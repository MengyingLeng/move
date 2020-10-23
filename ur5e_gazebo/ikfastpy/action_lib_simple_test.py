#!/usr/bin/env python
# import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import String

import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# import ikfastpy
# import sys
# sys.path.append('~/mingying_ws/src/UR5e_simulation/ur5e_gazebo/ikfastpy')
import ikfastpy

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
client = None
t = 0
last = [0,0,0,0,0,0]
x = []
y = []
ee_post_left = []
ee_post_right = []
diff = 0
def getXY():
    global x, y, ee_post_left, ee_post_right, diff
    ee_post_left = np.array([[  0.3077197, -0.9420444, 0.1336445, 0.141997206704],
    [-0.9409226, -0.3221526, -0.1043186,-0.32326325541],
    [0.1413267, -0.0936482, -0.9855236, 0.273180083213]])

    ee_post_right = np.array([[  0.3077197, -0.9420444,  0.1336445, -0.14408809336],
    [-0.9409226, -0.3221526, -0.1043186,-0.331634687949],
    [0.1413267, -0.0936482, -0.9855236, 0.26008363963]])
    
    diff = ee_post_left[0][3] - ee_post_right[0][3]
    # x = []
    # y = []
    
    for i in range(31):
        x.append(ee_post_right[0][3] + diff * i / 30)
        y.append(ee_post_left[1][3])

    for i in range(31):
        x.append(ee_post_left[0][3] - diff * i / 30)
        y.append(ee_post_left[1][3])

def move():
    global curr_angle, count, last
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # given the end-effector position, get joint angles
    ur5_kin = ikfastpy.PyKinematics()
    n_joints = ur5_kin.getDOF()
    
    global x, y, ee_post_left, ee_post_right, diff, t
    t = (t + 1)%len(x)
    ee_pose = ee_post_left
    ee_pose[0][3] = x[t]
    ee_pose[1][3] = y[t]

    print("pose",ee_pose)

    joint_configs = ur5_kin.inverse(ee_pose.reshape(-1).tolist())
    n_solutions = int(len(joint_configs)/n_joints)
    print(t, ee_pose[0][3], ee_pose[1][3], ee_pose[2][3])
    print(" %d solutions found:"%n_solutions)
    joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)

    if(len(joint_configs) == 0):
        joint_angles = last
        # print("not found")
    else:
        dis = []
        for i in range(len(joint_configs)):
            dis.append(np.linalg.norm(last - joint_configs[i],2))
        min_dis_index = dis.index(min(dis))
        joint_angles = joint_configs[min_dis_index]
        last = joint_angles
    # publish goal
    print ("angles",joint_angles)
    g.trajectory.points = [
        JointTrajectoryPoint(positions=joint_angles, velocities=[0]*6, time_from_start=rospy.Duration(1.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise


def process(msg):
    global curr_angle, count, last
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    # given the end-effector position, get joint angles
    ur5_kin = ikfastpy.PyKinematics()
    n_joints = ur5_kin.getDOF()
    # theta = np.linspace(0.1, 2 * np.pi, 13)
    # x = []
    # y = []
    # for i in range(len(theta)):
    #     x.append(np.sin(theta[i]))
    #     y.append(np.cos(theta[i]))
    # last = np.array([ 1.14939153,-2.84839082,-0.97130454,-1.93128979,-1.18349659, 2.71537089],dtype='float32')

    # global t
    # t = (t + 1)%13
    # ee_pose = np.array([[ 0.56230879,0.48327458,0.67101002,x[t]* 0.6 + 0.1],
    # [-0.80670726,0.14224425,0.57357645,y[t] *0.6 + 0.1],
    # [ 0.18174762,-0.86383575,0.46984631, 0.6]])
    ee_pose = np.array(msg.data).reshape(3,4)
    print("post",ee_pose)

    joint_configs = ur5_kin.inverse(ee_pose.reshape(-1).tolist())
    n_solutions = int(len(joint_configs)/n_joints)
    print(t, ee_pose[0][3], ee_pose[1][3], ee_pose[2][3])
    print(" %d solutions found:"%n_solutions)
    joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)

    if(len(joint_configs) == 0):
        joint_angles = last
        # print("not found")
    else:
        dis = []
        for i in range(len(joint_configs)):
            dis.append(np.linalg.norm(last - joint_configs[i],2))
        min_dis_index = dis.index(min(dis))
        joint_angles = joint_configs[min_dis_index]
        last = joint_angles
    # publish goal
    print ("angles",joint_angles)
    g.trajectory.points = [
        JointTrajectoryPoint(positions=joint_angles, velocities=[0]*6, time_from_start=rospy.Duration(1.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        getXY()
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        # client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"

        state = client.get_state
        print "state"
        print(state)
        
        # subscriber =rospy.Subscriber("angle",Float32MultiArray, process,queue_size=1)
        rate = rospy.Rate(5)
        # move()
        print('2')

        while not rospy.is_shutdown():
            move()
            rate.sleep()
            print('1')
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
