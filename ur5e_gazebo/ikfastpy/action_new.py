#!/usr/bin/env python
# import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import String

import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState
import message_filters
import ikfastpy
# import sys
# sys.path.append('/home/mengying/catkin_ws/src/UR5e_simulation/ur5e_gazebo/ikfastpy')
# import ikfastpy

class Listener:

    def __init__(self):
        # client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)

        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        self.t = 0
        self.last = [0,0,0,0,0,0]
        self.time_step = 0.05
        self.max_acc = 1
 
        self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"

        # self.state = client.get_state
        # print "state"
        # print(self.state)
        # subscriber =rospy.Subscriber("target_angles",String, process,queue_size=1)
        # print("last 1")
        # print(last)
        # rospy.Subscriber("joint_states", JointState, self.joint_state_callback, queue_size=1)
        # print("last 2")
        # print(last)
        # rospy.Subscriber("angle",Float32MultiArray, self.process,queue_size=1)
        state_sub = message_filters.Subscriber("joint_states", JointState,  queue_size=1)
        target_sub = message_filters.Subscriber("angle",Float32MultiArray, queue_size=1)
        ats = message_filters.ApproximateTimeSynchronizer([state_sub,target_sub], queue_size=5, slop = 0.1, allow_headerless=True)
        ats.registerCallback(self.atsCallback)
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            rate.sleep()

    def atsCallback(self, cur_state, target):
 
        print "listen to joint state"
        msg = cur_state
        state = {}

        #get current state
        for i, name in enumerate(msg.name):
            if name in self.JOINT_NAMES:
                # print(i, name, msg.position[i])
                state[name] = msg.position[i]

        for i in range(len(self.JOINT_NAMES)):
            self.last[i] = state.get(self.JOINT_NAMES[i])
        print(self.last)

        # initial ik solver
        
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        # g.trajectory.header = Header()
        # g.trajectory.header.stamp = rospy.Time.now()
        g.trajectory.joint_names = self.JOINT_NAMES

        # given the end-effector position, get joint angles
        ur5_kin = ikfastpy.PyKinematics()
        n_joints = ur5_kin.getDOF()

        prev_ee_pose = ur5_kin.forward(self.last)
        prev_ee_pose = np.asarray(prev_ee_pose).reshape(3,4)
        
        ee_pose = np.array(target.data).reshape(3,4)

        # get distance between current state to target, velocity
        diff = prev_ee_pose - ee_pose
        dist = np.linalg.norm(diff[:,-1])
        dist_max = 1/2.0 * 1 * self.time_step**2
        print(ee_pose)
        print("dist", dist, "dist max", dist_max)

        joint_configs = ur5_kin.inverse(ee_pose.reshape(-1).tolist())
        n_solutions = int(len(joint_configs)/n_joints)
        # print(t, ee_pose[0][3], ee_pose[1][3], ee_pose[2][3])
        # print(" %d solutions found:"%n_solutions)
        joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)
        print("diff")
        print(diff)
        if(len(joint_configs) == 0):
            joint_angles = self.last
            # print("not found")
        # if (dist < dist_max):
        #     diff_list = 
        else:
            for i in range(10):
                # divided a target into 10 small target 
                if (dist < dist_max):
                    diff_i = diff[0,3] * i / 10
                    print('1')
                else:
                    # d = 1/2 * a * (t+dt)^2 - 1/2 * a * t^2 = a * dt + a/2
                    # d1 = ((i + 1) * self.time_step / 10)**2 
                    # d2 = (i * self.time_step / 10)**2 
                    # diff_i = self.max_acc * (d1-d2)/2.0
                    # diff_i = dist_max/20.0 * i
                    diff_i = diff[0,3] * i / 10
                    # print('2', self.max_acc, self.time_step, diff_i, d1, d2)
                # print("diff_i ", diff_i)
                
                ee_pose_i = ee_pose
       
                ee_pose_i[1,3] = prev_ee_pose[0,3] + diff_i
                # print("ee_pose_i",ee_pose_i)
                # prev_ee_pose = ee_pose_i
                
                joint_configs = ur5_kin.inverse(ee_pose_i.reshape(-1).tolist())
                # n_solutions = int(len(joint_configs)/n_joints)
                # joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)

                if(len(joint_configs) == 0):
                    joint_angles = self.last
                    continue
                else:
                    n_solutions = int(len(joint_configs)/n_joints)
                    joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)
                    dis = []
                    for j in range(len(joint_configs)):
                        dis.append(np.linalg.norm(self.last - joint_configs[j],2))
                    min_dis_index = dis.index(min(dis))
                    joint_angles = joint_configs[min_dis_index]
                    self.last = joint_angles
                    # publish goal
                    # print ("angles",joint_angles)
                    # print("i ", i)
                    #         g.trajectory.points = [
                    # JointTrajectoryPoint(positions=joint_angles, velocities=[0]*6, time_from_start=rospy.Duration(1.0))]
                    # g.trajectory.joint_names.append(self.JOINT_NAMES)
                point = JointTrajectoryPoint(positions=joint_angles, velocities=[0]*6, time_from_start=rospy.Duration(i * 0.05 + 1.0))

                g.trajectory.points.append(point)
        # else:
        #     dis = []
        #     for i in range(len(joint_configs)):
        #         dis.append(np.linalg.norm(self.last - joint_configs[i],2))
        #     min_dis_index = dis.index(min(dis))
        #     joint_angles = joint_configs[min_dis_index]
        #     self.last = joint_angles
        # # publish goal
        # print ("angles",joint_angles)
        # g.trajectory.points = [
        #     JointTrajectoryPoint(positions=joint_angles, velocities=[0]*6, time_from_start=rospy.Duration(1.0))]
        print("g",g)
        self.client.send_goal(g)
        # try:
        #     self.client.wait_for_result()
        # except KeyboardInterrupt:
        #     self.client.cancel_goal()
        #     raise
        
    def joint_state_callback(self,msg):
        global last
        print "listen to joint state"
        state = {}
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES:
                # print(i, name, msg.position[i])
                state[name] = msg.position[i]
        # # newlist = sorted(state, key = JOINT_NAMES)
        # # for name in JOINT_NAMES:
        # #     print(name, msg.name.position)
        # # state.sort(key = lambda i[0] : JOINT_NAMES.index(i[0]))
        # print("state")
        # print(state)
        for i in range(len(JOINT_NAMES)):
            last[i] = state.get(JOINT_NAMES[i])

        return
    def process(self,msg):
        global curr_angle, count, last, JOINT_NAMES
        # rospy.init_node("joint")
        # print("last 1")
        # print(last)
        # rospy.Subscriber("joint_states", JointState, joint_state_callback, queue_size=1)
        
        # print("last 2")
        # print(last)
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
        # print("post",ee_pose)

        joint_configs = ur5_kin.inverse(ee_pose.reshape(-1).tolist())
        n_solutions = int(len(joint_configs)/n_joints)
        # print(t, ee_pose[0][3], ee_pose[1][3], ee_pose[2][3])
        # print(" %d solutions found:"%n_solutions)
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
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        listener = Listener()
        # # client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        # client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # print "Waiting for server..."
        # client.wait_for_server()
        # print "Connected to server"

        # state = client.get_state
        # print "state"
        # print(state)
        # # subscriber =rospy.Subscriber("target_angles",String, process,queue_size=1)
        # subscriber =rospy.Subscriber("angle",Float32MultiArray, process,queue_size=1)
        # rate = rospy.Rate(2)

        # while not rospy.is_shutdown():
        #     rate.sleep()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()