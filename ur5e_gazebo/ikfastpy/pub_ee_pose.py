import numpy as np


import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
# import ikfastpy
# import sys
# sys.path.append('~/mingying_ws/src/UR5e_simulation/ur5e_gazebo/ikfastpy')
import ikfastpy

t = 0
def talker():
    rospy.init_node("pub_location", anonymous=True)
    angles_pub = rospy.Publisher("angle",Float32MultiArray, queue_size=1) 
    rate = rospy.Rate(5)

    # ur5_kin = ikfastpy.PyKinematics()
    # n_joints = ur5_kin.getDOF()
    # theta = np.linspace(0.1, 2 * np.pi, 13)
    ee_post_left = np.array([[  0.3077197, -0.9420444, 0.1336445, 0.141997206704],
    [-0.9409226, -0.3221526, -0.1043186,-0.32326325541],
    [0.1413267, -0.0936482, -0.9855236, 0.273180083213]])

    ee_post_right = np.array([[  0.3077197, -0.9420444,  0.1336445, -0.14408809336],
    [-0.9409226, -0.3221526, -0.1043186,-0.331634687949],
    [0.1413267, -0.0936482, -0.9855236, 0.26008363963]])
    
    diff = ee_post_left[0][3] - ee_post_right[0][3]
    x = []
    y = []
    
    for i in range(301):
        x.append(ee_post_right[0][3] + diff * i / 300)
        y.append(ee_post_left[1][3])

    for i in range(301):
        x.append(ee_post_left[0][3] - diff * i / 300)
        y.append(ee_post_left[1][3])
    # last = np.array([ 1.14939153,-2.84839082,-0.97130454,-1.93128979,-1.18349659, 2.71537089],dtype='float32')
    while not rospy.is_shutdown():
    #while True:
        global t
        t = (t+1)%len(x)
        # ee_pose = np.array([[ 1,0,0,x[t]],
        # [0,0,-1,-0.2 + y[t]],
        # [1,1,0, 0.6]])
        # ee_pose = np.array([[ 0.56230879,0.48327458,0.67101002,x[t]],
        # [-0.80670726,0.14224425,0.57357645,y[t]],
        # [ 0.18174762,-0.86383575,0.46984631, 0.6]])
        # ee_pose = np.array([[ 0.56230879,0.48327458,0.67101002,x[t]* 0.6],
        # [-0.80670726,0.14224425,0.57357645,0.3],
        # [ 0.18174762,-0.86383575,0.46984631, 0.6]])
        # ee_pose = np.array([[ 1.0,0.0,0.0,x[t]],
        # [0.0,np.sqrt(2)/2,-np.sqrt(2)/2,y[t]],
        # [ 0.0,np.sqrt(2)/2,np.sqrt(2)/2,0.7]])
        ee_pose = ee_post_left
        ee_pose[0][3] = x[t]
        ee_pose[1][3] = y[t]
        # anglesmsg.data = np.frombuffer(angles.tobytes(),'float32') ## serializes
        anglesmsg = Float32MultiArray()
        # anglesmsg.data = np.frombuffer(ee_pose.tobytes(),'float32')
        anglesmsg.data = ee_pose.reshape(-1)
        angles_pub.publish(anglesmsg)
        print(ee_pose)

        rate.sleep()

if __name__ == '__main__':
    t = 1
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
