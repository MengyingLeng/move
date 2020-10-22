import numpy as np
import ikfastpy

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def talker():
    rospy.init_node('pub_location', anonymous=True)
    angles_pub = rospy.Publisher("angle",Float32MultiArray, queue_size=1) 

    angles = np.array([0.0, -65.0, -80.0, 0.0, 125.0, 10.0],dtype='float32')
    #if the dimensions vary, then the the layout will need to be updated as well
    anglesmsg = Float32MultiArray()
    anglesmsg.layout.dim = []
    dims = np.array(angles.shape)
    anglesize = dims.prod()/float(angles.nbytes) # this is my attempt to normalize the strides size depending on .nbytes. not sure this is correct

    for i in range(0,len(dims)): #should be rather fast. 
        # gets the num. of dims of nparray to construct the message   
        anglesmsg.layout.dim.append(MultiArrayDimension())
        anglesmsg.layout.dim[i].size = dims[i]
        anglesmsg.layout.dim[i].stride = dims[i:].prod()/anglesize
        anglesmsg.layout.dim[i].label = 'dim_%d'%i

    while not rospy.is_shutdown():
        anglesmsg.data = np.frombuffer(angles.tobytes(),'float32') ## serializes
        angles_pub.publish(anglesmsg)


def callback(data):
    a = np.array(data.data)
    for i in range(len(a)):
        print(a[i])
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("angle", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()