import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from time import sleep
from std_msgs.msg import Float64

class Logger:
    def __init__(self):
        rospy.init_node("logger")
        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")

        self.tfl.wait_for_server()
        
        self.state_pitch = 0
        self.state_yaw = 0
        self.input_arm = 0
        self.input_base = 0

        rospy.Subscriber("/sample_pcl/diabolo/pitch", Float64, self.callbackPitch)
        rospy.Subscriber("/sample_pcl/diabolo/yaw", Float64, self.callbackYaw)
        rospy.Subscriber("/base_odometry/odom", Odometry, self.callbackOdom)
    
    def callbackPitch(self, msg):
        self.state_pitch = msg.data
    
    def callbackYaw(self, msg):
        self.state_yaw = msg.data
    
    def callbackOdom(self, msg):
        self.input_base = msg.pose.pose.orientation.w
    
    def execute(self):
        while True:
            transform = self.tfl.lookup_transform("base_footprint", "r_gripper_tool_frame", rospy.Time(0))
            self.input_arm = transform.transform.translation.x
            print self.input_arm, self.input_base, self.state_pitch, self.state_yaw
            sleep(0.05)

if __name__ == '__main__':
    logger = Logger()
    logger.execute()
