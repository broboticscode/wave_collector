import rospy
import roslib
import numpy as np
import tf.transformations
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Path
import cv2


class Recorder:
    def __init__(self):
        rospy.init_node('cmd_vel_listener')
        rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, velocity)
        rospy.Subscriber("/webcam/image_raw", Image, image_raw)

        # rospy.init_node("lane_detector")
        # self.bridge = CvBridge()

        # subscribe to images
        # rospy.Subscriber("left/image_rect_color", Image, self.on_left_image)
        # rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, self.on_teleop)
        rospy.loginfo("Received a /cmd_vel message!")


        self.image = None
        self.linear = None
        self.angular = None
        #No publishing just yet
        #self.publish()
        rospy.spin()

    def velocity(self, msg):
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        self.linear = msg.linear
        self.angular = msg.angular
    def image_raw(self, msg):
        rospy.loginfo("Captured image")
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # def on_left_image(self, data):
    #     try:
    #         self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         rospy.logerr(e)
    #
    # def on_teleop(self, msg):
    #     self.steer = msg.drive.steering_angle
    #     self.speed = msg.drive.speed

    def publish(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.left_image is not None and self.steer is not None and self.speed < 0:
                randint = round(np.random.rand(), 5)
                try:
                    rospy.logwarn("saving image with steer: %s, speed: %s", self.steer, self.speed)
                    cv2.imwrite("./" + str(randint) + "-leftimage_" + str(self.steer) + ".jpg", self.left_image)
                except Exception as e:
                    rospy.logerr(e)

            rate.sleep()




if __name__ == "__main__":
    try:
        Recorder()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start recording program")
