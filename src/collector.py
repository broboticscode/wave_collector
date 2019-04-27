import rospy
import roslib
import numpy as np
import tf.transformations
from geometry_msgs.msg import Twist
import os
from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Path
import cv2
from rospy_message_converter import json_message_converter
import json


class Recorder:
    def __init__(self):
        rospy.init_node('cmd_vel_listener')
        rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, self.velocity)
        rospy.Subscriber("/webcam/image_raw", Image, self.image_raw)

        # rospy.init_node("lane_detector")
        self.bridge = CvBridge()

        # subscribe to images
        # rospy.Subscriber("left/image_rect_color", Image, self.on_left_image)
        # rospy.Subscriber("/vesc/low_level/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, self.on_teleop)
        rospy.loginfo("Received a /cmd_vel message!")


        self.image = None
        self.linear = None
        self.angular = None
        self.twist = None
        #No publishing just yet
        self.publish()
        rospy.spin()

    def velocity(self, msg):
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        self.linear = msg.linear
        self.angular = msg.angular
        self.twist = msg
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
        count = 0
        dir_number = len(os.walk("images").next()[1])
        save_path=os.path.join("images","collection_" + str(dir_number))
        os.mkdir(save_path)

        while not rospy.is_shutdown():

            if self.image is not None and self.linear is not None and self.angular is not None:
                rospy.loginfo("All vars initialised")

                try:
                    rospy.logwarn("saving image %s", count)
                    # cv2.imshow('image',self.image)
                    # cv2.waitKey(0)

                    image_path = os.path.join(save_path,str(count) + ".jpg")
                    json_path = os.path.join(save_path,str(count) + ".json")

                    cv2.imwrite(image_path, self.image)
                    json_str = json_message_converter.convert_ros_message_to_json(self.twist)
                    rospy.loginfo(json_str)

                    with open(json_path, 'w') as outfile:
                        outfile.write(json_str)
                         # json.dump(json_str, outfile)
                    count+=1
                except Exception as e:
                    rospy.logerr(e)

            rate.sleep()




if __name__ == "__main__":
    try:
        Recorder()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start recording program")
