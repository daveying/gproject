#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose2D, TwistStamped

class getVelocity:
    """listion pose2D topic and caculate velocity"""

    def __init__(self):
        rospy.init_node('getVelocity', anonymous=True)
        self.laservelocity_pub = rospy.Publisher('/laserplatform_speeds',
                                                 TwistStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/platform_speeds', TwistStamped, queue_size=10)
        self.duration_time = 0.033
        self.poseArray = [Pose2D(), Pose2D(), Pose2D(), Pose2D(), Pose2D(), Pose2D(), Pose2D(),
                          Pose2D(), Pose2D(), Pose2D()]
        self.current_position = Pose2D()
        self.last_position = Pose2D()
        self.velocity_laser = TwistStamped()
        self.velocity = TwistStamped()
        self.count = 0
        self.bias = 0.165

    def getPose(self):
        """listion to pose2D topic"""
        rospy.Subscriber("pose2D", Pose2D, self.callback)
        rospy.spin()

    def moveAverage(self, pose):
        """get average pose every 5 positions"""
        temp_x = 0
        temp_y = 0
        temp_theta = 0
        # update pose array and sum 5 position
        for i in range(9):
            self.poseArray[i] = self.poseArray[i + 1]
            temp_x = temp_x + self.poseArray[i].x
            temp_y = temp_y + self.poseArray[i].y
            temp_theta = temp_theta + self.poseArray[i].theta
        self.poseArray[9] = pose
        temp_x = temp_x + self.poseArray[9].x
        temp_y = temp_y + self.poseArray[9].y
        temp_theta = temp_theta + self.poseArray[9].theta
        # get average position
        self.current_position.x = temp_x/10.0
        self.current_position.y = temp_y/10.0
        self.current_position.theta = temp_theta/10.0
        #return self.current_position

    def callback(self, pose):
        """callback funciton."""
        if self.count < 10:
            self.poseArray[self.count] = pose
            self.count = self.count + 1
            self.last_position = pose
        if self.count == 10:
            # using moving average to caculate position of laser platform
            self.moveAverage(pose)

            # caculate laser velocity
            self.velocity_laser.twist.linear.x = (self.current_position.x - self.last_position.x) / self.duration_time
            self.velocity_laser.twist.linear.y = (self.current_position.y - self.last_position.y) / self.duration_time
            self.velocity_laser.twist.angular.z = (self.current_position.theta - self.last_position.theta) / self.duration_time
            print "laser velocity: ", self.velocity_laser

            # caculate AIMM platform velocity
            theta = self.current_position.theta
            self.velocity.twist.linear.x = self.velocity_laser.twist.linear.x + self.velocity_laser.twist.angular.z * self.bias * math.sin(theta)
            self.velocity.twist.linear.y = self.velocity_laser.twist.linear.y - self.velocity_laser.twist.angular.z * self.bias * math.cos(theta)
            self.velocity.twist.angular.z = self.velocity_laser.twist.angular.z
            print "platform velocity: ", self.velocity

            # set time stamp
            self.velocity_laser.header.stamp = rospy.get_rostime()
            self.velocity.header.stamp = rospy.get_rostime()

            # publish topic
            self.laservelocity_pub.publish(self.velocity_laser)
            self.velocity_pub.publish(self.velocity)

            # save current position as last_position
            self.last_position.x = self.current_position.x
            self.last_position.y = self.current_position.y
            self.last_position.theta = self.current_position.theta

if __name__ == '__main__':
    robotVel = getVelocity()
    robotVel.getPose()
