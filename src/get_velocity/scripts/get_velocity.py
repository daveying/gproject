#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose2D, TwistStamped

length = 8

class getVelocity:
    """listion pose2D topic and caculate velocity"""

    def __init__(self):
        rospy.init_node('getVelocity', anonymous=True)
        self.laservelocity_pub = rospy.Publisher('/laserplatform_speeds',
                                                 TwistStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/platform_speeds', TwistStamped, queue_size=10)
        self.duration_time = 1.0/31.9
        self.poseArray = []
        self.velocityArray = []
        for i in range(length):
            self.poseArray.append(Pose2D()) 
            #= [Pose2D(), Pose2D(), Pose2D(), Pose2D(), Pose2D(), Pose2D(), Pose2D(),
            #              Pose2D(), Pose2D(), Pose2D()]
            self.velocityArray.append(TwistStamped())
        # = [TwistStamped(), TwistStamped(), TwistStamped(), TwistStamped(), TwistStamped(), 
         #                     TwistStamped(), TwistStamped(), TwistStamped(), TwistStamped(), TwistStamped()]
        self.current_position = Pose2D()
        self.last_position = Pose2D()
        self.velocity_laser = TwistStamped()
        self.velocity = TwistStamped()
        self.pose_count = 0
        self.velocity_count = 0
        self.bias = 0.164 - 0.56

    def getPose(self):
        """listion to pose2D topic"""
        rospy.Subscriber("pose2D", Pose2D, self.callback)
        rospy.spin()

    def pose_moveAverage(self, pose):
        """get average pose every 5 positions"""
        temp_x = 0
        temp_y = 0
        temp_theta = 0
        current_position = Pose2D()
        # update pose array and sum length position
        for i in range(length - 1):
            self.poseArray[i] = self.poseArray[i + 1]
            temp_x = temp_x + self.poseArray[i].x
            temp_y = temp_y + self.poseArray[i].y
            temp_theta = temp_theta + self.poseArray[i].theta
        self.poseArray[length - 1] = pose
        temp_x = temp_x + self.poseArray[length - 1].x
        temp_y = temp_y + self.poseArray[length - 1].y
        temp_theta = temp_theta + self.poseArray[length - 1].theta
        # get average position
        current_position.x = temp_x/length
        current_position.y = temp_y/length
        current_position.theta = temp_theta/length
        return current_position

    def velocity_moveAverage(self, velocity):
        """get average pose every 5 positions"""
        temp_x = 0
        temp_y = 0
        temp_theta = 0
        current_velocity = TwistStamped()
        # update pose array and sum 5 position
        for i in range(length - 1):
            self.velocityArray[i] = self.velocityArray[i + 1]
            temp_x = temp_x + self.velocityArray[i].twist.linear.x
            temp_y = temp_y + self.velocityArray[i].twist.linear.y
            temp_theta = temp_theta + self.velocityArray[i].twist.angular.z
        self.velocityArray[length - 1] = velocity
        temp_x = temp_x + self.velocityArray[length - 1].twist.linear.x
        temp_y = temp_y + self.velocityArray[length - 1].twist.linear.y
        temp_theta = temp_theta + self.velocityArray[length - 1].twist.angular.z
        # get average position
        current_velocity.twist.linear.x = temp_x/length
        current_velocity.twist.linear.y = temp_y/length
        current_velocity.twist.angular.z = temp_theta/length
        return current_velocity

    def callback(self, pose):
        """callback funciton."""
        if self.pose_count < length:
            self.poseArray[self.pose_count] = pose
            self.pose_count = self.pose_count + 1
            self.last_position = pose
        if self.pose_count == length:
            # using moving average to caculate velocity of laser platform
            self.current_position = self.pose_moveAverage(pose)

            # caculate velocity of laser
            self.velocity_laser.twist.linear.x = (self.current_position.x - self.last_position.x) / self.duration_time
            self.velocity_laser.twist.linear.y = (self.current_position.y - self.last_position.y) / self.duration_time
            self.velocity_laser.twist.angular.z = (self.current_position.theta - self.last_position.theta) / self.duration_time
           # print "velocity: ", self.velocity_laser

            # caculate AIMM platform velocity
            theta = self.current_position.theta
            self.velocity.twist.linear.x = self.velocity_laser.twist.linear.x + self.velocity_laser.twist.angular.z * self.bias * math.sin(theta)
            self.velocity.twist.linear.y = self.velocity_laser.twist.linear.y - self.velocity_laser.twist.angular.z * self.bias * math.cos(theta)
            self.velocity.twist.angular.z = self.velocity_laser.twist.angular.z
            print "velocity: ", self.velocity

            # save velocity to velocityArray
            if self.velocity_count < length:
                self.velocityArray[self.velocity_count] = self.velocity
                self.velocity_count = self.velocity_count + 1

            # use move average on velocity
            if self.velocity_count == length:
                self.velocity = self.velocity_moveAverage(self.velocity)

            #publish velocity topic
            self.velocity_laser.header.stamp = rospy.get_rostime()
            self.velocity.header.stamp = rospy.get_rostime()
            self.laservelocity_pub.publish(self.velocity_laser)
            self.velocity_pub.publish(self.velocity)

            # save current position as last_position
            self.last_position.x = self.current_position.x
            self.last_position.y = self.current_position.y
            self.last_position.theta = self.current_position.theta

if __name__ == '__main__':
    robotVel = getVelocity()
    robotVel.getPose()

