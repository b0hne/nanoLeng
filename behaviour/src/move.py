#!/usr/bin/env python

import rospy

from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist

from smach import State, StateMachine
from time import sleep
from math import atan2, sqrt

ros_rate = 20
PI = 3.1415926535897


def from_degree(degree):
    return degree*PI/360


def scan_callback(tag):
    return

# def callback(self, tags):
#         # for tag in tags.msg:
#     print 'callback\n'
#     print len(tag)
# class leave_dockingbay(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['approach', 'done'], input_keys=['rate','found'], output_keys=['rate','found'])
#         print 'leave_dockingbay\n'

#     def execute(self, userdata)

# class take_position_0(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['succeeded', 'lost_visual'], input_keys=['rate','found'], output_keys=['rate','found'])
#         print 'take_position_0\n'

#     def execute(self, userdata)

# class searching_for_bay_0(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['approach', 'done'], input_keys=['rate','found'], output_keys=['rate','found'])
#         print 'search_for_bay_0\n'

#     def execute(self, userdata)
# 'approach', 'done', 'uncertain'

# look for tag nr. 1(leng)


class Looking_for_first_target(State):
    def __init__(self):
        State.__init__(self, outcomes=['found', 'not_found', 'failure'], input_keys=[
                       'tag'], output_keys=['tag'])
        self.outcome = None
        self.turn = 0

    def find_tag(self, tags, userdata):
        print 'found 1\n'
        for tag in tags.detections:
            print "tag :"
            print tag.id
            if tag.id == 0:
                self.tag = tag
                self.outcome = 'found'

    def execute(self, userdata):
        print 'looking_for_first_target\n'
        subscriber = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ros_rate)
        self.turn_left = Twist()
        self.turn_left.angular.z = 0.3  # from_degree(45)

        while self.outcome == None:
            # count if turns in self.turn
            if self.turn == 2:
                self.outcome = 'not_found'

            cmd_vel_pub.publish(self.turn_left)
            # print 'a'
            self.rate.sleep()
        print 'found end '
        userdata.tag = self.tag
        subscriber.unregister()
        cmd_vel_pub.unregister()
        return self.outcome


# approach tag in input
class Approaching_target(State):
    def __init__(self):
        State.__init__(self, outcomes=['reached', 'lost_visual'], input_keys=[
                       'tag'], output_keys=['tag'])
        print 'approaching_target\n'
        self.block = 0
        self.twist = None
        self.newPosition = False
        self.outcome = None
        self.tag = None
        self.counter = 0

    def calculate_twist(self):
        localTag = None
        while True:
            count = self.block
            if (count % 2) == 0:
                localTag = self.tag
            if self.block == count:
                break
        self.twist = Twist()

        # fix hight
        vertical_angle_to_tag = localTag.pose.pose.position.y
        print "vertical"
        print vertical_angle_to_tag

        horizontal_angle_to_tag = atan2(
            localTag.pose.pose.position.x, localTag.pose.pose.position.z)
        print "horrzontal"
        print horizontal_angle_to_tag

        distance_to_tag = sqrt(
            pow(localTag.pose.pose.position.x, 2) + pow(localTag.pose.pose.position.y, 2) + pow(localTag.pose.pose.position.z, 2))
        print 'distance'
        print distance_to_tag

        angle_to_tag = localTag.pose.pose.orientation.z
        print 'angle'
        print angle_to_tag

        # finish once tag is in reach
        if distance_to_tag < 1 and vertical_angle_to_tag < (0.1) and vertical_angle_to_tag > (-0.1) and horizontal_angle_to_tag < 0.1 and horizontal_angle_to_tag > -0.1 and angle_to_tag < 0.1 and angle_to_tag > -0.1:
            self.outcome = 'reached'
            return

        if vertical_angle_to_tag > 0.1:
            self.twist.linear.z = -0.1

        elif vertical_angle_to_tag < -0.1:
            self.twist.linear.z = 0.1

        if distance_to_tag < 1:
            if angle_to_tag < -0.1:
                self.twist.linear.y = 0.1

            elif angle_to_tag > 0.1:
                self.twist.linear.y = -0.1

        if angle_to_tag < 0.1:
            self.twist.linear

        # turn towards tag
        if horizontal_angle_to_tag < 0.1:
            self.twist.angular.z = 0.1

        elif horizontal_angle_to_tag > 0.1:
            self.twist.angular.z = -0.1

        # otherwise move forward
        if distance_to_tag < 0.5:
            self.twist.linear.x = -0.05
        elif distance_to_tag < 1.5:
            self.twist.linear.x = 0.05
        else:
            self.twist.linear.x = 0.1

    def callback(self, tags):
        print 'found 2\n'
        self.counter += 1
        for tag in tags.detections:
            if tag.id == self.id:
                print 'found tag'
                self.block += 1
                self.tag = tag
                self.block += 1
                self.counter = 0
                self.newPosition = True
        if self.counter > 15:
            self.twist = Twist()
            self.outcome = 'lost_visual'

    def execute(self, userdata):
        self.id = userdata.tag.id
        self.tag = userdata.tag
        self.calculate_twist()
        subscriber = rospy.Subscriber(
            '/tag_detections', AprilTagDetectionArray, self.callback)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(ros_rate)

        while self.outcome == None:
            if self.newPosition:
                self.calculate_twist()
                self.newPosition = False

            cmd_vel_pub.publish(self.twist)
            rate.sleep()
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        cmd_vel_pub.publish(self.twist)
        subscriber.unregister()
        cmd_vel_pub.unregister()
        userdata.tag = self.tag
        return self.outcome


# class Looking_for_target(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['found', 'not_found', 'failure'], input_keys=[
#                        'tag'], output_keys=['tag'])
#         self.outcome = None
#         self.turn = 0

#     def callback(self, tags):
#         print 'found \n'
#         for tag in tags.detections:
#             print "tag :" + tag.id
#             if tag.id == self.userdata.tag.id:
#                 self.userdata.tag = tag
#                 self.outcome = 'found'

#     def execute(self, userdata):
#         print 'looking_for_target' + self.userdata.tag.id + '\n'
#         april_sub = rospy.Subscriber(
#             '/tag_detections', AprilTagDetectionArray, self.callback)
#         cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
#         self.rate = rospy.Rate(ros_rate)
#         self.turn_left = Twist()
#         self.turn_left.angular.z = from_degree(45)

#         while self.outcome == None:
#             # count if turns in self.turn
#             if(turn == 2)
#                 self.outcome = 'not_found'

#             cmd_vel_pub.publish(self.turn_left)
#             # print 'a'
#             self.rate.sleep()

#         return self.outcome


# class facing_target(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['found', 'done'], input_keys=[
#                        'rate', 'found'], output_keys=['rate', 'found'])
#         print 'facing_target\n'

#     def callback(tags):
#         pass

#     def execute(self, userdata):
#         sub = rospy.Subscriber('tags_found', AprilTagDetectionArray, callback)
#         while True:
#             continue


# class searching_for_next_target(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['found', 'done'], input_keys=[
#                        'rate', 'found'], output_keys=['rate', 'found'])
#         print 'searching_for_next_target\n'

#     def callback(tags):
#         pass

#     def execute(self, userdata):
#         sub = rospy.Subscriber('tags_found', AprilTagDetectionArray, callback)
#         while True:
#             continue


# class searching_for_bay(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['found', 'done'], input_keys=[
#                        'rate', 'found'], output_keys=['rate', 'found'])
#         print 'searching_for_bay\n'

#     def callback(tags):
#         pass

#     def execute(self, userdata):
#         sub = rospy.Subscriber('tags_found', AprilTagDetectionArray, callback)
#         while True:
#             continue


# class docking(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['found', 'done'], input_keys=[
#                        'rate', 'found'], output_keys=['rate', 'found'])
#         print 'docking\n'

#     def callback(tags):
#         pass

#     def execute(self, userdata):
#         sub = rospy.Subscriber('tags_found', AprilTagDetectionArray, callback)
#         while True:
#             continue


# class Two(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['visible', 'not_visible'], input_keys=['counter', 'rate'], output_keys=['counter'])
#     def execute(self, userdata):
#         print 'two'
#         print userdata.counter
#         sleep(1)
#         if (userdata.counter % 5 == 0):
#             userdata.counter = userdata.counter + 1
#             return 'visible'
#         else:
#             userdata.counter = userdata.counter + 1
#             return 'not_visible'

# class Three(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['visible', 'not_visible'], input_keys=['counter', 'rate'], output_keys=['counter'])
#     def execute(self, userdata):
#         print 'two'
#         sleep(1)
#         if (userdata.counter % 2 == 0):
#             userdata.counter = userdata.counter + 1
#             return 'visible'
#         else:
#             userdata.counter = userdata.counter + 1
#             return 'not_visible'

if __name__ == '__main__':
    rospy.init_node("behaviour_output")
    discovered = []
    rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    sm = StateMachine(outcomes=['success', 'failure'])
    sm.userdata.tag = None
    with sm:
        # def result(userdata, sta)
        StateMachine.add('LOOKING_FOR_FIRST_TARGET', Looking_for_first_target(), transitions={
                         'found': 'APPROACHING_TARGET', 'not_found': 'LOOKING_FOR_FIRST_TARGET'}, remapping={'tag': 'tag', 'tag': 'tag'})
        StateMachine.add('APPROACHING_TARGET', Approaching_target(), transitions={
                         'reached': 'success', 'lost_visual': 'failure'}, remapping={'tag': 'tag', 'tag': 'tag'})
        # StateMachine.add('THREE', Three(), transitions={'visible':'ONE', 'not_visible':'TWO'}, remapping={'counter':'counter'})
    sm.execute()
