#!/usr/bin/env python

import rospy

from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from smach import State, StateMachine
from time import sleep
from math import atan2, sqrt
# from operator import itemgetter

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
                       'tags', 'furthest_tag_id'], output_keys=['tags', 'furthest_tag_id'])
        self.outcome = None
        self.turn = 0

    def find_tag(self, tags, userdata):
        for tag in tags.detections:
            print "tag :"
            print tag.id
            if tag.id == 0:
                userdata.tags.append(tag)
                userdata.furthest_tag_id = tag.id
                self.outcome = 'found'
                return
    def update_rotation(self, imu):
        pass
        # print 'heading :'
        # print imu.orientation.w

    def execute(self, userdata):
        print 'looking_for_first_target :\n'
        subscriber = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)
        subscriber = rospy.Subscriber('/rexrov/imu',
                                      Imu, self.update_rotation)
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
        subscriber.unregister()
        cmd_vel_pub.unregister()
        return self.outcome


# approach tag in input
class Approaching_target(State):
    def __init__(self):
        State.__init__(self, outcomes=['reached', 'lost_visual'], input_keys=[
                       'tags', 'furthest_tag_id'], output_keys=['tags', 'furthest_tag_id'])
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
        print 'moving towards:'
        print localTag.id
        # fix hight
        vertical_angle_to_tag = localTag.pose.pose.position.y
        # print "vertical"
        # print vertical_angle_to_tag

        horizontal_angle_to_tag = atan2(
            localTag.pose.pose.position.x, localTag.pose.pose.position.z)
        # print "horrzontal"
        # print horizontal_angle_to_tag

        distance_to_tag = sqrt(
            pow(localTag.pose.pose.position.x, 2) + pow(localTag.pose.pose.position.y, 2) + pow(localTag.pose.pose.position.z, 2))
        # print 'distance'
        # print distance_to_tag

        angle_to_tag = localTag.pose.pose.orientation.z
        # print 'angle'
        # print angle_to_tag

        # finish once tag is in reach
        if distance_to_tag < 1 and vertical_angle_to_tag < (0.1) and vertical_angle_to_tag > (-0.1) and horizontal_angle_to_tag < 0.1 and horizontal_angle_to_tag > -0.5 and angle_to_tag < 0.5 and angle_to_tag > -0.5:
            self.outcome = 'reached'
            return


        if vertical_angle_to_tag > 0.1:
            if vertical_angle_to_tag < 0.1:
                self.twist.linear.z = -0.1
            else:
                self.twist.linear.z = -0.15

        #get tag into middle of camera
        if vertical_angle_to_tag < -0.1:
            if vertical_angle_to_tag > -0.1:
                self.twist.linear.z = 0.1
            else:
                self.twist.linear.z = 0.15


        if distance_to_tag < 1.5:
            if angle_to_tag < -0.1:
                self.twist.linear.y = 0.05

            elif angle_to_tag > 0.1:
                self.twist.linear.y = -0.05
        else:
            if angle_to_tag < -0.1:
                self.twist.linear.y = 0.1

            elif angle_to_tag > 0.1:
                self.twist.linear.y = -0.1

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
        print 'found\n'
        self.counter += 1
        for tag in tags.detections:
            if tag.id == self.tag.id:
                print 'found tag'
                self.block += 1
                self.tag = tag
                self.block += 1
                self.counter = 0
                self.newPosition = True
        if self.counter > 20:
            self.twist = Twist()
            self.outcome = 'lost_visual'

    def execute(self, userdata):
        self.block = 0
        self.twist = None
        self.newPosition = False
        self.outcome = None
        self.tag = None
        self.counter = 0

        self.tag = userdata.tags[-1]
        subscriber = rospy.Subscriber(
            '/tag_detections', AprilTagDetectionArray, self.callback)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(ros_rate)

        self.calculate_twist()
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
        # userdata.tags.append(self.tag)
        return self.outcome


class Find_tag_again(State):
    def __init__(self):
        State.__init__(self, outcomes=['found', 'not_found', 'failure'], input_keys=[
                       'tags', 'furthest_tag_id'], output_keys=['tags', 'furthest_tag_id'])


    def find_tag(self, tags, userdata):
        for tag in tags.detections:
            print "tag :"
            print tag.id
            if tag.id == userdata.tags[-1].id:
                self.outcome = 'found'
                return
    def update_rotation(self, imu):
        pass
        # print 'heading :'
        # print imu.orientation.w

    def execute(self, userdata):
        self.outcome = None
        self.turn = 0
        print 'looking_for_first_target :\n'
        subscriber = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)
        subscriber = rospy.Subscriber('/rexrov/imu',
                                      Imu, self.update_rotation)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ros_rate)
        self.turn_right = Twist()
        self.turn_right.angular.z = -0.3  # from_degree(45)

        while self.outcome == None:
            # count if turns in self.turn
            if self.turn == 2:
                self.outcome = 'not_found'

            cmd_vel_pub.publish(self.turn_right)
            # print 'a'
            self.rate.sleep()
        subscriber.unregister()
        cmd_vel_pub.unregister()
        return self.outcome


# class Corner(State):
#      def __init__(self):
#         State.__init__(self, outcomes=['reached', 'lost_visual'], input_keys=[
#                        'tags', 'furthest_tag_id'], output_keys=['tags', 'furthest_tag_id'])
#         print 'approaching_target\n'
#         self.block = 0
#         self.twist = None
#         self.newPosition = False
#         self.outcome = None
#         self.tag = None
#         self.counter = 0

#     def calculate_twist(self):
#         localTag = None
#         while True:
#             count = self.block
#             if (count % 2) == 0:
#                 localTag = self.tag
#             if self.block == count:
#                 break
#         self.twist = Twist()
#         print 'moving towards:'
#         print localTag.id
#         # fix hight
#         # vertical_angle_to_tag = localTag.pose.pose.position.y
#         # print "vertical"
#         # print vertical_angle_to_tag

#         horizontal_angle_to_tag = atan2(
#             localTag.pose.pose.position.x, localTag.pose.pose.position.z)
#         # print "horrzontal"
#         # print horizontal_angle_to_tag

#         distance_to_tag = sqrt(
#             pow(localTag.pose.pose.position.x, 2) + pow(localTag.pose.pose.position.y, 2) + pow(localTag.pose.pose.position.z, 2))
#         # print 'distance'
#         # print distance_to_tag

#         angle_to_tag = localTag.pose.pose.orientation.z
#         # print 'angle'
#         # print angle_to_tag

#         # finish once tag is in reach
#         # if distance_to_tag < 1 and vertical_angle_to_tag < (0.1) and vertical_angle_to_tag > (-0.1) and horizontal_angle_to_tag < 0.1 and horizontal_angle_to_tag > -0.5 and angle_to_tag < 0.5 and angle_to_tag > -0.5:
#         #     self.outcome = 'reached'
#         #     return


#         if vertical_angle_to_tag > 0.1:
#             if vertical_angle_to_tag < 0.1:
#                 self.twist.linear.z = -0.1
#             else:
#                 self.twist.linear.z = -0.15

#         #get tag into middle of camera
#         if vertical_angle_to_tag < -0.1:
#             if vertical_angle_to_tag > -0.1:
#                 self.twist.linear.z = 0.1
#             else:
#                 self.twist.linear.z = 0.15


#         if distance_to_tag < 1.5:
#             if angle_to_tag < -0.1:
#                 self.twist.linear.y = 0.05

#             elif angle_to_tag > 0.1:
#                 self.twist.linear.y = -0.05
#         else:
#             if angle_to_tag < -0.1:
#                 self.twist.linear.y = 0.1

#             elif angle_to_tag > 0.1:
#                 self.twist.linear.y = -0.1

#         # turn towards tag
#         if horizontal_angle_to_tag < 0.1:
#             self.twist.angular.z = 0.1

#         elif horizontal_angle_to_tag > 0.1:
#             self.twist.angular.z = -0.1

#         # otherwise move forward
#         if distance_to_tag < 0.5:
#             self.twist.linear.x = -0.05
#         elif distance_to_tag < 1.5:
#             self.twist.linear.x = 0.05
#         else:
#             self.twist.linear.x = 0.1

#     def callback(self, tags):
#         print 'found\n'
#         self.counter += 1
#         for tag in tags.detections:
#             if tag.id == self.tag.id:
#                 print 'found tag'
#                 self.block += 1
#                 self.tag = tag
#                 self.block += 1
#                 self.counter = 0
#                 self.newPosition = True
#         if self.counter > 20:
#             self.twist = Twist()
#             self.outcome = 'lost_visual'

#     def execute(self, userdata):
#         self.block = 0
#         self.twist = None
#         self.newPosition = False
#         self.outcome = None
#         self.tag = None
#         self.counter = 0

#         self.tag = userdata.tags[-1]
#         subscriber = rospy.Subscriber(
#             '/tag_detections', AprilTagDetectionArray, self.callback)
#         cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
#         rate = rospy.Rate(ros_rate)

#         self.calculate_twist()
#         while self.outcome == None:
#             if self.newPosition:
#                 self.calculate_twist()
#                 self.newPosition = False

#             cmd_vel_pub.publish(self.twist)
#             rate.sleep()
#         self.twist.angular.x = 0.0
#         self.twist.angular.y = 0.0
#         self.twist.angular.z = 0.0
#         self.twist.linear.x = 0.0
#         self.twist.linear.y = 0.0
#         self.twist.linear.z = 0.0
#         cmd_vel_pub.publish(self.twist)
#         subscriber.unregister()
#         cmd_vel_pub.unregister()
#         # userdata.tags.append(self.tag)
#         return self.outcome


class Looking_for_next_target(State):
    def __init__(self):
        State.__init__(self, outcomes=['found_next', 'corner', '', 'failure'], input_keys=[
                       'tags', 'furthest_tag_id'], output_keys=['tags', 'furthest_tag_id'])
        self.outcome = None
        self.turn = 0

    def find_tag(self, tags, userdata):
        print 'findtag_callback\n'
        counter = -1
        last_used_tag = -1
        rightest_new_tag = -1
        rightest_position = -1

        for tag in tags.detections:
            print "found tag :"
            print tag.id
            counter += 1
            new_tag = True
            for old_tag in userdata.tags:
                if tag.id == old_tag.id:
                    print 'found old'
                    new_tag = False
            if tag.id == self.old_tag_id:
                print 'found last used'
                last_used_tag = counter
            if new_tag:
                print 'found new tag'
                if tag.pose.pose.position.x > rightest_position:
                    print 'new rightest tag'
                    rightest_position = tag.pose.pose.position.x
                    rightest_new_tag = counter

        print 'rigtest new tag'
        print rightest_new_tag
        if rightest_new_tag >= 0:
            # save new leftest element
            if self.old_tag_id == userdata.furthest_tag_id and rightest_new_tag < last_used_tag:
                userdata.furthest_tag_id = tags.detections[rightest_new_tag].id
                print 'furthest id'
                print userdata.furthest_tag_id

            userdata.tags.append(tags.detections[rightest_new_tag])
            self.outcome = 'found_next'

        # print 'findtag_callback\n'
        # last_used_tag = False
        # rightest_new_tag = -1
        # rightest_position = -1
        # tag_order = []

        # for tag in tags.detections:
        #     print "found tag :"
        #     print tag.id
        #     new_tag = True
        #     for old_tag in userdata.tags:
        #         if tag.id == old_tag.id:
        #             print 'found old'
        #             new_tag = False
        #     tag_order.append(tag)

        #     if tag.id == self.old_tag_id:
        #         print 'found last used'
        #         last_used_tag = True
        # if last_used_tag:
        #     if new_tag:
        #         print 'found new tag'
                            
        # else:
        #     self.outcome = 'lost_last_tag'
        #     # check for id 0

        # print 'rigtest new tag'
        # print rightest_new_tag
        # if rightest_new_tag >= 0:
        #     # save new leftest element
        #     if self.old_tag_id == userdata.furthest_tag_id and rightest_new_tag < last_used_tag:
        #         userdata.furthest_tag_id = tags.detections[rightest_new_tag].id
        #         print 'furthest id'
        #         print userdata.furthest_tag_id

        #     userdata.tags.append(tags.detections[rightest_new_tag])
        #     self.outcome = 'found_next'

    def execute(self, userdata):
        self.outcome = None
        self.turn = 0
        self.old_tag_id = userdata.tags[-1].id
        print 'looking_for_next_target\n'
        subscriber = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ros_rate)
        self.turn_left = Twist()
        self.turn_left.angular.z = 0.3  # from_degree(45)

        while self.outcome == None:
            # count if turns in self.turn
            # if self.turn == 2:
            #     self.outcome = 'not_found'

            cmd_vel_pub.publish(self.turn_left)
            print 'a'
            self.rate.sleep()
        print 'found next'
        subscriber.unregister()
        cmd_vel_pub.unregister()
        print 'tags'
        print len(userdata.tags)
        return self.outcome


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

    sm = StateMachine(outcomes=['success', 'failure', 'out_of_tags'])
    sm.userdata.tags = []
    sm.userdata.furthest_tag_id = -1
    with sm:
        # def result(userdata, sta)
        StateMachine.add('LOOKING_FOR_FIRST_TARGET', Looking_for_first_target(), transitions={
                         'found': 'APPROACHING_TARGET', 'not_found': 'failure'}, remapping={'tags': 'tags', 'furthest_tag_id': 'furthest_tag_id'})
        StateMachine.add('APPROACHING_TARGET', Approaching_target(), transitions={
                         'reached': 'LOOKING_FOR_NEXT_TARGET', 'lost_visual': 'FIND_TAG_AGAIN'}, remapping={'tags': 'tags', 'furthest_tag_id': 'furthest_tag_id'})
        StateMachine.add('LOOKING_FOR_NEXT_TARGET', Looking_for_next_target(), transitions={
                         'found_next': 'APPROACHING_TARGET', 'not_found': 'out_of_tags'}, remapping={'tags': 'tags', 'furthest_tag_id': 'furthest_tag_id'})
                         
        StateMachine.add('FIND_TAG_AGAIN', Find_tag_again(), transitions={
                         'found': 'APPROACHING_TARGET', 'not_found': 'failure'}, remapping={'tags': 'tags', 'furthest_tag_id': 'furthest_tag_id'})
        StateMachine.add('CORNER', Corner(), transitions={
                         'found': 'APPROACHING_TARGET', 'not_found': 'failure'}, remapping={'tags': 'tags', 'furthest_tag_id': 'furthest_tag_id'})
    sm.execute()
