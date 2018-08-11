#!/usr/bin/env python

import rospy

from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from smach import State, StateMachine
from time import sleep
from math import atan2, sqrt
from operator import itemgetter

ROS_RATE = 20
PI = 3.1415926535897
MAX_TAG_DISTANCE = 0.6
ABBORT_TAG = 586


def from_degree(degree):
    return degree*PI/360


# def scan_callback(tag):
#     return

    #count rotations for 'not_found' or 'corner'
def update_rotation(imu, self):
    if self.positive and imu.orientation.w > 0:
        self.half_turn +=1
        self.positive = False
    if not self.positive and imu.orientation.w < 0:
        self.half_turn +=1
        self.positive = True

def check_for_signal(tag):
    if tag.id == ABBORT_TAG:
        return 'abbort'
    else:
        return None

def calculate_twist(self):
    self.localTag = None
    while True:
        count = self.block_counter
        if (count % 2) == 0:
            self.localTag = self.main_tag
        if self.block_counter == count:
            break
    self.twist = Twist()
    
    #get tag into vertical middle of camera
    vertical_angle_to_tag = self.localTag.pose.pose.position.y
    self.vertical_angle_to_tag = vertical_angle_to_tag
    if vertical_angle_to_tag > 0.1:
        self.twist.linear.z = -0.15
    elif vertical_angle_to_tag < -0.1:
        self.twist.linear.z = 0.15

    #get tag into horizontal middle of camera
    horizontal_angle_to_tag = atan2(self.localTag.pose.pose.position.x, self.localTag.pose.pose.position.z)
    self.horizontal_angle_to_tag = horizontal_angle_to_tag
    if horizontal_angle_to_tag < -0.1:
        self.twist.angular.z = 0.15
    elif horizontal_angle_to_tag > 0.1:
        self.twist.angular.z = -0.15

    #move forwards
    distance_to_tag = sqrt(
        pow(self.localTag.pose.pose.position.x, 2) + pow(self.localTag.pose.pose.position.y, 2) + pow(self.localTag.pose.pose.position.z, 2))
    self.distance_to_tag = distance_to_tag
    if distance_to_tag < 0.5:
        self.twist.linear.x = -0.05
    elif distance_to_tag < 0.8:
        self.twist.linear.x = 0.05
    else:
        self.twist.linear.x = 0.1

    #         # allign to tag
    # if distance_to_tag < 0.6:
    #         self.twist.linear.y = 0.05



class Looking_for_first_target(State):
    def __init__(self):
        State.__init__(self, outcomes=['found', 'not_found', 'failure'], input_keys=[
                       'tags'], output_keys=['tags'])

    def find_tag(self, tags, userdata):
        for tag in tags.detections:
            if tag.id == 0 and self.outcome == None:
                userdata.tags.append(tag)
                self.outcome = 'found'
                return



    def execute(self, userdata):
        self.outcome = None
        self.half_turn = 0
        self.positive = True
        print 'looking_for_first_target :\n'
        subscriber = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)
        subscriber = rospy.Subscriber('/rexrov/imu',
                                      Imu, update_rotation, self)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ROS_RATE)
        self.turn_left = Twist()
        self.turn_left.angular.z = 0.3  # from_degree(45)

        while self.outcome == None:
            # count if turns in self.turn
            if self.half_turn > 2:
                self.outcome = 'not_found'

            cmd_vel_pub.publish(self.turn_left)
            self.rate.sleep()
        print 'found initial tag'
        cmd_vel_pub.publish(Twist())
        cmd_vel_pub.publish(Twist())
        subscriber.unregister()
        cmd_vel_pub.unregister()
        return self.outcome


# approach tag in input
class Approaching_target(State):
    def __init__(self):
        State.__init__(self, outcomes=['reached', 'lost_visual', 'abbort'], input_keys=[
                       'tags'], output_keys=['tags'])

    def calculate_twist(self):
        # localTag = None
        # while True:
        #     count = self.block_counter
        #     if (count % 2) == 0:
        #         localTag = self.tag
        #     if self.block_counter == count:
        #         break
        # self.twist = Twist()
        # print 'moving towards:'
        # print localTag.id

        # #get tag into vertical middle of camera
        # vertical_angle_to_tag = localTag.pose.pose.position.y
        # if vertical_angle_to_tag > 0.1:
        #     self.twist.linear.z = -0.15
        # elif vertical_angle_to_tag < -0.1:
        #     self.twist.linear.z = 0.15

        # #get tag into horizontal middle of camera
        # horizontal_angle_to_tag = atan2(localTag.pose.pose.position.x, localTag.pose.pose.position.z)

        # if horizontal_angle_to_tag < -0.1:
        #     self.twist.angular.z = 0.15

        # elif horizontal_angle_to_tag > 0.1:
        #     self.twist.angular.z = -0.15

        # #move forwards
        # distance_to_tag = sqrt(
        #     pow(localTag.pose.pose.position.x, 2) + pow(localTag.pose.pose.position.y, 2) + pow(localTag.pose.pose.position.z, 2))
        # if distance_to_tag < 0.8:
        #     self.twist.linear.x = -0.05
        # elif distance_to_tag < 0.9:
        #     self.twist.linear.x = 0.05
        # else:
        #     self.twist.linear.x = 0.1
        calculate_twist(self)

                # allign to tag
        angle_to_tag = self.main_tag.pose.pose.orientation.z
        if self.distance_to_tag < 1.5:
            if angle_to_tag < -0.1:
                self.twist.linear.y = 0.05

            elif angle_to_tag > 0.1:
                self.twist.linear.y = -0.05
        else:
            if angle_to_tag < -0.1:
                self.twist.linear.y = 0.1

            elif angle_to_tag > 0.1:
                self.twist.linear.y = -0.1  

         # finish once tag is in reach
        if self.distance_to_tag < 1 and self.vertical_angle_to_tag < (0.1) and self.vertical_angle_to_tag > (-0.1) and self.horizontal_angle_to_tag < 0.1 and self.horizontal_angle_to_tag > -0.1 and angle_to_tag < 0.2 and angle_to_tag > -0.3:
            self.outcome = 'reached'
            return



    def get_tags(self, tags):
        self.counter += 1
        for tag in tags.detections:
            if check_for_signal(tag) != None:
                self.outcome = check_for_signal(tag)
                return
            if tag.id == self.main_tag.id:
                self.block_counter += 1
                self.main_tag = tag
                self.block_counter += 1
                self.counter = 0
                self.newPosition = True
        if self.counter > 15:
            self.outcome = 'lost_visual'

    def execute(self, userdata):
        self.block_counter = 0
        self.twist = None
        self.newPosition = False
        self.outcome = None
        self.main_tag = userdata.tags[-1]
        self.counter = 0
        subscriber_tag = rospy.Subscriber(
            '/tag_detections', AprilTagDetectionArray, self.get_tags)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(ROS_RATE)

        self.calculate_twist()
        while self.outcome == None:
            if self.newPosition:
                self.calculate_twist()
                self.newPosition = False

            cmd_vel_pub.publish(self.twist)
            rate.sleep()
        print 'reached'
        print self.main_tag.id
        self.twist = Twist()
        cmd_vel_pub.publish(self.twist)
        subscriber_tag.unregister()
        cmd_vel_pub.unregister()
        return self.outcome


class Find_tag_again(State):
    def __init__(self):
        State.__init__(self, outcomes=['found', 'not_found', 'failure', 'abbort'], input_keys=[
                       'tags'], output_keys=['tags'])


    def find_tag(self, tags, userdata):
        for tag in tags.detections:
            if check_for_signal != None:
                self.outcome = check_for_signal(tag)
                return
            if tag.id == userdata.tags[-1].id:
                self.outcome = 'found'
                # return


    def execute(self, userdata):
        self.outcome = None
        self.half_turn = 0
        self.positive = True
        print 'looking_for_lost_tag'
        subscriber_tag = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)
        subscriber_imu = rospy.Subscriber('/rexrov/imu',
                                      Imu, update_rotation, self)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ROS_RATE)
        self.turn_right = Twist()
        self.turn_right.angular.z = -0.3  # from_degree(45)

        while self.outcome == None:
            if self.half_turn > 2:
                self.outcome = 'not_found'
            cmd_vel_pub.publish(self.turn_right)
            self.rate.sleep()
        print 'outcome :'
        print self.outcome
        twist = Twist()
        cmd_vel_pub.publish(twist)
        subscriber_tag.unregister()
        subscriber_imu.unregister()
        cmd_vel_pub.unregister()
        return self.outcome


class Corner(State):
    def __init__(self):
        State.__init__(self, outcomes=['found_next', 'failure', 'abbort'], input_keys=[
                       'tags'], output_keys=['tags'])

    def find_tag(self, tags, userdata):
        #order from left to right
        tags.detections.sort(key=lambda x: x.pose.pose.position.x)
        counter = -1
        corner = -1
        new_tag = -1
        for tag in tags.detections:
            signal = check_for_signal(tag)
            if signal != None:
                self.outcome = signal
                return
            counter += 1
            if tag.id == self.corner_tag.id:
                self.block_counter += 1
                self.corner_tag = tag
                self.block_counter += 1
                corner = counter
                # break
        if corner > 0:
            new_tag = corner - 1
            dif_x = tags.detections[new_tag].pose.pose.position.x - tags.detections[corner].pose.pose.position.x
            dif_y = tags.detections[new_tag].pose.pose.position.y - tags.detections[corner].pose.pose.position.y
            distance_tags = sqrt(pow(dif_x, 2) + pow(dif_y, 2))
            distance_to_new_tag = sqrt(pow(tags.detections[corner -1].pose.pose.position.x, 2) + pow(tags.detections[corner -1].pose.pose.position.y, 2) + pow(tags.detections[corner -1].pose.pose.position.z, 2))

            # if distance_tags > MAX_TAG_DISTANCE:
            if distance_tags < 0.8 and distance_to_new_tag < 1 and self.outcome == None:
                new_tag = corner - 1
                userdata.tags.append(tags.detections[new_tag])
                self.outcome = 'found_next'
        if corner < 0:
            self.timeout += 1

    def calculate_twist(self):
        localTag = None
        while True:
            count = self.block_counter
            if (count % 2) == 0:
                localTag = self.corner_tag
            if self.block_counter == count:
                break
        self.twist = Twist()
        # fix hight
        #get tag into vertical middle of camera
        vertical_angle_to_tag = localTag.pose.pose.position.y
        if vertical_angle_to_tag > 0.1:
            self.twist.linear.z = -0.15
        elif vertical_angle_to_tag < -0.1:
            self.twist.linear.z = 0.15

        #get tag into horizontal middle of camera
        horizontal_angle_to_tag = atan2(localTag.pose.pose.position.x, localTag.pose.pose.position.z)

        if horizontal_angle_to_tag < -0.1:
            self.twist.angular.z = 0.15

        elif horizontal_angle_to_tag > 0.1:
            self.twist.angular.z = -0.15

        #move forwards
        distance_to_tag = sqrt(
            pow(localTag.pose.pose.position.x, 2) + pow(localTag.pose.pose.position.y, 2) + pow(localTag.pose.pose.position.z, 2))
        if distance_to_tag < 0.5:
            self.twist.linear.x = -0.05
        elif distance_to_tag > 0.7:
            self.twist.linear.x = 0.05

                # allign to tag
        if distance_to_tag < 0.7:
                self.twist.linear.y = 0.05


    def execute(self, userdata):
        self.outcome = None
        self.half_turn = 0
        self.positive = True
        self.corner_tag = userdata.tags[-1]
        self.block_counter = 0
        self.timeout = 0
        self.twist = None
        print 'corner handling'

        subscriber_tag = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)

        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ROS_RATE)

        while self.outcome == None:
            # count if turns in self.turn
            
            self.calculate_twist()
            
            cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
            if self.timeout > 8:
                self.outcome = 'failure'
        print 'found next'
        print userdata.tags[-1].id
        twist = Twist()
        cmd_vel_pub.publish(twist)
        subscriber_tag.unregister()
        cmd_vel_pub.unregister()
        return self.outcome



class Looking_for_next_target(State):
    def __init__(self):
        State.__init__(self, outcomes=['found_next', 'corner', 'finished_row', 'failure', 'abbort'], input_keys=[
                       'tags'], output_keys=['tags'])


    def find_tag(self, tags, userdata):
        #order from left to right
        counter = -1
        last_used_tag = -1
        new_tag = -1
        tags.detections.sort(key=lambda x: x.pose.pose.position.x)
        for tag in tags.detections:
            signal = check_for_signal(tag)
            if signal != None:
                self.outcome = signal
                return
            counter += 1
            if tag.id == self.old_tag_id:
                last_used_tag = counter
                # break
        if last_used_tag >= 0 and self.half_turn >= 2:
            self.outcome = 'corner'
            return
        if last_used_tag > 0:
            new_tag = last_used_tag - 1
            dif_x = tags.detections[new_tag].pose.pose.position.x - tags.detections[last_used_tag].pose.pose.position.x
            dif_y = tags.detections[new_tag].pose.pose.position.y - tags.detections[last_used_tag].pose.pose.position.y
            distance_tags = sqrt(pow(dif_x, 2) + pow(dif_y, 2))
            if distance_tags < MAX_TAG_DISTANCE and self.outcome == None:
                new_tag = last_used_tag - 1
                userdata.tags.append(tags.detections[new_tag])
                if tags.detections[new_tag].id == 0:
                    self.outcome = 'finished_row'
                else:
                    self.outcome = 'found_next'
        
        

    def execute(self, userdata):
        self.outcome = None
        self.half_turn = 0
        self.positive = True
        self.old_tag_id = userdata.tags[-1].id
        print 'looking_for_next_target\n'
        subscriber_tag = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)
        subscriber_imu = rospy.Subscriber('/rexrov/imu',
                                      Imu, update_rotation, self)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ROS_RATE)
        self.turn_left = Twist()
        self.turn_left.angular.z = 0.3  # from_degree(45)

        while self.outcome == None:
            # count if turns in self.turn
            if self.half_turn > 2:
                self.outcome = 'failure'

            cmd_vel_pub.publish(self.turn_left)
            self.rate.sleep()
        print 'outcome :'
        print self.outcome
        twist = Twist()
        cmd_vel_pub.publish(twist)
        subscriber_tag.unregister()
        subscriber_imu.unregister()
        cmd_vel_pub.unregister()
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


class Abbort(State):
    def __init__(self):
        State.__init__(self, outcomes=['ready_to_dock', 'corner', 'no_old_tags', 'failure'], input_keys=['tags'], output_keys=['tags'])     
    
    def find_oldest_tag(self, tags, userdata):
        counter = -1
        if self.outcome == None:
            for old_tag in userdata.tags:
                counter += 1
                for tag in tags.detections:
                    #if oldest tag is found
                    if counter > self.index:
                        self.lost += 1
                        return
                    if self.main_tag != None and tag.id == self.main_tag.id:
                        self.block_counter +=1
                        self.main_tag = tag
                        self.block_counter +=1
                        distance_to_tag = sqrt(pow(tag.pose.pose.position.x, 2) + pow(tag.pose.pose.position.y, 2) + pow(tag.pose.pose.position.z, 2))
                        if distance_to_tag < 0.7:
                            self.reached = True
                            if tag.id == 0:
                                self.outcome = 'ready_to_dock'
                        self.lost = 0
                        return
                    if tag.id == old_tag.id:
                        self.index = counter
                        if self.main_tag != None and tag.id != self.main_tag.id:
                            self.reached = False
                        counter = 0
                        self.block_counter +=1
                        self.main_tag = tag
                        self.block_counter +=1
                        print 'tag :'
                        print self.main_tag
                        self.lost = 0
                        return

    def execute(self, userdata):
        self.outcome = None
        self.half_turn = 0
        self.positive = True
        self.block_counter = 0
        self.index = len(userdata.tags) -1
        self.main_tag = None
        self.reached = False
        self.twist = None
        self.lost = 0
        print 'abbort, returning home'
        subscriber_tag = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_oldest_tag, userdata)
        subscriber_imu = rospy.Subscriber('/rexrov/imu',
                                      Imu, update_rotation, self)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ROS_RATE)
        self.turn_right = Twist()
        # while self.main_tag == None:
        #     pass
        while self.outcome == None:
            if self.main_tag != None and not self.reached:
                calculate_twist(self)
            else:
                self.twist = Twist()
                self.twist.angular.z = -0.3
            cmd_vel_pub.publish(self.twist)
            if self.half_turn > 2 and self.lost == 0:
                self.outcome = 'corner'
            self.rate.sleep()
        print 'outcome :'
        print self.outcome
        twist = Twist()
        cmd_vel_pub.publish(twist)
        subscriber_tag.unregister()
        subscriber_imu.unregister()
        cmd_vel_pub.unregister()
        return self.outcome

class Abbort_lost_tags(State):
    def __init__(self):
        State.__init__(self, outcomes=['found', 'failure'], input_keys=['tags'], output_keys=['tags'])
    def find_tag(self, tags, userdata):
        if self.outcome == None:
            for old_tag in userdata.tags:
                for tag in tags.detections:
                    if tag.id == old_tag.id:
                        self.outcome = 'found'
                        return


    def execute(self, userdata):
        self.outcome = None
        self.half_turn = 0
        self.positive = True
        print 'abbort_lost_tags'
        subscriber_tag = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_tag, userdata)
        subscriber_imu = rospy.Subscriber('/rexrov/imu',
                                      Imu, update_rotation, self)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ROS_RATE)
        self.turn_right = Twist()
        self.turn_right.angular.z = -0.3  # from_degree(45)

        while self.outcome == None:
            if self.half_turn > 2:
                self.outcome = 'not_found'
            cmd_vel_pub.publish(self.turn_right)
            self.rate.sleep()
        print 'outcome :'
        print self.outcome
        twist = Twist()
        cmd_vel_pub.publish(twist)
        subscriber_tag.unregister()
        subscriber_imu.unregister()
        cmd_vel_pub.unregister()
        return self.outcome


class Abbort_corner(State):
    def __init__(self):
        State.__init__(self, outcomes=['found', 'failure'], input_keys=['tags'], output_keys=['tags'])

    def find_oldest_tag(self, tags, userdata):
        counter = -1
        if self.outcome == None:
            for old_tag in userdata.tags:
                counter += 1
                for tag in tags.detections:
                    #if oldest tag is found
                    if counter > self.index:
                        self.lost += 1
                        return
                    if self.main_tag != None and tag.id == self.main_tag.id:
                        self.block_counter +=1
                        self.main_tag = tag
                        self.block_counter +=1
                        self.lost = 0
                        return
                    if self.main_tag != None and tag.id != self.main_tag.id:
                        self.outcome = 'found'
                        return
                    # find cornertag
                    if tag.id == old_tag.id:
                        self.index = counter
                        self.block_counter +=1
                        self.main_tag = tag
                        self.block_counter +=1
                        print 'tag :'
                        print self.main_tag
                        self.lost = 0
                        return

    def calculate_twist(self):
        localTag = None
        while True:
            count = self.block_counter
            if (count % 2) == 0:
                localTag = self.main_tag
            if self.block_counter == count:
                break
        self.twist = Twist()
        # fix hight
        #get tag into vertical middle of camera
        vertical_angle_to_tag = localTag.pose.pose.position.y
        if vertical_angle_to_tag > 0.1:
            self.twist.linear.z = -0.15
        elif vertical_angle_to_tag < -0.1:
            self.twist.linear.z = 0.15

        #get tag into horizontal middle of camera
        horizontal_angle_to_tag = atan2(localTag.pose.pose.position.x, localTag.pose.pose.position.z)

        if horizontal_angle_to_tag < -0.1:
            self.twist.angular.z = 0.15

        elif horizontal_angle_to_tag > 0.1:
            self.twist.angular.z = -0.15

        #move forwards
        self.distance_to_tag = sqrt(
            pow(localTag.pose.pose.position.x, 2) + pow(localTag.pose.pose.position.y, 2) + pow(localTag.pose.pose.position.z, 2))
        if self.distance_to_tag < 0.5:
            self.twist.linear.x = -0.05
        elif self.distance_to_tag > 0.7:
            self.twist.linear.x = 0.05

                # allign to tag
        if self.distance_to_tag < 0.7:
                self.twist.linear.y = -0.05

    def execute(self, userdata):
        self.outcome = None
        self.half_turn = 0
        self.positive = True
        self.block_counter = 0
        self.index = len(userdata.tags) -1
        self.main_tag = None
        self.lost = 0
        self.twist = None
        print 'abbort found corner'
        subscriber_tag = rospy.Subscriber('/tag_detections',
                                      AprilTagDetectionArray, self.find_oldest_tag, userdata)
        subscriber_imu = rospy.Subscriber('/rexrov/imu',
                                      Imu, update_rotation, self)
        cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(ROS_RATE)
        while self.main_tag == None:
            pass
        while self.outcome == None:
            self.calculate_twist()
            # if self.distance_to_tag < 0.7:
                # self.twist = Twist()
                # # self.twist.linear.y = 0.15
                # self.twist.angular.z = 0.05
            cmd_vel_pub.publish(self.twist)
            if self.lost > 15:
                self.outcome = 'failure'
            self.rate.sleep()
        print 'outcome :'
        print self.outcome 
        twist = Twist()
        cmd_vel_pub.publish(twist)
        subscriber_tag.unregister()
        subscriber_imu.unregister()
        cmd_vel_pub.unregister()
        return self.outcome





class Docking(State):
    def __init__(self):
        State.__init__(self, outcomes=['docked', 'failure'], input_keys=['tags'], output_keys=['tags'])

    def execute(self, userdata):
        print 'docking'
        return 'docked'

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

    with sm:
        # def result(userdata, sta)
        StateMachine.add('LOOKING_FOR_FIRST_TARGET', Looking_for_first_target(), transitions={
                         'found': 'APPROACHING_TARGET', 'not_found': 'failure'}, remapping={'tags': 'tags'})
        StateMachine.add('APPROACHING_TARGET', Approaching_target(), transitions={
                         'reached': 'LOOKING_FOR_NEXT_TARGET', 'lost_visual': 'failure', 'abbort':'ABBORT'}, remapping={'tags': 'tags'})
        StateMachine.add('LOOKING_FOR_NEXT_TARGET', Looking_for_next_target(), transitions={
                         'found_next': 'APPROACHING_TARGET', 'corner':'CORNER', 'failure': 'failure', 'finished_row' : 'success', 'abbort':'ABBORT'}, remapping={'tags': 'tags'})
                         
        StateMachine.add('FIND_TAG_AGAIN', Find_tag_again(), transitions={
                         'found': 'APPROACHING_TARGET', 'not_found': 'failure', 'abbort':'ABBORT'}, remapping={'tags': 'tags'})
        StateMachine.add('CORNER', Corner(), transitions={
                         'found_next': 'APPROACHING_TARGET', 'failure': 'failure', 'abbort':'ABBORT'}, remapping={'tags': 'tags'})
        StateMachine.add('ABBORT', Abbort(), transitions={
                         'ready_to_dock': 'DOCKING', 'corner':'ABBORT_CORNER', 'no_old_tags':'ABBORT_LOST_TAGS', 'failure': 'failure'}, remapping={'tags': 'tags'})
        StateMachine.add('ABBORT_LOST_TAGS', Abbort_lost_tags(), transitions={
                         'found': 'ABBORT', 'failure': 'failure'}, remapping={'tags': 'tags'})
        StateMachine.add('ABBORT_CORNER', Abbort_corner(), transitions={
                         'found': 'ABBORT', 'failure': 'failure'}, remapping={'tags': 'tags'})
        StateMachine.add('DOCKING', Docking(), transitions={
                         'docked':'success', 'failure': 'failure'}, remapping={'tags': 'tags'})
                         
    sm.execute()
