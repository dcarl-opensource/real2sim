# coding=utf-8

import rospy
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
from argparse import RawTextHelpFormatter
import copy
import math
from conception import Object, EgoState
from zzz_driver_msgs.msg import RigidBodyStateStamped, FrenetSerretState2D
from zzz_common.kinematics import get_frenet_state
from zzz_common.geometry import dense_polyline2d

DISTANCE_TOLERANCE = 3
TIME_GAP_TOLERANCE = 2

def quaterion2eular(x, y, z, w):
    # r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    # p = math.asin(2*(w*y-z*x))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    angleY = y*180/math.pi

    return angleY

class BagParser:
    def __init__(self, name):
        self.bag = rosbag.Bag(name)
        self.get_reference_path()
        self.time_axis = []
        self.ego_state = EgoState(self.reference_path, self.reference_tangent)
        self.object_list = []
        self.merging_object_list = None
        self.object_list_after_removing = None
        self.parse_object_info_cognition()
        self.parse_ego_info()
        self.export_data()
        self.observe()

    def parse_object_info_cognition(self):
        for topic, msg, t in self.bag.read_messages(topics=['/zzz/cognition/local_dynamic_map/map_with_ref']):
            self.time_axis.append(round(t.to_time(), 2))
            for surrounding_obj in msg.jmap.obstacles:
                object_found = False
                for obj in self.object_list:
                    if surrounding_obj.uid == obj.id:
                        object_found = True
                        yaw = quaterion2eular(surrounding_obj.state.pose.pose.orientation.x, surrounding_obj.state.pose.pose.orientation.y,
                            surrounding_obj.state.pose.pose.orientation.z, surrounding_obj.state.pose.pose.orientation.w)
                        v = (surrounding_obj.state.twist.twist.linear.x**2 + surrounding_obj.state.twist.twist.linear.y**2)**0.5
                        obj.add_info(time=round(t.to_time(), 2) - self.time_axis[0], x_pos=surrounding_obj.state.pose.pose.position.x,
                                     y_pos=surrounding_obj.state.pose.pose.position.y, yaw=yaw, velocity=v)
                        obj.updatesd(surrounding_obj.state)

                if not object_found:
                    temp_obj = Object(surrounding_obj.uid, surrounding_obj.cls.classid, self.reference_path, self.reference_tangent)
                    self.object_list.append(temp_obj)
                    yaw = quaterion2eular(surrounding_obj.state.pose.pose.orientation.x, surrounding_obj.state.pose.pose.orientation.y,
                        surrounding_obj.state.pose.pose.orientation.z, surrounding_obj.state.pose.pose.orientation.w)
                    v = (surrounding_obj.state.twist.twist.linear.x**2 + surrounding_obj.state.twist.twist.linear.y**2)**0.5
                    temp_obj.add_info(time=round(t.to_time(), 2) - self.time_axis[0], x_pos=surrounding_obj.state.pose.pose.position.x,
                                      y_pos=surrounding_obj.state.pose.pose.position.y, yaw=yaw, velocity=v)
                    temp_obj.updatesd(surrounding_obj.state)
                
            # the object info is in 10hz, but the topic info is in 20hz, the object info should be interpolated.
            for obj in self.object_list:
                obj.interpolate_infomation()
        self.merge_perception_sections()

    def parse_object_info_perception(self):
        for topic, msg, t in self.bag.read_messages(topics=['/zzz/perception/objects_tracked']):
            # print(topic, msg, t)
            self.time_axis.append(t.to_time())
            self.time_axis.append(t.to_time()+0.05)
            for surrounding_obj in msg.targets:
                object_found = False
                for i in range(2):
                    for obj in self.object_list:
                        if surrounding_obj.uid == obj.id:
                            object_found = True
                            yaw = quaterion2eular(surrounding_obj.bbox.pose.pose.orientation.x, surrounding_obj.bbox.pose.pose.orientation.y,
                                surrounding_obj.bbox.pose.pose.orientation.z, surrounding_obj.bbox.pose.pose.orientation.w)
                            v = (surrounding_obj.twist.twist.linear.x**2 + surrounding_obj.twist.twist.linear.y**2)**0.5
                            obj.add_info(time=t.to_time() + 0.05 * i - self.time_axis[0], x_pos=surrounding_obj.bbox.pose.pose.position.x,
                                        y_pos=surrounding_obj.bbox.pose.pose.position.y, yaw=yaw, velocity=v)
                            temp_state = RigidBodyStateStamped()
                            temp_state.state.pose = surrounding_obj.bbox.pose
                            temp_state.state.twist = surrounding_obj.twist
                            temp_state.state.accel = surrounding_obj.accel
                            obj.updatesd(temp_state)

                    if not object_found:
                        temp_obj = Object(surrounding_obj.uid, surrounding_obj.classes[0].classid, self.reference_path, self.reference_tangent)
                        self.object_list.append(temp_obj)
                        yaw = quaterion2eular(surrounding_obj.bbox.pose.pose.orientation.x, surrounding_obj.bbox.pose.pose.orientation.y,
                            surrounding_obj.bbox.pose.pose.orientation.z, surrounding_obj.bbox.pose.pose.orientation.w)
                        v = (surrounding_obj.twist.twist.linear.x**2 + surrounding_obj.twist.twist.linear.y**2)**0.5
                        temp_obj.add_info(time=t.to_time() + 0.05 * i - self.time_axis[0], x_pos=surrounding_obj.bbox.pose.pose.position.x,
                                        y_pos=surrounding_obj.bbox.pose.pose.position.y, yaw=yaw, velocity=v)
                        temp_state = RigidBodyStateStamped()
                        temp_state.state.pose = surrounding_obj.bbox.pose
                        temp_state.state.twist = surrounding_obj.twist
                        temp_state.state.accel = surrounding_obj.accel
                        temp_obj.updatesd(temp_state)
        self.merge_perception_sections()

    def get_reference_path(self):
        # reference_path = []
        # last_trajactory = None
        # for topic, msg, t in self.bag.read_messages(topics=["/zzz/cognition/sent_ref_path"]):
        #     if not last_trajactory:
        #         last_trajactory = msg
        #         continue
        #     for path_point in last_trajactory.poses:
        #         if msg.poses[0].pose.position.x == path_point.pose.position.x and \
        #             msg.poses[0].pose.position.y == path_point.pose.position.y:
        #             break
        #         reference_path.append([path_point.pose.position.x, path_point.pose.position.y])
        #     last_trajactory = msg
        
        # for path_point in last_trajactory.poses[0:30]:
        #     reference_path.append([path_point.pose.position.x, path_point.pose.position.y])
        
        # self.reference_path = dense_polyline2d(np.array(reference_path), 1)
        # self.reference_tangent = np.zeros(len(self.reference_path))

        self.reference_path = None
        self.reference_tangent = None

    def merge_perception_sections(self):
        # merge function is removed here
        merged_object_list = copy.deepcopy(self.object_list)
        print("[Length of object list BEFORE merging]:", len(merged_object_list))
        self.merged_object_list = merged_object_list
        self.object_list_after_removing = self.remove_noisy(copy.deepcopy(merged_object_list))

    # 20hz-5second
    def remove_noisy(self, merging_list):
        final_object_list = []
        for item in merging_list:
            if len(item.timestamp) > 400:
                final_object_list.append(item)
        return final_object_list

    def observe(self):
        fig2 = plt.figure("Surroundings From Perception")
        plt.subplot(1, 2, 1)
        plt.title("Surroundings Before Filtering")
        plt.xlabel("X in Cartesian Coordinates")
        plt.ylabel("Y in Cartesian Coordinates")
        for item in self.merged_object_list:
            plt.plot(item.x_pos, item.y_pos)
        plt.subplot(1, 2, 2)
        plt.title("Surroundings After Filtering")
        plt.xlabel("X in Cartesian Coordinates")
        plt.ylabel("Y in Cartesian Coordinates")
        for item in self.object_list_after_removing:
            plt.plot(item.x_pos, item.y_pos)
        fig2.show()
        
        # fig3 = plt.figure("Global Path")
        # plt.title("Global Path")
        # plt.xlabel("X in Cartesian Coordinates")
        # plt.ylabel("Y in Cartesian Coordinates")
        # plt.plot(self.reference_path[:,0], self.reference_path[:,1])
        # fig3.show()

        # fig4 = plt.figure("ego_v-t")
        # plt.plot(self.ego_state.t, self.ego_state.v)
        #     # plt.xlim(-750, -680)
        #     # plt.ylim(200, 1000)
        # fig4.show()

        # fig5 = plt.figure("s-t")
        # # plt.plot(self.ego_state.t, self.ego_state.s)
        # plt.plot(self.ego_state.t, self.ego_state.s_trend)
        # # plt.xlim(-750, -680)
        # # plt.ylim(200, 1000)
        # fig5.show()

        # fig6 = plt.figure("d-t")
        # plt.plot(self.ego_state.t, self.ego_state.d)
        # plt.plot(self.ego_state.t, self.ego_state.d_trend)
        # # plt.xlim(-750, -680)
        # # plt.ylim(200, 1000)
        # fig6.show()

        # fig7 = plt.figure("relative_s-t")
        # plt.plot(self.object_list_after_removing[1].timestamp, self.object_list_after_removing[1].relative_s)
        # # plt.xlim(-750, -680)
        # # plt.ylim(200, 1000)
        # fig7.show()

        # fig8 = plt.figure("relative_d-t")
        # plt.plot(self.object_list_after_removing[1].timestamp, self.object_list_after_removing[1].relative_d)
        # # plt.xlim(-750, -680)
        # # plt.ylim(200, 1000)
        # fig8.show()

        plt.show()

    def parse_ego_info(self):
        time_index = 0
        for topic, msg, t in self.bag.read_messages(topics=["/zzz/navigation/ego_pose"]):
            if t.to_time() > self.time_axis[-5]:
                break
            if t.to_time() > self.time_axis[time_index]:
                self.ego_state.timestamp.append(self.time_axis[time_index] - self.time_axis[0])
                self.ego_state.x.append(msg.state.pose.pose.position.x)
                self.ego_state.y.append(msg.state.pose.pose.position.y)
                self.ego_state.yaw.append(quaterion2eular(msg.state.pose.pose.orientation.x, msg.state.pose.pose.orientation.y,
                                msg.state.pose.pose.orientation.z, msg.state.pose.pose.orientation.w))
                self.ego_state.v.append((msg.state.twist.twist.linear.x**2 + msg.state.twist.twist.linear.y**2)**0.5)
                self.ego_state.updatesd(msg)
                time_index = time_index + 1
            else:
                continue

    def export_data(self):
        surroundings = []
        for obj in self.object_list_after_removing:
            result = obj.dump()
            surroundings.append([result[0], result[1]])
        print(surroundings)
        np.savetxt("/".join(__file__.split("/")[0:-2]) + "/data/raw/" + "surroundings", surroundings, fmt="%d,%d", delimiter=',', newline='\n')
        self.ego_state.dump()


if __name__ == "__main__":
    PARSER = argparse.ArgumentParser(description="Parsing rosbag for reconstruction", formatter_class=RawTextHelpFormatter)
    PARSER.add_argument('--bag', type=str, help="Rosbag absolute path")
    ARGUMENTS = PARSER.parse_args()
    BagParser(ARGUMENTS.bag)
