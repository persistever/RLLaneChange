# coding:utf-8
import numpy as np
import tensorflow as tf
import random
import math
import operator


class DataProcess:
    def __init__(self):
        self.leftLeaderNeighborList = []
        self.leftFollowerNeighborList = []
        self.rightLeaderNeighborList = []
        self.rightFollowerNeighborList = []
        self.midLeaderNeighborList = []
        self.midFollowerNeighborList = []
        self.leftVehicleData = []
        self.rightVehicleData = []
        self.midVehicleData = []
        self.laneData = []
        self.speed = 10
        self.laneIndex = 0
        self.targetLane = 0
        self.targetGap = 0
        self.gapVehicleList = []

    def _chosen_vehicle(self, vehicle, number=3):
        if vehicle!=None:
            sored_vehicle = sorted(vehicle, key=operator.itemgetter('relative_lane_position_abs'))
        else:
            sored_vehicle = []
        if len(sored_vehicle) > number:
            return sored_vehicle[0:number]
        else:
            return sored_vehicle

    def _vehicle_data_process(self, leader, follower, speed,lane):
        lat = (lane-1)*3.2
        vehicle_data = np.array([200.0,lat,speed,200.0,lat,speed,200.0,lat,speed,-200.0,lat,speed,-200.0,lat,speed,-200.0,lat,speed])
        for i in range(3):
            if i < len(leader):
                vehicle_data[6 - 3 * i] = leader[i]['relative_lane_position']
                vehicle_data[7 - 3 * i] = leader[i]['relative_position_y']
                vehicle_data[8 - 3 * i] = leader[i]['speed']-speed
            if i < len(follower):
                vehicle_data[9 + 3 * i] = follower[i]['relative_lane_position']
                vehicle_data[10 - 3 * i] = follower[i]['relative_position_y']
                vehicle_data[11 + 3 * i] = follower[i]['speed']-speed
        return vehicle_data

    def set_surrounding_data(self, surrounding, speed):
        self.leftLeaderNeighborList = self._chosen_vehicle(surrounding.get_left_leader_neighbor_list())
        self.leftFollowerNeighborList = self._chosen_vehicle(surrounding.get_left_follower_neighbor_list())
        self.midLeaderNeighborList = self._chosen_vehicle(surrounding.get_mid_leader_neighbor_list())
        self.midFollowerNeighborList = self._chosen_vehicle(surrounding.get_mid_follower_neighbor_list())
        self.rightLeaderNeighborList = self._chosen_vehicle(surrounding.get_right_leader_neighbor_list())
        self.rightFollowerNeighborList = self._chosen_vehicle(surrounding.get_right_follower_neighbor_list())
        self.speed = speed

    def vehicle_surrounding_data_process(self):
        self.leftVehicleData = self._vehicle_data_process(self.leftLeaderNeighborList, self.leftFollowerNeighborList, self.speed,0)
        self.midVehicleData = self._vehicle_data_process(self.midLeaderNeighborList, self.midFollowerNeighborList, self.speed,1)
        self.rightVehicleData = self._vehicle_data_process(self.rightLeaderNeighborList, self.rightFollowerNeighborList,
                                                           self.speed, 2)

    def get_left_vehicle_data(self):
        return self.leftVehicleData

    def get_mid_vehicle_data(self):
        return self.midVehicleData

    def get_right_vehicle_data(self):
        return self.rightVehicleData

    def set_rl_result_data(self, lane, gap):
        self.targetLane = lane
        self.targetGap = gap

    def _create_virtual_vehicle(self, name, place, lane):
        vehicle_dict = {'name': name,
                        'relative_position_x': place,
                        'relative_position_y': (lane-1)*3.2,
                        'speed': self.speed,
                        'lane_index_relative': lane,
                        'relative_lane_position': place,
                        'relative_lane_position_abs': math.fabs(place),
                        'virtual': True
        }
        return vehicle_dict

    def _create_real_vehicle(self, vehicle, lane):
        vehicle_dict = {'name': vehicle['name'],
                        'relative_position_x': vehicle['relative_position_x'],
                        'relative_position_y': vehicle['relative_position_y'],
                        'speed': vehicle['speed'],
                        'lane_index_relative': lane,
                        'relative_lane_position': vehicle['relative_lane_position'],
                        'relative_lane_position_abs': vehicle['relative_lane_position_abs'],
                        'virtual': False
        }
        return vehicle_dict

    def _gap_data_process(self):
        self.gapVehicleList = []
        if self.targetLane == 0:
            targetLeaderNeighborList = self.leftLeaderNeighborList
            targetFollowerNeighborList = self.leftFollowerNeighborList
            lane = 0
        elif self.targetLane == 1:
            targetLeaderNeighborList = self.midLeaderNeighborList
            targetFollowerNeighborList = self.midFollowerNeighborList
            lane = 1
        elif self.targetLane == 2:
            targetLeaderNeighborList = self.rightLeaderNeighborList
            targetFollowerNeighborList = self.rightFollowerNeighborList
            lane = 2
        else:
            return
        if self.targetGap == 0:
            if len(targetLeaderNeighborList) == 3:
                self.gapVehicleList.append(self._create_real_vehicle(targetLeaderNeighborList[2], lane))
                self.gapVehicleList.append(self._create_real_vehicle(targetLeaderNeighborList[1], lane))
            elif len(targetLeaderNeighborList) == 2:
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_l", 200, lane))
                self.gapVehicleList.append(self._create_real_vehicle(targetLeaderNeighborList[1], lane))
            else:
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_l", 200, lane))
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_f", 200, lane))
        if self.targetGap == 1:
            if len(targetLeaderNeighborList) >= 2:
                self.gapVehicleList.append(self._create_real_vehicle(targetLeaderNeighborList[1], lane))
                self.gapVehicleList.append(self._create_real_vehicle(targetLeaderNeighborList[0], lane))
            elif len(targetLeaderNeighborList) == 1:
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_l", 200, lane))
                self.gapVehicleList.append(self._create_real_vehicle(targetLeaderNeighborList[0], lane))
            elif len(targetLeaderNeighborList) == 0:
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_l", 200, lane))
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_f", 200, lane))
        if self.targetGap == 2:
            if len(targetLeaderNeighborList) >= 1:
                self.gapVehicleList.append(self._create_real_vehicle(targetLeaderNeighborList[0], lane))
            else:
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_l", 200, lane))
            if len(targetFollowerNeighborList) >= 1:
                self.gapVehicleList.append(self._create_real_vehicle(targetFollowerNeighborList[0], lane))
            else:
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_l", -200, lane))
        if self.targetGap == 3:
            if len(targetFollowerNeighborList) >= 2:
                self.gapVehicleList.append(self._create_real_vehicle(targetFollowerNeighborList[0], lane))
                self.gapVehicleList.append(self._create_real_vehicle(targetFollowerNeighborList[1], lane))
            elif len(targetFollowerNeighborList) == 1:
                self.gapVehicleList.append(self._create_real_vehicle(targetFollowerNeighborList[0], lane))
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_f", -200, lane))
            elif len(targetFollowerNeighborList) == 0:
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_l", -200, lane))
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_f", -200, lane))
        if self.targetGap == 4:
            if len(targetFollowerNeighborList) == 3:
                self.gapVehicleList.append(self._create_real_vehicle(targetFollowerNeighborList[1], lane))
                self.gapVehicleList.append(self._create_real_vehicle(targetFollowerNeighborList[2], lane))
            elif len(targetFollowerNeighborList) == 2:
                self.gapVehicleList.append(self._create_real_vehicle(targetFollowerNeighborList[1], lane))
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_f", -200, lane))
            else:
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_l", -200, lane))
                self.gapVehicleList.append(self._create_virtual_vehicle("virtual_f", -200, lane))

    def rl_result_process(self):
        self._gap_data_process()

    def get_gap_vehicle_list(self):
        return self.gapVehicleList
