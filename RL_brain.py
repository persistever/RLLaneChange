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


    def _chosen_vehicle(self, vehicle, number=3):
        if vehicle!=None:
            sored_vehicle = sorted(vehicle, key=operator.itemgetter('relative_lane_position_abs'))
        else:
            sored_vehicle = []
        if len(sored_vehicle) > number:
            return sored_vehicle[0:number]
        else:
            return sored_vehicle

    def _vehicle_data_process(self, leader, follower, speed):
        vehicle_data = np.array([200.0,10.0,200.0,10.0,200.0,10.0,-200.0,10.0,-200.0,10.0,-200.0,10.0])
        for i in range(3):
            if i < len(leader):
                vehicle_data[4 - 2 * i] = leader[i]['relative_lane_position']
                vehicle_data[5 - 2 * i] = leader[i]['speed']-speed
            if i < len(follower):
                vehicle_data[6 + 2 * i] = follower[i]['relative_lane_position']
                vehicle_data[7 + 2 * i] = follower[i]['speed']-speed
        return vehicle_data

    def set_surrounding_data(self, surrounding):
        self.leftLeaderNeighborList = self._chosen_vehicle(surrounding.get_left_leader_neighbor_list())
        self.leftFollowerNeighborList = self._chosen_vehicle(surrounding.get_left_follower_neighbor_list())
        self.midLeaderNeighborList = self._chosen_vehicle(surrounding.get_mid_leader_neighbor_list())
        self.midFollowerNeighborList = self._chosen_vehicle(surrounding.get_mid_follower_neighbor_list())
        self.rightLeaderNeighborList = self._chosen_vehicle(surrounding.get_right_leader_neighbor_list())
        self.rightFollowerNeighborList = self._chosen_vehicle(surrounding.get_right_follower_neighbor_list())

    def vehicle_surrounding_data_process(self, speed):
        self.leftVehicleData = self._vehicle_data_process(self.leftLeaderNeighborList, self.leftFollowerNeighborList, speed)
        self.rightVehicleData = self._vehicle_data_process(self.rightLeaderNeighborList, self.rightFollowerNeighborList, speed)
        self.midVehicleData = self._vehicle_data_process(self.midLeaderNeighborList, self.midFollowerNeighborList, speed)

    def get_left_vehicle_data(self):
        return self.leftVehicleData

    def get_mid_vehicle_data(self):
        return self.midVehicleData

    def get_right_vehicle_data(self):
        return self.rightVehicleData






