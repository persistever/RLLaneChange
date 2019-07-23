# coding:utf-8
import numpy as np
import tensorflow as tf
import random
import math
import operator

class DataProcess:
    def __init__(self):
        self.leftVehicleData = []
        self.rightVehicleData = []
        self.MidVehicleData = []
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








