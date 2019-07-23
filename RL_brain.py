# coding:utf-8
import numpy as np
import pandas as pd
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
        self.leftVehicleList = []

    def _chosen_vehicle(self, number, vehicle=None):
        vehicle_list = []
        sored_vehicle = sorted(vehicle, key=operator.itemgetter('relative_lane_position_abs'))
        if len(sored_vehicle) > number:
            return sored_vehicle[0:number]
        else:
            return sored_vehicle






