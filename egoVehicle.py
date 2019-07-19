# coding:utf-8


import math
import traci
import traci.constants as tc

N_LANE = 4
LANE_WIDTH = 3.2


class EgoVehicle:
    def __init__(self, vehicle_id):
        self.id = vehicle_id
        self._subscribe_ego_vehicle()
        self.data = None  # 从subscribe订阅的所有数据
        self.preX = 0  # 之前的一个位置，用来估算纵向车速
        self.preY = 0  # 之前的一个位置，用来横向车速
        self.x = 0  # 当前的x全局坐标
        self.y = 0  # 当前的y全局坐标
        self.yLane = 0  # 车辆在当前车道的相对横向位置
        self.vx0 = 0
        self.vx = 0
        self.vy = 0
        self.preLaneIndex = -1
        self.laneIndex = -1
        self.goalLaneIndex = -1
        self.laneID = ''
        self.laneX = 0
        self.laneY = 0
        self.timeStep = 0.01
        self.goalX = 0
        self.goalY = 0
        self.axCtl = 0
        self.ayCtl = 0
        self.vyCtl = 0
        self.vxCtl = 10 - self.vx0
        self.angleCtl = 90
        self.lastChangeLaneTime = 0

    def _subscribe_ego_vehicle(self):
        traci.vehicle.subscribe(self.id, (tc.VAR_POSITION, tc.VAR_SPEED))

    def get_data(self):
        self.data = traci.vehicle.getSubscriptionResults(self.id)
        if self.data is not None:
            self._get_xy()
            self.vx = (self.x - self.preX) / self.timeStep
            self.vy = (self.y - self.preY) / self.timeStep
            self.vx0 = self.data[tc.VAR_SPEED]
            self._get_lane_index()
            self._get_y_lane_lateral()
            self._get_angle()

    def print_data(self):
        print(self.data)
        print(self.laneIndex)

    def _get_xy(self):
        self.preX = self.x
        self.preY = self.y
        self.x = self.data[tc.VAR_POSITION][0]
        self.y = self.data[tc.VAR_POSITION][1]

    def _get_lane_index(self):
        # self.laneIndex = traci.vehicle.getLaneIndex(self.id)

        self.laneIndex = N_LANE - math.ceil(-self.y / LANE_WIDTH)

    def _get_y_lane_lateral(self):
        self.yLane = self.y - (- N_LANE + self.laneIndex + 0.5) * LANE_WIDTH
        print('yLane:'+str(self.yLane))

    def _get_angle(self):
        self.angleCtl = 90 - math.atan(self.vy/self.vx)/math.pi*180.0
        print('angleCtl'+str(self.angleCtl))

    def drive(self):
        traci.vehicle.moveToXY(self.id, 'gneE0', 2, self.x + self.timeStep * self.vxCtl,
                               self.y + self.timeStep * self.vyCtl, self.angleCtl, 2)
        self.has_lane_change_complete()

    # def is_need_change_lane(self):
    #
    # def decide_change_to_lane(self):

    def change_to_lane(self, lane_index):
        self.goalLaneIndex = lane_index
        if lane_index > self.laneIndex:
            self.vyCtl = 21.0 / self.vxCtl
        elif lane_index < self.laneIndex:
            self.vyCtl = - 21.0 / self.vxCtl

    def has_lane_change_complete(self):
        if self.laneIndex == self.goalLaneIndex and abs(self.yLane) < 0.1:
            self.preLaneIndex = self.laneIndex
            self.vyCtl = 0
            return True
        else:
            return False



