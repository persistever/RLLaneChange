# coding:utf-8

import math
import traci
import traci.constants as tc
from surrounding import Surrounding

LANE_WIDTH = 3.2
RADAR_LIMIT = 200


class EgoVehicle:
    def __init__(self, vehicle_id):
        self.id = vehicle_id
        self.data = None  # 从subscribe订阅的所有数据
        # self.surroundings = Surrounding("ego")
        # self.neighbourVehicles = None
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
        self.edgeID = ''
        self.nextEdgeID = ''
        self.nLane = 0
        self.nNextLane = 0
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
        self.missionList = []
        self.frontVehicleList = []
        self.rearVehicleList = []
        self.leftVehicleList = []
        self.rightVehicleList = []
        self.targetGapFront = None
        self.targetGapRear = None
        self._subscribe_ego_vehicle()

    def _subscribe_ego_vehicle(self):
        traci.vehicle.subscribe(self.id, (tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_ROAD_ID))
        # self.surroundings.surrounding_init()
        # self.surroundings.subscribe_ego_vehicle_surrounding()
        # traci.vehicle.subscribeContext(self.id, tc.CMD_GET_VEHICLE_VARIABLE, 100,
        #                                [tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_LANE_INDEX])
        # traci.vehicle.addSubscriptionFilterLanes([-2, -1, 0, 1, 2], noOpposite=True,
        #                                          downstreamDist=200.0, upstreamDist=200.0)
        # traci.vehicle.subscribeContext(self.id, tc.CMD_GET_EDGE_VARIABLE, 100, [tc.VAR_NEXT_EDGE])

    def get_data(self):
        self.data = traci.vehicle.getSubscriptionResults(self.id)
        # self.surroundings.get_surroundings()
        # self.neighbourVehicles = self.surroundings.get_neighbor_list()

        if self.data is not None:
            self._get_xy()
            if self.x > 0:
                self._get_speed()
                self._get_lane_index()
                self._get_y_lane_lateral()
                self._get_angle()
                self._get_road_id()
                self._get_n_lane()
            # print('roadid: '+str(traci.vehicle.getRoadID(self.id)))

        # if self.otherData is not None:
            # self.nextEdgeID = self.otherData[tc.VAR_NEXT_EDGE]
        # if self.surroundings.get_neighbor_list() is not None:
        #     self.neighbourVehicles = self.surroundings.get_neighbor_list()
        #     self.frontVehicleList = self.surroundings.get_leader_neighbor_list()
        #     self.rearVehicleList = self.surroundings.get_follower_neighbor_list()
        #     self.leftVehicleList = self.surroundings.get_left_neighbor_list()
        #     self.rightVehicleList = self.surroundings.get_right_neighbor_list()

    def print_data(self):
        print("自车信息："+str(self.data))
        # print("他车信息"+str(self.neighbourVehicles))

    def _get_xy(self):
        self.preX = self.x
        self.preY = self.y
        self.x = self.data[tc.VAR_POSITION][0]
        self.y = self.data[tc.VAR_POSITION][1]

    def _get_speed(self):
        self.vx = (self.x - self.preX) / self.timeStep
        self.vy = (self.y - self.preY) / self.timeStep
        self.vx0 = self.data[tc.VAR_SPEED]

    def _get_lane_index(self):
        self.laneIndex = self.nLane - math.ceil(-self.y / LANE_WIDTH)

    def _get_y_lane_lateral(self):
        self.yLane = self.y - (- self.nLane + self.laneIndex + 0.5) * LANE_WIDTH
        # print('yLane:'+str(self.yLane))

    def _get_angle(self):
        self.angleCtl = 90 - math.atan(self.vy/self.vx)/math.pi*180.0
        # print('angleCtl'+str(self.angleCtl))

    def _get_road_id(self):
        if self.edgeID != self.data[tc.VAR_ROAD_ID]:
            self.edgeID = self.data[tc.VAR_ROAD_ID]

    def _get_n_lane(self):
        self.nLane = traci.edge.getLaneNumber(self.edgeID)

    def drive(self):
        traci.vehicle.moveToXY(self.id, 'gneE0', 2, self.x + self.timeStep * self.vxCtl,
                               self.y + self.timeStep * self.vyCtl, self.angleCtl, 2)
        self.has_lane_change_complete()

    # def decide_change_lane_process(self):

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

    # def pre_change_to_lane(self):
    #
    #
    # def has_pre_change_to_lane_complete(self):
    #
    #
    # def post_change_to_lane(self):
    #
    #
    # def has_post_change_to_lane(self):







