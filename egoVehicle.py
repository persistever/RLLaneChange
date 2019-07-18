# coding:utf-8

import traci
import traci.constants as tc


class EgoVehicle:
    def __init__(self, vehicle_id):
        self.id = vehicle_id
        self._subscribe_ego_vehicle()
        self.data = None
        self.x = 0
        self.y = 0
        self.vx = 1
        self.vy = 0
        self.angle = 0
        self.laneIndex = -1
        self.laneID = ''
        self.laneX = 0
        self.laneY = 0
        self.timeStep = 0.01

    def _subscribe_ego_vehicle(self):
        traci.vehicle.subscribe(self.id, (tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_LANE_INDEX))

    def get_data(self):
        self.data = traci.vehicle.getSubscriptionResults(self.id)
        if self.data is not None:
            self._get_xy()

    def print_data(self):
        print(self.data)

    def _get_xy(self):
        self.x = self.data[tc.VAR_POSITION][0]
        self.y = self.data[tc.VAR_POSITION][1]

    def drive(self):
        traci.vehicle.moveToXY(self.id, 'gneE0', 0, self.x + self.timeStep * self.vx,
                               self.y + self.timeStep * self.vy, 90, 2)

