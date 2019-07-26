# coding:utf-8

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import math
import numpy
import time

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci
import traci.constants as tc
from egoVehicle import EgoVehicle
from surrounding import Traffic
from  data_process import DataProcess

TIME_STEP = 0.01


class Env:
    def __init__(self, ego_start_time=0):
        self.ego_vehicle = None
        self.n_action = [3, 5]
        self.n_feature = [18, 18, 18, 3]
        self.ego_start_time = ego_start_time
        self.sumo_step = 0
        self.nogui = False
        self.data_process = DataProcess()

    def _get_options(self):
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--nogui", action="store_true",
                              default=self.nogui, help="run the commandline version of sumo")
        options, args = opt_parser.parse_args()
        return options

    def reset(self, nogui=False):
        self.nogui = nogui
        self.sumo_step = 0
        options = self._get_options()
        # this script has been called from the command line. It will start sumo as a
        # server, then connect and run
        if options.nogui:
            sumo_binary = checkBinary('sumo')
        else:
            sumo_binary = checkBinary('sumo-gui')
        # first, generate the route file for this simulation
        # traffics = Traffic(trafficBase=0.4, trafficList=None)
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        traci.start([sumo_binary, "-c", "data/motorway.sumocfg"])
        ego_start_step = math.ceil(self.ego_start_time/TIME_STEP+1)
        while self.sumo_step < ego_start_step:
            traci.simulationStep()
            self.sumo_step += 1
        self.ego_vehicle = EgoVehicle('ego')
        self.ego_vehicle.subscribe_ego_vehicle()
        self.ego_vehicle.fresh_data()
        self.ego_vehicle.drive()
        traci.simulationStep()
        self.sumo_step += 1
        self.data_process.set_surrounding_data(self.ego_vehicle.surroundings, self.ego_vehicle.get_speed())
        self.data_process.vehicle_surrounding_data_process()
        observation = [self.data_process.get_left_vehicle_data(), self.data_process.get_mid_vehicle_data(),
                       self.data_process.get_right_vehicle_data()]
        return 0

    def step(self, action, gap_front_vehicle=None, gap_rear_vehicle=None):
        done = False
        if action[0] == 1:
            self.ego_vehicle.lane_keep_plan()
        else:
            self.ego_vehicle.lane_change_plan(gsap_front_vehicle, gap_rear_vehicle)
        while self.ego_vehicle.get_state():
            self.ego_vehicle.fresh_data()
            self.ego_vehicle.print_data()
            self.ego_vehicle.drive()
            traci.simulationStep()
            self.sumo_step += 1

        if self.sumo_step > 1e5 or traci.simulation.getMinExpectedNumber() <= 0 or self.ego_vehicle.is_outof_map():
            done = True
            traci.close(wait=False)
        return done

        # observation = 0
        # reward = 0
        # done = False
        # info = {}
        # return observation, reward, done, info


