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
TIME_OUT = 1500
KEEP_LANE_TIME = 1000


class Env:
    def __init__(self, ego_start_time=0):
        self.ego_vehicle = None
        # self.n_action = [3, 5]
        # self.n_feature = [18, 18, 18, 3]
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
        observation = []
        self.nogui = nogui
        self.sumo_step = 0
        self.ego_vehicle = None
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
        traci.start([sumo_binary, "-c", "data/motorway.sumocfg", "--no-step-log", "--no-warnings"])
        ego_start_step = math.ceil(self.ego_start_time/TIME_STEP+5)
        while self.sumo_step < ego_start_step:
            traci.simulationStep()
            self.sumo_step += 1
        traci.vehicle.moveToXY('ego', 'gneE0', 2, 0.5, -4.8, 90, 2)
        traci.simulationStep()
        self.sumo_step += 1
        self.ego_vehicle = EgoVehicle('ego')
        self.ego_vehicle.subscribe_ego_vehicle()
        while self.sumo_step < ego_start_step + 5:
            self.ego_vehicle.fresh_data()
            self.ego_vehicle.drive()
            traci.simulationStep()
            self.sumo_step += 1
        self.data_process.set_surrounding_data(self.ego_vehicle.surroundings, self.ego_vehicle.get_speed())
        self.data_process.vehicle_surrounding_data_process()
        observation.extend(self.data_process.get_left_vehicle_data())
        observation.extend(self.data_process.get_mid_vehicle_data())
        observation.extend(self.data_process.get_right_vehicle_data())
        observation.extend([self.ego_vehicle.get_lane_index(), self.ego_vehicle.get_n_lane(),
                            self.ego_vehicle.get_next_n_lane()])
        observation.extend(self.ego_vehicle.get_lmr_speed_limit())
        self.ego_vehicle.get_lmr_speed_limit()
        return observation

    def step(self, action_high, action_low):
        speed_before = self.ego_vehicle.get_speed()
        observation = []
        reward = 0
        current_step = 0
        done = False
        n_collision = 0
        info = {}
        keep_step = 0
        self.ego_vehicle.clear_mission()
        self.ego_vehicle.print_current_lane_index()
        self.ego_vehicle.clear_gap_vehicle()
        # self.ego_vehicle.print_data()
        self.data_process.set_surrounding_data(self.ego_vehicle.surroundings, self.ego_vehicle.get_speed())
        self.data_process.vehicle_surrounding_data_process()
        # print("step中的车速："+str(self.ego_vehicle.get_speed()))
        if action_high == 1:
            self.ego_vehicle.clear_mission()
            self.ego_vehicle.lane_keep_plan()
            while self.ego_vehicle.is_outof_map() is False and self.ego_vehicle.check_outof_road() is False and \
                    self.ego_vehicle.get_state() and current_step < TIME_OUT:
                self.ego_vehicle.fresh_data()
                # self.ego_vehicle.print_data()
                self.ego_vehicle.drive()
                traci.simulationStep()
                if self.ego_vehicle.check_collision():
                    n_collision += 1
                self.sumo_step += 1
                current_step += 1
            reward += 1
            info['endState'] = 'Choose ego lane, action is to keep lane'
        else:
            self.data_process.set_rl_result_data(action_high, action_low)
            self.data_process.rl_result_process()
            gap_front_vehicle, gap_rear_vehicle = self.data_process.get_gap_vehicle_list()
            print("gap前车："+str(gap_front_vehicle))
            print("gap后车："+str(gap_rear_vehicle))
            self.ego_vehicle.clear_mission()
            self.ego_vehicle.lane_change_plan(gap_front_vehicle, gap_rear_vehicle)
            if self.ego_vehicle.check_can_change_lane(action_high) is True and \
                    self.ego_vehicle.check_can_insert_into_gap() is True:
                while self.ego_vehicle.get_state() and current_step < TIME_OUT \
                        and self.ego_vehicle.check_can_insert_into_gap() is True and \
                        self.ego_vehicle.is_outof_map() is False and self.ego_vehicle.check_outof_road() is False:
                    self.ego_vehicle.fresh_data()
                    # self.ego_vehicle.print_data()
                    self.ego_vehicle.drive()
                    traci.simulationStep()
                    if self.ego_vehicle.check_collision():
                        n_collision += 1
                    self.sumo_step += 1
                    current_step += 1
                if self.ego_vehicle.check_change_lane_successful():
                    reward += 5
                    info['endState'] = 'Change to the target gap successful'
                else:
                    if current_step >= TIME_OUT:
                        info['endState'] = 'Change Lane Timeout, the plan has been tried'
                        reward -= 5
                    if self.ego_vehicle.check_can_insert_into_gap() is False:
                        info['endState'] = 'Change to the target gap failed, the plan has been tried'
                        reward -= 5
            else:
                self.ego_vehicle.clear_mission()
                self.ego_vehicle.lane_keep_plan()
                while self.ego_vehicle.is_outof_map() is False and self.ego_vehicle.check_outof_road() is False and \
                        self.ego_vehicle.get_state() and current_step < KEEP_LANE_TIME:
                    self.ego_vehicle.fresh_data()
                    # self.ego_vehicle.print_data()
                    self.ego_vehicle.drive()
                    traci.simulationStep()
                    if self.ego_vehicle.check_collision():
                        n_collision += 1
                    self.sumo_step += 1
                    current_step += 1
                if self.ego_vehicle.check_can_change_lane(action_high) is False:
                    info['endState'] = 'Cannot change to the target lane, because it\'s out of map, lane keep instead'
                    reward = -20
                if self.ego_vehicle.check_can_insert_into_gap() is False:
                    info['endState'] = 'Cannot change to the target lane, because the gap is too narrow'
                    reward = -10

        while keep_step < 20:
            self.ego_vehicle.clear_mission()
            self.ego_vehicle.lane_keep_plan()
            while self.ego_vehicle.get_state() and self.ego_vehicle.is_outof_map() is False and \
                    self.ego_vehicle.check_outof_road() is False and keep_step < 50:
                self.ego_vehicle.fresh_data()
                # self.ego_vehicle.print_data()
                self.ego_vehicle.drive()
                traci.simulationStep()
                if self.ego_vehicle.check_collision():
                    n_collision += 1
                self.sumo_step += 1
                keep_step += 1
            if self.ego_vehicle.is_outof_map() or self.ego_vehicle.check_outof_road():
                break

        if self.ego_vehicle.check_outof_road():
            reward -= 50
            info['endState'] = 'Vehicle is out of map in lateral direction'
        if self.ego_vehicle.get_lane_index() == 0:
            reward -= 20
            info['emergencyLane'] = 'Vehicle change to the emergency lane'

        reward -= min(n_collision * 5, 50)

        speed_after = self.ego_vehicle.get_speed()
        if speed_after > speed_before:
            reward += min((speed_after - speed_before) * 3, 20)
        elif speed_after < speed_before:
            reward += max((speed_after - speed_before) * 2, -10)

        if self.sumo_step > 1e5 or traci.simulation.getMinExpectedNumber() <= 0 \
                or self.ego_vehicle.is_outof_map() or self.ego_vehicle.check_outof_road():
            done = True
            self.ego_vehicle.clear_mission()
            traci.close()
        else:
            self.data_process.set_surrounding_data(self.ego_vehicle.surroundings, self.ego_vehicle.get_speed())
            self.data_process.vehicle_surrounding_data_process()
            observation.extend(self.data_process.get_left_vehicle_data())
            observation.extend(self.data_process.get_mid_vehicle_data())
            observation.extend(self.data_process.get_right_vehicle_data())
            observation.extend([self.ego_vehicle.get_lane_index(), self.ego_vehicle.get_n_lane(),
                                self.ego_vehicle.get_next_n_lane()])
            observation.extend(self.ego_vehicle.get_lmr_speed_limit())

        info['nCollision'] = n_collision
        info['SUMO_Time: '] = self.sumo_step*0.01
        # self.ego_vehicle.print_data()
        print("\n执行结束")
        self.ego_vehicle.print_goal_lane_index()
        self.ego_vehicle.print_current_lane_index()
        return observation, reward, done, info

    def test_step(self, action_high, action_low):
        speed_before = self.ego_vehicle.get_speed()
        observation = []
        reward = 0
        current_step = 0
        done = False
        n_collision = 0
        info = {}
        keep_step = 0
        self.ego_vehicle.clear_mission()
        self.ego_vehicle.print_current_lane_index()
        self.ego_vehicle.clear_gap_vehicle()
        # self.ego_vehicle.print_data()
        self.data_process.set_surrounding_data(self.ego_vehicle.surroundings, self.ego_vehicle.get_speed())
        self.data_process.vehicle_surrounding_data_process()
        # print("step中的车速："+str(self.ego_vehicle.get_speed()))
        if action_high == 1:
            self.ego_vehicle.clear_mission()
            self.ego_vehicle.lane_keep_plan()
            while self.ego_vehicle.is_outof_map() is False and self.ego_vehicle.check_outof_road() is False and \
                    self.ego_vehicle.get_state() and current_step < TIME_OUT and self.ego_vehicle.is_safe():
                self.ego_vehicle.fresh_data()
                # self.ego_vehicle.print_data()
                self.ego_vehicle.drive()
                traci.simulationStep()
                if self.ego_vehicle.check_collision():
                    n_collision += 1
                self.sumo_step += 1
                current_step += 1
            reward += 1
            info['endState'] = 'Choose ego lane, action is to keep lane'
        else:
            self.data_process.set_rl_result_data(action_high, action_low)
            self.data_process.rl_result_process()
            gap_front_vehicle, gap_rear_vehicle = self.data_process.get_gap_vehicle_list()
            print("gap前车："+str(gap_front_vehicle))
            print("gap后车："+str(gap_rear_vehicle))
            self.ego_vehicle.clear_mission()
            self.ego_vehicle.lane_change_plan(gap_front_vehicle, gap_rear_vehicle)
            if self.ego_vehicle.check_can_change_lane(action_high) is True and \
                    self.ego_vehicle.check_can_insert_into_gap() is True:
                while self.ego_vehicle.get_state() and current_step < TIME_OUT \
                        and self.ego_vehicle.check_can_insert_into_gap() is True and \
                        self.ego_vehicle.is_outof_map() is False and self.ego_vehicle.check_outof_road() is False and \
                        self.ego_vehicle.is_safe():
                    self.ego_vehicle.fresh_data()
                    # self.ego_vehicle.print_data()
                    self.ego_vehicle.drive()
                    traci.simulationStep()
                    if self.ego_vehicle.check_collision():
                        n_collision += 1
                        break
                    self.sumo_step += 1
                    current_step += 1
                if self.ego_vehicle.check_change_lane_successful():
                    reward += 20
                    info['endState'] = 'Change to the target gap successful'
                else:
                    if current_step >= TIME_OUT:
                        info['endState'] = 'Change Lane Timeout, the plan has been tried'
                        reward -= 5
                    if self.ego_vehicle.check_can_insert_into_gap() is False:
                        info['endState'] = 'Change to the target gap failed, the plan has been tried'
                        reward -= 5
                    if n_collision != 0:
                        info['collision'] = 'collision happened during lane change'
            else:
                self.ego_vehicle.clear_mission()
                self.ego_vehicle.lane_keep_plan()
                while self.ego_vehicle.is_outof_map() is False and self.ego_vehicle.check_outof_road() is False and \
                        self.ego_vehicle.get_state() and current_step < KEEP_LANE_TIME and self.ego_vehicle.is_safe():
                    self.ego_vehicle.fresh_data()
                    # self.ego_vehicle.print_data()
                    self.ego_vehicle.drive()
                    traci.simulationStep()
                    if self.ego_vehicle.check_collision():
                        n_collision += 1
                        break
                    self.sumo_step += 1
                    current_step += 1
                if self.ego_vehicle.check_can_change_lane(action_high) is False:
                    info['endState'] = 'Cannot change to the target lane, because it\'s out of map, lane keep instead'
                    reward = -20
                if self.ego_vehicle.check_can_insert_into_gap() is False:
                    info['endState'] = 'Cannot change to the target lane, because the gap is too narrow'
                    reward = -10

        if self.ego_vehicle.is_safe() is False:
            self.ego_vehicle.clear_mission()
            self.ego_vehicle.lane_keep_plan()
            while self.ego_vehicle.get_state() and self.ego_vehicle.is_outof_map() is False and \
                    self.ego_vehicle.check_outof_road() is False:
                self.ego_vehicle.fresh_data()
                # self.ego_vehicle.print_data()
                self.ego_vehicle.drive()
                traci.simulationStep()
                if self.ego_vehicle.check_collision():
                    n_collision += 1
                self.sumo_step += 1

        while keep_step < 20:
            self.ego_vehicle.clear_mission()
            self.ego_vehicle.lane_keep_plan()
            while self.ego_vehicle.get_state() and self.ego_vehicle.is_outof_map() is False and \
                    self.ego_vehicle.check_outof_road() is False and keep_step < 50:
                self.ego_vehicle.fresh_data()
                # self.ego_vehicle.print_data()
                self.ego_vehicle.drive()
                traci.simulationStep()
                if self.ego_vehicle.check_collision():
                    n_collision += 1
                self.sumo_step += 1
                keep_step += 1
            if self.ego_vehicle.is_outof_map() or self.ego_vehicle.check_outof_road():
                break

        if self.ego_vehicle.check_outof_road():
            reward -= 50
            info['endState'] = 'Vehicle is out of map in lateral direction'
        if self.ego_vehicle.get_lane_index() == 0:
            reward -= 20
            info['emergencyLane'] = 'Vehicle change to the emergency lane'

        if n_collision < 10:
            reward -= n_collision * 5
        elif n_collision >= 10:
            reward -= 50

        speed_after = self.ego_vehicle.get_speed()
        if speed_after > speed_before:
            reward += 5
        elif speed_after < speed_before - 5:
            reward -= 5

        if self.sumo_step > 1e5 or traci.simulation.getMinExpectedNumber() <= 0 \
                or self.ego_vehicle.is_outof_map() or self.ego_vehicle.check_outof_road():
            done = True
            self.ego_vehicle.clear_mission()
            traci.close()
        else:
            self.data_process.set_surrounding_data(self.ego_vehicle.surroundings, self.ego_vehicle.get_speed())
            self.data_process.vehicle_surrounding_data_process()
            observation.extend(self.data_process.get_left_vehicle_data())
            observation.extend(self.data_process.get_mid_vehicle_data())
            observation.extend(self.data_process.get_right_vehicle_data())
            observation.extend([self.ego_vehicle.get_lane_index(), self.ego_vehicle.get_n_lane(),
                                self.ego_vehicle.get_next_n_lane()])
            observation.extend(self.ego_vehicle.get_lmr_speed_limit())

        info['nCollision'] = n_collision
        info['SUMO_Time: '] = self.sumo_step*0.01
        # self.ego_vehicle.print_data()
        print("\n执行结束")
        self.ego_vehicle.print_goal_lane_index()
        self.ego_vehicle.print_current_lane_index()
        return observation, reward, done, info


