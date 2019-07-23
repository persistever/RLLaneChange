#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2019 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
# SPDX-License-Identifier: EPL-2.0

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import math
import numpy


# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import traci.constants as tc
from egoVehicle import EgoVehicle
from surrounding import Surrounding
from surrounding import Traffic
from  RL_brain import DataProcess

surroundings = Surrounding("ego")
data_process = DataProcess()
def run():
    """execute the TraCI control loop"""
    step = 0
    ego_vehicle = None
    surroundings.surrounding_init()
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        if step == 3001:
            ego_vehicle = EgoVehicle('ego')
        if ego_vehicle is not None:
            ego_vehicle.get_data()
            # ego_vehicle.print_data()
            ego_vehicle.drive()
            if step == 3200:
                ego_vehicle.change_to_lane(2)
            if step == 3500:
                ego_vehicle.change_to_lane(3)
            if step == 3800:
                ego_vehicle.change_to_lane(2)

            surroundings.get_surroundings()
            data_process.set_surrounding_data(surroundings,10)
            data_process.vehicle_surrounding_data_process(10)
            print("lane_index:%d" % surroundings.get_lane_index())
            print(surroundings.get_mid_leader_neighbor_list())
            print(surroundings.get_mid_follower_neighbor_list())
            # print(data_process.get_left_vehicle_data())
            print(data_process.get_mid_vehicle_data())
            # print(data_process.get_right_vehicle_data())
            print(step)
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    traffics = Traffic(trafficBase=0.4, trafficList=None)
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/motorway.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])

    run()
