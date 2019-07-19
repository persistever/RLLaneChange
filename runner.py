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


def generate_routefile():
    traffic_base = 0.4
    f_rate = random.uniform(0.4,0.6)
    s_rate = random.uniform(0.05,0.1)
    m_rate = 1-f_rate-s_rate
    p_s1 = random.uniform(0.9,0.95)
    p_s2 = 1-p_s1
    p_e1 = random.uniform(0.05,0.1)
    p_e2 = random.uniform(0.3,0.4)
    p_e3 = 1 - p_e1 - p_e2

    with open("data/motorway.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="pkw_special" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="0.000001" \
guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="blue"/>""", file=routes)
        print('     	<trip id="ego" type="pkw_special" depart="0" from="gneE0" to="gneE7" departLane="free" departSpeed ="random"/> ',file=routes)
        print("</routes>", file=routes)


def run():
    """execute the TraCI control loop"""
    step = 0
    ego_vehicle = EgoVehicle('ego')
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        ego_vehicle.get_data()
        ego_vehicle.print_data()
        ego_vehicle.drive()
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
    surroundings = Surrounding(0.5)
    surroundings.traffic_init_general()
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/motorway.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
