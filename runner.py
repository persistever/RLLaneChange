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



# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import traci.constants as tc

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

    with open("data/highway.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="pkw_f" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="35" \
guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="green"/>
        <vType id="pkw_m" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="25" \
guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="yellow"/>
        <vType id="bus" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="2.5" maxSpeed="20" \
guiShape="bus" laneChangeModel="SL2015" latAlignment="center" color="red"/>
        <vType id="pkw_special" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="30" \
guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="blue"/>""", file=routes)
        print(
            '     	<flow id="pkw11_f" type="pkw_f" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e1 * f_rate), file=routes)
        print(
            '     	<flow id="pkw12_f" type="pkw_f" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e2 * f_rate), file=routes)
        print(
            '     	<flow id="pkw13_f" type="pkw_f" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e3 * f_rate), file=routes)
        print(
            '     	<flow id="pkw21_f" type="pkw_f" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e1 * f_rate), file=routes)
        print(
            '     	<flow id="pkw22_f" type="pkw_f" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e2 * f_rate), file=routes)
        print(
            '     	<flow id="pkw23_f" type="pkw_f" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e3 * f_rate), file=routes)
        print(
            '     	<flow id="pkw11_m" type="pkw_m" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e1 * m_rate), file=routes)
        print(
            '     	<flow id="pkw12_m" type="pkw_m" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e2 * m_rate), file=routes)
        print(
            '     	<flow id="pkw13_m" type="pkw_m" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e3 * m_rate), file=routes)
        print(
            '     	<flow id="pkw21_m" type="pkw_m" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e1 * m_rate), file=routes)
        print(
            '     	<flow id="pkw22_m" type="pkw_m" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e2 * m_rate), file=routes)
        print(
            '     	<flow id="pkw23_m" type="pkw_m" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e3 * m_rate), file=routes)
        print(
            '     	<flow id="bus11" type="bus" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e1 * s_rate), file=routes)
        print(
            '     	<flow id="bus12" type="bus" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e2 * s_rate), file=routes)
        print(
            '     	<flow id="bus13" type="bus" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s1 * p_e3 * s_rate), file=routes)
        print(
            '     	<flow id="bus21" type="bus" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e1 * s_rate), file=routes)
        print(
            '     	<flow id="bus22" type="bus" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e2 * s_rate), file=routes)
        print(
            '     	<flow id="bus23" type="bus" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        traffic_base * p_s2 * p_e3 * s_rate), file=routes)

        print('     	<trip id="pkw_my" type="pkw_special" depart="100" from="gneE0" to="gneE7" departLane="free" departSpeed ="random"/> ',file=routes)
        print("</routes>", file=routes)


def run():
    """execute the TraCI control loop"""
    # traci.vehicle.subscribe("pkw_my", (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
    # print(traci.vehicle.getSubscriptionResults("pkw_my"))
    step = 0
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        #print(traci.vehicle.getSubscriptionResults("left_0"))
        #print(traci.vehicle.getPosition("pkw_my"))
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
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/highway.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
run()
