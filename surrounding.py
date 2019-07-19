# coding:utf-8
import os
import sys

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import traci.constants as tc
import random


class Surrounding:
    def __init__(self, id,downstreamDist=200.0, upstreamDist=200.0):
        self.id = id
        self.downstreamDist = downstreamDist
        self.upstreamDist = upstreamDist
        self.neighborList = None
        self.edge = "gneE0"
        self.maxSpeedList = []

    def get_neighbor_list(self):
        dicts = traci.vehicle.getContextSubscriptionResults(self.id)
        if dicts != None:
            self.neighborList = []
            for name, value in dicts.items():
                self.neighborList.append({'name': name, 'lane': value[tc.VAR_LANE_INDEX],
                                          'position_x': value[tc.VAR_POSITION][0], 'position_y': value[tc.VAR_POSITION][1], 'speed': value[tc.VAR_SPEED]})
            return self.neighborList
        else:
            self.neighborList = []
            return self.neighborList

    def get_MaxSpeed_list(self):
        self.maxSpeedList = []
        self.edge = traci.vehicle.getRoadID(self.id)
        for i in range(traci.edge.getLaneNumber(self.edge)):
            self.maxSpeedList.append(traci.lane.getMaxSpeed(self.edge + "_"+str(i)))
        return self.maxSpeedList

    def subscribe_ego_vehicle_surrounding(self):
        traci.vehicle.subscribeContext(self.id, tc.CMD_GET_VEHICLE_VARIABLE, 0.0, [tc.VAR_LANE_INDEX, tc.VAR_POSITION, tc.VAR_SPEED])
        traci.vehicle.addSubscriptionFilterLanes([-2, -1, 0, 1, 2], noOpposite=True, downstreamDist=self.downstreamDist,
                                                 upstreamDist=self.upstreamDist)


class Traffic:
    def __init__(self, trafficBase=0.5, trafficList=None):
        self.trafficBase = trafficBase
        self.trafficList = trafficList
        if self.trafficList is None:
            self.traffic_init_general()
        else:
            self.traffic_init_custom()

    def traffic_init_custom(self):
        with open("data/motorway.rou.xml", "w") as routes:
            print("""<routes>
                    <vType id="pkw_f" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="35" \
            guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="green"/>
                    <vType id="pkw_m" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="25" \
            guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="yellow"/>
                    <vType id="bus" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="2.5" maxSpeed="20" \
            guiShape="bus" laneChangeModel="SL2015" latAlignment="center" color="red"/>
                    <vType id="pkw_special" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="0.000001" \
            guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="blue"/>""", file=routes)
            print("""
                    <route id="route_ego" color="1,1,0" edges="gneE0 gneE1 gneE2 gneE3 gneE4 gneE5 gneE6 gneE8"/>""", file=routes)
            for traffic in self.trafficList:
                print(
                    '     <flow id="%s" type="%s" from="%s" to="%s" begin="%d" end="%d" probability="%f" departLane="free" departSpeed ="random"/> '
                        %( traffic['id'], traffic['type'], traffic['from'], traffic['to'], traffic['begin'], traffic['end'], traffic['possbability']), file=routes)
            # print(
            #     '     	<trip id="ego" type="pkw_special" depart="10" from="gneE0" to="gneE7" departLane="free" departSpeed ="random"/> ',
            #     file=routes)
            print("""       <vehicle id="ego" type="pkw_special" route="route_ego" depart="30" color="blue"/>""",
                  file=routes)
            print("</routes>", file=routes)

    def traffic_init_general(self):
        f_rate = random.uniform(0.4, 0.6)
        s_rate = random.uniform(0.05, 0.1)
        m_rate = 1 - f_rate - s_rate
        p_s1 = random.uniform(0.9, 0.95)
        p_s2 = 1 - p_s1
        p_e1 = random.uniform(0.05, 0.1)
        p_e2 = random.uniform(0.3, 0.4)
        p_e3 = 1 - p_e1 - p_e2
        with open("data/motorway.rou.xml", "w") as routes:
            print("""<routes>
                <vType id="pkw_f" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="35" \
        guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="green"/>
                <vType id="pkw_m" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="25" \
        guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="yellow"/>
                <vType id="bus" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="2.5" maxSpeed="20" \
        guiShape="bus" laneChangeModel="SL2015" latAlignment="center" color="red"/>
                <vType id="pkw_special" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="0.000001" \
        guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="blue"/>""", file=routes)
            print(
                """<route id="route_ego" color="1,1,0" edges="gneE0 gneE1 gneE2 gneE3 gneE4 gneE5 gneE6 gneE8"/>""", file=routes
            )
            print(
                '     	<flow id="pkw11_f" type="pkw_f" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e1 * f_rate), file=routes)
            print(
                '     	<flow id="pkw12_f" type="pkw_f" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e2 * f_rate), file=routes)
            print(
                '     	<flow id="pkw13_f" type="pkw_f" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e3 * f_rate), file=routes)
            print(
                '     	<flow id="pkw21_f" type="pkw_f" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e1 * f_rate), file=routes)
            print(
                '     	<flow id="pkw22_f" type="pkw_f" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e2 * f_rate), file=routes)
            print(
                '     	<flow id="pkw23_f" type="pkw_f" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e3 * f_rate), file=routes)
            print(
                '     	<flow id="pkw11_m" type="pkw_m" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e1 * m_rate), file=routes)
            print(
                '     	<flow id="pkw12_m" type="pkw_m" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e2 * m_rate), file=routes)
            print(
                '     	<flow id="pkw13_m" type="pkw_m" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e3 * m_rate), file=routes)
            print(
                '     	<flow id="pkw21_m" type="pkw_m" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e1 * m_rate), file=routes)
            print(
                '     	<flow id="pkw22_m" type="pkw_m" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e2 * m_rate), file=routes)
            print(
                '     	<flow id="pkw23_m" type="pkw_m" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e3 * m_rate), file=routes)
            print(
                '     	<flow id="bus11" type="bus" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e1 * s_rate), file=routes)
            print(
                '     	<flow id="bus12" type="bus" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e2 * s_rate), file=routes)
            print(
                '     	<flow id="bus13" type="bus" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e3 * s_rate), file=routes)
            print(
                '     	<flow id="bus21" type="bus" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e1 * s_rate), file=routes)
            print(
                '     	<flow id="bus22" type="bus" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e2 * s_rate), file=routes)
            print(
                '     	<flow id="bus23" type="bus" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e3 * s_rate), file=routes)
            # print(
            #     '     	<trip id="ego" type="pkw_special" depart="10" from="gneE0" to="gneE7" departLane="free" departSpeed ="random"/> ',file=routes)
            print("""       <vehicle id="ego" type="pkw_special" route="route_ego" depart="30" color="blue"/>""", file=routes)
            print("</routes>", file=routes)

