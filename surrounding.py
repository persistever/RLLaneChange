# coding:utf-8
import os
import sys
import random
import math

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import traci.constants as tc


class Surrounding:
    def __init__(self, id,downstreamDist=200.0, upstreamDist=200.0):
        self.id = id
        self.downstreamDist = downstreamDist
        self.upstreamDist = upstreamDist
        self.neighborList = None
        self.edge = "gneE0"
        self.maxSpeedList = []
        self.neighborDict = None
        self.x = 0
        self.y = 0
        self.laneNumber = 4
        self.laneWidth = 3.2
        self.laneIndex = 0
        self.leftLeaderNeighborList = []
        self.leftFollowerNeighborList = []
        self.rightLeaderNeighborList = []
        self.rightFollowerNeighborList = []
        self.midLeaderNeighborList = []
        self.midFollowerNeighborList = []
        self.edgeList = []  # for route
        self.edgeDict = None  # for route
        self.edgeLengthList = []  # for route
        self.edgeLengthDict = None  # for route
        self.laneNumberDict = None  # for all edge
        self.edgeIdList = []  # for for all edge

    def get_neighbor_list(self):
        return self.neighborList

    def get_left_leader_neighbor_list(self):
        return self.leftLeaderNeighborList

    def get_left_follower_neighbor_list(self):
        return self.leftFollowerNeighborList

    def get_right_leader_neighbor_list(self):
        return self.rightLeaderNeighborList

    def get_right_follower_neighbor_list(self):
        return self.rightFollowerNeighborList

    def get_mid_leader_neighbor_list(self):
        return self.midLeaderNeighborList

    def get_mid_follower_neighbor_list(self):
        return self.midFollowerNeighborList

    def get_max_speed_list(self):
        return self.maxSpeedList

    def get_edge_list(self):
        return self.edgeList

    def get_edge_dict(self):
        return self.edgeDict

    def get_all_edge_lane_number_dict(self):
        return self.laneNumberDict

    def get_lane_index(self):
        return self.laneIndex

    def _get_neighbor_list(self):
        self.neighborDict = traci.vehicle.getContextSubscriptionResults(self.id)
        self.x = traci.vehicle.getPosition("ego")[0]
        self.y = traci.vehicle.getPosition("ego")[1]
        if self.neighborDict != None:
            self.neighborList = []
            for name, value in self.neighborDict.items():
                self.neighborList.append({'name': name,
                                          'position_x': value[tc.VAR_POSITION][0],
                                          'position_y': value[tc.VAR_POSITION][1],
                                          'relative_position_x': value[tc.VAR_POSITION][0] - self.x,
                                          'relative_position_y': value[tc.VAR_POSITION][1] - self.y,
                                          'speed': value[tc.VAR_SPEED],
                                          'edge': value[tc.VAR_ROAD_ID],
                                          'edges': value[tc.VAR_EDGES],
                                          'lane_index': value[tc.VAR_LANE_INDEX],
                                          'lane_number': self.laneNumberDict[value[tc.VAR_ROAD_ID]],
                                          'lane_position': value[tc.VAR_LANEPOSITION],
                                          'lane_position_lat': value[tc.VAR_LANEPOSITION_LAT],
                                          'relative_lane_position': value[tc.VAR_POSITION][0] - self.x,
                                          'relative_lane_position_abs': math.fabs(value[tc.VAR_POSITION][0] - self.x),
                                          # 'relative_speed': value[tc.VAR_SPEED] - self.neighborDict[self.id][tc.VAR_SPEED]
                                          })
        else:
            self.neighborList = []

    def _get_edge(self):
        self.edge = traci.vehicle.getRoadID(self.id)

    def _get_lane_number(self):
        if self.edge != '':
            self.laneNumber = traci.edge.getLaneNumber(self.edge)
        else:
            self.laneNumber = 4

    def _get_lane_index(self):
        self.laneIndex = self.laneNumber - math.ceil(-self.y / self.laneWidth)

    def _get_edge_list(self):
        self.edgeList = list(traci.vehicle.getRoute(self.id))

    def _get_edge_dict(self):
        self.edgeDict = None
        # self.get_edge_list() # for use alone
        self.edgeDict = dict(zip(self.edgeList, range(len(self.edgeList))))

    def _get_edge_length_list(self):
        self.edgeLengthList = []
        # self.get_edge_list() # for use alone
        for i in range(len(self.edgeList)):
            self.edgeLengthList.append(traci.lane.getLength(self.edgeList[i]+"_"+str(0)))

    def _get_edge_length_dict(self):
        self.edgeLengthDict = {}
        # self.get_edge_list() # for use alone
        for edge in self.edgeList:
            self.edgeLengthDict[edge] = traci.lane.getLength(edge+"_"+str(0))

    def _get_max_speed_list(self):
        self.maxSpeedList = []
        # self.get_edge()  # for use alone
        # self.get_lane_index()  # for use alone
        if self.edge == '':
            self.edge = 'gneE1'
        for i in range(traci.edge.getLaneNumber(self.edge)):
            self.maxSpeedList.append(traci.lane.getMaxSpeed(self.edge + "_"+str(i)))

    def _classify(self):  # can not use alone
        self.leftLeaderNeighborList = []
        self.leftFollowerNeighborList = []
        self.rightLeaderNeighborList = []
        self.rightFollowerNeighborList = []
        self.midLeaderNeighborList = []
        self.midFollowerNeighborList = []
        for vehicle in self.neighborList:
            if vehicle['lane_number'] - vehicle['lane_index'] == self.laneNumber - self.laneIndex:
                if vehicle['relative_lane_position'] > 0:
                    self.midLeaderNeighborList.append(vehicle)
                if vehicle['relative_lane_position'] < 0:
                    self.midFollowerNeighborList.append(vehicle)
            if vehicle['lane_number'] - vehicle['lane_index'] == self.laneNumber - self.laneIndex + 1:
                if vehicle['relative_lane_position'] > 0:
                    self.rightLeaderNeighborList.append(vehicle)
                if vehicle['relative_lane_position'] < 0:
                    self.rightFollowerNeighborList.append(vehicle)
            if vehicle['lane_number'] - vehicle['lane_index'] == self.laneNumber - self.laneIndex - 1:
                if vehicle['relative_lane_position'] > 0:
                    self.leftLeaderNeighborList.append(vehicle)
                if vehicle['relative_lane_position'] < 0:
                    self.leftFollowerNeighborList.append(vehicle)

    def get_surroundings(self):
        self._get_edge()
        self._get_lane_number()
        self._get_lane_index()
        self._get_edge_list()
        self._get_edge_dict()
        self._get_edge_length_list()
        self._get_edge_length_dict()
        # self._get_max_speed_list()
        self._get_neighbor_list()
        self._classify()

    def _subscribe_ego_vehicle_surrounding(self):
        traci.vehicle.subscribeContext(self.id, tc.CMD_GET_VEHICLE_VARIABLE, 200.0,
                                       [tc.VAR_LANE_INDEX, tc.VAR_POSITION,
                                        tc.VAR_SPEED, tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION,
                                        tc.VAR_LANEPOSITION_LAT, tc.VAR_EDGES])
        # traci.vehicle.addSubscriptionFilterLanes([-2, -1, 0, 1, 2], noOpposite=True,
        #                                          downstreamDist=self.downstreamDist,
        #                                          upstreamDist=self.upstreamDist)

    def _get_lane_number_dict(self):
        self.edgeIdList = traci.edge.getIDList()
        self.laneNumberDict = {}
        for edge in self.edgeIdList:
            self.laneNumberDict[edge] = traci.edge.getLaneNumber(edge)

    def surrounding_init(self):
        self._subscribe_ego_vehicle_surrounding()
        self._get_lane_number_dict()


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
                    <vType id="pkw_h" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="30" \
            guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="green"/>
                    <vType id="pkw_f" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="20" \
            guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="green"/>
                    <vType id="pkw_m" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="12" \
            guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="yellow"/>
                    <vType id="bus" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="2.5" maxSpeed="5" \
            guiShape="bus" laneChangeModel="SL2015" latAlignment="center" color="red"/>
                    <vType id="pkw_special" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="0.000001" \
            guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="blue"/>""", file=routes)
            print("""
                    <route id="route_ego" color="1,1,0" edges="gneE0 gneE1 gneE2 gneE3 gneE4 gneE5 gneE6 gneE7"/>""", file=routes)
            for traffic in self.trafficList:
                print(
                    '     <flow id="%s" type="%s" from="%s" to="%s" begin="%d" end="%d" probability="%f" departLane="free" departSpeed ="random"/> '
                        %( traffic['id'], traffic['type'], traffic['from'], traffic['to'], traffic['begin'], traffic['end'], traffic['possbability']), file=routes)
            print(
                '     	<trip id="ego" type="pkw_special" depart="100" from="gneE0" to="gneE2" departLane="free" departSpeed ="random"/> ',
                file=routes)
            # print("""       <vehicle id="ego" type="pkw_special" route="route_ego" depart="30" color="blue"/>""",
            #       file=routes)
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
                <vType id="pkw_h" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="30" \
        guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="orange"/>
                <vType id="pkw_f" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="20" \
        guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="green"/>
                <vType id="pkw_m" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="12" \
        guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="yellow"/>
                <vType id="bus" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="2.5" maxSpeed="5" \
        guiShape="bus" laneChangeModel="SL2015" latAlignment="center" color="red"/>
                <vType id="pkw_special" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="0.000001" \
        guiShape="passenger" laneChangeModel="SL2015" latAlignment="center" color="blue"/>""", file=routes)
            print(
                """<route id="route_ego" color="1,1,0" edges="gneE0 gneE1 gneE2"/>""", file=routes
            )
            print(
                '     	<flow id="pkw11_f" type="pkw_f" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e1 * f_rate), file=routes)

            print(
                '     	<flow id="pkw13_f" type="pkw_f" from="gneE0" to="gneE2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e3 * f_rate), file=routes)
            print(
                '     	<flow id="pkw21_f" type="pkw_f" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e1 * f_rate), file=routes)

            print(
                '     	<flow id="pkw23_f" type="pkw_f" from="Zadao1" to="gneE2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e3 * f_rate), file=routes)
            print(
                '     	<flow id="pkw11_m" type="pkw_m" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e1 * m_rate), file=routes)

            print(
                '     	<flow id="pkw13_m" type="pkw_m" from="gneE0" to="gneE2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e3 * m_rate), file=routes)
            print(
                '     	<flow id="pkw21_m" type="pkw_m" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e1 * m_rate), file=routes)

            print(
                '     	<flow id="pkw23_m" type="pkw_m" from="Zadao1" to="gneE2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e3 * m_rate), file=routes)
            print(
                '     	<flow id="pkw11_h" type="pkw_h" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e1 * m_rate), file=routes)

            print(
                '     	<flow id="pkw13_h" type="pkw_h" from="gneE0" to="gneE2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e3 * m_rate), file=routes)
            print(
                '     	<flow id="pkw21_h" type="pkw_h" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e1 * m_rate), file=routes)

            print(
                '     	<flow id="pkw23_h" type="pkw_h" from="Zadao1" to="gneE2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e3 * m_rate), file=routes)
            print(
                '     	<flow id="bus11" type="bus" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e1 * s_rate), file=routes)

            print(
                '     	<flow id="bus13" type="bus" from="gneE0" to="gneE2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s1 * p_e3 * s_rate), file=routes)
            print(
                '     	<flow id="bus21" type="bus" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e1 * s_rate), file=routes)

            print(
                '     	<flow id="bus23" type="bus" from="Zadao1" to="gneE2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.trafficBase * p_s2 * p_e3 * s_rate), file=routes)
            print(
                '     	<trip id="ego" type="pkw_special" depart="100" from="gneE0" to="gneE2" departLane="1" departSpeed ="random"/> ',file=routes)
            # print("""       <vehicle id="ego" type="pkw_special" route="route_ego" depart="30" color="blue"/>""", file=routes)
            print("</routes>", file=routes)

