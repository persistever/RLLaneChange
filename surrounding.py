# coding:utf-8

import traci
import traci.constants as tc
import random


class Surrounding:
    def __init__(self, traffic_base, traffic_list=[]):
        self.traffic_base = traffic_base
        self.traffic_list = traffic_list

    def traffic_init_custom(self):
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
            for traffic in self.traffic_list:
                print(
                    '     <flow id="%s" type="%s" from="%s" to="%s" begin="%d" end="%d" probability="%f" departLane="free" departSpeed ="random"/> '
                        %( traffic['id'], traffic['type'], traffic['from'], traffic['to'], traffic['begin'], traffic['end'], traffic['possbability']), file=routes)
            print(
                '     	<trip id="ego" type="pkw_special" depart="0" from="gneE0" to="gneE7" departLane="free" departSpeed ="random"/> ',
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
                        self.traffic_base * p_s1 * p_e1 * f_rate), file=routes)
            print(
                '     	<flow id="pkw12_f" type="pkw_f" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s1 * p_e2 * f_rate), file=routes)
            print(
                '     	<flow id="pkw13_f" type="pkw_f" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s1 * p_e3 * f_rate), file=routes)
            print(
                '     	<flow id="pkw21_f" type="pkw_f" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e1 * f_rate), file=routes)
            print(
                '     	<flow id="pkw22_f" type="pkw_f" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e2 * f_rate), file=routes)
            print(
                '     	<flow id="pkw23_f" type="pkw_f" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e3 * f_rate), file=routes)
            print(
                '     	<flow id="pkw11_m" type="pkw_m" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s1 * p_e1 * m_rate), file=routes)
            print(
                '     	<flow id="pkw12_m" type="pkw_m" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s1 * p_e2 * m_rate), file=routes)
            print(
                '     	<flow id="pkw13_m" type="pkw_m" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s1 * p_e3 * m_rate), file=routes)
            print(
                '     	<flow id="pkw21_m" type="pkw_m" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e1 * m_rate), file=routes)
            print(
                '     	<flow id="pkw22_m" type="pkw_m" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e2 * m_rate), file=routes)
            print(
                '     	<flow id="pkw23_m" type="pkw_m" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e3 * m_rate), file=routes)
            print(
                '     	<flow id="bus11" type="bus" from="gneE0" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s1 * p_e1 * s_rate), file=routes)
            print(
                '     	<flow id="bus12" type="bus" from="gneE0" to="gneE8" begin="0" end="500" probability="%f" epartLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s1 * p_e2 * s_rate), file=routes)
            print(
                '     	<flow id="bus13" type="bus" from="gneE0" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s1 * p_e3 * s_rate), file=routes)
            print(
                '     	<flow id="bus21" type="bus" from="Zadao1" to="Zadao2" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e1 * s_rate), file=routes)
            print(
                '     	<flow id="bus22" type="bus" from="Zadao1" to="gneE8" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e2 * s_rate), file=routes)
            print(
                '     	<flow id="bus23" type="bus" from="Zadao1" to="gneE7" begin="0" end="500" probability="%f" departLane="free" departSpeed ="random"/> ' % (
                        self.traffic_base * p_s2 * p_e3 * s_rate), file=routes)
            print(
                '     	<trip id="ego" type="pkw_special" depart="0" from="gneE0" to="gneE7" departLane="free" departSpeed ="random"/> ',file=routes)
            print("</routes>", file=routes)

