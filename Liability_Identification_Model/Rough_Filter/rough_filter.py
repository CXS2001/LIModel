import carla
import json
import os
import sys
import utils.extendmethod as em
from fault_monitor import Monitor

class RoughFilter:
    def __init__(self):
        self.fault_judgement = {}


    def judge_if_fault(self, msg):
        for i in len(msg['AgentNames']):
            self.fault_judgement[msg['AgentNames'][i]] = False

            self.monitor = Monitor(msg)
            list_fault = self.monitor.continuous_monitor_for_fault()

            violated_law_number = []

            for key, value in list_fault.items():
                if value >= 0.0:
                    self.law_judgement = True
                    violated_law_number.append(em.extract_characters(key))

        return self.fault_judgement
    
    def convert_trace_into_behavior(self, trace_list):
        behavior_list = []
        for i in range(len(trace_list)):

            behavior_list[i] = "Accelerate"
        return behavior_list
        