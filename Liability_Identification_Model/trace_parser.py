class Parser:
    def __init__(self, trace):
        self.init_trace = trace

        self.time = self.init_scenario['time']
        self.weather = self.init_scenario['weather']

        # The status of vehicle in every frame includes position, orientation, velocity, acceleration and size(bounding box)
        # Init trace includes the truth(in real world) and perception(in the view of ego) value

        self.map_info = {
            'MapName': self.init_scenario['MapName'],
            'time': self.time,
            'weather': self.weather,
        }

        self.parse()
        

    def parse(self):

        self.participant_info = dict()

        ego_init_info = self.init_trace['ego']

        self.participant_info['ego'] = ego_init_info
        self.participant_info['ego']['trace'] = []

        for _item in self.init_trace['AgentNames']:
            for i in self.init_trace['npcList']:
                if self.init_trace['npcList'][i]['ID'] == _item:
                    self.participant_info[_item] = self.init_trace['npcList'][i]
                    self.participant_info[_item]['trace'] = []
        
        
