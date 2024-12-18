from Liability_Identification_Model.trace_parser import Parser

class LImodel:
    def __init__(self, scenario, roughfilter, refilter, finefilter):
        
        self.parser = Parser(scenario)

        self.roughfilter = roughfilter
        self.refilter = refilter
        self.finefilter = finefilter

        
    def run(self):
        fault_judgement = self.roughfilter.judge_if_fault(self.scenario)
        ego_fault_judgement = self.roughfilter.judge_if_fault(self.data["ego"])
        npc_fault_judgement = self.roughfilter.judge_if_fault(self.data["npc"])
        
        if ego_fault_judgement == False and npc_fault_judgement == True:
            # ego无过错,ego无责,npc全责
            print("\nego vehicle is not liable for the accident!")
            print("\nNPC vehicle is fully liable for the accident!")
        
        if ego_fault_judgement == True and npc_fault_judgement == False:
            # npc无过错,ego全责,npc无责
            print("\nego vehicle is fully liable for the accident!")
            print("\nNPC vehicle is not liable for the accident!")

        else:
            # 双方都有过错,CAT- causal analysis tool
            # 先将trace转换成behavior
            ego_behavior = self.roughfilter.convert_trace_into_behavior(self.data["ego"]["trace"])
            npc_behavior = self.roughfilter.convert_trace_into_behavior(self.data["npc"]["trace"])

            ego_causal_relationship = self.refilter.judge_if_has_causal_relationship(ego_behavior)
            npc_causal_relationship = self.refilter.judge_if_has_causal_relationship(npc_behavior)

            if ego_causal_relationship == False:
                # ego过错不是引起事故的原因,ego无责,npc全责
                print("\nego vehicle has no liability for the accident!")
                print("\nNPC vehicle has fully liability for the accident!")
            if npc_causal_relationship == False:
                # npc过错不是引起事故的原因,ego全责,npc无责
                print("\nego vehicle has fully liability for the accident!")
                print("\nNPC vehicle has no liability for the accident!")

            else:
                # 双方过错都是引起事故的原因,进行更精细的责任划分
                ego_severity = self.finefilter.judge_severity_of_behavior(ego_behavior)
                npc_severity = self.finefilter.judge_severity_of_behavior(npc_behavior)

                if ego_severity > npc_severity:
                    # ego主责,npc次责
                    print("\nego vehicle has primary liability for the accident!")
                    print("\nNPC vehicle has secondary liability for the accident!")

                if ego_severity < npc_severity:
                    # ego次责,npc主责
                    print("\nego vehicle has secondary liability for the accident!")
                    print("\nNPC vehicle has primary liability for the accident!")

                if ego_severity == npc_severity:
                    # 同等责任
                    print("\nBoth ego and NPC vehicles have equal liability for the accident!")
            
    
