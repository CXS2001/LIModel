import os
import json

from Liability_Identification_Model import LImodel
from Liability_Identification_Model.Rough_Filter.rough_filter import RoughFilter
from Liability_Identification_Model.Re_Filter.refilter import ReFilter
from Liability_Identification_Model.Fine_Filter.fine_filter import FineFilter


def load_scenario_script():
    path = f"saved_scenario/test_scenario.json"
    
    if os.path.isfile(path):
        input_file = path
        with open(input_file) as f:
            scenario = json.load(f)
    
    return scenario

def set_LImodel(scenario):

    roughfilter = RoughFilter
    refilter = ReFilter
    finefilter = FineFilter
    
    return LImodel(scenario, roughfilter, refilter, finefilter)    
def main():
    final_liability = {}
    # load the scnerario script
    scenario = load_scenario_script()

    limodel = set_LImodel(scenario)
    
    limodel.run()

        
    
    
if __name__ == '__main__':
    main()