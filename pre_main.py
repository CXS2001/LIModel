import carla
import random
import torch
from Cognitive_Driver_Model.model.mask_lstm import NaiveLSTM
from Cognitive_Driver_Model.vehicles.realvehicle import RealVehicle
import Cognitive_Driver_Model.utils.globalvalues as gv
import Cognitive_Driver_Model.decisionmodels.CDM.cognitivedrivermodel as cdm
import Cognitive_Driver_Model.decisionmodels.CDM.observer as obs
import Cognitive_Driver_Model.decisionmodels.CDM.risk as rsk
import Cognitive_Driver_Model.decisionmodels.CDM.reward as rwd
import Cognitive_Driver_Model.decisionmodels.CDM.decision as dcs
import Cognitive_Driver_Model.dstructures.enumeratetree as tree
from data_bridge import DataBridge


# Run this program before running main.py
# get the accident scenario script (json file)

def initialize():
    print("Initializing...")
    
    full_realvehicle_dict = {}
    
    (world, map, vehicle_bp, spectator) = simulator_initialize()

    device = "cuda" if torch.cuda.is_available else "cpu"
    pred_net = NaiveLSTM(
        gv.INPUT_SIZE, gv.HIDDEN_SIZE, gv.NUM_CLASSES, gv.NUM_LAYERS
    ).to(device)
    pred_net.load_state_dict(torch.load("model/Traj_2lanes.pt"))
    
    (full_realvehicle_dict, ego_id) = vehicle_initialize(world, map, vehicle_bp, full_realvehicle_dict)

    print("Initialize done.")
    return (world, map, full_realvehicle_dict, ego_id, spectator, pred_net)

def simulator_initialize():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = gv.STEP_DT
    world.apply_settings(settings)
    
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("model3")[0]  # set all vehicles Model3
    spectator = world.get_spectator()

    return (world, carla_map, vehicle_bp, spectator)
    
def vehicle_initialize(world, map, vehicle_bp, full_realvehicle_dict):
    """
    生成ego和npc
    Param:
    world -> carla.World: 当前world
    map -> carla.Map: 当前map
    vehicle_bp -> carla.Blueprint: 车辆蓝图
    full_realvehicle_dict -> dict: 所有车辆字典
    """
    spawn_points = map.get_spawn_points()
    rel_distance = 30.0
    (ego_spawn_point, npc_spawn_point) = get_spawn_point(spawn_points, rel_distance)

    destination = map.get_waypoint(ego_spawn_point.location).next(300)[-1]

    # 生成ego
    ego_vehicle = world.spawn_actor(vehicle_bp, ego_spawn_point)
    ego_driving_style = 50
    full_realvehicle_dict[ego_vehicle.id] = set_acdm_vehicle(ego_vehicle, ego_driving_style, ego_vehicle.id, map, ego_spawn_point, 25, destination)
    # 生成npc
    npc_vehicle = world.spawn_actor(vehicle_bp.set_attribute("color", "255, 0, 0"), npc_spawn_point)
    npc_driving_style = 80
    full_realvehicle_dict[npc_vehicle.id] = set_acdm_vehicle(npc_vehicle, npc_driving_style, ego_vehicle.id, map, npc_spawn_point, 30, destination)
    
    return (full_realvehicle_dict, ego_vehicle.id)
    

def get_spawn_point(spawn_points, rel_distance):
    """
    以ego为中心获取车辆生成点
    Param:
    spawn_points -> list: carla.Waypoint() 当前map可生成的waypoint列表
    rel_distance -> float 相对距离
    """
    ego_spawn_point = random.choice(spawn_points)
    ego_location = ego_spawn_point.transform.location
    filtered_spawn_points = [point for point in spawn_points if ego_location.distance(point.transform.location) <= rel_distance and point != ego_spawn_point]
    npc_spawn_point = random.choice(filtered_spawn_points)
    return (ego_spawn_point, npc_spawn_point)


def set_acdm_vehicle(vehicle, driving_style, main_id, map, spawn_point, init_velocity, destination):
    id = vehicle.id
    observer = obs.CIPO_Observer(id, main_id, map)
    enumtree = tree.EnumerateTree(id, main_id, map)
    risk_cal = rsk.CDMRisk(id, main_id, map)
    reward_cal = rwd.ACDMReward(id, main_id, map)
    decision = dcs.LaplaceDecision()
    controller = cdm.NormalCognitiveDriverModel(
        driving_style, destination, observer, enumtree, risk_cal, reward_cal, decision
    )
    return RealVehicle(map, vehicle, spawn_point, controller, init_velocity)

def cdm_run(world, map, full_realvehicle_dict, ego_id, spectator, pred_net, round_number):
    ###################
    # 仿真步长
    step = 0

    # 终止指令
    stop = False

    # 终止后场景继续运行的时间: step
    stop_lifetime = 500

    full_realvehicle_id_list = list(full_realvehicle_dict.keys())
    num_vehicles = len(full_realvehicle_id_list)

    # 视角移动的时候预设的ego
    ego_vehicle = full_realvehicle_dict[ego_id]
    
    data_bridge = DataBridge(world, map, full_realvehicle_dict, full_realvehicle_id_list)
    
    data_bridge.init_scenario()
    
    # 模拟循环
    while True:
        # 正常运行
        if stop == False:
            
            # CDM决策
            if step % (gv.DECISION_DT / gv.STEP_DT) == 0:
                for car in full_realvehicle_dict.values():
                    car.run_step(full_realvehicle_dict, pred_net)
                    
            # 场景运行时间
            if step > 5000:
                stop = True
                cur_step = step
                
            # 控制车辆
            for car in full_realvehicle_dict.values():
                car.descrete_control()

                data_bridge.update_scenario()
                
            # 移动观察者视角
            spectator_tranform = carla.Transform(
                ego_vehicle.get_location() + carla.Location(z=150),
                carla.Rotation(pitch=-90, yaw=180),
            )
            spectator.set_transform(spectator_tranform)
            
            # 碰撞检测
            for i in range(num_vehicles):
                for j in range(i + 1, num_vehicles):
                    if full_realvehicle_dict[
                        full_realvehicle_id_list[i]
                    ].collision_callback(
                        full_realvehicle_dict[full_realvehicle_id_list[j]]._vehicle
                    ):
                        print("Collide!")
                        stop = True
                        cur_step = step
                        
        # 终止运行
        if stop == True:

            data_bridge.end_scenario(round_number)
            
            # LifeTime结束
            if step - cur_step >= stop_lifetime:
                # 删除所有车辆, 释放内存
                for carid in full_realvehicle_id_list:
                    full_realvehicle_dict[carid]._vehicle.destroy()
                    del full_realvehicle_dict[carid]
                full_realvehicle_id_list = []
                num_vehicles = 0
                raise KeyboardInterrupt()
        # 通知服务器进行一次模拟迭代
        world.tick()
        # time.sleep(0.01)  # 等待一帧的时间
        step += 1
    
    


def main():
    
    (world, map, full_realvehicle_dict, ego_id, spectator, pred_net) = initialize()
    
    for round_number in range(0, 10):
        cdm_run(world, map, full_realvehicle_dict, ego_id, spectator, pred_net, round_number)
    

if __name__ == '__main__':
    main()