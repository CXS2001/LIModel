import carla
import math
import Cognitive_Driver_Model.utils.globalvalues as gv
from Cognitive_Driver_Model.vehicles.virtualvehicle import VirtualVehicle
from Cognitive_Driver_Model.utils.extendmath import cal_yaw_by_location




class RealVehicle:
    """
    RealVehicle implements a vehicle in carla simulator that navigates the scene.
    """
    def __init__(self, vehicle, map, target_speed, opt_dict, controller):
        """
        Initialization the RealVehicle paramters and CDM controller.

            :param vehicle: carla vehicle actor
            :param map: carla.Map
            :param target_speed: speed (in Km/h) at which the vehicle will reach
            :param opt_dict: dictionary in case some of its parameters want to be changed.
            :param controller: CDM instance

        """
        self._vehicle = vehicle
        self._map = map
        self._world = self._vehicle.get_world()
        self._target_speed = target_speed
        self._controller = controller

        # Base control parameters
        self._control = carla.VehicleControl(
            throttle = opt_dict['throttle'],
            steer = opt_dict['steer'],
            brake = opt_dict['brake'],
            reverse = opt_dict['reverse'],
        )

        self._lights_list = self._world.get_actors().filter("*traffic_light*")


    def get_cdm_controller(self, full_vehicle_dict, nnet):
        self._control_action = self._controller.run_forward(full_vehicle_dict, nnet)
        # control_action includes maintain, accelerate, decelerate, turn_left, turn_right, slide_left, sidle_right, emergency_brake, reverse, etc...

    def convert_action_into_route(self):
        self._control = self._vehicle.get_control()
        if self._control == "MAINTAIN":
            pass
        if self._control_action == "ACCELERATE":
            self._control.throttle = 

        if self._control_action == "Emergency_Stop":
            self._control.throttle = 0
            self._control.brake = 1
            self._control.hand_brake = False


    def run_step(self):
        """Run a step of navigation"""
        vehicle_speed = 



    def clone_to_virtual(self):
        """
        创建一个状态与自身相同的VirtualVehicle
        """
        return VirtualVehicle(
            self._vehicle.id,
            self._map.get_waypoint(self._vehicle.get_location()),
            self._vehicle.get_transform(),
            self._scalar_velocity,
            self._control_action,
        )

    def cal_lanechanging_route(self):
        """
        计算变道过程中的路径
        Return: List[carla.Transform]
        """
        traj_length = int(gv.LANE_CHANGE_TIME / gv.STEP_DT)
        route = [None] * (traj_length + 1)
        # Route的首个元素包含变道开始前最后一个wp, 在调用时轨迹应从route[1]开始
        route[0] = self._vehicle.get_transform()
        route[-1] = self._target_dest_wp.transform
        # 计算变道时车辆的朝向
        direction_vector = route[-1].location - route[0].location
        yaw = cal_yaw_by_location(route[-1].location, route[0].location)
        rotation = carla.Rotation(pitch=0, yaw=yaw, roll=0)
        for i in range(1, traj_length):
            location = carla.Location(
                x=route[0].location.x + i * direction_vector.x / (traj_length - 1),
                y=route[0].location.y + i * direction_vector.y / (traj_length - 1),
                z=route[0].location.z,
            )
            wp = self._map.get_waypoint(location)
            location.z = wp.transform.location.z
            rotation.pitch = wp.transform.rotation.pitch
            route[i] = carla.Transform(
                location=location,
                rotation=rotation,
            )
        return route

    def descrete_control(self):
        """
        离散动作转换为路径规划
        """
        longitude_action_list = ["MAINTAIN", "ACCELERATE", "DECELERATE"]
        lateral_action_list = ["SLIDE_LEFT", "SLIDE_RIGHT"]

        if self._next_waypoint == None:
            self._next_waypoint = self._map.get_waypoint(self._spawn_point.location)

        if self._control_action in longitude_action_list:
            # 加速, 减速, 保持车道, 过程离散化
            self._scalar_velocity += (gv.LON_ACC_DICT.get(self._control_action) * gv.STEP_DT)
            wp_gap = self._scalar_velocity * gv.STEP_DT

            if wp_gap == 0:
                pass
            if wp_gap > 0:
                self._next_waypoint = self._next_waypoint.next(wp_gap)[0]
            if wp_gap < 0:
                self._next_waypoint = self._next_waypoint.previous(-wp_gap)[0]

            self._target_lane_id = self._next_waypoint.lane_id
            self._vehicle.set_transform(self._next_waypoint.transform)

        if self._control_action in lateral_action_list:
            # 涉及到车道变换的动作, 设定一个目标路点, 一秒行驶到目标位置
            current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
            self._current_dest_wp = current_waypoint.next(self._scalar_velocity)[0]

            if self._control_action == "SLIDE_LEFT":
                if self._current_dest_wp.get_left_lane().lane_type != "Shoulder":
                    self._target_dest_wp = self._current_dest_wp.get_left_lane()
                else:
                    self._target_dest_wp = self._current_dest_wp

            if self._control_action == "SLIDE_RIGHT":
                if self._current_dest_wp.get_right_lane().lane_type != "Shoulder":
                    self._target_dest_wp = self._current_dest_wp.get_right_lane()
                else:
                    self._target_dest_wp = self._current_dest_wp
                    
            if self._changing_lane_pace == 0:
                self._lane_changing_route = self.cal_lanechanging_route()
                self._target_lane_id = self._map.get_waypoint(self._lane_changing_route[-1].location).lane_id

            self._changing_lane_pace += 1

            if self._changing_lane_pace < len(self._lane_changing_route) - 1:
                self._next_waypoint = self._map.get_waypoint(self._lane_changing_route[self._changing_lane_pace].location)
                self._vehicle.set_transform(self._lane_changing_route[self._changing_lane_pace])
            else:
                # 变道完成
                self._changing_lane_pace = 0
                self._next_waypoint = self._map.get_waypoint(self._lane_changing_route[-1].location).next(self._scalar_velocity * gv.STEP_DT)[0]
                self._vehicle.set_transform(self._next_waypoint.transform)

    def collision_callback(self, other_vehicle):
        """
        判断两车是否发生碰撞
        """
        # 若两车水平距离大于bounding box对角线长度之和，则必然未碰撞
        if (
            self._vehicle.get_location() == carla.Location(0, 0, 0)
            or self._vehicle.get_location().distance_squared_2d(
                other_vehicle.get_location()
            )
            > (
                self._vehicle.bounding_box.extent.x**2
                + self._vehicle.bounding_box.extent.y**2
            )
            * 4
        ):
            return False
        # 否则计算两车相对位置向量在主车前进方向上的投影，与车身长度进行比较
        distance_vector = other_vehicle.get_location() - self._vehicle.get_location()
        forward_vector = self._vehicle.get_transform().get_forward_vector()
        projection = abs(
            distance_vector.dot_2d(forward_vector)
            / forward_vector.distance_2d(carla.Vector3D(0, 0, 0))
        )
        if (
            distance_vector.distance_squared_2d(carla.Vector3D(0, 0, 0))
            - projection * projection
            > 0.01
        ):
            normal = math.sqrt(
                distance_vector.distance_squared_2d(carla.Vector3D(0, 0, 0))
                - projection * projection
            )
        else:
            normal = 0
        if (
            projection
            <= self._vehicle.bounding_box.extent.x + other_vehicle.bounding_box.extent.x
            and normal
            <= self._vehicle.bounding_box.extent.y + other_vehicle.bounding_box.extent.y
        ):
            return True
        else:
            return False
        
    def is_destination(self, dest_wp):
        if self._next_waypoint == dest_wp:
            return True
        else:
            return False
