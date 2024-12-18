"""
Create Contact between Carla and LIModel
"""
import os
import re
import json
import math
import numpy as np
import shapely.geometry
import config
import carla
import utils.extendmethod as em


class DataBridge(object):
    def __init__(self, world):
        
        self._world = world
        self._map = self._world.get_map()

        map_name = self._map.name.split("/")[-1]
        
        weather = self._world.get_weather()
        rain = weather.precipitation
        cloudy = weather.cloudiness
        wetness = weather.wetness
        fog = weather.fog_density     

        self.trace = {
            "map": map_name,
            "time": {
                "hour": self.get_hour(),
                "minute": 0
            },
            "weather": {
                "rain": rain,
                "cloudy": cloudy,
                "wetness": wetness,
                "fog": fog
            }
        }

        self.ego_vehicle = None

    def update_ego_vehicle_start(self, ego:carla.Vehicle):
        """
        when scenario loaded, parses init information from ego vehicle
        """
        self.ego_vehicle = ego
        ego_velocity = self.ego_vehicle.get_velocity()
        ego_speed = em.convert_velocity_to_speed(ego_velocity)
        ego_wp = self._map.get_waypoint(self.ego_vehicle.get_location())

        self.trace['ego'] = {
            "ID": "ego_vehicle",
            "name": ego.type_id,
            "groundTruthPerception": True,
            "color": ego.attributes['color'],
            "start": {
                "lane_position": {
                    "lane": "lane_" + str(ego_wp.road_id),
                    "offset": ego_wp.s,
                    "roadID": ego_wp.lane_id
                },
                "heading": {
                    "ref_lane_point": {
                        "lane": "lane_" + str(ego_wp.road_id),
                        "offset": ego_wp.s,
                        "roadID": ego_wp.lane_id
                    },
                    "ref_angle": 0.0
                },
                "speed": ego_speed
            }
        }

        self.trace['npcList'] = []
        self.trace["pedestrianList"] = []
        self.trace["obstacleList"] = []
        self.trace["AgentNames"] = []
        self.trace["groundTruthPerception"] = True

        self.trace["minEgoObsDist"] = None
        self.trace["Crash"] = False


    
    def update_scenario(self, step):
        frame_dict = {}
        
        timestamp = step

        frame_dict['timestamp'] = timestamp


        frame_dict["ego"] = {
            "pose": {
                "position": {
                    ""
                }
            }
        }

        for i in range(len(self.full_realvehicle_id_list)):
            
            car = self.full_realvehicle_dict[self.full_realvehicle_id_list[i]]
            vehicle = self.full_realvehicle_dict[self.full_realvehicle_id_list[i]].vehicle

            transform = vehicle.get_transform()    
            waypoint = self._map.get_waypoint(transform.location)
            qx, qy, qz, qw = em.carla_to_ros2(transform.rotation)
            
            velocity = vehicle.get_velocity()
            speed = em.convert_velocity_to_speed(velocity)
            angular_velocity = vehicle.get_angular_velocity()
            linear_acceleration = vehicle.get_acceleration()
            acc = em.convert_velocity_to_speed(linear_acceleration)
            
            control = vehicle.get_control()
            is_engine_on = False
            
            if float(control.throttle) > 0:
                is_engine_on = True

            light_state = vehicle.get_light_state()

            junction_ahead = em.get_junction_ahead(waypoint)
            
            
            frame_dict["pose"] = {
                "position": {
                    "roadID": waypoint.road_id,
                    "laneID": "lane_" + str(waypoint.lane_id),
                    "offset": waypoint.s,
                    "coordinate": {
                        "x": transform.location.x,
                        "y": transform.location.y,
                        "z": transform.location.z
                    },
                    "orientation": {
                        "qx": qx,
                        "qy": qy,
                        "qz": qz,
                        "qw": qw
                    }
                },
                "velocity": {
                    "linearvelocity": {
                        "x": velocity.x,
                        "y": velocity.y,
                        "z": velocity.z
                    },
                    "angularvelocity": {
                        "x": angular_velocity.x,
                        "y": angular_velocity.y,
                        "z": angular_velocity.z
                    },
                    "speed": speed
                },
                "acceleration": {
                    "acc": acc,
                    "x": linear_acceleration.x,
                    "y": linear_acceleration.y,
                    "z": linear_acceleration.z
                }
            }
            
            frame_dict["carStatus"] = {
                "highBeamOn": self.get_highbeam(light_state),
                "lowBeamOn": self.get_lowbeam(light_state),
                "brakeLightOn": self.get_brake(light_state), 
                "turnSignal": self.get_turnsignal(light_state),
                "reversingLight": self.get_reserse(light_state),
                "fogLightOn": self.get_fog(light_state),
                "hornOn": False,
                "engineOn": is_engine_on,
            }
            
            frame_dict["drivingStatus"] = {
                "speed": speed,
                "accelerate": acc,
                "brake": 0,
                "isChangingLane": False,
                "isOverTaking": False,
                "isTurningAround": False
            }
            
            frame_dict["Chassis"] = {
                
                "lowBeamOn": self.get_lowbeam(light_state),
                "highBeamOn": self.get_highbeam(light_state),
                "turnSignal": self.get_turnsignal(light_state),
                "fogLightOn": self.get_fog(light_state),
                
                "hornOn": False,
                "engineOn": is_engine_on,
                "gear": control.gear,
                "brake": control.brake,
                
                "day": 1,
                "hours": self.get_hour(),
                "minutes": 0,
                "seconds": 0,
                "error_code": 0
            }
            
            # frame_dict["crosswalkAhead"] = -1,
            # frame_dict["junctionAhead"] = junction_ahead,
            # frame_dict["stopSignAhead"] = -1,
            # frame_dict["stoplineAhead"] = junction_ahead,
            # frame_dict["planning_of_turn"] = turn,
            # frame_dict["isTrafficJam"] = iscongesting,
            # frame_dict["isOverTaking"] = overtaking,
            # frame_dict["isLaneChange"] = lanechanging,
            # frame_dict["isTurningAround"] = isturningaround,
            # frame_dict["PriorityNPCAhead"] = Priority_of_lanechanging,
            # frame_dict["PriorityPedestrianAhead"] = False, 
        
            frame_dict.update(self.get_affected_traffic_light()) # 没修改
            
        self.scenario["participant" + str(i+1)]["trace"].append(frame_dict)

        

    def end_scenario(self, round_number):
        ego_velocity = self.ego_vehicle.get_velocity()
        ego_speed = em.convert_velocity_to_speed(ego_velocity)
        ego_wp = self._map.get_waypoint(self.ego_vehicle.get_location())
        

        for i in range(len(self.full_realvehicle_id_list)):
            vehicle = self.full_realvehicle_dict[self.full_realvehicle_id_list[i]].vehicle
            velocity = vehicle.get_velocity()
            speed = em.convert_velocity_to_speed(velocity)
            waypoint = self._map.get_waypoint(vehicle.get_location())

            self.scenario["participant" + str(i+1)]["destination"]["position"] = {
                "roadID": waypoint.road_id,
                "laneID": "lane_" + str(waypoint.lane_id),
                "offset": waypoint.s
            }
            self.scenario["participant" + str(i+1)]["destination"]["heading"] = {
                "position": {
                    "roadID": waypoint.road_id,
                    "laneID": "lane_" + str(waypoint.lane_id),
                    "offset": waypoint.s
                },
                "ref_angle": 0.0,
            }
            self.scenario["participant" + str(i+1)]["destination"]["speed"] = speed
        
        # self.scenario["accidentType"] = None # meeting, overtaking, changing lanes, intersections, U-turns, reversing, rear-end collisions, speeding, traffic light accidents

        folder_name = "saved_scenario"
        file_name = f"scenario_{round_number}.json"

        file_path = os.path.join(folder_name, file_name)
         
        with open(file_path, 'w') as json_file:
            json.dump(self.scenario, json_file, indent=4)

                

    def get_trafficlight_trigger_location(self, traffic_light):
        """
        Calculates the yaw of the waypoint that represents the trigger volume of the traffic light
        """

        def rotate_point(point, radians):
            """
            rotate a given point by a given angle
            """
            rotated_x = math.cos(radians) * point.x - math.sin(radians) * point.y
            rotated_y = math.sin(radians) * point.x - math.cos(radians) * point.y

            return carla.Vector3D(rotated_x, rotated_y, point.z)

        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)
        area_ext = traffic_light.trigger_volume.extent

        point = rotate_point(carla.Vector3D(0, 0, area_ext.z), math.radians(base_rot))
        point_location = area_loc + carla.Location(x=point.x, y=point.y)

        return carla.Location(point_location.x, point_location.y, point_location.z)

    def get_junction_distance(self, junction, ref_waypoint):
        """
        Calculates the yaw of the waypoint that represents the trigger volume of the traffic light
        """

        def rotate_point(point, radians):
            """
            rotate a given point by a given angle
            """
            rotated_x = math.cos(radians) * point.x - math.sin(radians) * point.y
            rotated_y = math.sin(radians) * point.x - math.cos(radians) * point.y

            return carla.Vector3D(rotated_x, rotated_y, point.z)

        base_rot = junction.bounding_box.rotation.yaw
        area_ext = junction.bounding_box.extent

        point = rotate_point(carla.Vector3D(0, 0, area_ext.z), math.radians(base_rot))
        point_location = junction.bounding_box.location + carla.Location(x=point.x, y=point.y)

        return self.get_distance_between_points(
            carla.Transform(carla.Location(point_location.x, point_location.y, point_location.z)),
            ref_waypoint.transform)


    def get_highbeam(self, light_state) -> bool:
        """
        get ego_vehicle highbeam state
        """
        if light_state is None:
            return False
        if light_state == carla.VehicleLightState.HighBeam:
            return True


    def get_lowbeam(self, light_state) -> bool:
        """
        get ego_vehicle lowbeam state
        """
        if light_state is None:
            return False
        if light_state == carla.VehicleLightState.LowBeam:
            return True


    def get_turnsignal(self, light_state) -> bool:
        """
        get ego_vehicle Blinker state
        """
        if light_state is None:
            return False
        if light_state == carla.VehicleLightState.RightBlinker or light_state == carla.VehicleLightState.LeftBlinker:
            return True


    def get_affected_traffic_light(self):

        frame_dict = {}
        traffic_light_list = self._world.get_actors().filter("*traffic_light*")

        for traffic_light in traffic_light_list:
            center, waypoints = self.get_traffic_light_waypoints(traffic_light)
            self._list_traffic_lights.append((traffic_light, center, waypoints))

        ego_transform = self.ego_vehicle.get_transform()
        location = ego_transform.location

        veh_extent = self.ego_vehicle.bounding_box.extent.x

        tail_close_pt = self.rotate_point(carla.Vector3D(-0.8 * veh_extent, 0.0, location.z),
                                          ego_transform.rotation.yaw)
        tail_close_pt = location + carla.Location(tail_close_pt)

        tail_far_pt = self.rotate_point(carla.Vector3D(-veh_extent - 1, 0.0, location.z), ego_transform.rotation.yaw)
        tail_far_pt = location + carla.Location(tail_far_pt)

        for traffic_light, center, waypoints in self._list_traffic_lights:

            center_loc = carla.Location(center)

            if center_loc.distance(location) > 5:
                continue

            for wp in waypoints:

                tail_wp = self._map.get_waypoint(tail_far_pt)

                # Calculate the dot product (Might be unscaled, as only its sign is important)
                ve_dir = self.ego_vehicle.get_transform().get_forward_vector()
                wp_dir = wp.transform.get_forward_vector()
                dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

                # Check the lane until all the "tail" has passed
                if tail_wp.road_id == wp.road_id and tail_wp.lane_id == wp.lane_id and dot_ve_wp > 0:
                    # This light is red and is affecting our lane
                    yaw_wp = wp.transform.rotation.yaw
                    lane_width = wp.lane_width
                    location_wp = wp.transform.location

                    lft_lane_wp = self.rotate_point(carla.Vector3D(0.4 * lane_width, 0.0, location_wp.z), yaw_wp + 90)
                    lft_lane_wp = location_wp + carla.Location(lft_lane_wp)
                    rgt_lane_wp = self.rotate_point(carla.Vector3D(0.4 * lane_width, 0.0, location_wp.z), yaw_wp - 90)
                    rgt_lane_wp = location_wp + carla.Location(rgt_lane_wp)

                    self._last_red_light = traffic_light

                    # Is the vehicle traversing the stop line?
                    if self.is_vehicle_crossing_line((tail_close_pt, tail_far_pt), (lft_lane_wp, rgt_lane_wp)):
                        self._last_red_light = None

                    if self._last_red_light is not None:

                        color = 0
                        if self._last_red_light.state == carla.TrafficLightState.Red:
                            color = 1
                        elif self._last_red_light.state == carla.TrafficLightState.Yellow:
                            color = 2
                        elif self._last_red_light.state == carla.TrafficLightState.Green:
                            color = 3

                        frame_dict["traffic_lights"] = {
                            "containLights": True,
                            "trafficLightList": [
                                {
                                    "color": color,
                                    "id": "signal_" + str(traffic_light.id),
                                    "blink": False
                                }
                            ],
                            "trafficLightStopLine": self.get_distance_between_points(wp.transform, tail_wp.transform)
                        }

        if "traffic_lights" in frame_dict:
            pass
        else:
            frame_dict["traffic_lights"] = {
                "containLights": False
            }

        return frame_dict

    def get_traffic_light_ahead(self) -> dict:
        """
        Parses distance between ego_vehicle and affected trafficlight
        """
        traffic_light_list = self._world.get_actors().filter("*traffic_light*")
        ego_wp = self._map.get_waypoint(self.ego_vehicle.get_transform().location)

        base_tlight_threshold = 5.0

        frame_dict = {}

        for traffic_light in traffic_light_list:
            object_location = self.get_trafficlight_trigger_location(traffic_light)
            object_waypoint = self._map.get_waypoint(object_location)

            if object_waypoint.road_id != ego_wp.road_id:
                continue

            ve_dir = ego_wp.transform.get_forward_vector()
            wp_dir = object_waypoint.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue

            if traffic_light.get_state() != carla.TrafficLightState.Red:
                continue

            if self.is_within_distance(object_waypoint.transform, ego_wp.transform, base_tlight_threshold, [0, 90]):
                traffic_light_stop_line = self.get_distance_between_points(ego_wp.transform,
                                                                           object_waypoint.transform) - self.ego_vehicle.bounding_box.extent.x
                traffic_light_stop_line = max(traffic_light_stop_line, 0)

                frame_dict["traffic_lights"] = {
                    "containLights": True,
                    "trafficLightList": [
                        {
                            "color": 1,
                            "id": "signal_" + str(traffic_light.id),
                            "blink": False
                        }
                    ],
                    "trafficLightStopLine": traffic_light_stop_line
                }

        if "traffic_lights" in frame_dict:
            pass
        else:
            frame_dict["traffic_lights"] = {
                "containLights": False
            }

        return frame_dict

    def get_traffic_light_waypoints(self, traffic_light):
        """
        get area of a given traffic light
        """
        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)

        # Discretize the trigger box into points
        area_ext = traffic_light.trigger_volume.extent
        x_values = np.arange(-0.9 * area_ext.x, 0.9 * area_ext.x, 1.0)  # 0.9 to avoid crossing to adjacent lanes

        area = []
        for x in x_values:
            point = self.rotate_point(carla.Vector3D(x, 0, area_ext.z), base_rot)
            point_location = area_loc + carla.Location(x=point.x, y=point.y)
            area.append(point_location)

        # Get the waypoints of these points, removing duplicates
        ini_wps = []
        for pt in area:
            wpx = self._map.get_waypoint(pt)
            # As x_values are arranged in order, only the last one has to be checked
            if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[-1].lane_id != wpx.lane_id:
                ini_wps.append(wpx)

        # Advance them until the intersection
        wps = []
        for wpx in ini_wps:
            while not wpx.is_intersection:
                next_wp = wpx.next(0.5)[0]
                if next_wp and not next_wp.is_intersection:
                    wpx = next_wp
                else:
                    break
            wps.append(wpx)

        return area_loc, wps

    def is_vehicle_crossing_line(self, seg1, seg2):
        """
        check if vehicle crosses a line segment
        """
        line1 = shapely.geometry.LineString([(seg1[0].x, seg1[0].y), (seg1[1].x, seg1[1].y)])
        line2 = shapely.geometry.LineString([(seg2[0].x, seg2[0].y), (seg2[1].x, seg2[1].y)])
        inter = line1.intersection(line2)

        return not inter.is_empty

    def rotate_point(self, point, angle):
        """
        rotate a given point by a given angle
        """
        x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
        y_ = math.sin(math.radians(angle)) * point.x + math.cos(math.radians(angle)) * point.y
        return carla.Vector3D(x_, y_, point.z)

    def get_hour(self) -> int:
        """
        get hour information from sun_altitude_angle
        """
        return int((90 + self._world.get_weather().sun_altitude_angle) * 0.06666666)

    def compute_connection(self, current_waypoint, next_waypoint, threshold=35):
        """
        Compute the type of topological connection between an active waypoint (current_waypoint) and a target waypoint
        (next_waypoint).

        :param current_waypoint: active waypoint
        :param next_waypoint: target waypoint
        :return: the type of topological connection encoded as a RoadOption enum:
                RoadOption.STRAIGHT
                RoadOption.LEFT
                RoadOption.RIGHT
        """

        n = next_waypoint.transform.rotation.yaw
        n = n % 360.0

        c = current_waypoint.rotation.yaw
        c = c % 360.0

        diff_angle = (n - c) % 180.0
        if diff_angle < threshold or diff_angle > (180 - threshold):
            return "STRAIGHT"
        elif diff_angle > 90.0:
            return "LEFT"
        else:
            return "RIGHT"
