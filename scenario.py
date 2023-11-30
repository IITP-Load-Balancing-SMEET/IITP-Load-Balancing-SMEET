import os
import random
import carla
import numpy as np
from utils.helpers import *


class SCENARIO:
    def __init__(self, yaml_path="./configs"):
        self.junction_list = []
        self.actor_list = []

        self.client = carla.Client("localhost", 2000)
        self.world = self.client.load_world("Town04")
        self.traffic_manager = self.client.get_trafficmanager()
        self.spectator = self.world.get_spectator()
        self.map = self.world.get_map()
        self.bp = self.world.get_blueprint_library()

        self.sensor_config = parse_config_yaml(
            os.path.join(yaml_path, "sensor_configs.yaml")
        )
        self.custom_config = parse_config_yaml(
            os.path.join(yaml_path, "customize.yaml")
        )

        # =============== Dataset Parameters =============== #
        self.dataset_path = self.custom_config["Dataset"]["path"]
        self.save = self.custom_config["Dataset"]["save"]
        self.show = self.custom_config["Dataset"]["show"]

        # =============== Scenario Parameters =============== #
        self.type = self.custom_config["Scenario"]["type"]
        self.weather_key = self.custom_config["Scenario"]["weather_key"]
        self.max_vehicles = self.custom_config["Scenario"]["max_vehicles"]
        self.junction_lidar = self.custom_config["Scenario"]["junction_lidar"]

        self.num_nv = self.custom_config["NV"]["num_nv"]
        self.aggressive_car_percenatge = self.custom_config["NV"][
            "aggressive_car_percentage"
        ]

        print("Scenario start, Press Ctrl+C to stop the scenario")

    def lidar_callback(self, lidar):
        timestamp = lidar.timestamp
        file_name = str(timestamp) + "_" + ".ply"
        lidar.save_to_disk(os.path.join(self.dataset_path, "LiDAR", str(self.junction_id), file_name))

    def spawn_nv(self, n, per):
        agg_num = int(n * per)
        ego_nv = self.bp.find("vehicle.lincoln.mkz_2017")

        for i in range(n):
            if i < agg_num:
                nv = self.world.spawn_actor(
                    ego_nv, np.random.choice(self.map.get_spawn_points())
                )
                nv.set_autopilot(True)
                self.traffic_manager.vehicle_percentage_speed_difference(nv, -20.0)
                self.traffic_manager.auto_lane_change(True)
                self.traffic_manager.ignore_lights_percentage(30)
                self.traffic_manager.ignore_vehicles_percentage(30)
                self.actor_list.append(nv)

            else:
                nv = self.world.spawn_actor(
                    ego_nv, np.random.choice(self.map.get_spawn_points())
                )
                nv.set_autopilot(True)
                self.traffic_manager.vehicle_percentage_speed_difference(nv, 80.0)
                self.traffic_manager.auto_lane_change(True)
                self.traffic_manager.ignore_lights_percentage(0)
                self.traffic_manager.ignore_vehicles_percentage(0)
                self.actor_list.append(nv)

    def set_world(self, synchronous=True):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

    def set_weatehr(self, key: int = 0):
        """
        0 - Default        1 - ClearNoon         2 - CloudyNoon\\
        3 - WetNoon        4 - WetCloudyNoon     5 - MidRainyNoon\\
        6 - HardRainNoon   7 - SoftRainNoon      8 - ClearSunset\\
        9 - CloudySunset   10 - WetSunset        11 - WetCloudySunset\\
        12 - MidRainSunset 1p3 - HardRainSunset   14 - SoftRainSunset\\
        """
        weather = {
            0: carla.WeatherParameters.Default,
            1: carla.WeatherParameters.ClearNoon,
            2: carla.WeatherParameters.CloudyNoon,
            3: carla.WeatherParameters.WetNoon,
            4: carla.WeatherParameters.WetCloudyNoon,
            5: carla.WeatherParameters.MidRainyNoon,
            6: carla.WeatherParameters.HardRainNoon,
            7: carla.WeatherParameters.SoftRainNoon,
            8: carla.WeatherParameters.ClearSunset,
            9: carla.WeatherParameters.CloudySunset,
            10: carla.WeatherParameters.WetSunset,
            11: carla.WeatherParameters.WetCloudySunset,
            12: carla.WeatherParameters.MidRainSunset,
            13: carla.WeatherParameters.HardRainSunset,
            14: carla.WeatherParameters.SoftRainSunset,
        }

        self.world.set_weather(weather[key])

    def set_traffic_manger(self, synchronous=True):
        self.traffic_manager.set_synchronous_mode(True)

        spawn_points = self.world.get_map().get_spawn_points()
        max_vehicles = min([self.max_vehicles, len(spawn_points)])

        platoon_spawn_points = {
            "merge1": self.map.get_waypoint_xodr(road_id=38, lane_id=-4, s=51.0).transform,
            "merge2": self.map.get_waypoint_xodr(road_id=40, lane_id=6, s=190.0).transform,
            "split1": self.map.get_waypoint_xodr(road_id=39, lane_id=6, s=40.0).transform,
            "split2": self.map.get_waypoint_xodr(road_id=39, lane_id=-4, s=80.0).transform
        }
        
        if self.type in platoon_spawn_points:
            platoon_spawn_point = platoon_spawn_points[self.type]
        
            if self.type == 'merge1' or self.type == 'merge2':
                platoon_spawn_point.location.z += 4.0
            
            elif self.type == 'split1' or self.type == 'split2':
                platoon_spawn_point.location.z += 11.0

            for _ in range(self.num_nv):
                vehicle_bp = random.choice(self.bp.filter("vehicle.bmw.grandtourer"))
                temp_vehicle = self.world.try_spawn_actor(vehicle_bp, platoon_spawn_point)

                if temp_vehicle is not None:
                    platoon_spawn_point.location.x -= 5.0
                    self.traffic_manager.auto_lane_change(temp_vehicle, False)
                    self.traffic_manager.vehicle_percentage_speed_difference(temp_vehicle, -20.0)
                    self.actor_list.append(temp_vehicle)

        for _, spawn_point in enumerate(random.sample(spawn_points, max_vehicles)):
            vehicle_bp = random.choice(self.bp.filter("vehicle.*.*"))
            temp_vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

            if temp_vehicle is not None:
                self.traffic_manager.ignore_lights_percentage(temp_vehicle, self.aggressive_car_percenatge)
                self.traffic_manager.ignore_vehicles_percentage(temp_vehicle, self.aggressive_car_percenatge)
                self.traffic_manager.vehicle_percentage_speed_difference(temp_vehicle, -20.0)
                self.actor_list.append(temp_vehicle)

        [actor.set_autopilot(True) for actor in self.actor_list[::-1]]

    def junction_lidar_spawn(self):
        """
        jucction 1 = x: 312 y: 170
        junction 2 = x: 312 y: 247
        junction 3 = x: 257 y: 170
        junction 4 = x: 257 y: 247
        junction 5 = x: 202 y: 170
        junction 6 = x: 202 y: 247
        """
        
        lidar_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
        lidar_bp.set_attribute("channels", str(self.sensor_config["Junction_LiDAR"]["channel"]))
        lidar_bp.set_attribute("points_per_second", str(self.sensor_config["Junction_LiDAR"]["points_per_second"]))
        lidar_bp.set_attribute("rotation_frequency", str(self.sensor_config["Junction_LiDAR"]["rotation_frequency"]))

        junction_lidar_1 = self.world.spawn_actor(
            lidar_bp, carla.Transform(carla.Location(x=312, y=170, z=3))
        )
        junction_lidar_2 = self.world.spawn_actor(
            lidar_bp, carla.Transform(carla.Location(x=312, y=247, z=3))
        )
        junction_lidar_3 = self.world.spawn_actor(
            lidar_bp, carla.Transform(carla.Location(x=257, y=170, z=3))
        )
        junction_lidar_4 = self.world.spawn_actor(
            lidar_bp, carla.Transform(carla.Location(x=257, y=247, z=3))
        )
        junction_lidar_5 = self.world.spawn_actor(
            lidar_bp, carla.Transform(carla.Location(x=202, y=170, z=3))
        )
        junction_lidar_6 = self.world.spawn_actor(
            lidar_bp, carla.Transform(carla.Location(x=202, y=247, z=3))
        )

        self.junction_list.append(junction_lidar_1)
        self.junction_list.append(junction_lidar_2)
        self.junction_list.append(junction_lidar_3)
        self.junction_list.append(junction_lidar_4)
        self.junction_list.append(junction_lidar_5)
        self.junction_list.append(junction_lidar_6)

        for i in range(1, 7):
            self.junction_id = 1
            self.junction_list[i].listen(self.lidar_callback)

    def main(self, synchronous=True):
        self.set_world(synchronous)
        self.set_weatehr(key=self.weather_key)
        self.set_traffic_manger(synchronous)

        # if self.num_nv != 0:
        #     self.spawn_nv(self.num_nv,self.aggressive_car_percenatge)

        if self.junction_lidar is True:
            self.junction_lidar_spawn()
