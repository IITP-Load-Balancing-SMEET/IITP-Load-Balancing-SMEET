import cv2
import math
import random
import carla
import numpy as np
import open3d as o3d
import os
from utils.helpers import *

class SCENARIO:
    def __init__(self, yaml_path='./configs/sensor_configs.yaml'):
        self.junction_list=[]
        self.actor_list = []

        self.client = carla.Client('localhost', 2000)
        self.world = self.client.load_world('Town04')
        self.spectator = self.world.get_spectator()
        self.map = self.world.get_map()
        self.bp = self.world.get_blueprint_library()
        

        self.sensor_config = parse_config_yaml(yaml_path)


        # =============== Vehicle Parameters =============== #
        self.nv_num = 10
        
        print("Scenario start, Press Ctrl+C to stop the scenario")

    def lidar_callback(self, lidar):
        point_list = o3d.geometry.PointCloud()
        pcd = np.copy(np.frombuffer(lidar.raw_data, dtype=np.dtype('f4')))
        pcd = np.reshape(pcd, (pcd.shape[0]//4, 4))
        
        intensity = pcd[:, -1]
        points = pcd[:, :-1]
        
        point_list.points = o3d.utility.Vector3dVector(points)

    def spawn_nv(self, n):
        for i in range(n):
            ego_nv = self.bp.find('vehicle.lincoln.mkz_2017')
            nv = self.world.spawn_actor(ego_nv, np.random.choice(self.map.get_spawn_points()))
            nv.set_autopilot(True)

            self.actor_list.append(nv)

    def set_world(self, synchronous=True):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

    def set_weatehr(self, key:int=0):
        '''
        0 - Default        1 - ClearNoon         2 - CloudyNoon\\
        3 - WetNoon        4 - WetCloudyNoon     5 - MidRainyNoon\\
        6 - HardRainNoon   7 - SoftRainNoon      8 - ClearSunset\\
        9 - CloudySunset   10 - WetSunset        11 - WetCloudySunset\\
        12 - MidRainSunset 13 - HardRainSunset   14 - SoftRainSunset\\
        '''
        weather = {0: carla.WeatherParameters.Default, 1: carla.WeatherParameters.ClearNoon, 2: carla.WeatherParameters.CloudyNoon,
                   3: carla.WeatherParameters.WetNoon, 4: carla.WeatherParameters.WetCloudyNoon, 5: carla.WeatherParameters.MidRainyNoon,
                   6: carla.WeatherParameters.HardRainNoon, 7: carla.WeatherParameters.SoftRainNoon, 8: carla.WeatherParameters.ClearSunset,
                   9: carla.WeatherParameters.CloudySunset, 10: carla.WeatherParameters.WetSunset, 11:carla.WeatherParameters.WetCloudySunset,
                   12:carla.WeatherParameters.MidRainSunset, 13: carla.WeatherParameters.HardRainSunset, 14: carla.WeatherParameters.SoftRainSunset}
        
        self.world.set_weather(weather[key])

    def set_traffic_manger(self, synchronous=True):
        self.traffic_manager = self.client.get_trafficmanager()
        self.traffic_manager.set_synchronous_mode(synchronous)

    def junction_lidar_spawn(self):
        '''
            jucction 1 = x: 312 y: 170
            junction 2 = x: 312 y: 247
            junction 3 = x: 257 y: 170
            junction 4 = x: 257 y: 247
            junction 5 = x: 202 y: 170
            junction 6 = x: 202 y: 247
        '''
        lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channerls',int(self.sensor_config['channel']))
        lidar_bp.set_attribute('channerls',int(self.sensor_config['points_per_second']))
        lidar_bp.set_attribute('channerls',int(self.sensor_config['rotation_frequency']))
        
        junction_lidar_1 = self.world.spawn_actor(lidar_bp,carla.Transform(carla.Location(x=312, y=170, z=3)))
        junction_lidar_2 = self.world.spawn_actor(lidar_bp,carla.Transform(carla.Location(x=312, y=247, z=3)))
        junction_lidar_3 = self.world.spawn_actor(lidar_bp,carla.Transform(carla.Location(x=257, y=170, z=3)))
        junction_lidar_4 = self.world.spawn_actor(lidar_bp,carla.Transform(carla.Location(x=257, y=247, z=3)))
        junction_lidar_5 = self.world.spawn_actor(lidar_bp,carla.Transform(carla.Location(x=202, y=170, z=3)))
        junction_lidar_6 = self.world.spawn_actor(lidar_bp,carla.Transform(carla.Location(x=202, y=247, z=3)))
        self.junction_list.append(junction_lidar_1,junction_lidar_2,junction_lidar_3,junction_lidar_4,junction_lidar_5,junction_lidar_6)





    def main(self, synchronous=True):
        self.set_world(synchronous)
        self.set_weatehr(key=0)
        self.set_traffic_manger(synchronous)

        if self.nv_num != 0:
            self.spawn_nv(self.nv_num) 

        self.junction_lidar_spawn()