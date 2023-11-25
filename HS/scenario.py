import cv2
import math
import random
import carla
import numpy as np
import open3d as o3d

from utils.helpers import *


class SCENARIO:
    
    def __init__(self):
        self.actor_list = []
        self.ego_list= []
        self.nv_list = []
        self.junction_list=[]
        self.client = carla.Client('localhost', 2000)
        self.world = self.client.load_world('Town04')
        self.spectator = self.world.get_spectator()
        self.map = self.world.get_map()
        self.bp = self.world.get_blueprint_library()
    
        # =============== Vehicle Parameters =============== #
        args = arg_parse()
        self.nv_num = 10

        # ===================================================== #
        


        # =============== Ego Sensor Parameters =============== #
        sensor_config = parse_config_yaml(args.config_path)
        
        self.tick = sensor_config['all_sensor_tick']
        
        # Camera Parameter
        self.width = sensor_config['CAMERA']['width']
        self.height = sensor_config['CAMERA']['height']
        self.fov = sensor_config['CAMERA']['fov']
        
        # GNSS Parameters 
        self.earth_radius_major = sensor_config['GNSS']['earth_radius_major']
        self.drad = sensor_config['GNSS']['drad']
        self.is_first = True

        ## Lidar Parameters
        self.lidar_ch = sensor_config['LiDAR']['channel']
        self.pps = sensor_config['LiDAR']['poinits_per_second']
        self.lidar_rotation = sensor_config['LiDAR']['rotation_frequency']
        # ===================================================== #
        
        print("Scenario start, Press Ctrl+C to stop the scenario")
    
    def gnss_spawn(self):
        gnss_bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        gnss_bp.set_attribute('sensor_tick', str(self.tick))
        
        gnss_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        ego_gnss = self.world.spawn_actor(gnss_bp, gnss_transform, attach_to=self.ego)
        ego_gnss.listen(lambda gnss: self.gnss_callback(gnss))
        
        self.actor_list.append(ego_gnss)

    def imu_spawn(self):
        imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', str(self.tick))
        
        imu_transform = carla.Transform(carla.Location(x=1.5, z=1.4))
        ego_imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.ego)
        ego_imu.listen(lambda imu: self.imu_callback(imu))
        
        self.actor_list.append(ego_imu)

    def camera_spawn(self):
        dcam_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        dcam_bp.set_attribute("image_size_x", str(self.width))
        dcam_bp.set_attribute("image_size_y", str(self.height))
        dcam_bp.set_attribute("fov", str(self.fov))
        dcam_bp.set_attribute("sensor_tick", str(self.tick))

        cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(self.width))
        cam_bp.set_attribute("image_size_y", str(self.height))
        cam_bp.set_attribute("fov", str(self.fov))
        cam_bp.set_attribute("sensor_tick", str(self.tick))

        cam_transform = carla.Transform(carla.Location(z=2))

        # ego_dcam = self.world.spawn_actor(dcam_bp, cam_transform, attach_to=self.ego)
        ego_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.ego)    
            
        # ego_dcam.listen(lambda depth_img: self.depth_callback(depth_img))
        ego_cam.listen(lambda rgb_img: self.rgb_callback(rgb_img))

        self.actor_list.append(ego_cam)
        # self.actor_list.append(ego_dcam)


    def rgb_callback(self, camera):
        img = np.copy(camera.raw_data)
        img = img.reshape(self.height, self.width, 4)
        img = img[:, :, :3]
        cv2.imshow('rgb', img)

    def depth_callback(self, camera):
        camera.convert(carla.ColorConverter.LogarithmicDepth)
        img = np.copy(camera.raw_data)
        img = img.reshape(self.height, self.width, 4)
        img = img[:, :, :3]

    def lidar_callback(self, lidar):
        point_list = o3d.geometry.PointCloud()
        pcd = np.copy(np.frombuffer(lidar.raw_data, dtype=np.dtype('f4')))
        pcd = np.reshape(pcd, (pcd.shape[0]//4, 4))
        
        intensity = pcd[:, -1]
        points = pcd[:, :-1]
        
        point_list.points = o3d.utility.Vector3dVector(points)
        
    def imu_callback(self, imu):
        acc = imu.accelerometer
        gyro = imu.gyroscope
        
        print(f"Acceleration [m/s^2] x: {acc.x:.6f} y: {acc.y:.6f}, z: {acc.z:.6f}\n")
        print(f"Angular rate [rad/s] x: {gyro.x:.6f} rad/sec, y: {gyro.y:.6f} rad/sec, z: {gyro.z:.6f} \n")

    def gnss_callback(self, gnss):
        lat = gnss.latitude
        lon = gnss.longitude
        alt = gnss.altitude

        radius = (self.earth_radius_major * self.earth_radius_major) / math.sqrt( \
            self.earth_radius_major * self.earth_radius_major * math.cos(lat * self.drad) * math.cos( \
                lat * self.drad) + self.earth_radius_major * self.earth_radius_major * math.sin(lat * self.drad) * math.sin(lat * self.drad)) + alt

        if self.is_first:
            self.INIT_LAT = 0.0
            self.INIT_LON = 0.0
            self.INIT_ALT = 0.0
            
            init_location = self.ego.get_location()
            x = init_location.x
            y = init_location.y
            z = init_location.z

            self.INIT_LAT = lat + y / (radius * self.drad)
            self.INIT_LON = lon - x / (radius * self.drad * math.cos(self.INIT_LAT * self.drad))
            self.INIT_ALT = alt - z
            
            self.is_first = False

        x = radius * (lon - self.INIT_LON) * self.drad * math.cos(self.INIT_LAT * self.drad)
        y = radius * ((-lat + self.INIT_LAT) * self.drad)
        z = alt - self.INIT_ALT

        print(f"Location [m] x: {x:.6f}, y:{y:.6f}, z:{z:.6f}\n")

    def radar_callback(radar):
        '''
        The sensor creates a conic view that is translated to a 2D point map of the elements in sight and their speed regarding the sensor. 
        '''
        points = np.frombuffer(radar.raw_data, dtype=np.dtype('f4'))
        
    def sensor_setup(self):
        args = arg_parse()
        sensor_config = parse_config_yaml(args.config_path)
        
        self.tick = sensor_config['all_sensor_tick']
        
        camera = sensor_config['CAMERA']
        self.width = camera['width']
        self.height = camera['height']
        self.fov = camera['fov']
        
        gnss = sensor_config['GNSS']
        self.earth_radius_major = gnss['earth_radius_major']
        self.drad = gnss['drad']
        
        self.camera_spawn()
        self.imu_spawn()
        self.gnss_spawn()
        
    def spawn_ego(self):
        ego_bp = self.bp.find('vehicle.lincoln.mkz_2017')
        self.ego = self.world.spawn_actor(ego_bp, np.random.choice(self.map.get_spawn_points()))
        self.actor_list.append(self.ego)
        self.ego_list.append(self.ego)
        self.sensor_setup()
        self.ego.set_autopilot(True)

    def spawn_nv(self,n):
        for i in range(len(n)):
            ego_nv = self.bp.find('vehicle.lincoln.mkz_2017')
            nv = self.world.spawn_actor(ego_nv, np.random.choice(self.map.get_spawn_points()))
            self.actor_list.append(nv)
            self.nv_list.append(nv)
            self.nv.set_autopilot(True)

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
        lidar_bp.set_attribute('channerls',int(self.lidar_ch))
        lidar_bp.set_attribute('channerls',int(self.pps))
        lidar_bp.set_attribute('channerls',int(self.lidar_rotation))
        
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
        self.spawn_ego()
        if self.nv_num != 0:
            self.spawn_nv(self.nv_num)
        #self.spawn_actor()
        # self.follow_path()

        while True:
            self.world.tick()
            self.update_view(self.actor_list[0])
            
            if cv2.waitKey(1) & 0xFF == 27:
                raise KeyboardInterrupt
                
            
    def __del__(self):
        self.set_world(synchronous=False)
        destroy_commands = [carla.command.DestroyActor(actor.id) for actor in self.actor_list]
        self.client.apply_batch(destroy_commands)
        
        print("Canceled by user...")

if __name__ == '__main__':
    try:
        scenario = SCENARIO()
        scenario.main()

    except KeyboardInterrupt:
        del scenario