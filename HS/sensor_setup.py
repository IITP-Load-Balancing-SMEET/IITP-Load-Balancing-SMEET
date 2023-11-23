import carla
import cv2
import numpy as np
import open3d as o3d
import math
from utils.helpers import * 

class SETUP:
    def __init__(self, world, ego):
        self.actor_list = []
        self.world = world
        self.ego = ego
        self.is_first = True
        
    def gnss_spawn(self, tick):
        gnss_bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        gnss_bp.set_attribute('sensor_tick', str(tick))
        
        gnss_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        ego_gnss = self.world.spawn_actor(gnss_bp, gnss_transform, attach_to=self.ego)
        ego_gnss.listen(lambda gnss: self.gnss_callback(gnss))
        self.actor_list.append(ego_gnss)

    def imu_spawn(self, tick):
        imu_bp = self.world.get_blueprint_library.find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', str(tick))
        
        imu_transform = carla.Transform(carla.Location(x=1.5, z=1.4))
        ego_imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.ego)
        ego_imu.listen(lambda imu: self.imu_callback(imu))
        self.actor_list.append(ego_imu)

    def camera_spawn(self, width, height, fov, tick):
        dcam_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        dcam_bp.set_attribute("image_size_x", str(width))
        dcam_bp.set_attribute("image_size_y", str(height))
        dcam_bp.set_attribute("fov", str(fov))
        dcam_bp.set_attribute("sensor_tick", str(tick))

        cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(width))
        cam_bp.set_attribute("image_size_y", str(height))
        cam_bp.set_attribute("fov", str(fov))
        cam_bp.set_attribute("sensor_tick", str(tick))

        cam_transform = carla.Transform(carla.Location(x=-0.1, z=1.7))

        ego_dcam = self.world.spawn_actor(dcam_bp, cam_transform, attach_to=self.ego)
        ego_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.ego)    
            
        ego_dcam.listen(self.camera_callback(type='Depth'))
        ego_cam.listen(self.camera_callback(type='RGB'))

        self.actor_list.append(ego_cam)
        self.actor_list.append(ego_dcam)


    def rgb_callback(self, camera):
        img = np.copy(camera.raw_data)
        img = img[:, :, :3]
        cv2.imshow('RGB', img)
        cv2.waitKey(1)

    def depth_callback(self, camera):
        camera.convert(carla.ColorConverter.LogarithmicDepth)
        img = np.copy(camera.raw_data)
        img = img[:, :, :3]
        cv2.imshow('Depth', img)
        cv2.waitKey(1)
    
    def lidar_callback(self, lidar):
        point_list = o3d.geometry.PointCloud()
        pcd = np.copy(np.frombuffer(lidar.raw_data, dtype=np.dtype('f4')))
        pcd = np.reshape(pcd, (pcd.shape[0]//4, 4))
        
        intensity = pcd[:, -1]
        points = pcd[:, :-1]
        
        point_list.points = o3d.utility.Vector3dVector(points)
        
    def imu_callback(self, imu):
        acc = imu.acclerometer
        gyro = imu.gyroscope
        
        print(f"Acceleration | x: {acc.x} m/s^2, y: {acc.y} m/s^2, z: {acc.z} m/s^2")
        print(f"Angular rate | x: {gyro.x} rad/sec, y: {gyro.y} rad/sec, z: {gyro.z}")

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

        print(f"Location | x: {x} m, y:{y} m, z:{z} m")

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
        self.img_width = camera['width']
        self.img_height = camera['height']
        self.img_fov = camera['fov']
        
        gnss = sensor_config['GNSS']
        self.earth_radius_major = gnss['self.EARTH_RADIUS_MAJOR']
        self.drad = gnss['self.drad']
        
        self.camera_spawn()
        self.imu_spawn()
        self.gnss_spawn()