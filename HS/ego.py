import carla
import cv2
import numpy as np
import open3d as o3d
import math

EARTH_RADIUS_MAJOR: 6378137.0
EARTH_RADIUS_MINOR: 6356752.3142
DRAD: 0.01745329251994329576923690768489

class Ego:
    def __init__(self, world, vehicle):
        self.actor_list = []
        self.world = world
        self.vehicle = vehicle
        self.is_first = True
        self.sensor_tick = 0.1 # 0.1 sec = 10Hz
        
    def gnss_spawn(self):
        gnss_bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        gnss_bp.set_attribute('sensor_tick', str(self.sensor_tick))
        
        gnss_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        ego_gnss = self.world.spawn_actor(gnss_bp, gnss_transform, attach_to=self.vehicle)
        ego_gnss.listen(lambda gnss: self.gnss_callback(gnss))
        self.actor_list.append(ego_gnss)

    def imu_spawn(self):
        imu_bp = self.world.get_blueprint_library.find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', str(self.sensor_tick))
        
        imu_transform = carla.Transform(carla.Location(x=1.5, z=1.4))
        ego_imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
        ego_imu.listen(lambda imu: self.imu_callback(imu))
        self.actor_list.append(ego_imu)

    def camera_spawn(self):
        dcam_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        dcam_bp.set_attribute("image_size_x", str(640))
        dcam_bp.set_attribute("image_size_y", str(480))
        dcam_bp.set_attribute("fov", str(110))
        dcam_bp.set_attribute("sensor_tick", str(self.sensor_tick))

        cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(640))
        cam_bp.set_attribute("image_size_y", str(480))
        cam_bp.set_attribute("fov", str(110))
        cam_bp.set_attribute("sensor_tick", str(self.sensor_tick))

        cam_transform = carla.Transform(carla.Location(x=-0.1, z=1.7))

        ego_dcam = self.world.spawn_actor(dcam_bp, cam_transform, attach_to=self.vehicle)
        ego_cam = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.vehicle)    
            
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

    def gnss_callback(self, gnss, vehicle):
        lat = gnss.latitude
        lon = gnss.longitude
        alt = gnss.altitude

        radius = (EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR) / math.sqrt( \
            EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR * math.cos(lat * DRAD) * math.cos( \
                lat * DRAD) + EARTH_RADIUS_MAJOR * EARTH_RADIUS_MAJOR * math.sin(lat * DRAD) * math.sin(lat * DRAD)) + alt

        if self.is_first:
            global INIT_LAT
            global INIT_LON
            global INIT_ALT
            init_location = self.vehicle.get_location()
            x = init_location.x
            y = init_location.y
            z = init_location.z

            INIT_LAT = lat + y / (radius * DRAD)
            INIT_LON = lon - x / (radius * DRAD * math.cos(INIT_LAT * DRAD))
            INIT_ALT = alt - z
            
            self.is_first = False

        x = radius * (lon - INIT_LON) * DRAD * math.cos(INIT_LAT * DRAD)
        y = radius * ((-lat + INIT_LAT) * DRAD)
        z = alt - INIT_ALT

        print(f"Location | x: {x} m, y:{y} m, z:{z} m")

    def radar_callback(radar):
        '''
        The sensor creates a conic view that is translated to a 2D point map of the elements in sight and their speed regarding the sensor. 
        '''
        points = np.frombuffer(radar.raw_data, dtype=np.dtype('f4'))


