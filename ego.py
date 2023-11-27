import os
import cv2
import math
import queue
import carla
import pandas as pd

from scenario import *
from utils.helpers import *

class EGO(SCENARIO):
    def __init__(self):
        super().__init__()
        
        # For all sensor synchronization
        self.tick = self.sensor_config['all_sensor_tick'] 
        
        # Camera Parameters
        self.width = self.sensor_config['Camera']['width']
        self.height = self.sensor_config['Camera']['height']
        self.fov = self.sensor_config['Camera']['fov']
       
        # GNSS Parameters 
        self.earth_radius_major = self.sensor_config['GNSS']['earth_radius_major']
        self.drad = self.sensor_config['GNSS']['drad']
        self.is_first = True

        # Radar Parameters
        self.r_vfov = self.sensor_config['Radar']['vfov']
        self.r_hfov = self.sensor_config['Radar']['hfov']
        self.r_range = self.sensor_config['Radar']['range']

        self.img_front = queue.Queue(maxsize=1)
        self.img_left = queue.Queue(maxsize=1)
        self.img_right = queue.Queue(maxsize=1)
        self.depth_img = queue.Queue(maxsize=1)

        # Log system
        self.imu_data = []
        self.gnss_data = []
        self.radar_data = []

    def spawn_ego(self, spawn_point='random'):
        # choice = {'merge1': self.map.get_waypoint_xodr(road_id=1097, lane_id=2, s=62.54).transform,
        #           'merge2': self.map.get_waypoint_xodr(road_id=1194, lane_id=2, s=53.86).transform,
        #           'merge3': self.map.get_waypoint_xodr(road_id=1080, lane_id=2, s=75.18).transform,
        #           'merge4': self.map.get_waypoint_xodr(road_id=779, lane_id=2, s=50.61).transform,
        #           'split1': self.map.get_waypoint_xodr(road_id=39, lane_id=-4, s=103.35).transform,
        #           'split2': self.map.get_waypoint_xodr(road_id=39, lane_id=6, s=26.95).transform,
        #           'split3': self.map.get_waypoint_xodr(road_id=47, lane_id=-4, s=86.03).transform,
        #           'split4': self.map.get_waypoint_xodr(road_id=1076, lane_id=2, s=66.25).transform,
        #           'junction1':self.map.get_waypoint_xodr(road_id=0.0, lane_id=0.0, s=0.0).transform,
        #           'junction2': self.map.get_waypoint_xodr(road_id=0.0, lane_id=0.0, s=0.0).transform,
        #           'junction3': self.map.get_waypoint_xodr(road_id=0.0, lane_id=0.0, s=0.0).transform,
        #           'junction4': self.map.get_waypoint_xodr(road_id=0.0, lane_id=0.0, s=0.0).transform,
        #           'junction5': self.map.get_waypoint_xodr(road_id=0.0, lane_id=0.0, s=0.0).transform,
        #           'random': np.random.choice(self.map.get_spawn_points())}
    
        ego_bp = self.bp.find("vehicle.lincoln.mkz_2017")
        
        #self.ego = self.world.spawn_actor(ego_bp, choice[spawn_point])
        self.ego = self.world.spawn_actor(ego_bp, np.random.choice(self.map.get_spawn_points()))
        
        self.ego.set_autopilot(True)

        self.actor_list.append(self.ego)

    def spawn_camera(self):
        cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        dcam_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        
        cam_bp.set_attribute("image_size_x", str(self.width))
        cam_bp.set_attribute("image_size_y", str(self.height))
        cam_bp.set_attribute("fov", str(self.fov))
        cam_bp.set_attribute("sensor_tick", str(self.tick))
        
        dcam_bp.set_attribute("image_size_x", str(self.width * 3))
        dcam_bp.set_attribute("image_size_y", str(self.height))
        dcam_bp.set_attribute("fov", str(self.fov))
        dcam_bp.set_attribute("sensor_tick", str(self.tick))

        cam1_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
        cam2_transform = carla.Transform(carla.Location(x=1.5, y=-2.0, z=2.0), carla.Rotation(yaw=-30.0))
        cam3_transform = carla.Transform(carla.Location(x=1.5, y=2.0, z=2.0), carla.Rotation(yaw=30.0))
        
        # front
        cam1_ego = self.world.spawn_actor(cam_bp, 
                                         cam1_transform, 
                                         attach_to=self.ego,
                                         attachment_type=carla.AttachmentType.Rigid)
        # left
        cam2_ego = self.world.spawn_actor(cam_bp, 
                                         cam2_transform, 
                                         attach_to=self.ego,
                                         attachment_type=carla.AttachmentType.Rigid)
        # right
        cam3_ego = self.world.spawn_actor(cam_bp, 
                                         cam3_transform, 
                                         attach_to=self.ego,
                                         attachment_type=carla.AttachmentType.Rigid) 
        # front depth
        dcam_ego = self.world.spawn_actor(dcam_bp, 
                                          cam1_transform, 
                                          attach_to=self.ego,
                                          attachment_type=carla.AttachmentType.Rigid)
        
        cam1_ego.listen(self.img_front.put)
        cam2_ego.listen(self.img_left.put)
        cam3_ego.listen(self.img_right.put)
        dcam_ego.listen(self.depth_img.put)

        self.actor_list.append(cam1_ego)
        self.actor_list.append(cam2_ego)
        self.actor_list.append(cam3_ego)
        self.actor_list.append(dcam_ego)

    def spawn_imu(self):
        imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
        
        imu_bp.set_attribute('sensor_tick', str(self.tick))
        
        imu_transform = carla.Transform(carla.Location(x=0.0, z=1.4))
        imu_ego = self.world.spawn_actor(imu_bp, 
                                         imu_transform, 
                                         attach_to=self.ego,
                                         attachment_type=carla.AttachmentType.Rigid)
        
        imu_ego.listen(self.imu_callback)

        self.actor_list.append(imu_ego)

    def spawn_gnss(self):
        gnss_bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        
        gnss_bp.set_attribute('sensor_tick', str(self.tick))
        
        gnss_transform = carla.Transform(carla.Location(x=0.0, z=1.4))
        gnss_ego = self.world.spawn_actor(gnss_bp, 
                                          gnss_transform, 
                                          attach_to=self.ego,
                                          attachment_type=carla.AttachmentType.Rigid)
        
        gnss_ego.listen(self.gnss_callback)
        
        self.actor_list.append(gnss_ego)

    def spawn_radar(self):
        radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
        radar_bp.set_attribute('sensor_tick', str(self.tick))
        radar_bp.set_attribute('horizontal_fov', str(self.r_hfov))
        radar_bp.set_attribute('vertical_fov', str(self.r_vfov))
        radar_bp.set_attribute('range', str(self.r_range))

        radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
        
        radar_ego = self.world.spawn_actor(radar_bp,
                                           radar_transform,
                                           attach_to=self.ego, 
                                           attachment_type=carla.AttachmentType.Rigid)

        radar_ego.listen(self.radar_callback)

        self.actor_list.append(radar_ego)

    def img_callback(self):
        img_front = None
        img_left = None
        img_right = None
        
        depth_img = None
    
        while not self.img_front.empty() and img_front is None:
            img_front = self.img_front.get_nowait()

        while not self.img_left.empty() and img_left is None:
            img_left = self.img_left.get_nowait()

        while not self.img_right.empty() and img_right is None:
            img_right = self.img_right.get_nowait()
            
        while not self.depth_img.empty() and depth_img is None:
            depth_img = self.depth_img.get_nowait()

        if (img_front is not None) and (img_left is not None) and (img_right is not None) and (depth_img is not None):
            img_front = np.reshape(np.copy(img_front.raw_data), (self.height, self.width, 4))
            img_left = np.reshape(np.copy(img_left.raw_data), (self.height, self.width, 4))
            img_right = np.reshape(np.copy(img_right.raw_data), (self.height, self.width, 4))
            
            depth_img = np.reshape(np.copy(depth_img.raw_data), (self.height, self.width*3, 4))
            depth_img = (depth_img / np.max(depth_img) * 255).astype(np.uint8)
            
            rgb_img = np.concatenate((img_left, img_front, img_right), axis=1)
            all_img = np.concatenate((rgb_img, depth_img), axis=0)
            
            if self.show:
                cv2.imshow('images', all_img)
                
            if self.save:
                cv2.imwrite(self.dataset_path + f"/images/depth/{int(self.world.get_snapshot().timestamp.elapsed_seconds):010d}.png", depth_img)
                cv2.imwrite(self.dataset_path + f"/images/rgb/{int(self.world.get_snapshot().timestamp.elapsed_seconds):010d}.png", rgb_img)
    
    def imu_callback(self, imu):
        acc = imu.accelerometer
        gyro = imu.gyroscope
        
        print(f"Acceleration [m/s^2] x: {acc.x:.6f} y: {acc.y:.6f}, z: {acc.z:.6f}\n")
        print(f"Angular rate [rad/s] x: {gyro.x:.6f} rad/sec, y: {gyro.y:.6f} rad/sec, z: {gyro.z:.6f} \n")
        
        if self.save:
            self.imu_data.append({'timestamp': imu.timestamp,
                                  'ax' : acc.x,
                                  'ay' : acc.y,
                                  'az' : acc.z,
                                  'wx' : gyro.x,
                                  'wy' : gyro.y,
                                  'wz' : gyro.z})
            
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
        
        if self.save:
            self.gnss_data.append({'timestamp': gnss.timestamp,
                                   'x' : x,
                                   'y' : y,
                                   'z' : z})
    
    def radar_callback(self, radar):
        current_rot = radar.transform.rotation
        
        for detect in radar:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(carla.Location(),
                            carla.Rotation(pitch=current_rot.pitch + alt,
                                           yaw=current_rot.yaw + azi,
                                           roll=current_rot.roll)).transform(fw_vec)
            
            self.world.debug.draw_point(
                radar.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(255, 0, 0))
            
            if self.save:
                self.radar_data.append({'timestamp': self.world.get_snapshot().timestamp.elapsed_seconds,
                                        'x' : fw_vec.x,
                                        'y' : fw_vec.y,
                                        'z' : fw_vec.z})

    def update_view(self):
        try:
            transform = self.ego.get_transform()
            self.spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-5.0, y=0.0, z=2.0)))

        except RuntimeError:
            raise KeyboardInterrupt
    
    def save_data(self):
        imu_df = pd.DataFrame(self.imu_data)
        gnss_df = pd.DataFrame(self.gnss_data)
        radar_df = pd.DataFrame(self.radar_data)
        
        excel_path = os.path.join(self.dataset_path, 'others', 'sensor_data.xlsx')
        
        with pd.ExcelWriter(excel_path, engine='xlsxwriter') as writer:
            imu_df.to_excel(writer, sheet_name='IMU')
            gnss_df.to_excel(writer, sheet_name='GNSS')
            radar_df.to_excel(writer, sheet_name='RADAR')

    def main(self):
        super().main()
        
        self.spawn_ego()
        self.spawn_camera()
        self.spawn_imu()
        self.spawn_gnss()
        self.spawn_radar()
        
        self.set_traffic_manger()
        
        while True:
            self.world.tick()
            self.update_view()
            self.img_callback()
            
            if self.show:
                if cv2.waitKey(1) & 0xff == ord('q'):
                    raise KeyboardInterrupt

    
    def cleanup(self): # __del__ is not working
        if self.save:
            self.save_data()
        
        self.set_world(synchronous=False)
        destroy_commands1 = [carla.command.DestroyActor(actor.id) for actor in self.actor_list]
        destroy_commands2 = [carla.command.DestroyActor(actor.id) for actor in self.junction_list]
        
        self.client.apply_batch(destroy_commands1)
        self.client.apply_batch(destroy_commands2)
            
if __name__ == '__main__':
    try:
        ego = EGO()
        ego.main()

    except KeyboardInterrupt:
        print('Canceld by user...')
        
    finally:
        ego.cleanup()
        del ego
