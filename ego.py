import carla
import queue
from scenario import *
from utils.helpers import *
import os
import pandas as pd
'''
All sensors are following SAE coordinate system
'''
class EGO(SCENARIO):
    def __init__(self, yaml_path='./configs/sensor_configs.yaml', log='./log', save=False):
        super().__init__()
        sensor_config = parse_config_yaml(yaml_path)
        
        # variable whether save or not
        self.save = save
        
        # For all sensor synchronization
        self.tick = sensor_config['all_sensor_tick'] 
        
        # Camera Parameters
        self.width = sensor_config['CAMERA']['width']
        self.height = sensor_config['CAMERA']['height']
        self.fov = sensor_config['CAMERA']['fov']
       
        # GNSS Parameters 
        self.earth_radius_major = sensor_config['GNSS']['earth_radius_major']
        self.drad = sensor_config['GNSS']['drad']
        self.is_first = True

        # Radar Parameters
        self.r_vfov = sensor_config['RADAR']['vfov']
        self.r_hfov = sensor_config['RADAR']['hfov']
        self.r_range = sensor_config['RADAR']['range']

        self.img_front = queue.Queue(maxsize=1)
        self.img_left = queue.Queue(maxsize=1)
        self.img_right = queue.Queue(maxsize=1)

        # Log system
        self.log_path = log
        self.imu_data = []
        self.gnss_data = []
        self.radar_data = []

    def spawn_ego(self, spawn_point=carla.Location(x=0.0, y=0.0, z=0.0)):
        ego_bp = self.bp.find("vehicle.lincoln.mkz_2017")
        
        self.ego = self.world.spawn_actor(ego_bp, np.random.choice(self.map.get_spawn_points())) # map point 바꿔야함
        self.ego.set_autopilot(True)

        self.actor_list.append(self.ego)

    def spawn_rgb_camera(self):
        cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x", str(self.width))
        cam_bp.set_attribute("image_size_y", str(self.height))
        cam_bp.set_attribute("fov", str(self.fov))
        cam_bp.set_attribute("sensor_tick", str(self.tick))

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
        
        cam1_ego.listen(self.img_front.put)
        cam2_ego.listen(self.img_left.put)
        cam3_ego.listen(self.img_right.put)

        self.actor_list.append(cam1_ego)
        self.actor_list.append(cam2_ego)
        self.actor_list.append(cam3_ego)

    def spawn_depth_camera(self):
        dcam_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        
        dcam_bp.set_attribute("image_size_x", str(self.width))
        dcam_bp.set_attribute("image_size_y", str(self.height))
        dcam_bp.set_attribute("fov", str(self.fov))
        dcam_bp.set_attribute("sensor_tick", str(self.tick))

        dcam_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
        dcam_ego = self.world.spawn_actor(dcam_bp, 
                                          dcam_transform, 
                                          attach_to=self.ego,
                                          attachment_type=carla.AttachmentType.Rigid)
        
        dcam_ego.listen(lambda image: image.save_to_disk('tutorial/new_depth_output/%.6d.jpg' % image.frame,carla.ColorConverter.LogarithmicDepth))

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
        img_front = self.img_front.get()
        img_left = self.img_left.get()
        img_right = self.img_right.get()

        img_front = np.reshape(np.copy(img_front.raw_data), (self.height, self.width, 4))
        img_left = np.reshape(np.copy(img_left.raw_data), (self.height, self.width, 4))
        img_right = np.reshape(np.copy(img_right.raw_data), (self.height, self.width, 4))

        all_img = np.concatenate((img_left, img_front, img_right), axis=1)
        
        if self.save:
            cv2.imwrite(f"{self.img_callback.counter:10d}.png", all_img)
        
        self.img_callback.counter += 1
        return all_img

    def depth_callback(self, camera):
        cc = camera.convert(carla.ColorConverter.LogarithmicDepth)
        img = np.copy(camera.raw_data)
        img = img.reshape(self.height, self.width, 4)
        img = img[:, :, :3]
    
    def imu_callback(self, imu):
        acc = imu.accelerometer
        gyro = imu.gyroscope
        
        if self.save:
            self.imu_data.append({
                'timestamp': self.world.get_snapshto().timestamp.elaped_seconds,
                'acc.x' : acc.x,
                'acc.y' : acc.y,
                'acc.z' : acc.z,
                'gyro.x' : gyro.x,
                'gyro.y' : gyro.y,
                'gyro.z' : gyro.z,
                
            })
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

        if self.save:
            self.gnss_data.append({
                'timestamp': self.world.get_snapshot().timestamp.elaped_seconds,
                'x' : x,
                'y' : y,
                'z' : z,
            })
            
        print(f"Location [m] x: {x:.6f}, y:{y:.6f}, z:{z:.6f}\n")
    
    def radar_callback(self, radar, save=False):
        current_rot = radar.transform.rotation
        for detect in radar:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)
            
            self.world.debug.draw_point(
                radar.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(255, 0, 0))

    def update_view(self):
        try:
            transform = self.ego.get_transform()
            self.spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-5.0, y=0.0, z=2.0)))

        except RuntimeError:
            raise KeyboardInterrupt
    
    def save_data(self):
        imu_df = pd.DataFrame(self.imu_data)
        self.imu_data.clear()
        gnss_df = pd.DataFrame(self.gnss_data)
        self.gnss_data.clear()

        with pd.ExcelWriter(os.path.join(self.log_path,'/sensor_data.xlsx')) as writer:
            imu_df.to_excel(writer,sheet_name='IMU')
            gnss_df.to_excel(writer,sheet_name='gnss')
        
        del imu_df
        del gnss_df


    def main(self):
        super().main()
        
        self.spawn_ego()
        self.spawn_rgb_camera()
        # self.spawn_depth_camera()
        self.spawn_imu()
        self.spawn_gnss()
        self.spawn_radar()

        while True:
            self.world.tick()
            self.update_view()
            self.img_callback()
    
    def __del__(self):
        super().__del__()
            
if __name__ == '__main__':
    try:
        ego = EGO(yaml_path="./configs/sensor_configs.yaml", save=True)
        ego.main()

    except KeyboardInterrupt:
        del ego
