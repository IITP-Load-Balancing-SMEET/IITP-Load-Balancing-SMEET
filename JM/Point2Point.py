import os
import numpy
import sys
import carla
import glob
import sys
import copy

try:
    sys.path.append(glob.glob('/home/smeet/Desktop/Carla/Carla/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

class Test:
    def __init__(self,HOST,PORT,TIMEOUT):
        self.client = carla.Client(HOST,PORT)
        self.client.set_timeout(TIMEOUT)
        print('Carla_setting')
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.actor_list = [] 

    def run(self): 
        
        # World Setting
        self.client.load_world('Scenario1_test')
        car = self.blueprint_library.filter('model3')[0]
        map= self.world.get_map()
        
        
        way_point1= map.get_waypoint_xodr(road_id=7,lane_id=-1,s=10).transform.location
        way_point2= map.get_waypoint_xodr(road_id=1,lane_id=1,s=0).transform.location
        route=[way_point1,way_point2]
        
        # Traffic Manager Setting        
        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        
        # Spawn Vehicle
        Spawn_point = map.get_waypoint_xodr(7,-1,s=10).transform
        Spawn_point.location.z = Spawn_point.location.z + 3
        vehicle = self.world.spawn_actor(car, Spawn_point)
        self.actor_list.append(vehicle)
        

        traffic_manager.update_vehicle_lights(vehicle, True)
        traffic_manager.random_left_lanechange_percentage(vehicle, 0)
        traffic_manager.random_right_lanechange_percentage(vehicle, 0)
        traffic_manager.auto_lane_change(vehicle, False)
        traffic_manager.ignore_lights_percentage(vehicle,0)

        
        # Autopilot Setting
        vehicle.set_autopilot(True)
        alt=False
        traffic_manager.set_path(vehicle,route)
        self.world.tick()


        while True: 
             self.world.tick()
             traffic_manager.set_path(vehicle,route)
            





def main():
    Te = Test('localhost',2000,10.0)
    Te.run()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
