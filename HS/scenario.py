import sys
import math
import random
import carla
from utils.helpers import *
from ego import *

try:
    sys.path.append('/home/smeet/carla/PythonAPI/carla/')
    from agents.navigation.global_route_planner import GlobalRoutePlanner

except IndexError:
    raise("GlobalRoutePlanner not found")


class SCENARIO:
    def __init__(self):
        self.actor_list = []
        self.vehicle_dict = {}
        self.vehicle_catalogue = ['vehicle.audi.etron', 'vehicle.bmw.grandtourer', 'vehicle.citroen.c3', 
                                  'vehicle.audi.tt', 'vehicle.lincoln.mkz_2017', 'vehicle.mercedes.coupe', 
                                  'vehicle.mini.cooper_s_2021', 'vehicle.tesla.model3', 'vehicle.citroen.c3']

        self.client = carla.Client('localhost', 2000)
        self.world = self.client.load_world('Town04')
        self.spectator = self.world.get_spectator()
        self.map = self.world.get_map()
        self.bp = self.world.get_blueprint_library()

        print("Scenario start, Press Ctrl+C to stop the scenario")

    def set_world(self, synchronous=True):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

    def set_weatehr(self, key=0):
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
        
    def spawn_ego(self):
        ego_bp = self.bp.find('vehicle.lincoln.mkz_2017')
        ego_bp = self.world.spawn_actor(ego_bp, np.random.choice(self.map.get_spawn_points()))
        ego_bp.set_autopilot(True)
        self.actor_list.append(ego_bp)
        
        self.ego = Ego(self.world, ego_bp)
        self.ego.sensor_setup()
        

    def spawn_actor(self):
        for entity_ref in self.vehicle_dict.keys():
            selected_vehicle_bp = random.choice(self.vehicle_catalogue)

            property = self.vehicle_dict[entity_ref]

            start, end = routes[0], routes[1]
            start.location.z += 4.0
            end.location.z += 4.0

            vehicle_bp = self.bp.find(selected_vehicle_bp)
            vehicle_bp = self.world.spawn_actor(
                vehicle_bp, start)  # spawn actor

            self.actor_list.append(vehicle_bp)
            self.actor_dict[vehicle_bp.id] = property

            self.traffic_manager.auto_lane_change(vehicle_bp, False)


    def set_traffic_manger(self, synchronous=True):
        self.traffic_manager = self.client.get_trafficmanager()
        self.traffic_manager.set_synchronous_mode(synchronous)

        for entity_ref in self.vehicle_dict.keys():
            selected_vehicle_bp = random.choice(self.vehicle_catalogue)

            property = self.vehicle_dict[entity_ref]
            performance = property[0]  # [max speed]
            routes = property[1]  # [route1, route2]

            start, end = routes[0], routes[1]
            start.location.z += 4.0
            end.location.z += 4.0

            vehicle_bp = self.bp.find(selected_vehicle_bp)
            vehicle_bp = self.world.spawn_actor(
                vehicle_bp, start)  # spawn actor

            self.actor_list.append(vehicle_bp)
            self.actor_dict[vehicle_bp.id] = property

            self.traffic_manager.auto_lane_change(vehicle_bp, False)

    def generate_path(self, actor: carla.Actor):
        routes = self.actor_dict[actor.id][1]
        routes = self.grp.trace_route(routes[0].location, routes[1].location)

        return routes

    def draw_path(self, routes):
        for point in routes:
            self.world.debug.draw_point(
                point[0].transform.location, size=0.05, life_time=0, color=carla.Color(255, 0, 0))

    def follow_path(self):
        for actor in self.actor_list:
            routes = self.generate_path(actor)

            self.draw_path(routes)

            start = routes[0][0].transform.location
            start.x += 3.5
            end = routes[-1][0].transform.location

            path = [start, end]
            self.traffic_manager.set_path(actor, path)

        [actor.set_autopilot(True) for actor in self.actor_list[::-1]]

    def update_view(self, actor: carla.Actor):
        try:
            transform = actor.get_transform()
            self.spectator.set_transform(carla.Transform(
                transform.location + carla.Location(x=-8.0, y=0.0, z=5.0), carla.Rotation(pitch=-15)))

        except RuntimeError:
            raise KeyboardInterrupt

    def main(self, synchronous=True):
        self.set_world(synchronous)
        self.set_weatehr()
        self.set_traffic_manger(synchronous)
        self.spawn_ego()
        #self.spawn_actor()
        # self.follow_path()

        while True:
            self.world.tick()
            self.update_view(self.actor_list[0])
            
    def __del__(self):
        try:
            self.set_world(synchronous=False)
            destroy_commands1 = [carla.command.DestroyActor(actor.id) for actor in self.actor_list]
            destory_commands2 = [carla.command.DestroyActor(actor.id) for actor in self.ego.actor_list]
            
            self.client.apply_batch(destroy_commands1)
            self.client.apply_batch(destory_commands2)
            print("Canceled by user...")

        except Exception as e:
            print(f"Error in __del__: {e}")


if __name__ == '__main__':
    try:
        scenario = SCENARIO()
        scenario.main()

    except KeyboardInterrupt:
        del scenario
