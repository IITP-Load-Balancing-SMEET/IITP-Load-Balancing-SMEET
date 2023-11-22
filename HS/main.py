import sys
import math
import random
import carla
from utils.helpers import *


class SCENARIO:
    '''
    Args:
        xosc_path(`str`): enter the xosc file path, for example `scenario01.xosc`

        map_name(`str`): enter the map name, for example `scenario01`
    '''
    walker_list = []
    actor_list = []
    actor_dict = {}

    weather_params = {'sunny': carla.WeatherParameters.ClearNoon,
                      'rainy': carla.WeatherParameters.HardRainNoon,
                      'sunset': carla.WeatherParameters.WetCloudySunset}

    vehicle_catalogue = ['vehicle.audi.etron',
                         'vehicle.bmw.grandtourer',
                         'vehicle.citroen.c3',
                         'vehicle.audi.tt',
                         'vehicle.lincoln.mkz_2017',
                         'vehicle.mercedes.coupe',
                         'vehicle.mini.cooper_s_2021',
                         'vehicle.tesla.model3',
                         'vehicle.citroen.c3']

    def __init__(self, xosc_path: str, map_name: str):
        super().__init__(xosc_path=xosc_path)
        print("Scenario start, Press Ctrl+C to stop the scenario")

        self.lane_change_scenario = True if map_name == 'scenario05' else False
        self.lane_change_scenario2 = True if map_name == 'scenario21' else False
        self.stop_scenario = True if map_name == 'scenario11' else False
        self.blinker_scenario = True if map_name == 'scenario18' else False
        self.pedestiran_scenario = True if map_name == 'scenario06' else False
        self.weather = self.weather_params['rainy' if map_name ==
                                           'scenario10' else 'sunny']

        self.client = carla.Client('localhost', 2000)
        self.world = self.client.load_world(map_name)
        self.spectator = self.world.get_spectator()
        self.map = self.world.get_map()
        self.bp = self.world.get_blueprint_library()

        super().get_performance()
        super().get_routeposition(self.map)

        if self.lane_change_scenario:
            self.lane_change_location = self.map.get_waypoint_xodr(
                road_id=6, lane_id=2, s=140).transform.location
        
        elif self.lane_change_scenario2:
            self.lane_change_location = self.map.get_waypoint_xodr(road_id=2, lane_id=-1, s=290.0).transform.location

        elif self.stop_scenario:
            self.stop_location = self.map.get_waypoint_xodr(road_id=10, lane_id=1, s=11).transform.location

        elif self.blinker_scenario:
            self.blinker_location = [self.map.get_waypoint_xodr(road_id=1, lane_id=-1, s=300).transform.location,
                                     self.map.get_waypoint_xodr(road_id=2, lane_id=1, s=130).transform.location]

        elif self.pedestiran_scenario:
            self.pedestrian_location = [self.map.get_waypoint_xodr(road_id=21, lane_id=-1, s=16).transform, 
                                        self.map.get_waypoint_xodr(road_id=23, lane_id=-1, s=10).transform]

        else:
            pass

    def set_world(self, synchronous=True):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

    def set_weatehr(self):
        self.world.set_weather(self.weather)

    def set_traffic_manger(self, synchronous=True):
        self.traffic_manager = self.client.get_trafficmanager()
        self.traffic_manager.set_synchronous_mode(synchronous)

    def spawn_actor(self):
        if self.pedestiran_scenario:
            start, end = self.pedestrian_location[0], self.pedestrian_location[1]

            start.location.z += 4.0
            end.location.z += 4.0

            for _ in range(3): # 3 pedestirans
                start.location.x += 1.5
                walker_bp = random.choice(self.bp.filter('walker.*'))
                walker = self.world.spawn_actor(walker_bp, start)

                control = carla.WalkerControl()
                control.speed = 0.5  # Set the speed of the walker (in m/s)
                control.direction.x = -1.0
                walker.apply_control(control)

                self.walker_list.append(walker)


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

    def visualize_specific_point(self):
        if self.lane_change_scenario:
            self.world.debug.draw_point(
                self.lane_change_location, size=0.2, life_time=0, color=carla.Color(0, 255, 0))

        if self.stop_scenario:
            self.world.debug.draw_point(
                self.stop_location, size=0.2, life_time=0, color=carla.Color(0, 255, 0))

        if self.blinker_scenario:
            for point in self.blinker_location:
                self.world.debug.draw_point(
                    point, size=0.2, life_time=0, color=carla.Color(0, 255, 0))

    def lane_change(self, actor_idx: list, radius: float = 5.0):
        for i in actor_idx:
            actor = self.actor_list[i]
            current_location = actor.get_location()
            dist = math.sqrt((self.lane_change_location.x - current_location.x)
                             ** 2 + (self.lane_change_location.y - current_location.y)**2)

            if dist <= radius:
                self.traffic_manager.random_left_lanechange_percentage(
                    actor, 100)

    def stop(self, actor_idx: list, radius: float = 5.0, stop_duration: float = 5.0):
        for i in actor_idx:
            actor = self.actor_list[i]
            current_location = actor.get_location()
            dist = math.sqrt((self.stop_location.x - current_location.x)
                             ** 2 + (self.stop_location.y - current_location.y)**2)

            if dist <= radius:
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                control.steer = 0.0

                actor.set_autopilot(False)
                actor.apply_control(control)

                start_time = self.world.get_snapshot().timestamp.elapsed_seconds

                while (self.world.get_snapshot().timestamp.elapsed_seconds - start_time) < stop_duration:
                    self.world.tick()

                actor.set_autopilot(True)

    def main(self, synchronous=True):
        self.set_world(synchronous)
        self.set_weatehr()
        self.set_traffic_manger(synchronous)
        self.spawn_actor()
        self.follow_path()
        self.visualize_specific_point()

        while True:
            self.world.tick()
            self.update_view(self.actor_list[0])

            if self.lane_change_scenario:
                self.lane_change(actor_idx=[1, 2], radius=5.0)

            if self.lane_change_scenario2:
                self.lane_change(actor_idx=[0], radius=5.0)

            if self.stop_scenario:
                self.stop(actor_idx=[1], radius=5.0, stop_duration=5.0)

    def __del__(self):
        try:
            self.set_world(synchronous=False)
            destroy_commands1 = [carla.command.DestroyActor(
                actor.id) for actor in self.actor_list]
            destroy_commands2 = [carla.command.DestroyActor(
                walker.id) for walker in self.walker_list]
            self.client.apply_batch(destroy_commands1)
            self.client.apply_batch(destroy_commands2)
            print("Canceled by user...")

        except Exception as e:
            print(f"Error in __del__: {e}")


if __name__ == '__main__':
    '''
    <완성된 시나리오>
        - 01, 02, 03, 05, 06-2, 07, 09, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21

    <유턴 시나리오>
        - 해결 불가
        - 04, 08
    '''
    try:
        scenario_num = "scenario21"
        scenario = SCENARIO(xosc_path=f'./{scenario_num}/{scenario_num}.xosc', map_name=scenario_num)
        scenario.main()

    except KeyboardInterrupt:
        del scenario
