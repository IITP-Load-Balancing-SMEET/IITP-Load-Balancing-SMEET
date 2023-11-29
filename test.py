import carla
import numpy as np

client = carla.Client('localhost', 2000)

world = client.load_world('Town04')
map = world.get_map()

bp = world.get_blueprint_library()
ego = bp.find('vehicle.lincoln.mkz_2017')
for i in range(10):
    world.spawn_actor(ego, np.random.choice(map.get_spawn_points()))
    print('Vehicle respnse {}',i)