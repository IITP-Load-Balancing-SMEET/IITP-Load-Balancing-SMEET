## This is the test file for carla simulator
import os
import numpy
import sys
import glob
import sys
import carla
try:
    sys.path.append(glob.glob('D:\CARLA_0.9.13\WindowsNoEditor\PythonAPI\carla\dist\carla-*%d.%d-%s.egg' % (
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
        self.actor_list = self.world.get_actors()
        self.map= self.world.get_map()
        self.xodr=self.map.to_opendrive()
    def set_map(self,name):
        self.client.load_world(name)
    
    
    def get_waypoint(self):
        map = self.world.get_map()
        self.way_points = map.generate_waypoints(distance=2)
        print(len(self.way_points))
    
    def draw_waypoint(self):
        for w in self.way_points:
            self.world.debug.draw_point(w.transform.location, size=0.1,
                                       color=carla.Color(r=255, g=0, b=0))
    def get_traffic_lights(self):
        Landmarks=self.map.get_all_landmarks()
        print(len(Landmarks))
        for i in Landmarks:
            print("type :",i.type, "id:", i.id, "value:", i.value, "name:" ,i.name,"is_dynamic:" ,i.is_dynamic, "sub_type:", i.sub_type, "road_id:",i.road_id)
    
    def get_actor(self):
        actor_list = self.world.get_actors()
        for str in actor_list:
            print(actor_list)

    def get_offset_transform(self,transform, offset):
        """
        This function adjusts the give transform by offset and returns the new transform.
        """
        if offset != 0:
            forward_vector = transform.rotation.get_forward_vector()
            orthogonal_vector = carla.Vector3D(x=-forward_vector.y, y=forward_vector.x, z=forward_vector.z)
            transform.location.x = transform.location.x + offset * orthogonal_vector.x
            transform.location.y = transform.location.y + offset * orthogonal_vector.y
        return transform

    def find_roadmarks(self):
        xml_tree = ET.fromstring(self.xodr)
        roads = xml_tree.iter('road')
        for road in roads:
            objects = road.find('objects')
            road_id = int(road.get('id'))
            if objects is not None:
                for object in objects.iter('object'):
                    object_name= object.get('name')
                    object_s = float(object.get('s'))
                    object_t = float(object.get('t'))
                    object_z = float(object.get('zOffset'))
                    transform = self.map.get_waypoint_xodr(road_id,0,object_s).transform
                    transform = self.get_offset_transform(transform, object_t)                    
                    print(f"name : {object_name}, road_id :{road_id} s : {object_s}, t : {object_t}, z : {object_z}, transform: {transform.location}")
            

    def run(self): 
       
        # World Setting
        car = self.blueprint_library.filter('model3')[0]


Te = Test('localhost',2000,10.0)

Te.set_map("Town04")
