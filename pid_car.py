import glob
import os
import sys
import random
import time
import numpy as np
import cv2
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
def search_min_waypoints(waypoints,point):
    count_waypoints = list()
    for w in waypoints:
        count = ((point.transform.location.x - w.transform.location.x)** 2 + (point.transform.location.y - w.transform.location.y) ** 2)**0.5
        count_waypoints.append(count)
    result = np.argmin(count_waypoints)
    return result

def calc_waypoints(waypoints):
    final_waypoints, current_waypoints= list(), list()
    for w in waypoints:
        current_waypoints.append(w)
    next_point = current_waypoints[0]
    current_waypoints.pop(0)# not need to find the same , e.g first one pick [0] , find [1-100] , so the loop need pop the first
    for i, w in enumerate(waypoints):
        if(current_waypoints): #for the last one is empty
            min_result = search_min_waypoints(current_waypoints,next_point)
            final_waypoints.append(current_waypoints[min_result])
            next_point = current_waypoints[min_result]
            current_waypoints.pop(min_result)
    return final_waypoints

def draw_points(drawing_material,map,seconds,color_array = [0,255,0],option=1):
    if(option == 1):
        for idx, w in enumerate(drawing_material):
            #about the point different , the location not like the waypoints
            world.debug.draw_string(w.location, '0 {}'.format(idx), draw_shadow=False,color=carla.Color(r=color_array[0], g=color_array[1], b=color_array[2]),life_time=seconds,persistent_lines=True)
    else:
        world.debug.draw_string(drawing_material.location, 'O start point', draw_shadow=False,color=carla.Color(r=color_array[0], g=color_array[1], b=color_array[2]),life_time=seconds,persistent_lines=True)

def draw_waypoints(waypoints,map,seconds):
    left_lane_w,right_lane_w,all_lane_w,testing_lane_w = list(),list(),list(),list()
    for idx, w in enumerate(waypoints):
        if idx % 2 == 0:
            right_lane_w.append(w)
        else:
            left_lane_w.append(w)
        all_lane_w.append(w)

    planning_route = all_lane_w[492:616:2] + all_lane_w[1354:1384:2] + all_lane_w[426:458:2] + all_lane_w[1594:1720:2] + all_lane_w[1386:1576:2] + all_lane_w[1932:1962:2] + all_lane_w[2608:2862:2]
    planning_route = planning_route + all_lane_w[793:1143:2] + all_lane_w[21:33:2] + all_lane_w[313:369:2] + all_lane_w[277:287:2]
    planning_route = calc_waypoints(planning_route)

    for i, w in enumerate(planning_route):
        # world.debug.draw_string(w.transform.location, '    {} x:{:.2f} y:{:.2f} pitch:{:.2f}'.format(i,w.transform.location.x,w.transform.location.y,w.transform.rotation.pitch) , draw_shadow=False, color=carla.Color(r=255, g=0, b=0),life_time=180.0,persistent_lines=True)
        # world.debug.draw_string(w.transform.location, '    {}'.format(i) , draw_shadow=False, color=carla.Color(r=255, g=0, b=0),life_time=180.0,persistent_lines=True)
        world.debug.draw_string(w.transform.location, 'O {}'.format(i), draw_shadow=False, color=carla.Color(r=0, g=0, b=255),life_time=seconds,persistent_lines=True)
        # world.debug.draw_string(w.transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255),life_time=300.0,persistent_lines=True)
    start_waypoint, end_waypoint = planning_route[0], planning_route[-1]    
    world.debug.draw_string(start_waypoint.transform.location, '###', draw_shadow=False, color=carla.Color(r=255, g=0, b=255),life_time=seconds,persistent_lines=True)
    world.debug.draw_string(end_waypoint.transform.location, '###', draw_shadow=False, color=carla.Color(r=255, g=0, b=255),life_time=seconds,persistent_lines=True)

if __name__ == "__main__":

    #--------------for set client , map , world , waypoints and the map-------------------
    client = carla.Client('localhost',2000)
    client.set_timeout(5)
    client.load_world('Town02')

    world = client.get_world()

    # settings = world.get_settings()
    # settings.fixed_delta_seconds = 0.05
    # world.apply_settings(settings)

    map = world.get_map()
    waypoints = map.generate_waypoints(distance=1)

    #for the draw_waypoints function 
    draw_waypoints(waypoints,map,900)

    #-----------------------set the spectator and draw the points----------------------

    #for draw the spawn_points
    spawn_points = world.get_map().get_spawn_points()
    draw_points(spawn_points,map,900,[255,0,0],1)

    #for setting the spectator position and the find the actors in the world
    actor_list = world.get_actors()
    draw_points(actor_list[2].get_transform(),map,900,[255,0,255],0)

    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(spawn_points[12].location + carla.Location(z=15,y=15),carla.Rotation(pitch=-45,yaw=-90)))

    #-----------------------try the vehicles----------------------
    blueprint_library = world.get_blueprint_library()

    vehicle_bp = blueprint_library.filter('vehicle.volkswagen.t2')[0]
    vehicle = world.spawn_actor(vehicle_bp,spawn_points[12])

    time.sleep(1)

    def throttle_go(vehicle_,number,time_):
        vehicle_.apply_control(carla.VehicleControl(throttle=number))
        time.sleep(time_)

    def brake_go(vehicle_,number,time_):
        vehicle_.apply_control(carla.VehicleControl(brake=number))
        time.sleep(time_)

    def left_go(vehicle_,number,time_):
        vehicle_.apply_control(carla.VehicleControl(steer=number))
        time.sleep(time_)

    #throttle , steer , brake , reverse

    # throttle_go(vehicle,1,6.6)
    # left_go(vehicle,-1,1)
    print(carla.VehicleControl())
    # left_go(vehicle,1,.5)
    # left_go(vehicle,-1,.4)
    # left_go(vehicle,1,.2)
    # left_go(vehicle,-1,.2)
    # left_go(vehicle,0,.3)
    # throttle_go(vehicle,7,1)
    












