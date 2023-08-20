import random
from math import dist

from cam_air_nav.cam_airnav_mod import TMA, create_flights

def flights(up,down,left,right):
    tma = TMA()
    create_flights(north=up, south=down, east=right, west=left, tma_coord=tma.coord, dest=dest, size=1, spread=0)

class Waypoint():
    def __init__(self,pos):
        self.pos=pos
        self.dis_en=0
        self.dis_ex=0
        self.dis_btw_wp=0

    def create_waypoint(self):
        pass

def create_subwaypoints(sub_box,n_wp,exit,min_dist):
    wps = [Waypoint(pos=[random.randint(*sub_box[0]), random.randint(*sub_box[1])])]
    wps_pos = [wps[0].pos]

    def swp():
        wp = Waypoint(pos=[random.randint(*sub_box[0]), random.randint(*sub_box[1])])
        if waypoint_proximity(wp.pos,wps_pos,min_dist) and wp.pos != exit:
            return wp
        else:
            return None

    for n in range(n_wp):
        wp=swp()
        if wp:
            wps += [wp]
            wps_pos += [wp.pos]
        else:
            for n in range(10):
               wp=swp()
               if wp:
                   wps += [wp]
                   wps_pos += [wp.pos]
                   break

    return wps_pos

def create_waypoints():
    waypoints=[]

def waypoint_proximity(wp_pos,wps_pos,min_dist):
    proximity=list(map(lambda x:dist(x,wp_pos),wps_pos))
    if any(c < min_dist for c in proximity):
        return False
    else:
        return True



tma=TMA()
tma.coord=[[-20,60],[15,80]]
dest=[0,40]
subwaypoints=create_subwaypoints([[5,15],[10,28]],5,[0,40],7)
print(subwaypoints)
