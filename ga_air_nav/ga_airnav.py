import random
from math import dist

from cam_air_nav.cam_airnav_mod import TMA, create_flights
from cam_air_nav.cam_airnavconfrules import conf_flight_movement as cfm

def create_aircrafts(up=0,down=0,left=0,right=0):
    create_flights(north=up, south=down, east=right, west=left, tma_coord=tma.coord, dest=dest, size=0, spread=0)

class Waypoint():
    def __init__(self,pos):
        self.pos=pos
        self.dis_en=0
        self.dis_ex=0
        self.dis_btw_wp=0

    def create_waypoint(self):
        pass

def group_wp_to_boundingbox(bounding_boxes,wps_pos):
    grouped_wp={f'box {b}':[] for b in range(1,len(bounding_boxes)+1)}
    _wps_pos_=[pos for pos in wps_pos]

    for n,box in enumerate(bounding_boxes,1):
        group_wp=[]
        for wp_pos in wps_pos:
            if wp_pos[0]>=box[0][0] and wp_pos[0]<=box[0][1] and wp_pos[1]>=box[1][0] and wp_pos[1]<=box[1][1]:
                group_wp+=[wp_pos]
                _wps_pos_.remove(wp_pos)
        grouped_wp[f'box {n}']=group_wp
    # if len(wps_pos[0])>1:
    #     grouped_wp['Others']=_wps_pos_
    return grouped_wp

# create individual waypoints while ensuring proximity rules are followed
def swp(sub_box,wps_pos,exit,min_dist):
    wp = Waypoint(pos=[random.randint(*sub_box[0]), random.randint(*sub_box[1])])
    if waypoint_proximity(wp.pos,wps_pos,min_dist) and wp.pos != exit:
        return wp
    else:
        return None

# create random positioned waypoints with a minimum distance between them
def subwaypoints(sub_box,wps_pos,n_wp,exit,min_dist):
    # wps = [Waypoint(pos=[random.randint(*box[0]), random.randint(*box[1])])]
    wps = []
    # wps_pos = [wps[0].pos]

    for n in range(n_wp):
        wp=swp(sub_box,wps_pos,exit,min_dist)
        if wp:
            wps += [wp]
            wps_pos += [wp.pos]
        else:
            for n in range(30):
               wp=swp(sub_box,wps_pos,exit,min_dist)
               if wp:
                   wps += [wp]
                   wps_pos += [wp.pos]
                   break

    return wps,wps_pos

def create_waypoints(bounding_boxes,n_wp,exit,min_dist):
    wps_pos=[]
    sub_nwp=n_wp//len(bounding_boxes)

    for box in bounding_boxes:
        waypoints,wps_pos=subwaypoints(box,wps_pos,sub_nwp,exit,min_dist)

    last_bbox=random.choices(bounding_boxes,k=n_wp-len(wps_pos))
    sub_nwp = (n_wp-len(wps_pos))// len(last_bbox)
    for box in last_bbox:
        waypoints, wps_pos = subwaypoints(box, wps_pos, sub_nwp, exit, min_dist)
    return wps_pos

# to check proximity of waypoints to each other
def waypoint_proximity(wp_pos,wps_pos,min_dist):
    proximity=list(map(lambda x:dist(x,wp_pos),wps_pos))
    if any(c < min_dist for c in proximity):
        return False
    else:
        return True

# waypoints_param is a list of waypoint arguments
def select_fittest_wps(aircrafts, waypoints_param, n_individuals, it, n_flow):
    population=[]
    avg_flow=[]

    for p in range(n_individuals):
        waypoints = create_waypoints(*waypoints_param)
        population+=[waypoints]
        for ac in aircrafts:
            ac.way_p=[wp.pos for wp in waypoints]
        avg_flow+=[obj_func(aircrafts,it,n_flow)]
    max_afl_index=avg_flow.index(max(avg_flow))
    fittest_wps=population[max_afl_index]
    grouped_wp=group_wp_to_boundingbox(fittest_wps)
    fittest_wps=list(grouped_wp.values())

    for n,wps in enumerate(fittest_wps):
        if len(wps)>1:
            pass

    # wps_bin=[[bin(w.pos[0]),bin(w.pos[1])] for w in fittest_wps]

def crossover_wp(wps_bin,b_box):
    wps_bin=map(str,wps_bin)

def mutate_wp(wps_bin):
    pass

def obj_func(fls,it,n_flow):
    flow=[]
    obs=[]
    for n in range(n_flow):
        for i in range(it):
            for fl in fls:
                cfm(fls, fl, obs)
                fl.collect_pos()

                # cycle flight/boundary condition and count movement in agg_pos
                if fl.pos==fl.dest:
                    fl.pos[0],fl.pos[1]=fl.dept[0],fl.dept[1]
                    fl.agg_pos = fl.agg_pos + fl.dept
        flow+=cal_flow()
    avg_flow=sum(flow)/len(flow)
    return avg_flow

def sim(aircrafts,waypoints,iterations):
    wps_pos=[wp.pos for wp in waypoints]

    # write all waypoints in each aircrafts' waypoint variable
    for ac in aircrafts:
        ac.way_p=[wps_pos]

def cal_flow(agg_pos):
    pass

tma=TMA()
tma.coord=[[-20,60],[-15,80]]
dest=[0,40]
b_box=[
    [[-3,5],[40,45]],
    [[35,40],[0,45]],
    [[-2,8],[-3,35]],
    [[30,45],[-8,-4]]
        ]

waypoints=create_waypoints(b_box,13,[0,40],7)
Flights=create_aircrafts(up=25)
print(waypoints)
print(group_wp_to_boundingbox(b_box,waypoints))
