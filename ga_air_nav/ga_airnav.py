import random
from math import dist

from cam_air_nav.cam_airnav_mod import TMA, create_flights
from cam_air_nav.cam_airnavconfrules import conf_flight_movement as cfm

def create_aircrafts(up=0,down=0,left=0,right=0,ga=None):
    aircrafts = create_flights(north=up, south=down, east=right, west=left, tma_coord=tma.coord, dest=dest, size=0, spread=0, ga=ga)
    return aircrafts

class Waypoint():
    def __init__(self,pos):
        self.pos=pos
        self.dis_en=0
        self.dis_ex=0
        self.dis_btw_wp=0

def group_wp_to_boundingbox(bounding_boxes,wps_pos):
    grouped_wp={f'box {b}':[] for b in range(1,len(bounding_boxes)+1)}
    _wps_pos_=[pos for pos in wps_pos]

    for n,box in enumerate(bounding_boxes,1):
        group_wp=[]
        for wp_pos in wps_pos:
            if box[0][0]<=wp_pos[0]<=box[0][1] and box[1][0]<=wp_pos[1]<=box[1][1]:
                group_wp+=[wp_pos]
                _wps_pos_.remove(wp_pos)
        grouped_wp[f'box {n}']=group_wp
    return grouped_wp

# create individual waypoints while ensuring proximity rules are followed
def swp(sub_box,wps_pos,exit,min_dist,elite_wp_pos=None,elite=False):
    if elite:
        wp = Waypoint(pos=[elite_wp_pos[0], elite_wp_pos[1]])
    else:
        wp = Waypoint(pos=[random.randint(*sub_box[0]), random.randint(*sub_box[1])])
    if waypoint_proximity(wp.pos,wps_pos,min_dist) and wp.pos != exit:
        return wp
    else:
        return None

# create random positioned waypoints with a minimum distance between them
def subwaypoints(sub_box,wps_pos,n_wp,exit,min_dist,elite_wps_pos=None,elite=False):
    wps = []

    if elite:
        for elite_w_p in elite_wps_pos:
            wp = swp(sub_box, wps_pos, exit, min_dist, elite_w_p, elite)
            wps+=[wp]
            wps_pos+=[wp.pos]
        return wps,wps_pos
    else:
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

def create_waypoints(bounding_boxes,n_wp,exit,min_dist,elite_wps_pos=None,elite=False):
    all_waypoints=[]
    wps_pos=[]
    sub_nwp=n_wp//len(bounding_boxes)

    if elite:
        for box,sub_e_w_pos in zip(bounding_boxes,elite_wps_pos):
            waypoints,wps_pos=subwaypoints(box,wps_pos,len(sub_e_w_pos),exit,min_dist,sub_e_w_pos,elite)
            all_waypoints+=waypoints
        return all_waypoints,wps_pos
    else:
        for box in bounding_boxes:
            waypoints,wps_pos=subwaypoints(box,wps_pos,sub_nwp,exit,min_dist)
            all_waypoints+=waypoints
    last_bbox=random.choices(bounding_boxes,k=n_wp-len(wps_pos))
    sub_nwp = (n_wp-len(wps_pos))// len(last_bbox)
    if not elite:
        for box in last_bbox:
            waypoints,wps_pos=subwaypoints(box,wps_pos,sub_nwp,exit,min_dist)
            all_waypoints+=waypoints
    return all_waypoints,wps_pos

# to check proximity of waypoints to each other
def waypoint_proximity(wp_pos,wps_pos,min_dist):
    proximity=list(map(lambda x:dist(x,wp_pos),wps_pos))
    if any(c < min_dist for c in proximity):
        return False
    else:
        return True

def bin_same_length(wps_bin,xmax_b,ymax_b):
    bin=[b for b in wps_bin]
   # add zeros to left of waypoints binary pos so all wp binaries in a given direction are same length
    for wpb in bin:
        if wpb[0][0]=='-':
            if len(wpb[0])-1 < xmax_b:
                wpb[0] = '-' + (xmax_b - len(wpb[0])+1) * '0' + wpb[0][1:]
        else:
            if len(wpb[0]) < xmax_b:
                wpb[0] = (xmax_b - len(wpb[0])) * '0' + wpb[0]

        if wpb[1][0]=='-':
            if len(wpb[1])-1 < ymax_b:
                wpb[1] = '-' + (ymax_b - len(wpb[1])+1) * '0' + wpb[1][1:]
        else:
            if len(wpb[1]) < ymax_b:
                wpb[1] = (ymax_b - len(wpb[1])) * '0' + wpb[1]
    return bin

def crossover_wp(wps_bin,b_box_bin,single_offsprings=False):
    wps_bin=[list(map(str,wpb)) for wpb in wps_bin]

    xmax_b_len = len(str(b_box_bin[0][1]))
    ymax_b_len = len(str(b_box_bin[1][1]))
    wps_bin=bin_same_length(wps_bin,xmax_b_len,ymax_b_len)

    children=[]
    c_point=[]

    # choose crossover point
    c_xpoint = random.randint(1, xmax_b_len - 1)
    c_ypoint = random.randint(1, ymax_b_len - 1)
    for w in wps_bin:
        if '-' in w[0]:
            fir_x=w[0][0:c_xpoint+1]
        else:
            fir_x = w[0][0:c_xpoint]
        if '-' in w[1]:
            fir_y=w[1][0:c_ypoint+1]
        else:
            fir_y=w[1][0:c_ypoint]
        for m in wps_bin:
            if '-' in m[0]:
                sec_x = m[0][-(len(m[0]) - c_xpoint - 1):]
            else:
                sec_x=m[0][-(len(m[0])-c_xpoint):]
            if '-' in m[1]:
                sec_y = m[1][-(len(m[1]) - c_ypoint - 1):]
            else:
                sec_y=m[1][-(len(m[1])-c_ypoint):]
            child_x=fir_x+sec_x
            child_y=fir_y+sec_y
            children+=[[child_x,child_y]]
            c_point+=[[c_xpoint,c_ypoint]]
    # choose children within box_bounds
    elites = [child for child in children
            if int(b_box_bin[0][0]) < int(child[0]) < int(b_box_bin[0][1])
            and int(b_box_bin[1][0]) < int(child[1]) < int(b_box_bin[1][1])
            ]
    if single_offsprings:
        el_idx=list(range(len(elites)))
        random.shuffle(el_idx)
        elites_sing_idx=el_idx[0:len(wps_bin)]
        elites=[elites[n] for n in elites_sing_idx]
    return elites

def mutate_wp(wps_bin,b_box_bin,n_genes):
    xmax_b_len = len(str(b_box_bin[0][1]))
    ymax_b_len = len(str(b_box_bin[1][1]))

    population = bin_same_length(wps_bin, xmax_b_len, ymax_b_len)
    children=[]
    for n in range(5):
        for w in population:
            if '-' in w[0]:
                c_xpoints = random.choices(range(1,xmax_b_len), k=n_genes)
            else:
                c_xpoints = random.choices(range(xmax_b_len), k=n_genes)
            if '-' in w[1]:
                c_ypoints = random.choices(range(1,ymax_b_len), k=n_genes)
            else:
                c_ypoints = random.choices(range(ymax_b_len), k=n_genes)
            wlx = list(w[0])
            wly = list(w[1])
            for i in c_xpoints:
                if w[0][i]=='0':
                    wlx[i]='1'
                elif w[0][i]=='1':
                    wlx[i]='0'
            for i in c_ypoints:
                if w[1][i]=='0':
                    wly[i]='1'
                elif w[1][i]=='1':
                    wly[i]='0'
            indv=[''.join(wlx),''.join(wly)]
            children+=[indv]
    # choose children within box_bounds
    elites = [child for child in children
              if int(b_box_bin[0][0]) < int(child[0]) < int(b_box_bin[0][1])
              and int(b_box_bin[1][0]) < int(child[1]) < int(b_box_bin[1][1])]
    return elites

def convert_population_to_intpos(bin_pop):
    new_pop=[[int(p[0],2),int(p[1],2)] for p in bin_pop]

def obj_func(fls,it,n_flow):
    flow=[]
    obs=[]
    for n in range(n_flow):
        for i in range(it):
            for fl in fls:
                cfm(fls, fl, obs, 0)
                fl.collect_pos()

                # cycle flight/boundary condition and count movement in agg_pos
                if fl.pos==fl.dest:
                    fl.pos[0],fl.pos[1]=fl.dept[0],fl.dept[1]
                    fl.agg_pos = fl.agg_pos + fl.dept
        flow+=[cal_flow(fls,it)]
    avg_flow=sum(flow)/len(flow)
    return avg_flow

# waypoints_param is a list of waypoint arguments
def select_fittest_wps(aircrafts, waypoints, bounding_boxes, it, n_flow):
    avg_flow=[]

    for a,w in zip(aircrafts,waypoints):
        avg_flow+=[obj_func(a,it,n_flow)]
    max_avg_flow=max(avg_flow)
    max_afl_index = avg_flow.index(max_avg_flow)
    print('avg_flow',avg_flow)
    fittest_wps=waypoints[max_afl_index]
    fittest_wps_pos=[wp.pos for wp in fittest_wps]
    grouped_wp=group_wp_to_boundingbox(bounding_boxes,fittest_wps_pos)
    grouped_fittest_wps=list(grouped_wp.values())
    return max_avg_flow,fittest_wps

def maintain_wp_pos(wps_pos,exit,min_dist):
    new_wp_pos=[]
    pos_index=list(range(len(wps_pos)))
    random.shuffle(pos_index)
    for idx in pos_index:
        if waypoint_proximity(wps_pos[idx],new_wp_pos,min_dist) and wps_pos[idx]!=exit:
            new_wp_pos+=[wps_pos[idx]]
    return new_wp_pos

def sim_iter(n_pop,wp_arg,ac_arg,bounding_boxes,it_fittest,n_flow,n_genes_to_mutate):
    all_waypoints=[]
    all_aircrafts=[]
    all_max_aflow=[]
    aircrafts = create_aircrafts(up=ac_arg['up'], down=ac_arg['down'], left=ac_arg['left'], right=ac_arg['right'])
    acs_dept=[a.dept for a in aircrafts]
    for n in range(n_pop):
        waypoints,wps_pos = create_waypoints(bounding_boxes=wp_arg['bounding_boxes'],n_wp=wp_arg['n_waypoint'],exit=wp_arg['exit'],min_dist=wp_arg['min_dist'])

        # write all waypoints in each aircrafts' waypoint variable
        for ac in aircrafts:
            ac.way_p=wps_pos

        all_waypoints+=[waypoints]
        all_aircrafts+=[aircrafts]
        if len(all_aircrafts)<n_pop:
            aircrafts = create_aircrafts(up=ac_arg['up'], down=ac_arg['down'], left=ac_arg['left'], right=ac_arg['right'],ga=acs_dept)
    max_aflow,fit_wp=select_fittest_wps(all_aircrafts,all_waypoints,bounding_boxes,it_fittest,n_flow)
    bounding_boxes_bin=[[[bin(b[0]).replace('0b', ''), bin(b[1]).replace('0b', '')] for b in b_box] for b_box in bounding_boxes]
    all_max_aflow+=[max_aflow]
    print('First fit_wp', [f.pos for f in fit_wp])

    while True:
        all_children_wps = []
        all_aircrafts = []
        fit_wp_bin = [[bin(wp.pos[0]).replace('0b', ''), bin(wp.pos[1]).replace('0b', '')] for wp in fit_wp]
        print('fit_wp_bin', fit_wp_bin)
        for m in range(n_pop):
            all_altered_wp_bin = []
            aircrafts = create_aircrafts(up=ac_arg['up'], down=ac_arg['down'],
                                         left=ac_arg['left'], right=ac_arg['right'],ga=acs_dept)

            for n,b_box in enumerate(bounding_boxes):
                b_box_bin = bounding_boxes_bin[n]
                c_wp=crossover_wp(fit_wp_bin,b_box_bin,True)
                m_wp=mutate_wp(fit_wp_bin,b_box_bin,n_genes_to_mutate)
                altered_wp_bin=c_wp+m_wp
                all_altered_wp_bin+=altered_wp_bin

            all_altered_wp_pos = [[int(wp_bin[0], 2), int(wp_bin[1], 2)] for wp_bin in all_altered_wp_bin]
            children_wps_pos=maintain_wp_pos(all_altered_wp_pos,wp_arg['exit'],wp_arg['min_dist'])
            grouped_c_w_p_dict=group_wp_to_boundingbox(bounding_boxes,children_wps_pos)
            grouped_children_w_p = list(grouped_c_w_p_dict.values())

            children_wps,children_wps_pos=create_waypoints(bounding_boxes,wp_arg['n_waypoint'],wp_arg['exit'],wp_arg['min_dist'],grouped_children_w_p,elite=True)
            # write all waypoints in each aircrafts' waypoint variable
            for ac in aircrafts:
                ac.way_p = children_wps_pos
            print('grouped_children_w_p',grouped_children_w_p)
            all_children_wps+=[children_wps]
            all_aircrafts+=[aircrafts]
        max_aflow,fit_wp=select_fittest_wps(all_aircrafts,all_children_wps,bounding_boxes,it_fittest,n_flow)
        all_max_aflow+=[max_aflow]
        print('all_max_aflow',all_max_aflow)
        print('fit_wp_pos',[f.pos for f in fit_wp])

def simulate():
    pass

def aircrafts_movement(flights):
    ftl=[]
    flights_pos=[]
    for fl in flights:

        for m in range(0,len(fl.agg_pos),2):
            fl.sim_agg_pos+=[[fl.agg_pos[m],fl.agg_pos[m+1]]]
        flights_pos+=[fl.sim_agg_pos]
    for fp in flights_pos:
        ft=0
        for l in range(len(fp)):
            try:
                if fp[l]!=fp[l+1]:
                    ft+=1
            except:
                pass
        ftl+=[ft]
    return ftl

def cal_flow(flights,it):
    aircrafts_mov=aircrafts_movement(flights)
    flow=sum(aircrafts_mov)/it
    return flow


tma=TMA()
tma.coord=[[-25,100],[-30,300]]
if __name__=='__main__':
    dest=[0,40]
    b_box=[
        [[-3,43],[160,270]],
        [[57,92],[180,240]],
        [[0,40],[-25,70]],
        [[55,65],[-15,140]]
        ]

    waypoint_arg={'bounding_boxes':b_box,'n_waypoint':13,'exit':[0,40],'min_dist':5}
    aircraft_arg={'up':25,'down':0,'left':0,'right':0}
    sim_iter(5,waypoint_arg,aircraft_arg,b_box,100,5,1)
