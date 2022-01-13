from math import dist
import random

import numpy as np
from matplotlib import pyplot as plt


'''Object Classes'''
# dept:departure, dest:destination, size:object_size_class
# pos:current_position, fir_pos:first_position, sec_pos:second_position
# agg_pos:aggregate_positions, t_step:time_step
class Air_Object:

    def __init__(self,dept,dest):
        self.dept = dept
        self.dest = dest
        self.tdes = []
        self.size = 1
        self.con_rad = []
        self.ccrad = 0
        self.way_p = []
        self.fir_pos = []
        self.sec_pos = []
        self.pos = dept
        self.agg_pos = []
        self.t_step = 0
        self.distn = 0
        self.agg_distn = []

# to define object path; combining the object's different progressive positions
    def collect_pos(self):
        self.agg_pos=self.agg_pos+self.pos
        if self.agg_pos[-3:-1] != self.dest:
            self.t_step += 1

# the object path as a 2D array
    def flight_path(self):
        fp = np.array(self.agg_pos)
        nfp = fp.reshape(-1,2)
        self.fp = nfp.T

        # convert numpy to horizontal list for better result display
        ntl= fp.tolist()
        self.agg_pos=conv_to_2d(ntl,2)

# to calculate & collect distance of obj pos to dest
    def collect_distn(self):
        self.distn = dist(self.pos,self.dest)
        self.agg_distn+=[self.distn]

# class for other objects in the air during flight
class Free_Air_Object:

    def __init__(self,size,pos):
        self.pos = pos
        self.size = size
        self.con_rad = []
        self.dens = 0
        self.agg_dens = []
        self.t_step = 0

# class to instantiate aircraft
class Flight(Air_Object):
    pass

class Field:

    def __init__(self):
        self.av_dist = []
        self.velocity = []

class TMA(Field):
    pass

# class to instantiate waypoint
class Waypoint(Free_Air_Object):

    def coll_dens(self,flps):
        c=0
        for flp in flps:
            if flp in self.con_rad:
                c+=1
        self.dens=c/4
        self.t_step+=1
        self.agg_dens+=[self.dens]

# class to instantiate obstruction
class Obstruction(Free_Air_Object):
    pass


'''Global Variables'''
con_rad = []
ccrad = 0
gnum_sdf = [0,0,0,0]
flights_no = 0
flights_tstep = 0
hov_fli = 0


'''Collect Data'''
# calculate and collect density at waypoints
def wdens(wps,flps):

    for wp in wps:
        wp.con_rad=obj_radius(wp.size,wp.pos)
        wp.coll_dens(flps)

# to collect pos & dist of flights in flight agg_pos & agg_distn respectively
def col_dept(flights):
    for f in flights:
        f.collect_pos()
        f.collect_distn()

# collect number of flights moving per t_step
def col_fl_per_t(flights):
    ftl=[]

#create list of all agg_pos
    t_agpos=[flight.agg_pos for flight in flights]

# try..except to handle n which is beyond index for list elements
    for n in range(len(t_agpos[0])):
        ft=0
        for m in t_agpos:
            try:
                if m[n] != m[n+1]:
                    ft+=1
            except:
                pass
        ftl+=[ft]

    return ftl

def av_distn(tma,flights):
    # get total t_step of experiment
    maxt= max([fl.t_step for fl in flights])
    av_dist= []

    for n in range(maxt):
        t_distn = 0
        c = 0
        for f in flights:
            if f.t_step <= maxt:
                t_distn+=f.agg_distn[n]
                c+=1
        av_dist_t=t_distn/c
        av_dist+=[av_dist_t]

    tma.av_dist=av_dist

def vel_per_t(tma,flights):
    no_fl = col_fl_per_t(flights)

    velocity = [nf / len(flights) for nf in no_fl]

    tma.velocity = velocity


'''Conflicts Resolution'''
# resolving an object's conflict radius (size=1)
def obj_radius(size, pos):
    mpos=[0,pos[1]]
    mpos[0] = pos[0] + size
    rad = pos+mpos

    while (mpos[0]>=pos[0]-size) and (mpos[1]>=pos[1]-size):
        if mpos[1]-1>=pos[1]-size:
            mpos[1]=mpos[1]-1
            rad+=mpos
        if mpos[0]-1>=pos[0]-size:
            mpos[0]=mpos[0]-1
            rad+=mpos
        if mpos[0]==pos[0]-size and mpos[1]==pos[1]-size:
            break

    mpos[1] = mpos[1] + size
    rad+=mpos

    while (mpos[0]<=pos[0]+size) and (mpos[1]<=pos[1]+size):
        if mpos[1]+1<=pos[1]+size:
            mpos[1]=mpos[1]+1
            rad+=mpos
        if mpos[0]+1<=pos[0]+size:
            mpos[0]=mpos[0]+1
            rad+=mpos
        if mpos[0]==pos[0]+size and mpos[1]==pos[1]+size:
            break

    crad = conv_to_2d(rad,2)
    return crad

def conv_to_2d(arr,col):
    darr= [arr[i:i+col] for i in range(0,len(arr),col)]
    return darr

# to collect all possible destination of flight
def poss_temp_dest(flight,wpp):
    pt_dest = [flight.dest] + wpp
    return pt_dest

# determine coord. for best(considering waypoints & destination) path for flight
def cord_best_path(flight):
    wpp=flight.way_p
    pcord = flight.pos
    dest = poss_temp_dest(flight,wpp)

    sht_dist= min(map(lambda y: dist(pcord,y),dest))
    c_shdist = [i for i in dest if dist(pcord,i)==sht_dist][0]

    '''Update if flight should go straight to dest or wp dependent on distance'''
    wp_des  = dist(c_shdist,flight.dest)
    dep_des = dist(flight.pos,flight.dest)

    return c_shdist
    # flight with shortest distance
    # sht_dist = min(map(lambda x,y: dist(x,y), pcord,dest))

# to define conflict radius of flights
def objs_con_rad(objects):
    global con_rad
    con_rad = []

    # if-statement is so flight objects at destination have no conflict radius
    for obj in objects:
        if obj.pos != obj.dest:
            obj.con_rad=obj_radius(obj.size,obj.pos)
        else:
            obj.con_rad=[]
        con_rad+=obj.con_rad


'''Flight Navigation'''
# flight movement in no conflict
def nconf_flight_movement(dep_pos, des_pos, pos):
    if pos == des_pos:
        return

    if des_pos[0] - dep_pos[0] < 0:
        pos[0]=pos[0]-1
    if des_pos[1] - dep_pos[1] < 0:
        pos[1] = pos[1]-1

    if des_pos[0] - dep_pos[0] > 0:
        pos[0]=pos[0]+1
    if des_pos[1] - dep_pos[1] > 0:
        pos[1] = pos[1]+1

# return flight to former pos
def rev_nconf(dep_pos,des_pos,pos):

    if des_pos[0] - dep_pos[0] < 0:
        pos[0]=pos[0]+1
    if des_pos[1] - dep_pos[1] < 0:
        pos[1] = pos[1]+1

    if des_pos[0] - dep_pos[0] > 0:
        pos[0]=pos[0]-1
    if des_pos[1] - dep_pos[1] > 0:
        pos[1] = pos[1]-1

# SS
def SS(pos):
    pos[1] = pos[1] - 1

# SW
def SW(pos):
    pos[0] = pos[0] - 1
    pos[1] = pos[1] - 1

# WW
def WW(pos):
    pos[0] = pos[0] - 1

# NW
def NW(pos):
    pos[0] = pos[0] - 1
    pos[1] = pos[1] + 1

# SE
def SE(pos):
    pos[0] = pos[0] + 1
    pos[1] = pos[1] - 1

# EE
def EE(pos):
    pos[0] = pos[0] + 1

# NE
def NE(pos):
    pos[0] = pos[0] + 1
    pos[1] = pos[1] + 1

# NN
def NN(pos):
    pos[1] = pos[1] + 1

# flight movement in conflict and non-conflict
def conf_flight_movement(flights,fl,obs_pos):
    fls=[f for f in flights if f != fl]

    # one step destination of flight
    temp_des = cord_best_path(fl)
    fl.tdes = temp_des

    dep=fl.dept
    pos=fl.pos
    tdes=temp_des
    des=fl.dest

    # define conflict radius around destination
    con_des=obj_radius(1,des)

    # nconf_flight_movement(dep,tdes,fl.pos)
    if pos == des:
        return

    objs_con_rad(fls)
    global con_rad
    global hov_fli

    # add pos of obstructions to con_rad
    con_rad+=obs_pos
    print('con_rad:',con_rad)

    con_rad=[c for c in con_rad if c!=des]
    # flight waits once when destination conflicts up to len(flights)
    # if fl in fl.con_rad:
    #     fl.ccrad+=1
    # if fl.ccrad == len(flights)*3:
    #     print(fl.ccrad)
    #     fl.ccrad=0
    #     return
        # rev_nconf(dep,tdes,fl.pos)

    # best path SW
    if tdes[0]-dep[0]<0 and tdes[1]-dep[1]<0:
        # SW
        if [pos[0]-1,pos[1]-1] not in con_rad:
            SW(pos)
            return
        # SS
        if [pos[0],pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # WW
        if [pos[0] - 1,pos[1]] not in con_rad:
            WW(pos)
            return
        # NW
        if [pos[0]-1,pos[1]+1] not in con_rad:
            NW(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            hov_fli+=1
            return
        # SE
        if [pos[0]+1,pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # NN
        if [pos[0],pos[1]+1] not in con_rad:
            NN(pos)
            return
        # EE
        if [pos[0]+1,pos[1]] not in con_rad:
            EE(pos)
            return
        # NE
        if [pos[0]+1,pos[1]+1] not in con_rad:
            NE(pos)
            return
        else:
            return

    # best path WW
    if tdes[0]-dep[0]<0 and tdes[1]-dep[1]==0:
        # WW
        if [pos[0] - 1,pos[1]] not in con_rad:
            WW(pos)
            return
        # SW
        if [pos[0]-1,pos[1]-1] not in con_rad:
            SW(pos)
            return
        # NW
        if [pos[0]-1,pos[1]+1] not in con_rad:
            NW(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            hov_fli += 1
            return
        # SS
        if [pos[0],pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # NN
        if [pos[0],pos[1]+1] not in con_rad:
            NN(pos)
            return
        # SE
        if [pos[0]+1,pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # NE
        if [pos[0]+1,pos[1]+1] not in con_rad:
            NE(pos)
            return
        # EE
        if [pos[0]+1,pos[1]] not in con_rad:
            EE(pos)
            return
        else:
            return

    # best path NW
    if tdes[0]-dep[0]<0 and tdes[1]-dep[1]>0:
        # NW
        if [pos[0]-1,pos[1]+1] not in con_rad:
            NW(pos)
            return
        # WW
        if [pos[0] - 1,pos[1]] not in con_rad:
            WW(pos)
            return
        # NN
        if [pos[0],pos[1]+1] not in con_rad:
            NN(pos)
            return
        # SW
        if [pos[0]-1,pos[1]-1] not in con_rad:
            SW(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            hov_fli += 1
            return
        # NE
        if [pos[0]+1,pos[1]+1] not in con_rad:
            NE(pos)
            return
        # SS
        if [pos[0],pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # EE
        if [pos[0]+1,pos[1]] not in con_rad:
            EE(pos)
            return
        # SE
        if [pos[0]+1,pos[1] - 1] not in con_rad:
            SE(pos)
            return
        else:
            return

    # best path NN
    if tdes[0] - dep[0] == 0 and tdes[1] - dep[1] > 0:
        # NN
        if [pos[0],pos[1]+1] not in con_rad:
            NN(pos)
            return
        # NW
        if [pos[0]-1,pos[1]+1] not in con_rad:
            NW(pos)
            return
        # NE
        if [pos[0]+1,pos[1]+1] not in con_rad:
            NE(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            hov_fli += 1
            return
        # WW
        if [pos[0] - 1,pos[1]] not in con_rad:
            WW(pos)
            return
        # EE
        if [pos[0]+1,pos[1]] not in con_rad:
            EE(pos)
            return
        # SW
        if [pos[0]-1,pos[1]-1] not in con_rad:
            SW(pos)
            return
        # SE
        if [pos[0]+1,pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        else:
            return

    # best path NE
    if tdes[0] - dep[0] > 0 and tdes[1] - dep[1] > 0:
        # NE
        if [pos[0]+1,pos[1]+1] not in con_rad:
            NE(pos)
            return
        # EE
        if [pos[0]+1,pos[1]] not in con_rad:
            EE(pos)
            return
        # NN
        if [pos[0],pos[1]+1] not in con_rad:
            NN(pos)
            return
        # SE
        if [pos[0]+1,pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            hov_fli += 1
            return
        # NW
        if [pos[0]-1,pos[1]+1] not in con_rad:
            NW(pos)
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # WW
        if [pos[0] - 1,pos[1]] not in con_rad:
            WW(pos)
            return
        # SW
        if [pos[0]-1,pos[1]-1] not in con_rad:
            SW(pos)
            return
        else:
            return

    # best path EE
    if tdes[0] - dep[0] > 0 and tdes[1] - dep[1] == 0:
        # EE
        if [pos[0]+1,pos[1]] not in con_rad:
            EE(pos)
            return
        # SE
        if [pos[0]+1,pos[1]-1] not in con_rad:
            SE(pos)
            return
        # NE
        if [pos[0]+1,pos[1]+1] not in con_rad:
            NE(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            hov_fli += 1
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # NN
        if [pos[0],pos[1]+1] not in con_rad:
            NN(pos)
            return
        # SW
        if [pos[0]-1,pos[1]-1] not in con_rad:
            SW(pos)
            return
        # NW
        if [pos[0]-1,pos[1]+1] not in con_rad:
            NW(pos)
            return
        # WW
        if [pos[0] - 1,pos[1]] not in con_rad:
            WW(pos)
            return
        else:
            return

    # best path SE
    if tdes[0] - dep[0] > 0 and tdes[1] - dep[1] < 0:
        # SE
        if [pos[0]+1,pos[1]-1] not in con_rad:
            SE(pos)
            return
        # EE
        if [pos[0]+1,pos[1]] not in con_rad:
            EE(pos)
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # NE
        if [pos[0]+1,pos[1]+1] not in con_rad:
            NE(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            hov_fli += 1
            return
        # SW
        if [pos[0]-1,pos[1]-1] not in con_rad:
            SW(pos)
            return
        # NN
        if [pos[0],pos[1]+1] not in con_rad:
            NN(pos)
            return
        # NW
        if [pos[0]-1,pos[1]+1] not in con_rad:
            NW(pos)
            return
        # WW
        if [pos[0] - 1,pos[1]] not in con_rad:
            WW(pos)
            return

    # best path SS
    if tdes[0] - dep[0] == 0 and tdes[1] - dep[1] < 0:
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # SE
        if [pos[0]+1,pos[1]-1] not in con_rad:
            SE(pos)
            return
        # SW
        if [pos[0]-1,pos[1]-1] not in con_rad:
            SW(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            hov_fli += 1
            return
        # EE
        if [pos[0]+1,pos[1]] not in con_rad:
            EE(pos)
            return
        # WW
        if [pos[0] - 1,pos[1]] not in con_rad:
            WW(pos)
            return
        # NE
        if [pos[0]+1,pos[1]+1] not in con_rad:
            NE(pos)
            return
        # NW
        if [pos[0]-1,pos[1]+1] not in con_rad:
            NW(pos)
            return
        # NN
        if [pos[0],pos[1]+1] not in con_rad:
            NN(pos)
            return
        else:
            return


'''Plots'''
# to plot all the flight paths
def path_plots(fpaths):

    for fpath in fpaths:
        fpath.flight_path()
        x,y=fpath.fp
        plt.plot(x,y)

# to plot waypoints
def wp_plots(waypoints):
    wpp = [w.pos for w in waypoints]

    for wp in wpp:
        nwp=np.array(wp)
        x,y=nwp.T
        plt.plot(x,y,'r+')

# to plot obstructions
def obs_plots(obstructions):
    obp = [ob.pos for ob in obstructions]

    for ob in obp:
        nob = np.array(ob)
        x, y = nob.T
        plt.plot(x,y,'bo')


'''Display'''
# to display to user density at waypoints for each t_step
def disp_wp_dens(waypoints):
    n=0
    for w in waypoints:
        wdd = []
        t=0
        for wd in w.agg_dens:
            t+=1
            swdis=['time step:',t,'density:',wd]
            wdd+=[swdis]
        n+=1
        print('Waypoint',n,':',wdd,'\n')

# to display average distance of flights for each time step
def disp_av_distance(tma):
    savd=tma.av_dist

    avd=[['time step:',t,'average distance=',a] for (a,t) in zip(savd,range(len(savd)))]

    print(avd)

# to display all results
def display_results(tma,flights,waypoints):
    for f in flights:
        print('total time steps:',f.t_step,', flight aggregate positions:',f.agg_pos)

    disp_wp_dens(waypoints)
    disp_av_distance(tma)


'''User Interaction'''
# to split flights from a direction to subdir (e.g north to ne & nw)
def split_num(num):
    fir = num // 2
    sec = num - fir

    sn = [fir, sec]
    return sn

def inst_flights(num_flights,dpx,dpy,dest):
    # arrange data placement in rand function
    if dpx < 1:
        dpx=[dpx,0]
    else:
        dpx=[0,dpx]

    if dpy < 0:
        dpy=[dpy,0]
    else:
        dpy=[0,dpy]

    # random.seed(time.perf_counter_ns())
    # create lists of flights from random departure points
    flights=[Flight(dept=[random.randint(*dpx),random.randint(*dpy)],dest=dest)
             for _ in range(1,num_flights+1) if num_flights >= 1]
    return flights

# func for creating flights by subdir
def _nw_dep_flights(nw,nwd,dest):
    fl=inst_flights(nw, -nwd, nwd, dest)
    return fl

def _ne_dep_flights(ne,ned,dest):
    fl=inst_flights(ne, ned, ned, dest)
    return fl

def _sw_dep_flights(sw,swd,dest):
    fl=inst_flights(sw, -swd, -swd, dest)
    return fl

def _se_dep_flights(se,sed,dest):
    fl=inst_flights(se, sed, -sed, dest)
    return fl

# func to be used only for creating first set of flights
def same_dest_flights(north,south,east,west,dest):
    global gnum_sdf

    spn=split_num(north)
    n=_nw_dep_flights(spn[0],spn[0]*5,dest) + _ne_dep_flights(spn[1],spn[1]*5,dest)

    sps=split_num(south)
    s=_se_dep_flights(sps[0],sps[0]*5,dest) + _sw_dep_flights(sps[1],sps[1]*5,dest)

    spe=split_num(east)
    e=_ne_dep_flights(spe[0],(spn[1]+spe[0])*5,dest) + _se_dep_flights(spe[1],(sps[0]+spe[1])*5,dest)

    spw=split_num(west)
    w=_nw_dep_flights(spw[0],(spn[0]+spw[0])*5,dest) + _sw_dep_flights(spw[1],(sps[1]+spw[1])*5,dest)

    flights= n + s + e + w
    num_sdf= [north,south,east,west]

    gnum_sdf=[a+b for a,b in zip(num_sdf,gnum_sdf)]
    return flights

# func to create flights
def create_flights(north,south,east,west,dest,spread):
    global gnum_sdf

    num_sdf=[north,south,east,west]
    gnum_sdf = [a + b for a, b in zip(num_sdf, gnum_sdf)]

    spn = split_num(north)
    ospn = split_num(gnum_sdf[0])
    n=_nw_dep_flights(spn[0],ospn[0]*spread,dest) + _ne_dep_flights(spn[1],ospn[1]*spread,dest)

    sps = split_num(south)
    osps = split_num(gnum_sdf[1])
    s=_se_dep_flights(sps[0],osps[0]*spread,dest) + _sw_dep_flights(sps[1],osps[1]*spread,dest)

    spe = split_num(east)
    ospe = split_num(gnum_sdf[2])
    e=_ne_dep_flights(spe[0],(ospn[1]+ospe[0])*spread,dest) + _se_dep_flights(spe[1],(osps[0]+ospe[1])*spread,dest)

    spw = split_num(west)
    ospw = split_num(gnum_sdf[3])
    w=_nw_dep_flights(spw[0],(ospn[0]+ospw[0])*spread,dest) + _sw_dep_flights(spw[1],(osps[1]+ospw[1])*spread,dest)

    flights = n + s + e + w
    return flights


'''Simulation'''
# simulate flights
def sim_iter(tma,flights,flights_pos,waypoints,obstructions):
    # collect departures in flight agg_pos
    col_dept(flights)

    wpp=[w.pos for w in waypoints]
    obp = [ob.pos for ob in obstructions]

    # write all waypoints in each flights waypoint variable
    for f in flights:
        f.way_p=[w.pos for w in waypoints]

    while True:
        for flight in flights:
            print('flights_pos:', flights_pos)
            conf_flight_movement(flights,flight,obp)
            # nconf_flight_movement(flight.dept,flight.dest,flight.pos)
            flight.collect_pos()
            flight.collect_distn()
            if flight.pos in flight.way_p:
                flight.way_p.remove(flight.pos)

# collect flight densities at waypoint for each t_step
        wdens(waypoints,flights_pos)

# stop simulation when destination is reached
        flights_dest = [fd.dest for fd in flights]
        if flights_pos == flights_dest:
            break

    av_distn(tma,flights)
    vel_per_t(tma,flights)

    path_plots(flights)
    wp_plots(waypoints)
    obs_plots(obstructions)

    plt.show()

# container function for flight simulation
def simulate(tma,flights,waypoints,obstructions):
    global flights_no

    flights_pos=[f.pos for f in flights]
    flights_no=len(flights)

    sim_iter(tma,flights,flights_pos,waypoints,obstructions)


'''create flight, waypoint & obstruction objects f1,f2,f3,w1,w2,ob1,ob2...
and include all objects needed in respective experiment list'''
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    tma = TMA()

    dest = [5,5]
    f1 = Flight(dept=[-7,3], dest=dest)
    f2 = Flight(dept=[14,3], dest=dest)
    f3 = Flight(dept=[12,0], dest=[3,8])
    f4 = Flight(dept=[13,9],dest=dest)
    f5 = Flight(dept=[5,5], dest=[10,3])
    # f5 = Flight(dept=[26, 5], dest=[10, 3])
    f6 = Flight(dept=[1,1], dest=[15,2])

    fr1 = create_flights(north=2, south=1, east=1, west=1, dest=[5, 1], spread=5)
    fr2 = create_flights(north=2, south=2, east=1, west=0, dest=[7,15], spread=5)
    fr3 = create_flights(north=0, south=1, east=0, west=3, dest=[17, 20], spread=5)

    w1 = Waypoint(pos=[3,4], size=1)
    w2 = Waypoint(pos=[10,20], size=1)
    w3 = Waypoint(pos=[20,0], size=1)

    ob1 = Obstruction(pos=[14,4], size=1)
    ob2 = Obstruction(pos=[8,3], size=1)
    ob3 = Obstruction(pos=[-15,14], size=1)
    ob4 = Obstruction(pos=[4,18], size=1)

    flights = fr1+fr2+fr3+[f1,f2,f3,f4,f5]
    waypoints = [w1,w2,w3]
    obstructions = [ob1,ob2,ob3,ob4]


# to run simulation
    simulate(tma,flights,waypoints,obstructions)

    display_results(tma,flights,waypoints)