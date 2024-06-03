import math
import os
import random

import numpy as np
from matplotlib import pyplot as plt

from ga_air_nav import visualizations as vs
from cam_air_nav.cam_airnav_mod import Air_Object, Free_Air_Object, avg_trans_time, col_dept, col_dept_sing
from cam_air_nav.cam_airnavconfrules import obj_radius, conv_to_2d


'''Object Field Classes'''
class Obj_Field(Air_Object):

     def __init__(self,row,column,max_pot,grid_size,size,plt_color):
         mp_pos = [column, row]
         super(Obj_Field, self).__init__([mp_pos[0],mp_pos[1]],[0,0],size)

         self.g_size = [grid_size[1], grid_size[0]]
         self.color = plt_color
         self.max_pot = max_pot
         self.pot = 0
         self.fld_pot = res_field(self.g_size,mp_pos,max_pot)
         self.base_fld_pot = tuple(self.fld_pot)
         self.tfp = []

class Oth_Obj_Field(Free_Air_Object):

    def __init__(self,row,column,max_pot,grid_size,size):
        mp_pos = [column, row]
        super(Oth_Obj_Field, self).__init__(size,[mp_pos[0], mp_pos[1]])

        self.g_size = [grid_size[1],grid_size[0]]
        self.max_pot = max_pot
        self.pot = 0
        self.fld_pot = res_field(self.g_size, mp_pos, max_pot)
        self.base_fld_pot = tuple(self.fld_pot)

class TMA:
    def __init__(self,row,column,max_pot,grid_size):
        self.mp_pos = [column, row]
        self.g_size = [grid_size[1], grid_size[0]]
        self.max_pot = max_pot
        self.fld_pot = res_field(self.g_size,self.mp_pos,max_pot)
        self.base_fld_pot = tuple(self.fld_pot)
        self.fld = []
        self.dens = 0
        self.tot_tstep = 0
        self.transit_time=0
        self.vel = []
        self.avg_transit_time = 0
        self.avg_vel_a = 0
        self.avg_vel_b = 0
        self.sim_path_conf = {}

    def fd_dens(self,objs):
        self.dens=len(objs)/(self.g_size[0]*self.g_size[1])

    def store_sim_path(self,field, flights, clip_no):
        sim_clip = {}
        sim_clip['field'] = tuple(field)
        sim_clip['flight_path'] = tuple([flight.fp for flight in flights])
        self.sim_path_conf[clip_no] = sim_clip

class Aircraft(Obj_Field):
    def __init__(self,null_pont=False,**kwargs):
        super(Aircraft, self).__init__(**kwargs)
        # to create aircraft with completely zero potentials
        if null_pont: self.fld_pot=change_anydim_lst_sign(self.fld_pot,change_to_zero)

    def flight_path(self):
        super(Aircraft, self).flight_path()

        # adjusting coord of potn for field visualization
        for m in range(len(self.fp)):
            for n in range(len(self.fp[0])):
                self.fp[m,n]=self.fp[m,n]-1

        # collect aggregate position of flight in a better readable form
        self.coll_sim_agg_pos()
        # self.agg_pos=conv_to_2d(self.agg_pos,2)
        # self.agg_pos=[[m[1],m[0]] for m in self.agg_pos]


class Obstruction(Oth_Obj_Field):
    def __init__(self,**kwargs):
        super(Obstruction, self).__init__(**kwargs)
        # implement obstructions to have repulsive forces
        self.fld_pot = change_anydim_lst_sign(self.fld_pot,change_sign)

class Stat_Obstruction(Obstruction):
    pass

# moving obstructions like clouds
class Mov_Obstruction(Obstruction):
    def __init__(self,mob_rate=1,mob_span=0,mut_rate=1,mut_span=0,**kwargs):
        # default mobility and mutation rate is set to minimal i.e. 1
        super(Mov_Obstruction, self).__init__(**kwargs)
        self.mob_rate = mob_rate
        self.mob_span = mob_span
        self.mut_rate = mut_rate
        self.mut_span = mut_span
        self.original_pos = tuple(self.pos)

    # recreate the field potential matrix when the obstruction moves
    # mob_span to restrict no. of cells the obstruction can move in any random direction throughout the simulation
    def mov_obs(self):
        p0 = random.choice([self.pos[0]-1,self.pos[0],self.pos[0]+1])
        # reset coord so that obstruction never moves past user-defined span
        while p0>self.original_pos[0]+self.mob_span or p0<self.original_pos[0]-self.mob_span:
            p0 = random.choice([self.pos[0] - 1, self.pos[0], self.pos[0] + 1])

        p1 = random.choice([self.pos[1]-1,self.pos[1],self.pos[1]+1]) if p0!=self.pos[0] else random.choice([self.pos[1]-1,self.pos[1]+1])
        # reset coord so that obstruction never moves past user-defined span
        while p1>self.original_pos[1]+self.mob_span or p1<self.original_pos[1]-self.mob_span:
            p1 = random.choice([self.pos[1]-1,self.pos[1],self.pos[1]+1]) if p0!=self.pos[0] else random.choice([self.pos[1]-1,self.pos[1]+1])

        self.pos=[p0,p1]
        self.fld_pot=res_field(self.g_size,self.pos,self.max_pot)
        # change field potential to negative given obstructions have repulsive forces
        self.fld_pot = change_anydim_lst_sign(self.fld_pot, change_sign)

    def mut_obs(self):
        max_pot=random.choice(range(sum([self.max_pot,-self.mut_span]),sum([self.max_pot,self.mut_span,1])))
        self.max_pot=max_pot
        self.fld_pot = res_field(self.g_size, self.pos, self.max_pot)
        # change field potential to negative given obstructions have repulsive forces
        self.fld_pot = change_anydim_lst_sign(self.fld_pot, change_sign)

class Waypoint(Oth_Obj_Field):
    pass

class Point(Oth_Obj_Field):
    pass

loc_tma=None
con_rad=[]
dests=[]
acraft_info=[]
a_cord=[]

'''Abstract Modifiers/Correctors'''
# restrict magnitude of elements of a list within a range
def restrict_num(original_lst,lst,max_change):
    nlist = []
    # get indexes of elements that are out of range
    for n, ol, l in enumerate(zip(original_lst, lst)):
        p = l
        while p-ol > max_change or ol-p < max_change:
            p = l
            p = random.choice([p-1, p+1])
        nlist += [p]

# switch row & col to y & x respect.
def swi_cord_elem(cord):
    if isinstance(cord[0],list):
        ncord=[[cd[1],cd[0]] for cd in cord]
    else:
        ncord=[cord[1],cord[0]]
    return ncord

# get depth of list in a list_object
def no_lsts_in_lsts(lst):
    dlst=f'{lst}'
    no=len(dlst)-len(dlst.lstrip('['))
    return no

def change_to_zero(num):
    num=0*num
    return num

def change_sign(num):
    num=-1*num
    return num

def change_lst_sign(lst,func_change):
    li=[func_change(l) for l in lst]
    return li

# change polarity of elements in a list
# correction of obstruction potential field
def change_anydim_lst_sign(lst,func_change):
    li = []

    def pont_corr(lst):
        li=[]
        if not isinstance(lst[0], list):
            li += [change_lst_sign(lst,func_change)]
            return li

        elif isinstance(lst[0],list):
            for l in lst:
                li += pont_corr(l)
        return li

    li += pont_corr(lst)
    return li

# apply func on every unique sequence in 'i'
# !! update to more abstract approach (do away with arg)
def rem_dup(i,end,func,arg):
    con = []
    for el in i:
        con+=[el]
        if el in end:
            func(con,arg)
            con = []

# plot for 2-D list
def path_plot(path_cord, color):
    cn = np.array(path_cord)
    cnn = cn.T
    x, y = cnn
    plt.plot(x, y, color)
    return

# custom func to plot flight path
def fl_paths(flights,total_tstep):
    global dests
    dess = [[d[0]-1,d[1]-1] for d in dests]
    pdes = swi_cord_elem(dests)

    if total_tstep==1:
        for fpath in flights:
            fpath.flight_path()
        # path_plots(flights)
        # handle duplicates in agg_pos
        for f in flights:
            con = []
            for el in f.agg_pos:
                con += [el]
                if el in pdes:
                    f.agg_pos = con
                    break

    elif total_tstep>1 or total_tstep==0:
        for fpath in flights:
            fpath.flight_path()


'''Create Field, Enter Potentials & Calc Resultant'''
# define major field with arbitrary potential val
def maj_field(grid_size):
    gs = grid_size
    grid = [['x' for _ in range(gs[1])] for _ in range(gs[0])]
    return grid

# insert potential in major field
def maj_field_pot(max_pot, mp_pos):
    mps=mp_pos
    mpss=[(x*2)+1 for x in mps]
    p=max_pot
    low=max_pot-mps[0]

    maj_fld=maj_field(mpss)
    maj_fld[mps[0]][mps[1]]=p
    rad=[mps]

    for m in range(low,max_pot):
        rn=len(rad)
        p-=1
        for n,v in zip(range(rn),rad):
            nrad=[]
            if [v[0],v[1]+1] not in rad:
                nrad+=[[v[0],v[1]+1]]
                maj_fld[v[0]][v[1]+1]=p

            if [v[0]+1,v[1]+1] not in rad:
                nrad+=[[v[0]+1,v[1]+1]]
                maj_fld[v[0]+1][v[1]+1]=p

            if [v[0]+1,v[1]] not in rad:
                nrad+=[[v[0]+1,v[1]]]
                maj_fld[v[0]+1][v[1]]=p

            if [v[0]+1,v[1]-1] not in rad:
                nrad+=[[v[0]+1,v[1]-1]]
                maj_fld[v[0]+1][v[1]-1]=p

            if [v[0],v[1]-1] not in rad:
                nrad+=[[v[0],v[1]-1]]
                maj_fld[v[0]][v[1]-1]=p

            if [v[0]-1,v[1]-1] not in rad:
                nrad+=[[v[0]-1,v[1]-1]]
                maj_fld[v[0]-1][v[1]-1]=p

            if [v[0]-1,v[1]] not in rad:
                nrad+=[[v[0]-1,v[1]]]
                maj_fld[v[0]-1][v[1]]=p

            if [v[0]-1,v[1]+1] not in rad:
                nrad+=[[v[0]-1,v[1]+1]]
                maj_fld[v[0]-1][v[1]+1]=p
            rad+=nrad
    return maj_fld

# define obj field: creates major field and slices out relevant field
def res_field(grid_size,pos,max_pot):
    len_lst=[grid_size[0]-pos[0],grid_size[1]-pos[1],pos[0]-1,pos[1]-1]
    size = max(len_lst)
    mp_pos = [size,size]
    bg_grid = maj_field_pot(max_pot,mp_pos)
    grid = []

    # longest potential reduc. is l-r_end
    if size==len_lst[0]:
        for m in bg_grid:
            grid+=[m[-pos[0]-size:]]
        grid=grid[size-pos[1]+1:size-pos[1]+1+grid_size[1]]
            # grid=bg_grid[-pos[0]-1-size:-1][size-pos[1]:size-pos[1]+grid_size[1]]
    # longest potential reduc. is u-d_end
    elif size==len_lst[1]:
        for m in bg_grid:
            grid+=[m[size-pos[0]+1:size-pos[0]+1+grid_size[0]]]
        grid=grid[-pos[1]-size:]
        # grid=bg_grid[size-pos[0]:size-pos[0]+grid_size[0]][-pos[1]-size:]
    # longest potential reduc. is l_start-r
    elif size == len_lst[2]:
        for m in bg_grid:
            grid+=[m[0:grid_size[0]]]
        grid=grid[size-pos[1]+1:size-pos[1]+1+grid_size[1]]
    # longest potential reduc. is u_start-d
    elif size == len_lst[3]:
        for m in bg_grid:
            grid+=[m[size-pos[0]:size-pos[0]+grid_size[0]]]
        grid=grid[:grid_size[1]]
    return grid

# calc resultant potential
def reslt_pot(pots):
    pts=[np.matrix(p) for p in pots]
    res=sum(pts)
    res = res.tolist()
    return res


'''Collect Data'''
# store objects
def store_objects(store_container,*args):
    objs=list(store_container.keys())
    for n,k in enumerate(objs):
        store_container[k]+=[args[n]]
    return store_container

def col_dest(tma,max_pot):
    dests = []
    for m in range(len(tma)):
        for n in range(len(tma[0])):
            if tma[m][n]==max_pot:
                dests+=[[n+1,m+1]]
    return dests

# collect number of flights moving per t_step
def col_fl_per_t(tma,flights,foc_flights=None):
    tmv=[]

    # create list of all agg_pos
    if foc_flights:
        t_agpos=foc_flights
        tot_tstep=max([len(ts) for ts in t_agpos])
    else:
        t_agpos=[flight.agg_pos for flight in flights]
        tot_tstep=tma.tot_tstep=max([len(ts) for ts in t_agpos])
    # try..except to handle n which is beyond index for list elements
    for n in range(tot_tstep):
        mv=0
        for m in t_agpos:
            try:
                if m[n] != m[n+1]:
                    mv+=1
            except:
                pass
        tmv +=[mv]
    return tmv

def vel_per_t(tma,flights):
    no_fl = col_fl_per_t(tma,flights)

    velocity = [nf / len(flights) for nf in no_fl]

    tma.vel = velocity
    sum_vel = sum(tma.vel)
    tma.avg_vel_a = sum_vel/tma.tot_tstep
    tma.avg_vel_b = sum_vel/len(flights)

# calculate number of movement in simulation for a given number of timesteps
# for a given number of aircraft over number of maximum possible movement for stated aircraft
def cal_flow(t_steps,flights,tma=None):
    if not tma:
        global loc_tma
        tma=loc_tma
    # get position of flight in defined t_steps
    foc_flights=[fl.agg_pos[:t_steps] for fl in flights]
    flow=sum(col_fl_per_t(tma,flights,foc_flights))/(t_steps*len(flights))
    return flow

'''Conflict Resolution'''
# return content of different positions in object
def pos_vals(obj,cord):
    val=[]

    # try stmnt to return large -ve value if index is not found
    for c in cord:
        try:
            val+=[obj[c[1]][c[0]]]
        except:
            val+=[-10*math.exp(10**2)]
    return val

# resolve next position of flight
def nxt_pos(field,pos):
    fd=field
    crds= obj_radius(1,pos)
    cords=[]

    # in case of -ve dimen. in coord., change to original pos
    for c in crds:
        if all([_>=0 for _ in c]):
            cords+=[c]
        else:
            cords+=[pos]
    vals=pos_vals(fd,cords) # vals content sequence is clockwise i.e pos,E,SE,S,...,NE
    mvals=[i-vals[0] for i in vals]

    # try stmnt to keep a_craft in a pos when potn around it is lesser or same
    try:
        npos_i=mvals.index(min([j for j in mvals if j > 0]))
    except:
        npos_i=mvals.index(min([j for j in mvals if j >= 0]))
    return npos_i

# flight conflict resolution
def conf_resl(field,flights,flight):
    global con_rad, dests

    fls = []
    con_rad = []
    for f in flights:
        if f != flight:
            fls += [f]

    con_rad = [f.pos for f in flights if f.pos not in dests and f!=flight]

    pos = flight.pos
    # visualization coord starts from 1 but in simulation it is 0, hence the need to reduce pos by 1 for simulation
    fp = [pos[0]-1,pos[1]-1]
    rf = field

    npi = nxt_pos(rf,fp)

    # rf[pos[0]][pos[1] + 1] == pt + 1 former concept
    # no movement
    if npi==0:
        flight.pot = rf[fp[1]][fp[0]]
        return
    # Right
    if npi==1 and [pos[0],pos[1] + 1] not in con_rad:
        pos[1]=pos[1]+1
        flight.pot = rf[fp[1]+1][fp[0]]
        return
    # Left
    if npi==5 and [pos[0],pos[1] - 1] not in con_rad:
        pos[1]=pos[1] - 1
        flight.pot = rf[fp[1]-1][fp[0]]
        return
    # Down
    if npi==3 and [pos[0]+1,pos[1]] not in con_rad:
        pos[0]=pos[0] + 1
        flight.pot = rf[fp[1]][fp[0]+1]
        return
    # Up
    if npi==7 and [pos[0]-1,pos[1]] not in con_rad:
        pos[0]=pos[0] - 1
        flight.pot = rf[fp[1]][fp[0]-1]
        return
    # Up-right
    if npi==8 and [pos[0] - 1,pos[1] + 1] not in con_rad:
        pos[0] = pos[0] - 1
        pos[1] = pos[1] + 1
        flight.pot = rf[fp[1]+1][fp[0]-1]
        return
    # Down-right
    if npi==2 and [pos[0] + 1,pos[1] + 1] not in con_rad:
        pos[0] = pos[0] + 1
        pos[1] = pos[1] + 1
        flight.pot = rf[fp[1]+1][fp[0]+1]
        return
    # Up-left
    if npi==6 and [pos[0] - 1,pos[1] - 1] not in con_rad:
        pos[0] = pos[0] - 1
        pos[1] = pos[1] - 1
        flight.pot = rf[fp[1]-1][fp[0]-1]
        return
    # Down-left
    if npi==4 and [pos[0] + 1,pos[1] - 1] not in con_rad:
        pos[0] = pos[0] + 1
        pos[1] = pos[1] - 1
        flight.pot = rf[fp[1]-1][fp[0]+1]
        return


'''User Interaction'''
def disp_ran_acraft_info():
    global acraft_info
    for n in range(len(acraft_info)):
        print(f'a{n+1}: {acraft_info[n]}')

def rand_acraft_info():
    global acraft_info
    return acraft_info

def rand_ac_arg(grid_size,max_pot,max_size,plt_colors,side=None):
    row=None
    col=None
    if side=='up':
        row = 1
        col = random.randint(1, grid_size[1])
    if side=='down':
        row = grid_size[0]
        col = random.randint(1, grid_size[1])
    if side=='left':
        row = random.randint(1, grid_size[0])
        col = 1
    if side=='right':
        row = random.randint(1, grid_size[0])
        col = grid_size[1]
    if not side:
        row = random.randint(1, grid_size[0])
        col = random.randint(1, grid_size[1])
    mp = random.randint(max_pot // 2, max_pot)
    ms = random.randint(1, max_size)
    pc = random.choice(plt_colors)
    return row,col,mp,ms,pc

# create aircrafts by category, this function is used to create multiple aircrafts
def cat_aircrafts(no_aircrafts,grid_size, max_pot, max_size, plt_colors, side=None, null_pont=False):
    global acraft_info,a_cord
    acrafts=[]
    a_info=[]
    for _ in range(no_aircrafts):
        # one cell to one aircraft
        while True:
            row, col, mp, ms, pc = rand_ac_arg(grid_size, max_pot, max_size, plt_colors, side)
            if [row, col] not in a_cord:
                a_cord += [[row, col]]
                acrafts += [Aircraft(row=row, column=col, max_pot=mp, grid_size=grid_size, size=ms, plt_color=pc,
                                     null_pont=null_pont)]
                a_info += [{'row': row, 'column': col, 'max_pot': mp, 'max_size': ms, 'plt_color': pc}]
                break
    acraft_info += a_info
    return acrafts

def multiple_aircrafts(max_pot,grid_size,max_size,plt_colors,rand_aircrafts=None,start_sides=None,null_pont=False):
    acrafts=[]
    # check starting sides declared and number of starting sides for given aircrafts
    if start_sides:
        sides = start_sides.keys()
        for side in sides:
            no_aircrafts=start_sides[side]
            acrafts+=cat_aircrafts(no_aircrafts,grid_size,max_pot,max_size,plt_colors,side,null_pont)

    # create random aircrafts from any sides
    if rand_aircrafts:
        acrafts+=cat_aircrafts(rand_aircrafts, grid_size, max_pot, max_size, plt_colors, null_pont)

    return acrafts


'''Simulation'''
# to implement boundary condition for cycling aircraft movement
# at destination, flight starts again from closest to first starting point
def cyc_sim(field,fl):
    len_lst = [fl.g_size[0] - fl.dept[0], fl.g_size[1] - fl.dept[1], fl.dept[0], fl.dept[1]]
    prox = min(len_lst)

    if prox==len_lst[0]:
        fl.pos[0], fl.pos[1] = fl.g_size[0], fl.dept[1]
    elif prox==len_lst[1]:
        fl.pos[0], fl.pos[1] = fl.dept[0], fl.g_size[1]
    elif prox==len_lst[2]:
        fl.pos[0], fl.pos[1] = 1, fl.dept[1]
    elif prox==len_lst[3]:
        fl.pos[0], fl.pos[1] = fl.dept[0], 1
    # set new flight positions' potential
    # visualization coords starts from 1 but in simulation it is 0, hence the need to reduce pos by 1 for simulation
    fl.pot=field[fl.pos[1]-1][fl.pos[0]-1]

def sim_field_gen(tma,objs,mov_obstructions):
    objs=objs+mov_obstructions
    potns = [fd.fld_pot for fd in objs]

    tma.fld = reslt_pot(potns)
    tma.max_pot = max([n for m in tma.fld for n in m])

    dests = col_dest(tma.fld, tma.max_pot)
    return dests

def mov_obstr_sim(tma,objs,mov_obstructions,t_step):
    global dests
    for mob in mov_obstructions:
        # Encapsulated attributes for mobility and mutation handles cases of no mobility or mutation
        # however if-and condition is to ensure unnecessary call to the obstructions' mobility and mutation functions
        # mobility
        if t_step % mob.mob_rate==0 and mob.mob_rate!=1 and mob.mob_span!=0:
            mob.mov_obs()
        # mutation
        if t_step % mob.mut_rate==0 and mob.mut_rate!=1 and mob.mut_span!=0:
            mob.mut_obs()
    dests = []
    dests = sim_field_gen(tma, objs, mov_obstructions)

def plot_vis(t_step,tma):
    # plot visualization
    xat = [i for i in range(len(tma[0]))]
    yat = [i for i in range(len(tma))]
    xal = [i + 1 for i in range(len(tma[0]))]
    yal = [i + 1 for i in range(len(tma))]
    plt.xticks(xat, xal)
    plt.yticks(yat, yal)
    plt.tick_params(left=False, bottom=False, labelbottom=False, labelleft=False)
    plt.imshow(np.array(tma), cmap='binary')
    plt.colorbar()
    # plt.show()
    images_file = os.path.join(vs.image_dir, f'plot_{t_step}.png')
    plt.savefig(images_file)
    plt.close()

def sim_iter(flights,waypoints,stat_obstructions,mov_obstructions,tma,show_vis_clip,total_tstep=1):
    global dests,loc_tma

    col_dept(flights)

    # values of tma are set in the sim_field_gen method
    objs = flights + waypoints + tma + stat_obstructions
    dests = sim_field_gen(tma[0],objs,mov_obstructions)
    # capture first movement
    if show_vis_clip:
        fl_paths(flights, 0)
        tma[0].store_sim_path(tma[0].fld,flights,0)
        # plot_vis(0, flights, tma)

    if total_tstep>1:
        for t in range(1,total_tstep+1):
            mov_obstr_sim(tma[0],objs,mov_obstructions,t+1)
            for flight in flights:
                # if-else: Cycle flight/boundary condition and flight movement
                '''
                Update: single expected destination can be specified by user for situations
                where mutiple points have maximum potential. flight.pos==dests conditon can be included for this
                '''
                if flight.pot==tma[0].max_pot:
                    cyc_sim(tma[0].fld,flight)
                    flight.disp_agg_pos = []
                else:
                    # aircraft movement
                    conf_resl(tma[0].fld,flights,flight)
                flight.collect_pos()
                flight.collect_distn()

            # plot visualization
            if show_vis_clip:
                fl_paths(flights, t)
                tma[0].store_sim_path(tma[0].fld,flights, t)
                # plot_vis(t,flights,tma)

    t=0
    if total_tstep==1:
        while True:
            t+=1
            mov_obstr_sim(tma[0],objs,mov_obstructions,t)
            for flight in flights:
                conf_resl(tma[0].fld,flights,flight)
                flight.collect_pos()
                flight.collect_distn()

            # stop simulation when destination is reached
            if all([f.pot==tma[0].max_pot for f in flights]):
                break

            # plot visualization
            if show_vis_clip:
                pass
                # tma[0].store_sim_path(flights, clip_no)
                # plot_vis(t, flights, tma)

    # get field density
    tma[0].fd_dens(flights)

    # get average transit time of flights
    # derive sim_agg_pos first as it is needed to get transit time
    des=swi_cord_elem(dests)
    for f in flights:
        f.coll_sim_agg_pos()
        f.no_journ,f.avg_tnstime = avg_trans_time(f.sim_agg_pos,des)
        tma[0].transit_time+=f.avg_tnstime
    tma[0].avg_transit_time=tma[0].transit_time/len(flights)

    # av_distn(tma,flights) average distance func needs update
    vel_per_t(tma[0],flights)

    # set tma as a global variable loc_tma for further use acros the simulation
    loc_tma=tma[0]
    # create simulation video
    # if show_vis_clip:
    #     vs.make_video(f'potential{clip_no}.mp4')

def simulate(flights,waypoints,stat_obstructions,mov_obstructions,tma,show_vis_clip,total_tstep=1):
    # initialize visualization
    vs.clear_image_folder(vs.image_dir)

    sim_iter(flights,waypoints,stat_obstructions,mov_obstructions,tma,show_vis_clip,total_tstep)

