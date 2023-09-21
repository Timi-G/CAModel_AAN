import math
import random

import numpy as np
from matplotlib import pyplot as plt

from cam_air_nav.cam_airnav_mod import Air_Object, Free_Air_Object, avg_trans_time, col_dept, path_plots
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
         self.tfp = []

class Oth_Obj_Field(Free_Air_Object):

    def __init__(self,row,column,max_pot,grid_size,size):
        mp_pos = [column, row]
        super(Oth_Obj_Field, self).__init__(size,[mp_pos[0], mp_pos[1]])

        self.g_size = [grid_size[1],grid_size[0]]
        self.max_pot = max_pot
        self.pot = 0
        self.fld_pot = res_field(self.g_size, mp_pos, max_pot)

class TMA:
    def __init__(self,row,column,max_pot,grid_size):
        self.mp_pos = [column, row]
        self.g_size = [grid_size[1], grid_size[0]]
        self.max_pot = max_pot
        self.fld_pot = res_field(self.g_size,self.mp_pos,max_pot)
        self.fld = []
        self.dens = 0
        self.tot_tstep = 0
        self.vel = []
        self.avg_vel = 0

    def fd_dens(self,objs):
        self.dens=len(objs)/(self.g_size[0]*self.g_size[1])

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

        self.agg_pos=conv_to_2d(self.agg_pos,2)
        self.agg_pos=[[m[1],m[0]] for m in self.agg_pos]


class Obstruction(Oth_Obj_Field):
    def __init__(self,**kwargs):
        super(Obstruction, self).__init__(**kwargs)
        # implement obstructions to have repulsive forces
        self.fld_pot = change_anydim_lst_sign(self.fld_pot,change_sign)

class Stat_Obstruction(Obstruction):
    pass

class Mov_Obstruction(Obstruction):
    def __init__(self,travel_rate,**kwargs):
        super(Mov_Obstruction, self).__init__(**kwargs)
        self.trav_rate=travel_rate

    def mov_obs(self):
        self.pos=[random.randint(1,self.g_size[0]),random.randint(1,self.g_size[1])]
        self.fld_pot=res_field(self.g_size,self.pos,self.max_pot)

class Waypoint(Oth_Obj_Field):
    pass

class Point(Oth_Obj_Field):
    pass

con_rad=[]
dests=[]
acraft_info=[]
a_cord=[]

'''Abstract Modifiers/Correctors'''
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
        path_plots(flights)
        # handle duplicates in agg_pos
        for f in flights:
            con = []
            for el in f.agg_pos:
                con += [el]
                if el in pdes:
                    f.agg_pos = con
                    break

    elif total_tstep>1:
        for f in flights:
            f.flight_path()
            # conv. flight path coord to list
            pat= [list(_) for _ in zip(*f.fp.tolist())]
            rem_dup(pat,dess,path_plot,f.color)


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
def col_dest(tma,max_pot):
    dests = []
    for m in range(len(tma)):
        for n in range(len(tma[0])):
            if tma[m][n]==max_pot:
                dests+=[[n+1,m+1]]
    return dests

# collect number of flights moving per t_step
def col_fl_per_t(tma,flights):
    ftl=[]

    # create list of all agg_pos
    t_agpos=[flight.agg_pos for flight in flights]
    tma.tot_tstep=max([len(ts) for ts in t_agpos])
    # try..except to handle n which is beyond index for list elements
    for n in range(tma.tot_tstep):
        ft=0
        for m in t_agpos:
            try:
                if m[n] != m[n+1]:
                    ft+=1
            except:
                pass
        ftl+=[ft]
    return ftl

def vel_per_t(tma,flights):
    no_fl = col_fl_per_t(tma,flights)

    velocity = [nf / len(flights) for nf in no_fl]

    tma.vel = velocity
    tma.avg_vel = sum(tma.vel)/tma.tot_tstep


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
        if all(_>=0 for _ in c):
            cords+=[c]
        else:
            cords+=[crds[0]]

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

def rand_ac_arg(grid_size,max_pot,max_size,plt_colors):
    row = random.randint(1, grid_size[0])
    col = random.randint(1, grid_size[1])
    mp = random.randint(max_pot // 2, max_pot)
    ms = random.randint(1, max_size)
    pc = random.choice(plt_colors)
    return row,col,mp,ms,pc

def rand_aircrafts(no_aircrafts,max_pot,grid_size,max_size,plt_colors,null_pont=False):
    global acraft_info,a_cord
    acrafts=[]
    a_info=[]
    for _ in range(no_aircrafts):
    # one cell to one aircraft
        while True:
            row, col, mp, ms, pc = rand_ac_arg(grid_size, max_pot, max_size, plt_colors)
            if [row,col] not in a_cord:
                a_cord+=[[row,col]]
                acrafts+=[Aircraft(row=row,column=col,max_pot=mp,grid_size=grid_size,size=ms,plt_color=pc,null_pont=null_pont)]
                a_info += [{'row': row, 'column': col, 'max_pot': mp, 'max_size': ms, 'plt_color': pc}]
                # print(a_info)
                break
    acraft_info+=a_info
    return acrafts


'''Simulation'''
# to implement boundary condition for cycling aircraft movement
# at destination, flight starts again from closest to first starting point
def cyc_sim(fl):
    len_lst = [fl.g_size[0] - fl.dept[0], fl.g_size[1] - fl.dept[1], fl.dept[0], fl.dept[1]]
    prox = min(len_lst)

    if prox==len_lst[0]:
        fl.pos[0], fl.pos[1] = fl.g_size[0], fl.dept[1]
    elif prox==len_lst[1]:
        fl.pos[0], fl.pos[1] = fl.dept[0], fl.g_size[1]
    elif prox==len_lst[2]:
        fl.pos[0], fl.pos[1] = 0, fl.dept[1]
    elif prox==len_lst[3]:
        fl.pos[0], fl.pos[1] = fl.dept[0], 0

def sim_field_gen(tma,objs,mov_obstructions):
    global dests
    objs=objs+mov_obstructions
    potns = [fd.fld_pot for fd in objs]

    tma.fld = reslt_pot(potns)
    tma.max_pot = max([n for m in tma.fld for n in m])

    dests = col_dest(tma.fld, tma.max_pot)

def mov_obstr_sim(tma,objs,mov_obstructions,t_step):
    global dests
    for mob in mov_obstructions:
        if t_step % mob.trav_rate==0:
            mob.mov_obs()
            dests = []
            sim_field_gen(tma,objs,mov_obstructions)

def sim_iter(flights,waypoints,stat_obstructions,mov_obstructions,tma,total_tstep=1):
    global dests

    col_dept(flights)

    objs = flights + waypoints + tma + stat_obstructions
    sim_field_gen(tma[0],objs,mov_obstructions)

    if total_tstep>1:
        for t in range(total_tstep):
            mov_obstr_sim(tma[0],objs,mov_obstructions,t+1)
            for flight in flights:
                conf_resl(tma[0].fld,flights,flight)
                flight.collect_pos()
                flight.collect_distn()

                # cycle flight/boundary condition
                if flight.pot==tma[0].max_pot:
                    cyc_sim(flight)
                    flight.collect_pos()

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

    # get field density
    tma[0].fd_dens(flights)

    # plot visualization
    fl_paths(flights,total_tstep)
    xat=[i for i in range(len(tma[0].fld[0]))]
    yat= [i for i in range(len(tma[0].fld))]
    xal=[i+1 for i in range(len(tma[0].fld[0]))]
    yal=[i+1 for i in range(len(tma[0].fld))]
    plt.xticks(xat,xal)
    plt.yticks(yat,yal)
    plt.imshow(np.array(tma[0].fld),cmap='binary')
    plt.colorbar()
    plt.show()

    # get average transit time of flights
    des=swi_cord_elem(dests)
    for f in flights:
        f.avg_tnstime = avg_trans_time(f.agg_pos,des)

    # av_distn(tma,flights) average distance func needs update
    vel_per_t(tma[0],flights)


def simulate(flights,waypoints,stat_obstructions,mov_obstructions,tma,total_tstep=1):

    sim_iter(flights,waypoints,stat_obstructions,mov_obstructions,tma,total_tstep)

