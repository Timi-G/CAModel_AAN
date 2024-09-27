import math
import random

from cam_airnavconfrules import obj_radius


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
def nxt_pos(field,pos,con_rad):
    fd=field
    crds= obj_radius(1,pos)
    cords=[]

    # in case of -ve or zero dimen. in coord., change to original pos
    for c in crds:
        if all([_>0 for _ in c]):
            cords+=[c]
        else:
            cords+=[pos]
    # actual cords start from 0 by python convention though user-defined starts from 1
    fd_cords=[[c[0]-1,c[1]-1] for c in cords]
    vals=pos_vals(fd,fd_cords) # vals content sequence is clockwise i.e pos,E,SE,S,...,NE
    mvals=[i-vals[0] for i in vals]

    '''optimize selection of next position'''
    # get all valid positions: positions with same potentials which is the nearest higher potential than current potential

    # try stmnt to keep a_craft in a pos when potn around it is lesser or same
    try:
        valid_pot = min([j for j in mvals if j > 0])
        npos_idxs = [n for n, v in enumerate(mvals) if v == valid_pot]
    except:
        valid_pot = min([j for j in mvals if j == 0])
        npos_idxs = [n for n, v in enumerate(mvals) if v == valid_pot]

    # useful for getting coords of next possible position of flight
    npos_cords = [cords[npos_idx] for npos_idx in npos_idxs if cords[npos_idx] not in con_rad]
    # check if next position cords contains flight pos and remove flight pos
    npos_cords = [np for np in npos_cords if np != pos]
    # if npos_cords is empty because all npos_cords are flight pos, set next pos to flight pos
    npos_cords = npos_cords if len(npos_cords) else [pos]
    return npos_cords

# pos: position of flight in visualized field
# rpos: position of flight
def ac_movement(field,pos,rpos,npi,con_rad):
    # rf[pos[0]][pos[1] + 1] == pt + 1 former concept
    # no movement
    if npi==0:
        pot = field[rpos[1]][rpos[0]]
    # Right
    elif npi==1 and [pos[0],pos[1] + 1] not in con_rad:
        pos[1]=pos[1]+1
        pot = field[rpos[1]+1][rpos[0]]
    # Left
    elif npi==5 and [pos[0],pos[1] - 1] not in con_rad:
        pos[1]=pos[1] - 1
        pot = field[rpos[1]-1][rpos[0]]
    # Down
    elif npi==3 and [pos[0]+1,pos[1]] not in con_rad:
        pos[0]=pos[0] + 1
        pot = field[rpos[1]][rpos[0]+1]
    # Up
    elif npi==7 and [pos[0]-1,pos[1]] not in con_rad:
        pos[0]=pos[0] - 1
        pot = field[rpos[1]][rpos[0]-1]
    # Up-right
    elif npi==8 and [pos[0] - 1,pos[1] + 1] not in con_rad:
        pos[0] = pos[0] - 1
        pos[1] = pos[1] + 1
        pot = field[rpos[1]+1][rpos[0]-1]
    # Down-right
    elif npi==2 and [pos[0] + 1,pos[1] + 1] not in con_rad:
        pos[0] = pos[0] + 1
        pos[1] = pos[1] + 1
        pot = field[rpos[1]+1][rpos[0]+1]
    # Up-left
    elif npi==6 and [pos[0] - 1,pos[1] - 1] not in con_rad:
        pos[0] = pos[0] - 1
        pos[1] = pos[1] - 1
        pot = field[rpos[1]-1][rpos[0]-1]
    # Down-left
    elif npi==4 and [pos[0] + 1,pos[1] - 1] not in con_rad:
        pos[0] = pos[0] + 1
        pos[1] = pos[1] - 1
        pot = field[rpos[1]-1][rpos[0]+1]
    else:
        pot = field[rpos[1]][rpos[0]]
    return pot
