from math import dist


hov_fli=0

'''Conflicts Resolution'''
# resolving an object's conflict radius (size=1)
def obj_radius(size, pos):
    rad=[pos]

    for m in range(size):
        rn=len(rad)
        for n in range(rn):
            nrad=[]
            if [rad[n][0],rad[n][1]+1] not in rad:
                nrad+=[[rad[n][0],rad[n][1]+1]]
            if [rad[n][0]+1,rad[n][1]+1] not in rad:
                nrad+=[[rad[n][0]+1,rad[n][1]+1]]
            if [rad[n][0]+1,rad[n][1]] not in rad:
                nrad+=[[rad[n][0]+1,rad[n][1]]]
            if [rad[n][0]+1,rad[n][1]-1] not in rad:
                nrad+=[[rad[n][0]+1,rad[n][1]-1]]
            if [rad[n][0],rad[n][1]-1] not in rad:
                nrad+=[[rad[n][0],rad[n][1]-1]]
            if [rad[n][0]-1,rad[n][1]-1] not in rad:
                nrad+=[[rad[n][0]-1,rad[n][1]-1]]
            if [rad[n][0]-1,rad[n][1]] not in rad:
                nrad+=[[rad[n][0]-1,rad[n][1]]]
            if [rad[n][0]-1,rad[n][1]+1] not in rad:
                nrad+=[[rad[n][0]-1,rad[n][1]+1]]
            rad+=nrad
    return rad

def conv_to_2d(arr,col):
    darr= [arr[i:i+col] for i in range(0,len(arr),col)]
    return darr

# to collect all possible destination of flight
def poss_temp_dest(flight,wpp):
    pt_dest = [flight.t_down] + wpp
    return pt_dest

# determine coord. for best(considering waypoints & destination) path for flight
def cord_best_path(flight):
    wpp=flight.way_p
    pcord = flight.pos
    dest = poss_temp_dest(flight,wpp)
    # shortest distance from departure whether waypoint or destination
    # sort waypoint flow
    dest_sorted = sorted(wpp, key=lambda y: dist(pcord,y))
    # sorted(list(map(lambda y: dist(pcord,y),wpp)))
    
    if flight.trajectory == 'short':
        return [flight.t_down]
    elif flight.trajectory == 'medium':
        c_shdist = dest_sorted[0:-2]+[flight.t_down]
        # c_shdist = [i for i in dest if dist(pcord,i)==sht_dist][0]
    elif flight.trajectory == 'long':
        c_shdist = dest_sorted + [flight.t_down]
    '''Update if flight should go straight to dest or wp dependent on distance'''
    # wp_des  = dist(c_shdist,flight.dest)
    # dep_des = dist(flight.pos,flight.dest)

    return c_shdist
    # flight with shortest distance, to use variable later
    # sht_dist = min(map(lambda x,y: dist(x,y), pcord,dest))

# to define conflict radius of flights
def objs_con_rad(objects,con_rad):

    # if-statement is so flight objects at destination have no conflict radius
    for obj in objects:
        if obj.pos != obj.dest:
            obj.con_rad=obj_radius(obj.size,obj.pos)
        else:
            obj.con_rad=[]
        con_rad+=obj.con_rad


'''Flight Navigation'''
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
def conf_flight_movement(flights,fl,obs_pos,des_conf_r):
    fls=[f for f in flights if f != fl]

    # one step destination of flight
    # temp_des = cord_best_path(fl)
    # fl.tdes = temp_des

    pos=fl.pos
    print('flight',fl.size,' ',fl.pos,fl.way_p[0])
    fl.tdes=tdes=fl.way_p[0]
    des=fl.dest

    # define conflict radius around destination
    con_des=obj_radius(des_conf_r,des)

    if pos == des:
        return

    global con_rad
    global hov_fli

    con_rad=[]
    objs_con_rad(fls,con_rad)

    # add pos of obstructions to con_rad
    con_rad+=obs_pos

    # remove flight dest from con_rad
    con_rad=[c for c in con_rad if c!=des]

    # best path SW
    if tdes[0] - pos[0] < 0 and tdes[1] - pos[1] < 0:
        # SW
        if [pos[0] - 1, pos[1] - 1] not in con_rad:
            SW(pos)
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # WW
        if [pos[0] - 1, pos[1]] not in con_rad:
            WW(pos)
            return
        # NW
        if [pos[0] - 1, pos[1] + 1] not in con_rad:
            NW(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            if des_conf_r==0:
                pos[0],pos[1]=des
                hov_fli += 1
            else:
                hov_fli += 1
            return
        # SE
        if [pos[0] + 1, pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # NN
        if [pos[0], pos[1] + 1] not in con_rad:
            NN(pos)
            return
        # EE
        if [pos[0] + 1, pos[1]] not in con_rad:
            EE(pos)
            return
        # NE
        if [pos[0] + 1, pos[1] + 1] not in con_rad:
            NE(pos)
            return
        else:
            return

    # best path WW
    if tdes[0] - pos[0] < 0 and tdes[1] - pos[1] == 0:
        # WW
        if [pos[0] - 1, pos[1]] not in con_rad:
            WW(pos)
            return
        # SW
        if [pos[0] - 1, pos[1] - 1] not in con_rad:
            SW(pos)
            return
        # NW
        if [pos[0] - 1, pos[1] + 1] not in con_rad:
            NW(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            if des_conf_r==0:
                pos[0],pos[1]=des
                hov_fli += 1
            else:
                hov_fli += 1
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # NN
        if [pos[0], pos[1] + 1] not in con_rad:
            NN(pos)
            return
        # SE
        if [pos[0] + 1, pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # NE
        if [pos[0] + 1, pos[1] + 1] not in con_rad:
            NE(pos)
            return
        # EE
        if [pos[0] + 1, pos[1]] not in con_rad:
            EE(pos)
            return
        else:
            return

    # best path NW
    if tdes[0] - pos[0] < 0 and tdes[1] - pos[1] > 0:
        # NW
        if [pos[0] - 1, pos[1] + 1] not in con_rad:
            NW(pos)
            return
        # WW
        if [pos[0] - 1, pos[1]] not in con_rad:
            WW(pos)
            return
        # NN
        if [pos[0], pos[1] + 1] not in con_rad:
            NN(pos)
            return
        # SW
        if [pos[0] - 1, pos[1] - 1] not in con_rad:
            SW(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            if des_conf_r==0:
                pos[0],pos[1]=des
                hov_fli += 1
            else:
                hov_fli += 1
            return
        # NE
        if [pos[0] + 1, pos[1] + 1] not in con_rad:
            NE(pos)
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # EE
        if [pos[0] + 1, pos[1]] not in con_rad:
            EE(pos)
            return
        # SE
        if [pos[0] + 1, pos[1] - 1] not in con_rad:
            SE(pos)
            return
        else:
            return

    # best path NN
    if tdes[0] - pos[0] == 0 and tdes[1] - pos[1] > 0:
        # NN
        if [pos[0], pos[1] + 1] not in con_rad:
            NN(pos)
            return
        # NW
        if [pos[0] - 1, pos[1] + 1] not in con_rad:
            NW(pos)
            return
        # NE
        if [pos[0] + 1, pos[1] + 1] not in con_rad:
            NE(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            if des_conf_r==0:
                pos[0],pos[1]=des
                hov_fli += 1
            else:
                hov_fli += 1
            return
        # WW
        if [pos[0] - 1, pos[1]] not in con_rad:
            WW(pos)
            return
        # EE
        if [pos[0] + 1, pos[1]] not in con_rad:
            EE(pos)
            return
        # SW
        if [pos[0] - 1, pos[1] - 1] not in con_rad:
            SW(pos)
            return
        # SE
        if [pos[0] + 1, pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        else:
            return

    # best path NE
    if tdes[0] - pos[0] > 0 and tdes[1] - pos[1] > 0:
        # NE
        if [pos[0] + 1, pos[1] + 1] not in con_rad:
            NE(pos)
            return
        # EE
        if [pos[0] + 1, pos[1]] not in con_rad:
            EE(pos)
            return
        # NN
        if [pos[0], pos[1] + 1] not in con_rad:
            NN(pos)
            return
        # SE
        if [pos[0] + 1, pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            if des_conf_r==0:
                pos[0],pos[1]=des
                hov_fli += 1
            else:
                hov_fli += 1
            return
        # NW
        if [pos[0] - 1, pos[1] + 1] not in con_rad:
            NW(pos)
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # WW
        if [pos[0] - 1, pos[1]] not in con_rad:
            WW(pos)
            return
        # SW
        if [pos[0] - 1, pos[1] - 1] not in con_rad:
            SW(pos)
            return
        else:
            return

    # best path EE
    if tdes[0] - pos[0] > 0 and tdes[1] - pos[1] == 0:
        # EE
        if [pos[0] + 1, pos[1]] not in con_rad:
            EE(pos)
            return
        # SE
        if [pos[0] + 1, pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # NE
        if [pos[0] + 1, pos[1] + 1] not in con_rad:
            NE(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            if des_conf_r == 0:
                pos[0],pos[1] = des
                hov_fli += 1
            else:
                hov_fli += 1
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # NN
        if [pos[0], pos[1] + 1] not in con_rad:
            NN(pos)
            return
        # SW
        if [pos[0] - 1, pos[1] - 1] not in con_rad:
            SW(pos)
            return
        # NW
        if [pos[0] - 1, pos[1] + 1] not in con_rad:
            NW(pos)
            return
        # WW
        if [pos[0] - 1, pos[1]] not in con_rad:
            WW(pos)
            return
        else:
            return

    # best path SE
    if tdes[0] - pos[0] > 0 and tdes[1] - pos[1] < 0:
        # SE
        if [pos[0] + 1, pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # EE
        if [pos[0] + 1, pos[1]] not in con_rad:
            EE(pos)
            return
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # NE
        if [pos[0] + 1, pos[1] + 1] not in con_rad:
            NE(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            if des_conf_r == 0:
                pos[0],pos[1] = des
                hov_fli += 1
            else:
                hov_fli += 1
            return
        # SW
        if [pos[0] - 1, pos[1] - 1] not in con_rad:
            SW(pos)
            return
        # NN
        if [pos[0], pos[1] + 1] not in con_rad:
            NN(pos)
            return
        # NW
        if [pos[0] - 1, pos[1] + 1] not in con_rad:
            NW(pos)
            return
        # WW
        if [pos[0] - 1, pos[1]] not in con_rad:
            WW(pos)
            return

    # best path SS
    if tdes[0] - pos[0] == 0 and tdes[1] - pos[1] < 0:
        # SS
        if [pos[0], pos[1] - 1] not in con_rad:
            SS(pos)
            return
        # SE
        if [pos[0] + 1, pos[1] - 1] not in con_rad:
            SE(pos)
            return
        # SW
        if [pos[0] - 1, pos[1] - 1] not in con_rad:
            SW(pos)
            return
        # Hover (for conflict around destination)
        if pos in con_des:
            if des_conf_r==0:
                pos[0],pos[1] = des
                hov_fli += 1
            else:
                hov_fli += 1
            return
        # EE
        if [pos[0] + 1, pos[1]] not in con_rad:
            EE(pos)
            return
        # WW
        if [pos[0] - 1, pos[1]] not in con_rad:
            WW(pos)
            return
        # NE
        if [pos[0] + 1, pos[1] + 1] not in con_rad:
            NE(pos)
            return
        # NW
        if [pos[0] - 1, pos[1] + 1] not in con_rad:
            NW(pos)
            return
        # NN
        if [pos[0], pos[1] + 1] not in con_rad:
            NN(pos)
            return
        else:
            return
