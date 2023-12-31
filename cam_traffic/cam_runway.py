import random
import time


class Aircraft:
    def __init__(self,size,fuel):
        self.size = size
        self.fuel = fuel

    def fuel_depl(self):
        self.fuel=self.fuel-(self.size//2)

    def size_fuel_prop(self):
        siz_2_fue= [
                    [1,500,1000],
                    [2,1000,2000],
                    [3,2000,4000],
                    [4,4000,8000]
                     ]

class Runway:
    def __init__(self, no_cells, runway_group, mov_prob=100):
        self.no_cells = no_cells
        self.runway_group = runway_group
        self.runway_flights = [0] * no_cells
        self.cell1_mov = 0
        self.lvl = 0
        self.mov_prob = mov_prob
        self.exit = False
        self.ent = False
        self.tot_op_fl = 0
        self.tot_mov = 0
        self.agg_op_fl = []
        self.agg_dens = []


    def exit_runway(self):
        rw_fl = self.runway_flights
        exit_flight = rw_fl[-1]
        self.exit_flight = exit_flight

    def mov_per_tstep(self):
        self.moved = False
        rw_fl = self.runway_flights
        # return if runway is empty
        if rw_fl == [0] * self.no_cells:
            return
        # movement only when there is space in last cell
        if rw_fl[-1] == 0 or self.exit:
            mov_by_prob = random.choices([1, 0], weights=[self.mov_prob, 100 - self.mov_prob], k=1)[0]
            if mov_by_prob == 0:
                return
            for n in range(-1, -len(rw_fl), -1):
                self.runway_flights[n] = rw_fl[n - 1]
            self.runway_flights[0] = 0
            # calc no of aircraft that successfully move
            self.tot_mov += 1
            # confirm for entry func that movement occurred
            self.moved = True

            # check movement in first cell for cellular section_flow calc
            # cell_2 will be occupied by aircraft initially in cell_1
            if rw_fl[1] != 0:
                self.cell1_mov += 1

    def entry_runway(self, flight):
        rw_fl = self.runway_flights
        # entry is only for an aircraft
        if flight == 0:
            self.ent = False
            return
        # entry to an empty runway
        if rw_fl == [0] * self.no_cells:
            self.ent = True
            self.runway_flights[0] = flight
            return
        # entry
        for dx, n in enumerate(rw_fl):
            if n != 0:
                spacing = self.spacing_rules(flight.size, n.size)
                if dx <= spacing:
                    self.ent = False
                    return
                elif dx > spacing:
                    self.ent = True
                    self.runway_flights[0] = flight
                    # runway first cell gets occupied whenever there is an entry
                    self.cell1_mov += 1
                    return

    def spacing_rules(self, a, b):

        spacing_table = [
            [3, 3, 3, 3],
            [5, 3, 3, 3],
            [6, 5, 4, 3],
            [8, 7, 6, 3]
        ]
        spacing = spacing_table[a - 1][b - 1]
        return spacing

    # to compute extra spacing
    def extra_spacing(self):
        r_fl = self.runway_flights
        tot_ext_space = 0
        ext_space = 0
        prev_acs = [0] #[0] allows extra space to be derived when r_fl[0] is an aircraft
        bet_spc = []
        ac = []

        for a in r_fl:
            # collect first non_zero sized aircraft
            if a == 0:
                prev_acs += [a]
            if all([n == 0 for n in prev_acs]) and a != 0:
                ac += [a]
                prev_acs = []

            # collect spaces between aircraft
            if len(ac) == 1 and a == 0:
                bet_spc += [a]
            # get extra spaces
            if len(ac) > 1:
                space = self.spacing_rules(ac[0].size,ac[1].size)
                ext_space = len(bet_spc) - space
                ac = []
                bet_spc = []

            tot_ext_space += ext_space
            ext_space = 0
        self.tot_ext_space = tot_ext_space

    def get_tot_op_fl(self):
        op_fl_per_t = len([r for r in self.runway_flights if r != 0])
        self.agg_op_fl += [op_fl_per_t]

    def density(self,t_step):
        self.agg_dens=[c/self.no_cells for c in self.agg_op_fl]
        self.avg_dens=sum(self.agg_dens)/t_step


class Traffic_Control:

    def __init__(self):
        self.landing=[]

    def create_pool(self, size_dist):
        self.no_pool_cont = sum(size_dist)
        self.main_pool = self.create_aircraft(size_dist)
        random.shuffle(self.main_pool)

    def create_aircraft(self,sd):
        t_rwc=self.tot_rwcells
        thr_trwc=t_rwc//3  #trwc/t_rwc: total runway cells

        pool=[]
        for dx,ac in enumerate(sd,1):
            for n in range(ac):
                # ensuring fuel is proportional to aircraft size using dx
                pool+=[Aircraft(size=dx,fuel=random.randint(t_rwc+(2*thr_trwc)*dx,t_rwc*2*dx))]
        return pool

    def add_runway(self, level1_structure):
        # level1_structure should be in format [[50,35,60],[25,50,40],[45,50]]
        # in the above example, there are 3 groups and 8 runways
        # innermost lists represent groups 1,2,3... from left to right
        # and integers represent number of cells in each runway
        self.lvl1_struct = level1_structure
        self.tot_rwcells=sum([sum(rw) for rw in self.lvl1_struct])

    def test_runway(self):
        run = Runway(no_cells=50, runway_group=2)
        self.ipool = [r for r in self.main_pool]

        for t in range(100):
            # flight exits runway
            run.exit_runway()
            run.exit = True
            if run.exit and run.exit_flight != 0:
                self.main_pool = [run.exit_flight] + self.main_pool

            # flights move forward in runway
            run.mov_per_tstep()

            # flights enter runway from pool
            ent = self.main_pool.pop(-1)
            run.entry_runway(ent)
            # if runway rejects flight entry, return aircraft to pool
            if not run.ent:
                self.main_pool.insert(-1, ent)
            run.exit = False

    # change a level config. from groups in a list to list of lists
    def part_lvl(self, lvl):
        n_rw = len(lvl)
        lv = []
        rnway = []
        try:
            for n in range(n_rw):
                rnway += [lvl[n]]
                if lvl[n].runway_group != lvl[n + 1].runway_group:
                    lv += [rnway]
                    rnway = []
        except:
            if lvl[-1].runway_group == lvl[-2].runway_group:
                lv += [rnway]
            else:
                lv += [[lvl[-1]]]
        return lv

    def part_lvl_a(self, lvl):
        n_rw = len(lvl)
        lv = []
        rnway = []
        try:
            for n in range(n_rw):
                rnway += [lvl[n].runway_flights]
                if lvl[n].runway_group != lvl[n + 1].runway_group:
                    lv += [rnway]
                    rnway = []
        except:
            if lvl[-1].runway_group == lvl[-2].runway_group:
                lv += [rnway]
            else:
                lv += [[lvl[-1].runway_flights]]
        return lv

    def runway_per_tstep(self, t_step, node_prob, mov_prob, node_rule=None):
        if node_rule is None:
            node_rule = ['random']
        self.t_step = t_step
        lvl1, lvl2, lvl3 = self.converg_group(self.lvl1_struct,mov_prob)
        self.l1, self.l2, self.l3 = lvl1, lvl2, lvl3
        self.levels=[lvl1,lvl2,lvl3]
        # partition lvls as list of lists by runway_group
        lv1 = self.part_lvl(lvl1)
        lv2 = self.part_lvl(lvl2)

        lv1_a = self.part_lvl_a(lvl1)
        lv2_a = self.part_lvl_a(lvl2)

        l2out = 0
        l1out = [0] * len(lv1)

        self.ipool = [r for r in self.main_pool]
        for t in range(t_step):

            # lvl1
            for dx, gp in enumerate(lv1):
                if not all([rw.runway_flights[-1] == 0 for rw in gp]):
                    node_choice = random.choices([*node_rule, 'random'], weights=[*node_prob, 100 - sum(node_prob)],k=1)[0]
                    # get min sized aircraft
                    if node_choice == 'min':
                        ex_fl_s = [rw.runway_flights[-1].size for rw in gp if rw.runway_flights[-1] != 0]
                        min_fl_s = min(ex_fl_s)
                        ex_rw = ex_fl_s.index(min_fl_s)
                    # get max sized aircraft
                    elif node_choice == 'max':
                        ex_fl_s = [rw.runway_flights[-1].size for rw in gp if rw.runway_flights[-1] != 0]
                        max_fl_s=max(ex_fl_s)
                        ex_rw = ex_fl_s.index(max_fl_s)
                    # get random sized aircraft
                    elif node_choice == 'random':
                        ex_rw = self.rand_node_choice(gp)
                    # runway with aircrafts with least fuel_level
                    elif node_choice == 'fuel_level':
                        rw_fl = [sum([ac.fuel for ac in rw.runway_flights if ac!=0]) for rw in gp]
                        min_fl = min(rw_fl)
                        ex_rw = rw_fl.index(min_fl)
                    # get most healthy runway
                    elif node_choice == 'health':
                        for l in self.levels:
                            for rw in l:
                                rw.extra_spacing()
                        max_sp = max([rw.tot_ext_space for rw in gp])
                        ex_rw = [rw.tot_ext_space for rw in gp].index(max_sp)
                    # get runway having aircraft that first arrives at the node or randomly select
                    elif node_choice == 'first_arrival':
                        fls=[rw.runway_flights[-1] for rw in gp if rw.runway_flights[-1]!=0]
                        if len(fls) == 1:
                            ex_fl = fls[0]
                            ex_rw = [rw.runway_flights[-1] for rw in gp].index(ex_fl)
                        else:
                            ex_rw = self.rand_node_choice(gp)
                    else:
                        raise Exception('node_rule choice is invalid or empty. Argument can be \'min\', \'max\', \'random\','
                                        ' \'fuel_level\', \'health\' or \'first_arrival\'')

                    lv1[dx][ex_rw].exit_runway()
                    if l1out[dx] == 0:
                        l1out[dx] = lv1[dx][ex_rw].exit_flight
                        lv1[dx][ex_rw].exit = True

            for gp in range(len(lvl1)):
                lvl1[gp].mov_per_tstep()
                # ensure no flight exits level 1 if movement probability of chosen runway is 0
                if lvl1[gp].exit and not lvl1[gp].moved:
                    l1out[lvl1[gp].runway_group-1] = 0
                # condition and passing exception allows sim to keep running when there are no aircrafts in main_pool
                if self.main_pool!=[]:
                    ent = self.main_pool[-1]
                    lvl1[gp].entry_runway(ent)
                if lvl1[gp].ent:
                    try:
                        self.main_pool.pop(-1)
                    except IndexError:
                        pass
                # change all exit checkers back to default
                lvl1[gp].exit = False

            # lvl2
            # runway with least sized aircraft in last cell takes an exit
            if not all([rw.runway_flights[-1] == 0 for rw in lvl2]):
                node_choice = random.choices([*node_rule, 'random'], weights=[*node_prob, 100 - sum(node_prob)],k=1)[0]
                # get least sized aircraft
                if node_choice=='min':
                    ex_fl_s = [rw.runway_flights[-1].size for rw in lvl2 if rw.runway_flights[-1] != 0]
                    min_fl_s = min(ex_fl_s)
                    ex_rw = ex_fl_s.index(min_fl_s)
                # get max sized aircraft
                elif node_choice=='max':
                    ex_fl_s = [rw.runway_flights[-1].size for rw in lvl2 if rw.runway_flights[-1] != 0]
                    max_fl_s = max(ex_fl_s)
                    ex_rw = ex_fl_s.index(max_fl_s)
                # get random sized aircraft
                elif node_choice=='random':
                    ex_rw = self.rand_node_choice(lvl2)
                # runway with aircrafts with least fuel_level
                elif node_choice == 'fuel_level':
                    rw_fl = [sum([ac.fuel for ac in rw.runway_flights if ac!=0]) for rw in lvl2]
                    min_fl = min(rw_fl)
                    ex_rw = rw_fl.index(min_fl)
                # get most healthy runway
                elif node_choice == 'health':
                    for l in self.levels:
                        for rw in l:
                            rw.extra_spacing()
                    max_sp = max([rw.tot_ext_space for rw in lvl2])
                    ex_rw = [rw.tot_ext_space for rw in lvl2].index(max_sp)
                # runway having aircraft that first arrives at the node or randomly select
                elif node_choice == 'first_arrival':
                    fls = [rw.runway_flights[-1] for rw in lvl2 if rw.runway_flights[-1] != 0]
                    if len(fls) == 1:
                        ex_fl = fls[0]
                        ex_rw = [rw.runway_flights[-1] for rw in lvl2].index(ex_fl)
                    else:
                        ex_rw = self.rand_node_choice(lvl2)
                else:
                    raise Exception('node_rule choice is invalid or empty. Argument can be \'min\', \'max\', \'random\', '
                                    '\'fuel_level\', \'health\' or \'first_arrival\'')
                lvl2[ex_rw].exit_runway()
                if l2out == 0:
                    l2out = lvl2[ex_rw].exit_flight
                    lvl2[ex_rw].exit = True

            for rw in range(len(lvl2)):
                lvl2[rw].mov_per_tstep()
                # ensure no flight exits level 2 if movement probability of chosen runway is 0
                if lvl2[rw].exit and not lvl2[rw].moved:
                    l2out = 0
                lvl2[rw].entry_runway(l1out[rw])
                # change all exit checkers back to default
                lvl2[rw].exit = False
                if lvl2[rw].ent:
                    l1out[rw] = 0

            # lvl3
            # exit from runway if any flight has to
            lvl3[0].exit_runway()
            l3out = lvl3[0].exit_flight

            if l3out != 0:
                lvl3[0].exit = True
            # adjust position of flights in runway after exit
            lvl3[0].mov_per_tstep()
            # ensure no flight exits level 3 to main_pool if movement probability of chosen runway is 0
            if lvl3[0].exit and lvl3[0].moved:
                try:
                    # exit from last level goes to the main pool
                    self.main_pool = [l3out] + self.main_pool
                    self.landing+=[l3out]
                    # random.shuffle([self.main_pool])
                except AttributeError:
                    print('main_pool not created, create main_pool using object.create_pool command')

            if l2out != 0:
                lvl3[0].entry_runway(l2out)
            if lvl3[0].ent:
                l2out = 0
            # return exit checker to false
            lvl3[0].exit = False

            trans_lvl = l1out + [l2out]
            tl = len([n for n in trans_lvl if n != 0])

            # collect total operational flights across all levels and deplete aircraft fuel
            for l in self.levels:
                for rw in l:
                    rw.get_tot_op_fl()
                    # deplete aircraft fuel
                    for ac in rw.runway_flights:
                        if ac != 0:
                            ac.fuel_depl()

        # calc density of operational flights across all levels
        for l in self.levels:
            for rw in l:
                rw.density(t_step)

    def rand_node_choice(self, group):
        ex_fl = random.choice([rw.runway_flights[-1] for rw in group if rw.runway_flights[-1] != 0])
        ex_rw = [rw.runway_flights[-1] for rw in group].index(ex_fl)
        return ex_rw

    def converg_group(self, lvl1_strut, ac_mov_prob):
        lvl1 = self.create_rw_groups(lvl1_strut, ac_mov_prob)

        lvl2_gps, lvl2 = self.create_oth_lvl(lvl1_strut, ac_mov_prob)

        lvl3_gps, lvl3 = self.create_oth_lvl(lvl2_gps, ac_mov_prob)

        lvls_gps = [lvl1_strut, lvl2_gps, lvl3_gps]
        lvls = [lvl1, lvl2, lvl3]

        # name level attribute of runways accordingly
        for dx, lvl in enumerate(lvls, 1):
            for rw in lvl:
                rw.lvl = dx
        return lvl1, lvl2, lvl3

    def create_oth_lvl(self, group_strut, ac_mov_prob):
        no_group = len(group_strut)
        if any([len(gp) != 1 for gp in group_strut]):
            lvl_gps = [[sum(gp) // len(gp)] for gp in group_strut]
        else:
            lvl_gps = [[sum([nc for gp in group_strut for nc in gp]) // no_group]]
        lvl = self.create_rw_groups(lvl_gps, ac_mov_prob)
        return lvl_gps, lvl

    def create_rw_groups(self, group_strut, mov_prob=100, run_way=Runway):
        lvl = []
        for gdx, group in enumerate(group_strut, 1):
            for rw in group:
                if isinstance(rw, float):
                    break
                lvl += [run_way(rw, gdx, mov_prob)]
        return lvl

    def total_op_flights(self):
        tot_f = len(self.main_pool)
        tot_f += len([r for n in range(len(self.l1)) for r in self.l1[n].runway_flights if r != 0])
        tot_f += len([r for n in range(len(self.l2)) for r in self.l2[n].runway_flights if r != 0])
        tot_f += len([r for n in range(len(self.l3)) for r in self.l3[n].runway_flights if r != 0])
        return tot_f

    def section_density(self,section,type_choice='original'):
        if type_choice=='original':
            sec_agg_op_fl=[rw.agg_op_fl for l in section for rw in l]
            sec_tot_op_fl=[]
            for t in range(len(sec_agg_op_fl[0])):
                tot_op_fl=0
                for op_fl in sec_agg_op_fl:
                    tot_op_fl+=op_fl[t]
                sec_tot_op_fl+=[tot_op_fl]

            sec_density=sum(sec_tot_op_fl)/self.t_step
            return sec_density

        if type_choice == 'modified':
            sec_agg_dens = [rw.agg_dens for l in section for rw in l]
            sec_tot_dens = []
            for t in range(len(sec_agg_dens[0])):
                tot_dens = 0
                for dens in sec_agg_dens:
                    tot_dens += dens[t]
                sec_tot_dens += [tot_dens]

            sec_density = sum(sec_tot_dens) / self.t_step
            return sec_density

    def section_flow(self,section):
        sec_flow = 0
        for l in section:
            for rw in l:
                sec_flow += rw.tot_mov
        sec_flow = sec_flow/self.t_step
        return sec_flow

    def cell1_section_flow(self,section):
        c_sec_flow = 0
        for l in section:
            for rw in l:
                c_sec_flow += rw.cell1_mov
        c_sec_flow = c_sec_flow/self.t_step
        return c_sec_flow

    def landing_rate(self):
        landings=len(self.landing)
        landing_rate=landings/self.t_step
        return landing_rate


def total_op_flights(pool, l1, l2, l3, tl):
    tot_f = len(pool)
    tot_f += len([r for n in range(len(l1)) for r in l1[n].runway_flights if r != 0])
    tot_f += len([r for n in range(len(l2)) for r in l2[n].runway_flights if r != 0])
    tot_f += len([r for n in range(len(l3)) for r in l3[n].runway_flights if r != 0])
    tot_f += tl
    return tot_f


def obj_to_attr(obj, attr_name):
    attr = attr_name
    vables = vars()
    if hasattr(obj, attr_name):
        return obj.vables[attr]

# to change aircraft objects of runways in different levels to their respective sizes for better result display
def chng_list(raw_list):
    fin_list=[]

    def chng_elem(l):
        new_list = []
        for ac in l.runway_flights:
            if ac!=0:
                new_list += [ac.size]
            elif ac==0:
                new_list+=[0]
        return new_list

    for l in raw_list:
        if isinstance(l,list):
            chng_list(l)
        else:
            fin_list+=[chng_elem(l)]

    return fin_list


'''
Traffic_Control class represents experiment enviroment
#1 Create Traffic_Control object e.g tf=Traffic_Control()

#2 Setup group of runways by instantiating method add_runway
e.g tf.add_runway([[20,40,30,10],[30,50],[45]]) means
[20,40,30,10] is a group of runways with 20,40,30 & 10 steps, [30,45] is another group.
Note that no. of steps in any level2 runway is average of all the steps in corresponding level1 runway group

#3 Create pool of aircrafts (i.e create_pool method) to be used all through simulation.
Set distribution of aircraft sizes in pool using size_dist argument e.g size_dist=[40,50,45,65] means
40 size_1, 50 size_2, 45 size_3 & 65 size_4 aircrafts

#4 Run simulation with runway_per_tstep method.
i) node_rule parameter takes list of string argument 'min','max','random','fuel_level', 'health' and/or first_arrival
ii) node_prob argument is probability that chosen node_rule(s) will be used over 'random' with 100 being max probability variable
hint: node_prob can be set to 100 to force a singular chosen node_rule to be used
warning: total probability for chosen node_rules cannot exceed 100
iii) mov_prob parameter takes integer argument of up to 100 (signifying 100%) and is the probability of an aircraft moving
from one cell to another. Note that given mov_prob argument applies for all runways in the experiment.
examples:
tf.runway_per_tstep([t_step=200, node_rule=['health','min','first_arrival'], node_prob=[20,15,40], mov_prob=40)
tf.runway_per_tstep([t_step=200, node_rule=['max','min','fuel_level'], node_prob=[40,30,30], mov_prob=40)

#5 Calc density of sections with object.section_density method
section_density method takes 2 arguments:
i) section- a list of all levels or runways that constitute user desired section
ii) type_choice- a string argument 'original' or 'modified', defaults to 'original'.
Note that list of runways on each level has been flattened for user to l1,l2,l3
Note also that all entries must be a list of lists
examples:
tf.section_density([tf.l1],'modified')
tf.section_density([tf.l1,tf.l2],'original')
tf.section_density([[tf.l1[2]],tf.l2,tf.l3],'modified')
tf.section_density([tf.l1[0:2],tf.l2,tf.l3],'original')
tf.section_density([[tf.l1[2],tf.l1[4]],tf.l2,tf.l3],'modified')

#6 Calc flow with object.section_flow method.
examples:
tf.section_flow([tf.l1])
tf.section_flow([[tf.l1[2]],tf.l2,tf.l3])
tf.section_flow([tf.l1[0:2],tf.l2,tf.l3])

#7 Calc flow from first cells with object.cell1_section_flow method.
examples:
tf.cell1_section_flow([tf.l1])
tf.cell1_section_flow([tf.l1[0:2],tf.l2,tf.l3])

#8 Calc landing rate with function object.landing_rate()
e.g tf.landing_rate()
'''

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # start time
    st = time.process_time()

    tf = Traffic_Control()
    tf.add_runway([[20, 40, 30, 10], [30, 50], [20, 15, 25], [45]])
    tf.create_pool(size_dist=[40, 50, 45, 65])
    tf.runway_per_tstep(t_step=500, node_rule=['health', 'min', 'first_arrival'], node_prob=[30,10,40], mov_prob=40)

    # end time
    et = time.process_time()
    # execution time
    exc_time = et-st
