import random
import numpy as np
from matplotlib import pyplot as plt


class Runway:
    def __init__(self, no_cells, runway_group):
        self.no_cells = no_cells
        self.runway_group = runway_group
        self.runway_flights = [0] * no_cells
        self.lvl = 0
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
            for n in range(-1, -len(rw_fl), -1):
                self.runway_flights[n] = rw_fl[n - 1]
                # print('first:',self.runway_flights[n],' second',rw_fl[n-1])
            self.runway_flights[0] = 0
            # calc no of aircraft that successfully move
            self.tot_mov += 1
            # confirm for entry func that movement occurred
            self.moved = True

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
            if n > 0:
                spacing = self.spacing_rules(flight, n)
                if dx <= spacing:
                    self.ent = False
                    return
                elif dx > spacing:
                    self.ent = True
                    self.runway_flights[0] = flight
                    # self.runway_flights[dx-(spacing+1)]=flight
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
        prev_acs = []
        bet_spc = []
        ac = []

        for a in r_fl:
            # collect first non_zero sized aircraft
            if a == 0:
                prev_acs += [r_fl[a]]
            if all([n == 0 for n in prev_acs]) and a > 0:
                ac += [a]
                prev_acs = []

            # collect spaces between aircraft
            if len(ac) == 1 and a == 0:
                bet_spc += [a]
            # get extra spaces
            if len(ac) > 1:
                space = self.spacing_rules(*ac)
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
    #
    # def density(self, t_step):
    #     self.tot_op_fl = sum(self.agg_op_fl)
    #     self.dens = self.tot_op_fl / t_step


class Traffic_Control:

    def __init__(self):
        self.landing=[]

    def create_pool(self, size_dist):
        self.no_pool_cont = sum(size_dist)
        self.size_one = size_dist[0]
        self.size_two = size_dist[1]
        self.size_three = size_dist[2]
        self.size_four = size_dist[3]
        self.main_pool = [1] * self.size_one + [2] * self.size_two + [3] * self.size_three + [4] * self.size_four
        random.shuffle(self.main_pool)

    def test_runway(self):
        run = Runway(no_cells=50, runway_group=2)
        print(f'Initial Main Pool {len(self.main_pool)}:', self.main_pool)
        self.ipool = [r for r in self.main_pool]

        for t in range(100):
            # flight exits runway
            run.exit_runway()
            run.exit = True
            if run.exit and run.exit_flight != 0:
                self.main_pool = [run.exit_flight] + self.main_pool
                print(f'Main Pool {len(self.main_pool)}: ', self.main_pool)

            # flights move forward in runway
            run.mov_per_tstep()
            # print('current pool',self.main_pool)

            # flights enter runway from pool
            ent = self.main_pool.pop(-1)
            run.entry_runway(ent)
            # if runway rejects flight entry, return aircraft to pool
            if not run.ent:
                self.main_pool.insert(-1, ent)
            print(run.runway_flights)
            run.exit = False
            # random.shuffle(self.main_pool)
        print(f'Final Main Pool {len(self.main_pool)}:', self.main_pool)

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
        print('lv', lv)
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
        print(lv)
        return lv

    def runway_per_tstep(self, t_step, node_rule):
        self.t_step = t_step
        lvl1, lvl2, lvl3 = self.converg_group(self.lvl1_struct)
        self.l1, self.l2, self.l3 = lvl1, lvl2, lvl3
        self.levels=[lvl1,lvl2,lvl3]
        # partition lvls as list of lists by runway_group
        lv1 = self.part_lvl(lvl1)
        lv2 = self.part_lvl(lvl2)

        lv1_a = self.part_lvl_a(lvl1)
        lv2_a = self.part_lvl_a(lvl2)

        l2out = 0
        l1out = [0] * len(lv1)
        print('Initial Main Pool:', self.main_pool)
        print(
            f'Aircrafts In Initial Runway: {[rw.runway_flights for rw in lvl1]}\n{[rw.runway_flights for rw in lvl2]}\n{[rw.runway_flights for rw in lvl3]}')

        self.ipool = [r for r in self.main_pool]
        for t in range(t_step):

            # lvl1
            for dx, gp in enumerate(lv1):
                if not all([rw.runway_flights[-1] == 0 for rw in gp]):
                    # get min sized aircraft
                    if node_rule == 'min':
                        ex_fl = min([rw.runway_flights[-1] for rw in gp if rw.runway_flights[-1] != 0])
                    # get max sized aircraft
                    elif node_rule == 'max':
                        ex_fl = max([rw.runway_flights[-1] for rw in gp if rw.runway_flights[-1] != 0])
                    # get random sized aircraft
                    elif node_rule == 'random':
                        ex_fl = random.choice([rw.runway_flights[-1] for rw in gp if rw.runway_flights[-1] != 0])
                        # get random sized aircraft
                    elif node_rule == 'health':
                        for l in self.levels:
                            for rw in l:
                                rw.extra_spacing()
                        ex_fl = max([rw.tot_ext_space for rw in gp])
                    else:
                        raise Exception('node_rule choice is invalid or empty. Argument can be \'min\', \'max\', \'random\', \'health\'')

                    if node_rule == 'health':
                        ex_rw = [rw.tot_ext_space for rw in gp].index(ex_fl)
                    else:
                        ex_rw = [rw.runway_flights[-1] for rw in gp].index(ex_fl)

                    lv1[dx][ex_rw].exit_runway()
                    if l1out[dx] == 0:
                        l1out[dx] = lv1[dx][ex_rw].exit_flight
                        lv1[dx][ex_rw].exit = True

            for gp in range(len(lvl1)):
                lvl1[gp].mov_per_tstep()
                ent = self.main_pool[-1]
                # print('Aircrafts in main_pool is exhausted \nCreate sufficient with Traffic.create_pool()')
                lvl1[gp].entry_runway(ent)
                if lvl1[gp].ent:
                    try:
                        self.main_pool.pop(-1)
                    except IndexError:
                        raise Exception('Aircrafts in main_pool is exhausted \nCreate sufficient with Traffic.create_pool()')
                # change all exit checkers back to default
                lvl1[gp].exit = False

            # lvl2
            # runway with least sized aircraft in last cell takes an exit
            if not all([rw.runway_flights[-1] == 0 for rw in lvl2]):
                if node_rule=='min':
                    # get least sized aircraft
                    ex_fl = min([rw.runway_flights[-1] for rw in lvl2 if rw.runway_flights[-1] != 0])
                elif node_rule=='max':
                    # get max sized aircraft
                    ex_fl = max([rw.runway_flights[-1] for rw in lvl2 if rw.runway_flights[-1] != 0])
                elif node_rule=='random':
                    # get random sized aircraft
                    ex_fl = random.choice([rw.runway_flights[-1] for rw in lvl2 if rw.runway_flights[-1] != 0])
                elif node_rule == 'health':
                    for l in self.levels:
                        for rw in l:
                            rw.extra_spacing()
                    ex_fl = max([rw.tot_ext_space for rw in lvl2])
                else:
                    raise Exception('node_rule choice is invalid or empty. Argument can be \'min\', \'max\', \'random\', or \'health\'')
                # get runway with appropriate sized aircraft
                if node_rule == 'health':
                    ex_rw = [rw.tot_ext_space for rw in lvl2].index(ex_fl)
                else:
                    ex_rw = [rw.runway_flights[-1] for rw in lvl2].index(ex_fl)
                lvl2[ex_rw].exit_runway()
                if l2out == 0:
                    l2out = lvl2[ex_rw].exit_flight
                    lvl2[ex_rw].exit = True

            for rw in range(len(lvl2)):
                lvl2[rw].mov_per_tstep()
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
                try:
                    lvl3[0].exit = True
                    # exit from last level goes to the main pool
                    print('main_pool replenished')
                    self.main_pool = [l3out] + self.main_pool
                    self.landing+=[l3out]
                    # random.shuffle([self.main_pool])
                except AttributeError:
                    print('main_pool not created, create main_pool using object.create_pool command')
                    # adjust position of flights in runway after exit
            lvl3[0].mov_per_tstep()
            if l2out > 0:
                lvl3[0].entry_runway(l2out)
                lvl3[0].exit = False
            if lvl3[0].ent:
                l2out = 0
            trans_lvl = l1out + [l2out]
            tl = len([n for n in trans_lvl if n != 0])

            # collect total operational flights across all levels
            for l in self.levels:
                for rw in l:
                    rw.get_tot_op_fl()

            print('Main Pool No. ', len(self.main_pool))
            print('Total Flights ', total_op_flights(self.main_pool, lvl1, lvl2, lvl3, tl))
            print(f'Main Pool: {self.main_pool}')
            print(
                f'Aircrafts: {[rw.runway_flights for rw in lvl1]}\n{[rw.runway_flights for rw in lvl2]}\n{[rw.runway_flights for rw in lvl3]}')
            print('')

        # calc density of operational flights across all levels
        for l in self.levels:
            for rw in l:
                rw.density(t_step)
        print('Final Main Pool:', self.main_pool)

    def add_runway(self, level1_structure):
        # level1_structure should be in format [[50,35,60],[25,50,40],[45,50]]
        # in the above example, there are 3 groups and 8 runways
        # innermost lists represent groups 1,2,3... from left to right
        # and integers represent number of cells in each runway
        self.lvl1_struct = level1_structure

    def converg_group(self, lvl1_strut):
        lvl1 = self.create_rw_groups(lvl1_strut)

        lvl2_gps, lvl2 = self.create_oth_lvl(lvl1_strut)

        lvl3_gps, lvl3 = self.create_oth_lvl(lvl2_gps)

        lvls_gps = [lvl1_strut, lvl2_gps, lvl3_gps]
        lvls = [lvl1, lvl2, lvl3]

        # name level attribute of runways accordingly
        for dx, lvl in enumerate(lvls, 1):
            for rw in lvl:
                rw.lvl = dx
        return lvl1, lvl2, lvl3

    def create_oth_lvl(self, group_strut):
        no_group = len(group_strut)
        if any([len(gp) != 1 for gp in group_strut]):
            lvl_gps = [[sum(gp) // len(gp)] for gp in group_strut]
        else:
            lvl_gps = [[sum([nc for gp in group_strut for nc in gp]) // no_group]]
        lvl = self.create_rw_groups(lvl_gps)
        return lvl_gps, lvl

    def create_rw_groups(self, group_strut, run_way=Runway):
        lvl = []
        for gdx, group in enumerate(group_strut, 1):
            # print(group)
            for rw in group:
                if isinstance(rw, float):
                    break
                lvl += [run_way(rw, gdx)]
        # print(lvl)
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
        return sec_flow

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


def simulate(runways):
    pass


'''
Traffic_Control class represents experiment enviroment
#1 Create Traffic_Control object e.g tf=Traffic_Control()

#2 Create pool of aircrafts (i.e create_pool method) to be used all through simulation.
Set distribution of aircraft sizes in pool using size_dist argument e.g size_dist=[40,50,45,65] means
40 size_1, 50 size_2, 45 size_3 & 65 size_4 aircrafts

#3 Setup group of runways by instantiating method add.runway
e.g tf.add_runway([[20,40,30,10],[30,50],[45]]) means
[20,40,30,10] is a group of runways with 20,40,30 & 10 steps, [30,45] is another group.
Note that no. of steps in any level2 runway is average of all the steps in corresponding level1 runway group

#4 Run simulation with runway_per_tstep method.
node_rule parameter takes string argument 'min','max','random' or 'health'

#5 Calc density of sections with object.section_density method
section_density method takes 2 arguments:
i) section- a list of all levels or runways that constitute user desired section
ii) type_choice- a string argument 'original' or 'modified', defaults to 'original'.
Note that list of runways on each level has been flatened for user to l1,l2,l3
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

#7 Calc landing rate with function object.landing_rate()
e.g tf.landing_rate()
'''

if __name__ == '__main__':
    tf = Traffic_Control()
    tf.create_pool(size_dist=[40, 50, 45, 65])
    tf.add_runway([[20, 40, 30, 10], [30, 50], [20, 15, 25], [45]])
    tf.runway_per_tstep(t_step=200, node_rule='random')
    # tf.test_runway()
