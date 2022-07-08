import random


class Runway:
    def __init__(self,no_cells,runway_group):
        self.no_cells=no_cells
        self.runway_group=runway_group
        self.runway_flights=[0]*no_cells
        self.lvl=0

    def mov_per_tstep(self):
        self.moved = False
        rw_fl=self.runway_flights
        # return if runway is empty
        if rw_fl==[0]*self.no_cells:
            return
        # movement only when there is space in last cell
        if rw_fl[-1]==0:
            for n in range(-1,-len(rw_fl)):
                self.runway_flights[n]=rw_fl[n-1]
        # confirm for entry func that movement occurred
            self.moved = True

    def entry_runway(self,flight):
        rw_fl=self.runway_flights
        # entry is only for an aircraft
        if flight==0:
            self.ent=flight
            return
        # entry to an empty runway
        if rw_fl==[0]*self.no_cells:
            self.ent=0
            self.runway_flights[0]=flight
            return
        # entry
        for dx,n in enumerate(rw_fl):
            if n>0:
                spacing=self.spacing_rules(flight,n)
                if dx<spacing:
                    self.ent=flight
                    return
                elif dx>=spacing:
                    self.ent=0
                    self.runway_flights[dx-spacing]=flight

    def exit_runway(self):
        rw_fl=self.runway_flights
        if self.runway_flights[-1]>0:
            exit_flight=self.runway_flights[-1]
            return exit_flight
        else:
            return 0

    def spacing_rules(self,a,b):

        spacing_table=[
            [3,3,3,3],
            [5,3,3,3],
            [6,5,4,3],
            [8,7,6,3]
        ]
        spacing=spacing_table[a-1][b-1]
        return spacing

class Traffic_Control:

    def create_pool(self,size_dist):
        self.no_pool_cont=sum(size_dist)
        self.size_one=size_dist[0]
        self.size_two=size_dist[1]
        self.size_three=size_dist[2]
        self.size_four=size_dist[3]
        self.main_pool=[1]*self.size_one+[2]*self.size_two+[3]*self.size_three+[4]*self.size_four
        random.shuffle(self.main_pool)

    def runway_per_tstep(self):
        t_step=100
        lvl1,lvl2,lvl3=self.converg_group(self.lvl1_struct)
        l2out=0
        l1out=[0]*len(self.lvl1_struct)
        print('Initial Main Pool:',self.main_pool)
        print(f'Aircrafts In Initial Runway: {[rw.runway_flights for rw in lvl1]}\n{[rw.runway_flights for rw in lvl2]}\n{[rw.runway_flights for rw in lvl3]}')

        for t in range(t_step):
            # exit from runway if any flight has to
            if lvl3[0].exit_runway() > 0:
                try:
                    # exit from last level goes to the main pool
                    self.main_pool += lvl3[0].exit_runway()
                    random.shuffle([self.main_pool])
                except NameError:
                    print('main_pool not created, create main_pool using object.create_pool command')
                    # adjust position of flights in runway after exit
            lvl3[0].mov_per_tstep()
            if l2out>0:
                lvl3[0].entry_runway(l2out)

            # lvl2
            # runway with least sized aircraft in last cell takes an exit
            if not all([rw.runway_flights[-1]==0 for rw in lvl2]):
                rw_movdx=lvl2.index(min([rw.runway_flights[-1] for rw in lvl2 if rw.runway_flights[-1] != 0]))
                l2out=lvl2[rw_movdx].exit_runway()
            for rw in range(len(lvl2)):
                lvl2[rw].mov_per_tstep()
                lvl2[rw].entry_runway(l1out[rw])

            # lvl1
            g=[]
            for gp in range(len(lvl1)):
                g+=[lvl1[gp]]
                try:
                    if lvl1[gp].runway_group != lvl1[gp+1].runway_group:
                        if not all([rw.runway_flights[-1]==0 for rw in g]):
                            rw_movdx = g.index(min([rw.runway_flights[-1] for rw in g if rw.runway_flights[-1] != 0]))
                            l1out[lvl1[gp].runway_group-1]=g[rw_movdx].exit_runway()
                            g=[]
                except IndexError:
                    if not all([rw.runway_flights[-1] == 0 for rw in g]):
                        rw_movdx = g.index(min([rw.runway_flights[-1] for rw in g if rw.runway_flights[-1] != 0]))
                        l1out[lvl1[gp].runway_group-1]=g[rw_movdx].exit_runway()
                        g = []
            for gp in range(len(lvl1)):
                lvl1[gp].mov_per_tstep()
                if lvl1[gp].moved:
                    ent=self.main_pool[-1]
                    lvl1[gp].entry_runway(self.main_pool.pop(-1))
                    if lvl1[gp].ent==0:
                        self.main_pool.insert(-1,ent)
            print(f'Aircrafts: {[rw.runway_flights for rw in lvl1]}\n{[rw.runway_flights for rw in lvl2]}\n{[rw.runway_flights for rw in lvl3]}')
        print('Final Main Pool:', self.main_pool)
        # for gp in reversed(lvls):
        #         for rw in gp:
        #             # exit from runway if any flight has to
        #             if rw.exit_runway() > 0:
        #                 try:
        #                     # exit from last level goes to the main pool
        #                     if rw.lvl==3:
        #                         self.main_pool+=rw.exit_runway()
        #                         self.main_pool=random.shuffle([self.main_pool])
        #                 except NameError:
        #                     print('main_pool not created, create main_pool using object.create_pool command')

                    # adjust position of flights in runway after exit


    def display_traffic(self):
        pass

    def add_runway(self,level1_structure):
        # level1_structure should be in format [[50,35,60],[25,50,40],[45,50]]
        # where innermost lists represent groups 1,2,3... from left to right
        # and integers represent number of cells in each runway
        # in the above example, there are 3 groups and 8 runways
        self.lvl1_struct=level1_structure

    def converg_group(self,lvl1_strut):
        lvl1=self.create_rw_groups(lvl1_strut)

        lvl2_gps,lvl2=self.create_oth_lvl(lvl1_strut)

        lvl3_gps,lvl3=self.create_oth_lvl(lvl2_gps)

        lvls_gps=[lvl1_strut,lvl2_gps,lvl3_gps]
        lvls=[lvl1,lvl2,lvl3]

        # name level attribute of runways accordingly
        for dx,lvl in enumerate(lvls,1):
            for rw in lvl:
                rw.lvl= dx

        # for dx,gp in enumerate(no_group,1):
        #     lvl2=sum([rw for rw in group_rw if rw.runway_group==dx])

        # for group in group_strut:
        #     gp=[sum(group)/len(group)]
        #     lvl2+=[gp]
        #
        # gp=0
        # for group in group_strut:
        #     gp+=group[0]
        # lvl3=[gp]

        return lvl1,lvl2,lvl3

    def create_oth_lvl(self,group_strut):
        no_group = len(group_strut)
        if any([len(gp)!=1 for gp in group_strut]):
            lvl_gps = [[sum(gp) // len(gp)] for gp in group_strut]
        else:
            lvl_gps = [[sum([nc for gp in group_strut for nc in gp])//no_group]]
        lvl = self.create_rw_groups(lvl_gps)
        return lvl_gps,lvl

    def create_rw_groups(self,group_strut,run_way=Runway):
        lvl=[]
        for gdx,group in enumerate(group_strut,1):
            print(group)
            for rw in group:
                if isinstance(rw,float):
                    break
                lvl+=[run_way(rw,gdx)]
        print(lvl)
        return lvl


def simulate(runways):
    pass

if __name__=='__main__':
    tf=Traffic_Control()
    tf.create_pool(size_dist=[20,10,5,15])
    tf.add_runway([[20,40,30,10],[30,50],[20,15,25],[45]])
    tf.runway_per_tstep()

