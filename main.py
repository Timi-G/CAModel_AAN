import numpy as np
from matplotlib import pyplot as plt

# dept:departure, dest:destination, size:aircraft_size_class
# pos:current_position, fir_pos:first_position, sec_pos:second_position
# agg_pos:aggregate_positions, t_step:time_step
class Flight:

    def __init__(self,dept,dest):
        self.dept = dept
        self.dest = dest
        self.size = int
        self.fir_pos = []
        self.sec_pos = []
        self.pos = dept
        self.agg_pos = []
        self.t_step = 0

# to define flight path; combining the aircraft's different progressive positions
    def collect_pos(self):
        self.agg_pos=self.agg_pos+self.pos
        if self.agg_pos[-3:-1] != self.dest:
            self.t_step += 1

# the flight path as a 2D array
    def flight_path(self):
        fp = np.array(self.agg_pos)
        nfp = fp.reshape(-1,2)
        self.fp = nfp.T


class Nas:
    M = int
    N = int

def fir():
    pass

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

# flight movement in conflict
def conf_flight_movement(dep_pos, des_pos, pos):
    pass

# to plot all the flight paths
def path_plots(fpaths):

    for fpath in fpaths:
        fpath.flight_path()
        y,x=fpath.fp
        plt.plot(x,y)

# simulate flights
def sim_iter(flights,flights_pos):
    while True:
        for flight in flights:
            nconf_flight_movement(flight.dept,flight.dest,flight.pos)
            flight.collect_pos()

# stop simulation when destination is reached
        if flights_pos.count(flights_pos[0]) == len(flights_pos):
            break

    path_plots(flights)
    plt.show()

# container function for flight simulation
def simulate(flights):
    flights_pos=[]
    for f in flights:
        flights_pos.append(f.pos)

    sim_iter(flights,flights_pos)


'''create flight object f1,f2,f3...  and include all needed in flights experiment list'''
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    f1= Flight(dept=[10,3], dest=[1,1])
    f2 = Flight(dept=[14, 3], dest=[1, 1])
    f3 = Flight(dept=[12, 0], dest=[1, 1])
    f4= Flight(dept=[13,9],dest=[1,1])

    flights = [f1,f2,f3,f4]

    simulate(flights)

    for f in flights:
        print('time step:',f.t_step,'flight aggregate positions:',f.agg_pos)
