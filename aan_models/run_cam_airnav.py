from cam_airnav_mod import Flight, Waypoint, Obstruction, TMA, Point,\
                            simulate, display_results, create_flights

'''
Create flight, waypoint & obstruction objects f1,f2,f3,w1,w2,ob1,ob2...
and include all objects needed in respective experiment list

____________________________________________________________
#1 Implementation A of Cellular Automata Model

set total_tstep=1 to simulate until destination is reached

for random generated flights, follow convention in example fr1,fr2...
for density around points,use p1.agg_dens,p2.agg_dens,w1.agg_dens....
get average density around points with p1.avg_dens,p2.avg_dens,w1.avg_dens....
get average velocity with p1.avg_vel,p2.avg_vel...
get average distance with tma.av_dist
get average transit time with f1.avg_transit_time(), f2.avg_transit_time()...

'''
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # To Do
    # 1. Define airport block (5-coordinates)
    #       Airport block will be part of TMA
    #           OR
    # 1. User sets touchdown area and exit area
    #

    tma = TMA()
    tma.coord = [[-5,50],[-10,50]]
    a_cord = tma.avail_coords()

    dest = [5,5]
    t_down = [20,5]
    no_flyzone_size=7
    tdown_dest_path_size=2
    f1 = Flight(dept=[-7,3], dest=dest, t_down=t_down, trajectory='short', size=2)
    f2 = Flight(dept=[25,25], dest=dest, t_down=t_down, trajectory='short', size=1)
    f3 = Flight(dept=[12,0], dest=dest, t_down=t_down, trajectory='medium', size=3) #[3,8]
    f4 = Flight(dept=[13,9], dest=dest, t_down=t_down, trajectory='short', size=1)
    f5 = Flight(dept=[5,5], dest=dest, t_down=t_down, trajectory='short', size=1) #[10,3]
    f6 = Flight(dept=[1,1], dest=dest, t_down=t_down, trajectory='long', size=1) #[15,2]

    fr1 = create_flights(north=3, south=2, east=0, west=0, tma=tma, dest=dest, t_down=t_down, trajectory='long', size=1, spread=3) #[5, 1]
    fr2 = create_flights(north=1, south=0, east=2, west=4, tma=tma, dest=dest, t_down=t_down, trajectory='short', size=2, spread=1) #[7, 15]
    fr3 = create_flights(north=2, south=1, east=1, west=0, tma=tma, dest=dest, t_down=t_down, trajectory='medium', size=1, spread=2) #[17, 20]

    w1 = Waypoint(pos=[-13,20], size=1)
    w2 = Waypoint(pos=[10,20], size=1)
    w3 = Waypoint(pos=[-5,5], size=1)
    w4 = Waypoint(pos=[20,-8], size=1)
    w5 = Waypoint(pos=[-10,-8], size=1)

    ob1 = Obstruction(pos=[0,10], size=2)
    ob2 = Obstruction(pos=[5,10], size=3)
    ob3 = Obstruction(pos=[15,10], size=3)
    ob4 = Obstruction(pos=[-10,-4], size=2)
    ob5 = Obstruction(pos=[0,-4], size=2)
    ob6 = Obstruction(pos=[12,-4], size=1)

    p1 = Point(pos=[6,2], size=2)
    p2 = Point(pos=[10,5], size=3)

    flights = fr1+fr2+[f1,f2,f3,f4,f5,f6]
    waypoints = [w1,w2,w4,w5]
    obstructions = []
    points = [p1,p2]


# to run simulation
    simulate(tma,flights,waypoints,obstructions,points,no_flyzone_size,tdown_dest_path_size,total_tsteps=300)

    display_results(tma,flights,waypoints)