from matplotlib import pyplot as plt

import visualizations as vs
from pontential_field import Aircraft, Waypoint, Stat_Obstruction,Mov_Obstruction,\
                            TMA, simulate, multiple_aircrafts, cal_flow, store_objects,disp_ran_acraft_info, rand_acraft_info,\
                            plot_vis, a_cord


def create_clips_vis(sim_field):
    clips=sim_field.sim_path_conf
    clips_name=list(clips.keys())
    for k in clips_name:
        for fp in clips[k]['flight_path']:
            x,y=fp
            plt.plot(x,y)
        fld=clips[k]['field']
        plot_vis(k,fld)

sim_objects={}
def make_sim_video(video_no):
    sim_field=sim_objects['field'][video_no-1][0]
    create_clips_vis(sim_field)
    vs.make_video('potential.mp4')
'''
Create flight, waypoint, static obstructions and mobile obstructions objects f1,f2,f3,w1,w2,sob1,mob1...
and include all objects needed in respective experiment list

________________________________________________________________
#2 Implementation B Of Cellular Automata Model

create multiple randomly positioned aircrafts using multiple_aircrafts func.:
no_aircrafts: number of a_crafts, max_pot:max potential any of the a_craft can have
set rand_ac=True to generate random aircrafts with random position (defaults to False)
set starting_sides
max_size: max size any of the a_crafts can have, plt_colors:list of colors that will represent the flight paths
null_pont=True will create aircraft with completely zero potential (defaults to False)

display info of randomly generated a_crafts using disp_ran_acraft_info()
get list of parameters on randomly generated a_crafts using rand_acraft_info()

get aggregate positions of flight with f1.agg_pos,f2.agg_pos...
get flight transition time with f1.avg_tnstime,f2.avg_tnstime...
get total transit time in simulation with tma.transit_time
get average transit time in simulation with tma.avg_transit_time
get dens of resultant field with tma.dens
get velocity with tma.vel, tma.avg_vel_a and tma.avg_vel_b
get flow of a given number of aircraft within specified time steps using cal_flow(t_steps,flights) it takes an optional func. tma 

density is no. of flights/area of tma
velocity is no. of movements throughout the simulation
average velocity A is no. of aircraft movement/total time step
average velocity B is no. of aircraft movement/total no. of aircraft

show_vis_clip variable is used to set how many sims to run at a given time and which sim should have visualizations
'''

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    sim_objects_container={'field':[],'flights':[],'waypoints':[],'static_obstructions':[],'moving_obstructions':[]}

    t_steps = 20
    clip_no = 0
    grid_size = [40, 25]
    show_vis_clip = [True, False]
    # try the convention:
    # show_vis_clip = [True] * 3 + [True] * 5 + [False] * 2
    total_flows=[]
    all_avg_transit_time=[]

    # declare variables for aircraft generation during repetition of simulation
    no_rand_aircrafts=[1, 1, 1, 1]
    no_aircrafts_from_sides=[[20,0,0,1],[20,1,0,1],[20,0,0,1],[20,1,1,1]]
    max_size=[1,1,1,1]
    max_pot=[1,1,1,1]

    for n,(nra,nas,ms,mp) in enumerate(zip(no_rand_aircrafts,no_aircrafts_from_sides,max_size,max_pot),0):
        # define a collector for desired quantities here
        # e.g. flows=[]
        flows = []
        avg_transit_time = []
        for no, show_single_clip in enumerate(show_vis_clip, 1):
            '''clip_no is index of video visualization'''
            clip_no = (n*len(show_vis_clip))+no
            # clip_no+=1 if show_single_clip else clip_no

            tma = TMA(row=39,column=13,max_pot=500,grid_size=grid_size)

            # f1 = Aircraft(row=5,column=9,max_pot=9,grid_size=grid_size,size=1,plt_color='b',inf_rad=4,null_pont=True)
            # f2 = Aircraft(row=3,column=5, max_pot=10, grid_size=grid_size, size=1, inf_rad=4,plt_color='r')
            # f3 = Aircraft(row=8,column=14, max_pot=7, grid_size=grid_size, size=2, inf_rad=4, plt_color='g')
            # f4 = Aircraft(row=10, column=9, max_pot=5, grid_size=grid_size, size=2, inf_rad=4, plt_color='w')
            #
            # fr1 = multiple_aircrafts(rand_aircrafts=50,max_pot=5,grid_size=grid_size,max_size=2,plt_colors=['c','r','m','y'],null_pont=False)
            # fr2 = multiple_aircrafts(max_pot=5, grid_size=grid_size, max_size=2, plt_colors=['c', 'r', 'm', 'y'],
            #                         start_sides={'up':3,'down':1,'left':2,'right':1}, null_pont=False)

            # dynamic declaration of aircrafts for repetitive simulations only applies to variables fr3 & fr4
            fr3 = multiple_aircrafts(rand_aircrafts=nra,max_size=ms,max_pot=mp,grid_size=grid_size,plt_colors=['w','g','m','y'],inf_rad=[4,5],null_pont=False)
            fr4 = multiple_aircrafts(max_pot=mp, grid_size=grid_size, max_size=ms, plt_colors=['b', 'r', 'm', 'g'],inf_rad=[3,5],
                                    start_sides={'up':nas[0],'down':nas[1],'left':nas[2],'right':nas[3]}, null_pont=False)

            w1 = Waypoint(row=16, column=5, max_pot=7, grid_size=grid_size, size=3, inf_rad=5)
            w2 = Waypoint(row=15, column=20, max_pot=7, grid_size=grid_size, size=3, inf_rad=5)
            w3 = Waypoint(row=25, column=5, max_pot=7, grid_size=grid_size, size=3, inf_rad=5)
            w4 = Waypoint(row=27, column=20, max_pot=7, grid_size=grid_size, size=3, inf_rad=5)

            sob1 = Stat_Obstruction(row=8, column=13, max_pot=12, grid_size=grid_size,size=1,inf_rad=4)

            mob1 = Mov_Obstruction(row=20, column=5, max_pot=17, grid_size=grid_size, size=1, inf_rad=3, mob_rate=10, mob_span=1, mut_rate=2, mut_span=2)
            mob2 = Mov_Obstruction(row=25, column=19, max_pot=18, grid_size=grid_size, size=1, inf_rad=3, mob_rate=10,
                                   mob_span=1, mut_rate=2, mut_span=2)

            field = [tma]
            flights = fr3 + fr4
            waypoints = [w1,w2,w3,w4]
            stat_obstructions = [sob1]
            mov_obstructions = [mob1,mob2]

            simulate(flights,waypoints,stat_obstructions,mov_obstructions,field,show_single_clip,total_tstep=t_steps)

            # save objects in simulation
            if show_single_clip:
                sim_objects = store_objects(sim_objects_container, field, flights, waypoints, stat_obstructions, mov_obstructions)
            # put functions to calculate any desired quantities here
            # e.g. flows+=[cal_flow(15,fr1)]
            flows += [cal_flow(t_steps, flights)]

            # get transit time for repetitions
            avg_transit_time += [field[0].avg_transit_time]

        # find average of quantities here
        # e.g. avg_flow=sum(flows)/len(flows)
        total_flows += [flows]
        all_avg_transit_time += [avg_transit_time]
    # avg_flow = sum(flows) / len(flows)
    # make_sim_video(2)
