from pontential_field import Aircraft, Waypoint, Stat_Obstruction,Mov_Obstruction,\
                            TMA, simulate, multiple_aircrafts, cal_flow, disp_ran_acraft_info, rand_acraft_info

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
get dens of resultant field with tma.dens
get velocity with tma.vel and tma.avg_vel
get flow of a given number of aircraft within specified time steps using cal_flow(t_steps,flights) it takes an optional func. tma 

density is no. of flights/area of tma
velocity is no. of movements throughout the simulation
average velocity is no. of aircraft movement/total time step

show_vis_clip variable is used to set how many sims to run at a given time and which sim should have visualizations
'''

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    clip_no = 0
    grid_size = [20, 25]
    show_vis_clip = [True, False, False]
    # try the convention:
    # show_vis_clip = [True] * 3 + [True] * 5 + [False] * 2

    # declare variables for aircraft generation during repetition of simulation
    no_rand_aircrafts=[3, 2, 1, 2]
    no_aircrafts_from_sides=[[1,0,0,1],[2,2,2,1],[0,0,0,0],[1,1,1,1]]
    max_size=[3,2,4,2]
    max_pot=[5,4,6,10]

    for n,(nra,nas,ms,mp) in enumerate(zip(no_rand_aircrafts,no_aircrafts_from_sides,max_size,max_pot),0):
        # define a collector for desired quantities here
        # e.g. flows=[]
        for no, show_single_clip in enumerate(show_vis_clip, 1):
            '''clip_no is index of video visualization'''
            clip_no=(n*len(show_vis_clip))+no
            # clip_no+=1 if show_single_clip else clip_no

            tma = TMA(row=6,column=6,max_pot=50,grid_size=grid_size)

            # single aircraft object creation still works as shown in f1,f2,f3,f4
            # f1 = Aircraft(row=5,column=9,max_pot=9,grid_size=grid_size,size=1,plt_color='b',null_pont=True)
            # f2 = Aircraft(row=3,column=5, max_pot=10, grid_size=grid_size, size=1,plt_color='r')
            # f3 = Aircraft(row=8,column=14, max_pot=7, grid_size=grid_size, size=2,plt_color='g')
            # f4 = Aircraft(row=10, column=9, max_pot=5, grid_size=grid_size, size=2, plt_color='w')
            
            # fr1 and fr2 are examples of creating multiple aircrafts at once
            # fr1 = multiple_aircrafts(rand_aircrafts=50,max_pot=5,grid_size=grid_size,max_size=2,plt_colors=['c','r','m','y'],null_pont=False)
            # fr2 = multiple_aircrafts(max_pot=5, grid_size=grid_size, max_size=2, plt_colors=['c', 'r', 'm', 'y'],
            #                         start_sides={'up':3,'down':1,'left':2,'right':1}, null_pont=False)

            # dynamic declaration of aircrafts for repetitive simulations only applies to variables fr3 & fr4
            # this convention can be followed to define other multiple aircrafts variables
            fr3 = multiple_aircrafts(rand_aircrafts=nra,max_size=ms,max_pot=mp,grid_size=grid_size,plt_colors=['w','g','m','y'],null_pont=False)
            fr4 = multiple_aircrafts(max_pot=mp, grid_size=grid_size, max_size=ms, plt_colors=['b', 'r', 'm', 'g'],
                                    start_sides={'up':nas[0],'down':nas[1],'left':nas[2],'right':nas[3]}, null_pont=False)

            w1 = Waypoint(row=6,column=11,max_pot=6,grid_size=grid_size,size=2)

            sob1 = Stat_Obstruction(row=7,column=3,max_pot=8,grid_size=grid_size,size=1)

            mob1 = Mov_Obstruction(row=7, column=3, max_pot=8, grid_size=grid_size, size=1, mob_rate=5, mob_span=1, mut_rate=7, mut_span=3)

            field = [tma]
            flights = fr3 + fr4
            waypoints = [w1]
            stat_obstructions = [sob1]
            mov_obstructions = [mob1]

            simulate(flights,waypoints,stat_obstructions,mov_obstructions,field,show_single_clip,clip_no,total_tstep=10)

            # put functions to calculate any desired quantities here
            # e.g. flows+=[cal_flow(15,fr1)]

        # find average of quantities here
        # e.g. avg_flow=sum(flows)/len(flows)
