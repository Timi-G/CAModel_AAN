from pontential_field import Aircraft, Waypoint, Stat_Obstruction,Mov_Obstruction,\
                            TMA, simulate, rand_aircrafts

'''
Create flight, waypoint, static obstructions and mobile obstructions objects f1,f2,f3,w1,w2,sob1,mob1...
and include all objects needed in respective experiment list

________________________________________________________________
#2 Implementation B Of Cellular Automata Model

create multiple randomly positioned aircrafts using rand_aircrafts func.:
no_aircrafts: number of a_crafts, max_pot:max potential any of the a_craft can have
max_size: max size any of the a_crafts can have, plt_colors:list of colors that will represent the flight paths

display info of randomly generated a_crafts using disp_ran_acraft_info()
get list of parameters on randomly generated a_crafts using rand_acraft_info()

get aggregate positions of flight with f1.agg_pos,f2.agg_pos...
get flight transition time with f1.avg_tnstime,f2.avg_tnstime...
get dens of resultant field with tma.dens
get velocity with tma.vel and tma.avg_vel
'''

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    grid_size = [10, 15]

    tma = TMA(row=6,column=6,max_pot=50,grid_size=grid_size)

    f1 = Aircraft(row=5,column=9,max_pot=9,grid_size=grid_size,size=1,plt_color='b',null_pont=True)
    f2 = Aircraft(row=3,column=5, max_pot=10, grid_size=grid_size, size=1,plt_color='r')
    f3 = Aircraft(row=8,column=14, max_pot=7, grid_size=grid_size, size=2,plt_color='g')
    f4 = Aircraft(row=10, column=9, max_pot=5, grid_size=grid_size, size=2, plt_color='w')

    fr1 = rand_aircrafts(no_aircrafts=50,max_pot=5,grid_size=grid_size,max_size=2,plt_colors=['c','r','m','y'],null_pont=False)

    w1 = Waypoint(row=6,column=11,max_pot=6,grid_size=grid_size,size=2)

    sob1 = Stat_Obstruction(row=7,column=3,max_pot=8,grid_size=grid_size,size=1)

    mob1 = Mov_Obstruction(row=7, column=3, max_pot=8, grid_size=grid_size, size=1, travel_rate=5)

    field = [tma]
    flights = fr1+[f1, f2, f3, f4]
    waypoints = [w1]
    stat_obstructions = [sob1]
    mov_obstructions = [mob1]


    simulate(flights,waypoints,stat_obstructions,mov_obstructions,field,total_tstep=25)
