# AUTONOMOUS AIRCRAFT NAVIGATION
## CELLULAR AUTOMATA MODELLING FOR AUTONOMOUS AIRCRAFT NAVIGATION
CAM which is described as a dynamical system in which space and time are considered to be discrete is being applied to autonomous aircraft navigation.
It generally consists of a regular grid of cells, with individual cells being in a state(s) that gets updated simultaneously in discrete time steps.

To read more on CAM, check this well written thesis/article, which I have no affiliation to titled "Cellular Automata Modeling of En Route and Arrival Self-Spacing for Autonomous
Aircrafts" on www.mwftr.com/Charlesk.html

CAM approach to AAN as applied to this project is made to be as realistic as possible, with cells consisting of objects like aircrafts(different sizes), waypoints, moving obstructions (e.g clouds), static obstructions (e.g tall buildings).
*************************************************************************************************************************************************************************************************************
### About The Experiments
The experiments are carried out in a controlled environment (though virtual in this case) called TMA(Terminal Manoeuvring Area):
1. **cam_air_nav** is the first implementation created:
  - Flight object is instantiated with command `Flight(dept=[x,y], dest=[x,y], size=c)` dept meaning departure, dest meaning destination and size is to set size of aircraft. x and y are coordinates on the TMA
  - `create_flight(north=a,south=b,east=c,west=d,dest=[x,y],size=c,spread=s,aspace=[m,n])` is a function to instantiate large number of Flight object at once. Here there are 'a' number of aircrafts coming from the north, 'b' number of aircrafts coming from the south,'c' is the size of all aircrafts created with this function, 'spread' argument has a multiplier effect on how dispersed aircrafts are across the TMA and 'aspace' if given are exact boundary coordinates within which the aircrafts are positioned i.e [m,0] & [0,n]
  - The Waypoint object is a route for aircrafts while Obstruction objects are avoided during flights.
  - Point objects are referrence points to calculate quantities as desired for the project
  - Simulation can be run using the run_cam_airnav.py file.

2. **cam_pontential_field** is the second implementation: 
All the objects are resolved to individual 'grids' based on the position and potential given by the user. A resultant grid is calculated (note that all individual grid sizes have to be the same to successfully find a resultant grid) and cells in the resultant grid(TMA) have potential forces. Movement of flights tends towards cells with higher potentials with light boxes representing low potentials and darker bozes representing higher potentials
Following the convention in cam_air_nav
  - Aircrafts are created with command `Aircraft(row=a,column=b,max_pot=d,grid_size=grid_size,size=c,plt_color='b',null_pont=True)`, 'a' & 'b' being position of aircraft in the TMA, 'd' is the maximum potential an aircraft grid will have, 'null_pont' which defaults to False and defines if the object should have zero potential in all cells in it's grid.
  - Function `rand_aircrafts()` is used to create a large number of aircrafts at once with random max_pot, size and position.
  - Simulation can be run using the run_cam_pont.py file.

3. **cam_traffic** is the third implementation:
This implementation depicts flights converging as though moving in a runway. Future updates on this will include visualization of the experiment but results like landing rate, velocity and others can still be calculated. 

4. **ga_air_nav** is the fourth implementation:
It utilizes the Genetic Algorithm concept. In this implementation the waypoints is being optimized from one generation to another. At the moment this model does not produce the desired optimization after a given number of gemerations but the visualization applied using OpenCV to depict the movements of flights and other objects in the TMA will be applied across other implmentations in this project.


NOTE: For all these implementations, there are simple instructions on how to run the different experiments just before the `if __name__=='__main__':` codeline.
 
