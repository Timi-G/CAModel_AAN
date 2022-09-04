CELLULAR AUTOMATA MODELLING FOR AUTONOMOUS AIRCRAFT NAVIGATION
CAM which is described as a dynamical system in which space and time are considered to be discrete is being applied to autonomous aircraft navigation.
It generally consists of a regular grid of cells, with individual cells being in a state(s) that gets updated simultaneously in discrete time steps.

To read more on CAM, check this well written thesis/article, which I have no affiliation to titled "Cellular Automata Modeling of En Route and Arrival Self-Spacing for Autonomous
Aircrafts" on www.mwftr.com/Charlesk.html

CAM approach to AAN as applied to this project is made to be as realistic as possible, with cells consisting of objects like aircrafts(different sizes), waypoints, moving obstructions (e.g clouds), static obstructions (e.g tall buildings).

************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
The experiments are carried out in a controlled environment (though virtual in this case) called TMA(Terminal Manoeuvring Area)
1. cam_air_nav is the first implementation created.
i) Flight object is instantiated with command "Flight(dept=[x,y], dest=[x,y], size=c)" dept meaning departure, dest meaning destination and size is to set size of aircraft. x and y are coordinates on the TMA
ii) "create_flight(north=a,south=b,east=c,west=d,dest=[x,y],size=c,spread=s,aspace=[m,n])" is a function to instantiate large number of Flight object at once. Here there are 'a' number of aircrafts coming from the north, 'b' number of aircrafts coming from the south...
'c' is the size of all aircrafts created with this function, 'spread' argument has a multiplier effect on how dispersed aircrafts are across the TMA and 'aspace' if given are exact boundary coordinates within which the aircrafts are positioned i.e [m,0] & [0,n]
iii) The Waypoint object is a route for aircrafts while Obstruction objects are avoided during flights.
iv) Point objects are referrence points to calculate quantities as desired for the project

2. cam_pontential_field is the second implementation.
All the objects are resolved to a grid based on the position and potential given by the user. A resultant grid is calculated and cells in the resultant grid(TMA) here have potential forces with flight movement tending towards cells with higher potentials (note that all grid sizes have to be the same to successfully find a resultant grid).
Following the convention in cam_air_nav
i) Aircrafts are created with command "Aircraft(row=a,column=b,max_pot=d,grid_size=grid_size,size=c,plt_color='b',null_pont=True)", 'a' & 'b' being position of aircraft in the TMA, 'd' is the maximum potential an aircraft grid will have, 'null_pont' which defaults to False and defines if the object should have zero potential in all cells in it's grid.
ii) Function rand_aircrafts() is used to create a large number of aircrafts at once with random max_pot, size and position.

3. cam_traffic is the third implementation.
This implementation depicts flights converging as though moving in a runway.


************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
DISCLAIMER: This Project is a PHD thesis for Ikeoluwa Oreoluwa Ogedengbe, however the theories and calculations were interpreted and all codes written by Rotimi Olasehinde(Github handle 'Timi-G').
Agreement was reached and permission granted to share these codes.