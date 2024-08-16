Use different search algorithms to optimize traffic of air transport. Goal will be to optimize different quantities measured to judge good traffic.
Some of these quantities include
- Average Velocity during simulation (local)...
- No of Landings in a simulation (local)
- Flow (global)
- Arrival Transit Time (global)...

Intelligent Agent: 
- Aircraft

## Optimization Approaches
### Breadth-First Search

Start: Position of aircraft at beginning of simulation as defined by user

End: End of simulation or cell with the highest potential in TMA

Goal: Quantities measured in simulation
- when aircraft lands
- at end of simulation

Node: The attributes of the node
    
    State: Potential of cell aircraft occupies and simulation time elapsed

    Parent: Previous immediate Potential of cell aircraft occupies

    Action: Aircraft movement towards cell with highest potential in TMA or destination
    
    Path Cost: Measured traffic's local and global quantity from node to goal-state

Frontier: Conditions for queuing nodes
    
    Neighbours to be considered are those with the least potential that are higher than potential of current cell

#### How BFS will be applied to Aircraft Cellular Automata Model
The default mode for the movement of aircraft is movement from when one cell of lower potential to the next cell with a nearest higher
potential to that of the current cell. In a scenario whereby more than one destination cell has the same nearest higher potential to
the current cell, the aircraft moves to the first valid cell it encounters from a clockwise stance.

BFS is applied to instances of these competing choices described to ensure that the cell which results in the optimum path is chosen.
An optimum path in this model is the path that gives optimal result for global and local traffic quantities being observed during the simulation
as defined by the user. 