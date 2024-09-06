from pont_functions import nxt_pos, swi_cord_elem
from cam_airnav_mod import avg_trans_time

'''utility'''
class Node():
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action


class StackFrontier():
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node


class QueueFrontier(StackFrontier):

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node


'''breadth-first'''
def check_average_tns_time(positions,destinations):
    des=swi_cord_elem(destinations)
    journ,avg_tns_t=avg_trans_time(positions,des)
    return avg_tns_t

def check_average_velocity(positions):
    pass

# actions==positions
def check_goals(actions,destinations):
    avg_tt=check_average_tns_time(actions,destinations)

def optimization_bf(pos,poss_pos,tma,goal):
    pass

def best_path(first_pos,field,destinations,goal,pos_not_allowed):
    start = Node(state=field[first_pos[1]-1][first_pos[0]-1], action=first_pos, parent=None)
    frontier = QueueFrontier()
    frontier.add(start)
    explored=[]
    op_att=0
    op_cord=[]
    while True:
        if frontier.empty():
            if not len(op_cord):
                print('no_optimization')
                return first_pos

            print('optimized cord',op_cord)
            return op_cord

        node = frontier.remove()
        next_pos = nxt_pos(field, node.action,pos_not_allowed)
        explored += [node.action]
        # check if next_position is the same as initial

        if node.state == goal:
            actions = []
            while node.parent is not None:
                actions.append(node.action)
                node = node.parent
            # get the cost for reaching goal: average transit time
            att=check_average_tns_time(actions,destinations)
            # assign optimized cord from path with smallest average transit time
            if op_att > att or op_att == 0:
                op_att=att
                if len(actions)>1:
                    op_cord = actions[-2]
                else:
                    op_cord = node.action


        # create child node linked to parents
        for action in next_pos:
            if action not in explored:
                state = field[action[1]-1][action[0]-1]
                child = Node(state=state, action=action, parent=node)
                frontier.add(child)
