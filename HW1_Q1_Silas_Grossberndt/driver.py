import numpy as np
import resource as rs
import sys
import time

class StateRep: #class to represent the state of the board
    n_states=
    def __init__(self, board, counter):
         #tracks the state by assigning a number to the state that is given by order in which the state is visited
        self.parent=[Board(board), counter]
        self.child=child_nodes()
    def child_nodes(self): #generates first order child nodes from a board
        #this expands the state parent and puts the children on the frontier to later be expanded
        children =[]
        if self.parent.up() !=0 :
                children.append([self.parent.up(), "Up"])
        if self.parent.right() != 0:
            children.append([self.parent.right(), "Right"])
        if self.parent.down() !=0:
            children.append([self.parent.down(), "Down"])
        if self.parent.left() !=0:
            children.append([self.parent.left(), "Left"])
        return children
    def compare_state_to_board(self, board_to_compare):
        if self.parent[0,0]==board_to_compare: #compares the list form of the two boards
            return true
        else:
            return false
    def get_state: #allows for easy checking to see if state has been visited yet
        return counter
class Board: #class to give a board representation 
    
    def __init__(self, input_board):
        self.board=[input_board, convert_to_matrix(input_board)]
        #board contains representation of itself in list an matrix form to allow the zero position searcher to more quickly find a zero, but also makes the representation of the moves simpler to handle logically 
        self.zero_pos=get_zero_position() #gets the postion of the zero to give on which numbers the actions may act
        self.zero_matrix_pos=get_zero_matrix_position() #just a bit of algebra to get the zero postion in row/collumn form
    def convert_to_matrix(self, inp):
        matrix=[]
        row=[]
        for i in range(3):
            for j in range(3):
                k=i+j
                row.append(inp[k])
            matrix.append(row)
        return matrix
    #these four methods give board that results from taking an action 
    def up(self):
        if self.zero_matrix_pos[0] !=2:
            moved_val=self.board[1][self.zero_matrix_pos[0]+1][self.zero_matrix_pos[1]]
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]]=moved_val
            self.board[1][self.zero_matrix_pos[0]+1][self.zero_matrix_pos[1]]=0
            self.board[0]=[item for sublist in self.board[1] for item in sublist] #flattens board to an output
            del moved_val
            return self.board
        else return 0

    def down(self):
        if self.zero_matrix_pos[0] !=0:
            moved_val=self.board[1][self.zero_matrix_pos[0]-1][self.zero_matrix_pos[1]]
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]]=moved_val
            self.board[1][self.zero_matrix_pos[0]-1][self.zero_matrix_pos[1]]=0
            self.board[0]=[item for sublist in self.board[1] for item in sublist] #flattens board to an output
            del moved_val
            return self.board
        else return 0


    def left(self):
        if self.zero_matrix_pos[1] !=0:
            moved_val=self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]-1]
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]]=moved_val
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]-1]=0
            self.board[0]=[item for sublist in self.board[1] for item in sublist] #flattens board to an output
            del moved_val
            return self.board
        else return 0


    def right(self):
        if self.zero_matrix_pos[1] !=2:
            moved_val=self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]+1]
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]]=moved_val
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]+1]=0
            self.board[0]=[item for sublist in self.board[1] for item in sublist] #flattens board to an output
            del moved_val
            return self.board
        else return 0


    def get_zero_position(self): #finds the zero in the list as it is slightly faster than itterating over nested lists
        list_form=self.board[0]
        pos=0
        for i in range(len(list_form)):
            if list_form[i]==0:
                pos=i
                break
        return pos
    def get_zero_matrix_postition(self): #does algebra to return the list
        j=self.zero_pos%3
        i=int(self.zero_pos/3)
        return [i,j]
class SearchStratagies:
    def __init__(self, method, state, goal):
        self.method=method
        self.output=[]
        self.end_condition=goal
        self.state_to_expand=state
    def breadth(self): #this is a single layer breadth search
        childstates=self.state_to_expand.child
        for i in range(len(childstates)):
            if childstates[i][0][0]==goal:
                output.append([childstates[i][1], "End"])
                break
            else output.append(childstates[i][1]) 
            #outputs move leading to the child states
        return output 
    def depth(self, branch_number):
        childstates=self.state_to_expand.child
        if childstates[branch_number][0][0]==goal:
            output.append([childstates[branch_number][1], "End"])
        else output.append(childstates[branch_number][1])
        return output
    def astar(self):
        a

def bfs_itterate(node_number, state_board, goal_board, moves, state_path):
    node_number=node_number+1
    curr_state=StateRep(state_board, node_number)
    for i in range(len(state_path)):
        if curr_state.compare_state_to_board(state_path[i]):
            node_number=node_number-1
            return False
    state_path.append(state_board)
    searchstate=SearchStratgies("bfs", curr_state, goal_board)
    smoves=searchstate.breadth()
    for j in range(len(smoves)) moves.append(smoves)
    if len(moves[-1]) !=1:
        return True
    else return False

def bfs(board, goal_board):
    initial_state=StateRep(board, 0)
    child_states=inital_state.child
    moves=[]
    state_paths=[board]
    path_moves=[]
    output=[]
    cost_of_path=0  
    max_depth=0
    node_number=0
    at_goal=False
    if initial_state.compare_state_to_board(goal_board):
        moves.append(["None"])
        at_goal=True
    at_goal=bfs_iterate(node_number, board, goal_board, moves, state_paths)
    path_moves=moves
    while at_goal==False:
        cs=[] #holds frontier to expand
        cost_of_path=cost_of_path+1 #set maxium path length to then refine below
        for i in range(len(child_states)):
            at_goal=bfs_iterate(node_number, child_states[i][0], goal_board, moves, state_paths[i])
            cstaterep=StatesRep(child_state[i][0], node_number) #right now I am not properly finding unique states
            cs.append(cstaterep)
            if at_goal:
                break
        child_states=[]
        tstate_path=state_paths
        state_paths=[]
        tpath_moves=path_moves
        path_moves=[]
        for i in range(len(cs)): #sets up the next set of child states
            child_states.append(cs[i].child)
            for j in range(len(child_states)):
                path=tstate_path[i].append(child_states[j][0]) #creates a flat array of path
                state_paths.append(path) #new array of paths
                move=tpath_moves[i].append(child_states[j][1])
                path_moves.append(move) #flattens to an array of arrays from a depth-diimensional array
    final_move_set=[]
    #this next block is to select the shortest path and get deepest depth 
    for i in range(len(state_paths)):
        pl=len(state_paths[i])
        if pl < cost_of_path:
            cost_of_path=pl
            final_move_set=path_move[i]
        if len(state_paths[i]) > max_depth:
            max_depth=pl
    #outout of the final results, minus ram and time which are calculated in the main body
    output.append(final_move_set)
    output.append(cost_of_path)
    output.append(node_number)
    output.append(cost_of_path)
    output.append(max_depth)
    return output


def dfsitterate(node_number, board, goal_board, path, moves, branch_number):
    node_number=node_number+1
    current_state=StateRep(board, node_number)
    for i in range(len(path)):
        #check to see that the state has not been visited in this path
        if current_state.compare_state_to_board(path[i]):
            node_number=node_number-1
            return "looped" #need three cases as to move back up tree to search deeper
    path.append(board)
    searchstate=SearchStratagies("dfs", current_state, goal_board)
    moves.append(search_state.depth(branch_number)) #adds moves corresponding to the first unvisited branch of the tree
    if len(moves[-1]) != 1 
        return "goal"
    else return "keep going"

def dfs(board, goal_board):
    initial_state=StateRep(board, 0)
    child_states=inital_state.child
    current_state=StateRep(child_states[0][0],1)
    oldstate=initial_state
    moves=[]
    state_paths=[board]
    path_moves=[]
    output=[]
    cost_of_path=0
    max_depth=0
    node_number=0
    at_goal=False
    branch_number=0 
    path_number=0
    if initial_state.compare_state_to_board(goal_board):
        moves=["None"]
        path_moves=["None"]
        at_goal==True
    while at_goal==False:
        status=dfsitterate(node_number, current_state.parent[0], goal_board, state_paths[path_number], path_moves[path_number], branch_number) #itterates one step down a single path, returns status which can then be used to decided further iterations
        if status=="looped":
            branch_number=branch_number+1 #moves over one branch to the "right"
            path_number=path_number+1 #calls this a new path
            current_state=StateRep(oldstate.child[branch_number][0], node_number)
            state_paths[path_number]=state_paths[path_number-1]
            path_moves[path_number-1]
        if status=="goal":
            path_moves[branch_number].append(moves[-1])
            state_paths[branch_number].append(current_state.child[branch_number])
            at_goal=True
        if status=="keep going"
            path_moves[path_number].append(moves[-1])
            state_paths[path_number].append(current_state.child[branch_number])
            old_state=current_state
            current_state=StateRep(current_state.child[branch_number][0], node_number)
            branch_number=0 #goes back to the looking at the "left most"
            
    final_move_set=[]
    for i in range(len(state_paths)):
        pl=len(state_paths[i])
        if pl < cost_of_path:
            cost_of_path=pl
            final_move_set=path_move[i]
        if len(state_paths[i]) > max_depth:
            max_depth=pl
    output.append(moves)
    output.append(cost_of_path)
    output.append(node_number)
    output.append(cost_of_path)
    output.append(max_depth)
    return output


def ast(board):
    initial_state=StateRep(board, 0)
    child_states=inital_state.child
    moves=[]
    state_paths=[board]
    path_moves=[]
    output=[]
    cost_of_path=0
    max_depth=0
    node_number=0
    at_goal=False

    final_move_set=[]
    for i in range(len(state_paths)):
        pl=len(state_paths[i])
        if pl < cost_of_path:
            cost_of_path=pl
            final_move_set=path_move[i]
        if len(state_paths[i]) > max_depth:
            max_depth=pl
    output.append(moves)
    output.append(cost_of_path)
    output.append(node_number)
    output.append(cost_of_path)
    output.append(max_depth)
    return output
#take in arguments of the method and board
method=sys.argv[1] 
board=sys.argv[2]
start=time.time()
#create output file in write mode
outfile=open("output.txt", "w")
goal_board=[0,1,2,3,4,5,6,7,8] #current goal, leaving variable as to make code later more generalizable
out=[]
#process the method to determine approach 
if method=="BFS" or method=="bfs" or method=="Bfs":
    print "Solving board using breadth first search method"
    out=bfs(board, goal_board)
if method=="DFS" or method=="dfs" or method=="Dfs":
    print "Solving board using depth first search method"
    out=dfs(board, goal_board)
if method=="A*" or method=="a*" or method=="Astar" or method=="astar" or method=="A *" or method=="a *" or method=="A star" or method=="a star":
    print "Solving board using A* method with Manhattan Heuristic"
    out=ast(board, goal_board)
else:
    print "Method not recognized. \n Please enter method (bfs, dfs or a*)"
    return 0

end=time.time()
time_run=end-start #running time for code (aside from outputting)
out.append(time_run)
outfile.write("path_to_goal: " + out[0])
outfile.write("\n cost_of_path: " +out[1])
outfile.write("\n nodes_expanded: "+out[2])
outfile.write("\n search_depth: "+out[3])
outfile.write("\n max_search_depth: " + out[4])
outfile.write("\n running_time: " + out[5])
outfile.write("\n max_ram_usage: " +out[6])
outfile.close()
return 0

