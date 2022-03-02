import numpy as np
import resource as rs
import sys
import time

class StateRep: #class to represent the state of the board
 
    def __init__(self, board, counter):
         #tracks the state by assigning a number to the state that is given by order in which the state is visited
        self.parent=[Board(board), counter]
        self.child=self.child_nodes()
    def child_nodes(self): #generates first order child nodes from a board
        #this expands the state parent and puts the children on the frontier to later be expanded
        children =[]
        if self.parent[0].up() !=0 :
                children.append([self.parent[0].up(), "Up"])
        if self.parent[0].down() != 0:
            children.append([self.parent[0].down(), "Down"])
        if self.parent[0].down() !=0:
            children.append([self.parent[0].left(), "Left"])
        if self.parent[0].left() !=0:
            children.append([self.parent[0].right(), "Right"])
        return children
    def compare_state_to_board(self, board_to_compare):
        if self.parent[0].board[0]==board_to_compare: #compares the list form of the two boards
            return True 
        else:
            return False
    #def get_state: #allows for easy checking to see if state has been visited yet
     #   return counter
class Board: #class to give a board representation 
    
    def __init__(self, input_board):
        self.board=[input_board, self.convert_to_matrix(input_board)]
        self.h=self.manhattan()
        #board contains representation of itself in list an matrix form to allow the zero position searcher to more quickly find a zero, but also makes the representation of the moves simpler to handle logically 
        self.zero_pos=self.get_zero_position() #gets the postion of the zero to give on which numbers the actions may act
        self.zero_matrix_pos=self.get_zero_matrix_position() #just a bit of algebra to get the zero postion in row/collumn form
    def convert_to_matrix(self, inp):
        matrix=[]
        for i in range(3):
            row=[]
            for j in range(3):
                k=3*i+j
                row.append(inp[k])
            matrix.append(row)
        return matrix
    #these four methods give board that results from taking an action 
    def up(self):
        if self.zero_matrix_pos[0] !=2:
            moved_val=self.board[1][self.zero_matrix_pos[0]+1][self.zero_matrix_pos[1]]
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]]=moved_val
            self.board[1][self.zero_matrix_pos[0]+1][self.zero_matrix_pos[1]]=0
            self.board[0]=[]
            for i in range(len(self.board[1])):
                for j in range(len(self.board[1][i])):
                    self.board[0].append(self.board[1][i][j])
            del moved_val
            return self.board
        else:
            return 0

    def down(self):
        if self.zero_matrix_pos[0] !=0:
            moved_val=self.board[1][self.zero_matrix_pos[0]-1][self.zero_matrix_pos[1]]
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]]=moved_val
            self.board[1][self.zero_matrix_pos[0]-1][self.zero_matrix_pos[1]]=0
            self.board[0]=[item for sublist in self.board[1] for item in sublist] #flattens board to an output
            del moved_val
            return self.board
        else:
            return 0


    def left(self):
        if self.zero_matrix_pos[1] !=0:
            moved_val=self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]-1]
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]]=moved_val
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]-1]=0
            self.board[0]=[item for sublist in self.board[1] for item in sublist] #flattens board to an output
            del moved_val
            return self.board
        else:
            return 0


    def right(self):
        if self.zero_matrix_pos[1] !=2:
            moved_val=self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]+1]
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]]=moved_val
            self.board[1][self.zero_matrix_pos[0]][self.zero_matrix_pos[1]+1]=0
            self.board[0]=[item for sublist in self.board[1] for item in sublist] #flattens board to an output
            del moved_val
            return self.board
        else:
            return 0

    def manhattan(self): #defining the manhattan heuristic, which is going to be done in a sort of hard coded manner here
        #I am using the distance measure in the matrix
        htemp=0 
        for i in range(len(self.board[1])):
            for j in range(len(self.board[1])):
                k = self.board[1][i][j] #gets value of matrix at row i collumn j 
                cdif = abs((k%3) - j)%3 #diffence in postion of column #cant pass through the back, hence absolute value
                rdif = abs((int(k/3)-i)) #diffence in postion of rows 
                htemp+=cdif+rdif #manhattan measure is just row and column differnce summed, is admissible, see readme
        return int(htemp)
    def get_zero_position(self): #finds the zero in the list as it is slightly faster than itterating over nested lists
        list_form=self.board[0]
        pos=0
        for i in range(len(list_form)):
            if list_form[i]==0:
                pos=i
                break
        return pos
    def get_zero_matrix_position(self): #does algebra to return the zero's position in matrix from that in the list
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
            if childstates[i][0]==self.end_condition:
                self.output.append([childstates[i][1], "End"])
                break
            else:
                self.output.append(childstates[i][1]) 
            #outputs move leading to the child states
        return self.output 
    def depth(self, branch_number): #this is opening one note and going one layer down in depth
        childstates=self.state_to_expand.child
        if childstates[branch_number][0][0]==self.end_condition:
            self.output.append([childstates[branch_number][1], "End"])
        else:
            self.output.append(childstates[branch_number][1])
        return self.output
    def astar(self, max_allowed): 
        #searchs for nodes that are the next priority by looking at a single layer to find nodes that can be expanded
        #since the g score is cosntant at 1, we can just worry about the heuristic
        #aim is to output the score for each node in a child that is allowed at that particluar score
        childstates=self.state_to_expand.child
        allowed_nodes=[]
        for i in range(len(childstates)):
            if childstates[i][0][0]==self.end_condition:
                return [childstates[i][0][0],0, childstates[i][0][1]]
            b=Board(childstates[i][0])
            h=int(b.h)
            if h<=max_allowed:
                allowed_nodes.append(childstates[i][0],h, childstates[i][1])
        hmin=max_allowed+1
        next_node=[]
        for i in range(len(allowed_nodes)):
            if allowed_nodes[i][1]<hmin:
                hmin = allowed_nodes[i][1]
                next_node=allowed_nodes[i]
        if next_node==[]:
            return [self.state_to_expand.parent[0].board[0], -1, "NULL"]
        else:
            return next_node

        

def bfs_iterate(node_number, state_board, goal_board, moves, state_path):
    curr_state=StateRep(state_board, node_number)
    for i in range(len(state_path)):
        if curr_state.compare_state_to_board(state_path[i]):
            #node_number=node_number-1
            return False
    state_path.append(state_board)
    searchstate=SearchStratagies("bfs", curr_state, goal_board)
    smoves=searchstate.breadth()
    for j in range(len(smoves)):
        moves.append(smoves)
    if len(moves[-1]) !=1 and moves[-1][-1]=="End":
        return True
    else:
        return False

def bfs(board, goal_board):
    initial_state=StateRep(board, 0)
    child_states=initial_state.child
    print(child_states[1][0][1])
    moves=[]
    state_paths=[board]
    path_moves=[]
    output=[]
    cost_of_path=0  
    max_depth=0
    node_number=0
    at_goal=False
    if initial_state.compare_state_to_board(goal_board)==True:
        moves.append(["None"])
        at_goal=True
    at_goal=bfs_iterate(node_number, board, goal_board, moves, state_paths)
    print(at_goal)
    path_moves=moves
    print(moves)
    while at_goal==False:
        cs=[] #holds frontier to expand
        cost_of_path=cost_of_path+1 #set maxium path length to then refine below
        print([type(child_states[0][0][0]), type(goal_board), type(state_paths), type(node_number)])
        for i in range(len(child_states)):
            node_number+=1
            at_goal=bfs_iterate(node_number, child_states[i][0][0], goal_board, moves, state_paths)
            cstaterep=StateRep(child_states[i][0][0], node_number)
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
                path=tstate_path[i].append(child_states[j][0][0]) #creates a flat array of path
                state_paths.append(path) #new array of paths
                move=tpath_moves[i].append(child_states[j][0][1])
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
    if len(moves[-1]) != 1:
        return "goal"
    else:
        return "keep going"

def dfs(board, goal_board):
    initial_state=StateRep(board, 0)
    child_states=initial_state.child
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
        status=dfsitterate(node_number, current_state.parent[0].board[0], goal_board, state_paths[path_number], path_moves[path_number], branch_number) #itterates one step down a single path, returns status which can then be used to decided further iterations
        if status=="looped":
            branch_number=branch_number+1 #moves over one branch to the "right"
            path_number=path_number+1 #calls this a new path
            current_state=StateRep(oldstate.child[branch_number][0], node_number)
            state_paths[path_number]=state_paths[path_number-1]
            path_moves[path_number-1]
        elif status=="goal":
            path_moves[branch_number].append(moves[-1])
            state_paths[branch_number].append(current_state.child[branch_number])
            at_goal=True
        elif status=="keep going":
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
def get_children_min_cost(child_states):
    cost=10000
    new_state_number=0
    for i in range(len(child_states)):
        b=Board(child_states[i][0])
        if int(b.h) <cost:
            cost=int(b.h)
            if i>new_state_number:
                new_state_number=i
    return [cost, new_state_number]
def get_max_cost(older_states, current_state, cost): #this gets the maximum allowable cost to move forwards 
    current_states_children=current_state.child
    cn=ast_itterate(current_states_children)
    child_cost=cn[0]+cost
    old_cost=10000
    depth=len(older_states)
    for i in range(depth):
        depth_state=older_states[depth-i-1]
        for j in range(len(depth_state)):
            old_state_cost=depth_state[j][1]
            depth_states=depth_state[j][0]
            for k in range(len(depth_states)):
                old_state_cost+=depth_states[k][1]
            if old_state_cost<old_cost:
                old_cost=old_state_cost
    return min([child_cost, old_state_cost])

def ast_itterate(older_states, current_state, goal_board, cost):
    ss=SearchStratagies("astar", current_state, goal_board)
    mcost=get_max_cost(older_states, current_state, cost)
    next_node=ss.astar(mcost)
    return next_node

def ast(board, goal_board):
    initial_state=StateRep(board, 0)
    child_states=initial_state.child
    moves=[]
    state_paths=[board]
    path_moves=[]
    output=[]
    cost_of_path=0
    max_depth=0
    depth_of_path=1
    node_number=0
    at_goal=False
    states=[initial_state]
    cost=10000
    new_state_number=0
    otherboards_and_cost=[]
    board_step=[]
    for i in range(len(child_states)):
        b=Board(child_states[i][0][0])
        print(b.h)
        if int(b.h) <cost:
            cost=int(b.h)
            if i>new_state_number:
                new_state_number=i
    states.append([StateRep(child_states[new_state_number][0][0], node_number+1), cost+initial_step.parent[0].h+1])
    moves.append(child_states[new_state_number][1])
    for i in range(len(child_states)):
        if i!= new_state_number:
            b=Board(child_states[i][0])
            board_step.append([child_states[i][0], int(b.h)])
    otherboards_and_cost.append([board_step, int(initial_step.parent[0].h)])
    board_step=[]
    dummy_depth=1
    while at_goal==False:
        depth_of_path+=1
        dummy_depth+=1
        current_state=states[-1][0]
        node=ast_itterate(otherboards_and_cost, current_state, states[-1][1])
        if node[1]==0:
            cost_of_path=states[-1][1]+1
            moves.append(node[2])
            at_goal=True
        elif node[1]==-1:
            current_state=states[-2][0]
            moves.pop()
            if dummy_depth >max_depth:
                max_depth=dummy_depth
            dummy_depth=dummy_depth-1
        else:
            current_state=StateRep(node[0], node_number+1)
            moves.append(node[2])
            states.append([current_state, node[1]+1])

    #need to keep a record of all the parent states to be able to check costs 
    #otherboards has index of depth-1
    
    output.append(moves)
    output.append(cost_of_path)
    output.append(node_number)
    output.append(len(moves))
    output.append(max_depth)
    return output
#take in arguments of the method and board
method=str(sys.argv[1])
boardin=sys.argv[2]
board=[]
for i in range(len(boardin)):
    if boardin[i]==',':
        continue
    else:
        board.append(int(boardin[i]))
start=time.time()
#create output file in write mode
outfile=open("output.txt", "w")
goal_board=[0,1,2,3,4,5,6,7,8] #current goal, leaving variable as to make code later more generalizable
out=[]
#process the method to determine approach 
if method=="BFS" or method=="bfs" or method=="Bfs":
    print( "Solving board using breadth first search method")
    out=bfs(board, goal_board)
elif method=="DFS" or method=="dfs" or method=="Dfs":
    print ("Solving board using depth first search method")
    out=dfs(board, goal_board)
elif method=="A*" or method=="a*" or method=="Ast" or method=="ast" or method=="A *" or method=="a *" or method=="A star" or method=="a star":
    print( "Solving board using A* method with Manhattan Heuristic")
    out=ast(board, goal_board)
else:
    print ("Method not recognized. \n Please enter method (bfs, dfs or ast)" +method)
    

end=time.time()
time_run=end-start #running time for code (aside from outputting)
out.append(time_run)
out.append(rs.getrusage(0).ru_maxrss)
path="["
for i in range(len(out[0])):
    path+="'"+out[0][i]+"'"
    if i!=len(out[0])-1:
        path+=","
path+="]"
outfile.write("path_to_goal: " + path)
outfile.write("\n cost_of_path: " +str(out[1]))
outfile.write("\n nodes_expanded: "+str(out[2]))
outfile.write("\n search_depth: "+str(out[3]))
outfile.write("\n max_search_depth: " + str(out[4]))
outfile.write("\n running_time: " + str(out[5]))
outfile.write("\n max_ram_usage: " +str(out[6]))
outfile.close()



