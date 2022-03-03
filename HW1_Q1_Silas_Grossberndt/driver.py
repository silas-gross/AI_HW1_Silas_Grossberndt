import copy as cp
import resource as rs
import sys
import time

class StateRep: #class to represent the state of the board
 
    def __init__(self, board, counter):
         #tracks the state by assigning a number to the state that is given by order in which the state is visited
        self.parent=[Board(board), counter]
        self.child=self.child_nodes(self.parent[0].board[0])
    def child_nodes(self, board_val): #generates first order child nodes from a board
        #this expands the state parent and puts the children on the frontier to later be expanded
        children =[]
        bv=board_val.copy() #protect against action changing value via pointer
        chboard=Board(bv)
        print(chboard.right(chboard.board), "right move")
        if chboard.up(chboard.board) !=[0] :
                children.append([chboard.up(chboard.board), "Up"])
        if chboard.down(chboard.board) != [0]:
            children.append([chboard.down(chboard.board), "Down"])
        #print(chboard.board)
        if chboard.left(chboard.board) != [0]:
            chboard.append([chboard.left(chboard.board), "Left"])
        if chboard.right(chboard.board) != [0]:
            children.append([chboard.right(chboard.board), "Right"])
        #print(children, len(children))
        return children
    def compare_state_to_board(self, board_to_compare): #compares the list form of the two boards
        listform=self.parent[0].board[0]
        if listform==board_to_compare:
            print("matched")
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
        self.zero_pos=self.get_zero_position(input_board) #gets the postion of the zero to give on which numbers the actions may act
        self.zero_matrix_pos=self.get_zero_matrix_position(self.zero_pos) #just a bit of algebra to get the zero postion in row/collumn form
    def convert_to_matrix(self, inp):
        matrix=[]
        k=0
        if type(inp)==int:
            return matrix
        l=len(inp)
        #print(inp)
        if len(inp)<=2:
            return [[0]]
        for i in range(3):
            row=[]
            for j in range(3):
                k=3*i+j
                if k>len(inp)-1:
                    print("k too large", k)
                    break
                row.append(inp[k])
            matrix.append(row)
        return matrix
    #these four methods give board that results from taking an action 
    def down(self, board):
        pos=self.get_zero_position(board[0])
        matrix_pos=self.get_zero_matrix_position(pos)
        if matrix_pos[0] !=2:
            moved_val=0
            matrix=[]
            for i in range(len(board[1])):
                rm=[]
                for j in range(len(board[1][i])):
                    rm.append(board[1][i][j])
                matrix.append(rm)
            m2=board[1].copy()
       #     print("initial: ", matrix)
            if len(matrix) <=matrix_pos[0]+1:
                return [0]
            if len(matrix[matrix_pos[0]]) < matrix_pos[1]+1:
                return [0]
            moved_val=matrix[matrix_pos[0]+1][matrix_pos[1]]
            if moved_val==0:
                return [0]
            matrix[matrix_pos[0]][matrix_pos[1]]=moved_val
            matrix[matrix_pos[0]+1][matrix_pos[1]]=0
            listform=[item for sublist in matrix for item in sublist] #flattens board to an output
            board[1]=m2
            #print("changed", matrix)
            del moved_val
            return [listform, matrix]
        else:
            return [0]

    def up(self, board):
        pos=self.get_zero_position(board[0])
        matrix_pos=self.get_zero_matrix_position(pos)
        if matrix_pos[0] !=0:
            moved_val=0
           
            matrix=[]
            for i in range(len(board[1])):
                rm=[]
                for j in range(len(board[1][i])):
                    rm.append(board[1][i][j])
                matrix.append(rm)

        #    print(matrix)
            if matrix_pos[0] ==0:
                return [0]
            if len(matrix[matrix_pos[0]]) < matrix_pos[1]+1:
                return [0]
            moved_val=matrix[matrix_pos[0]-1][matrix_pos[1]]
            if moved_val==0:
                return 0
            matrix[matrix_pos[0]][matrix_pos[1]]=moved_val
            matrix[matrix_pos[0]-1][matrix_pos[1]]=0
            listform=[item for sublist in matrix for item in sublist] #flattens board to an output
            del moved_val
            return [listform, matrix]
        else:
            return[0]

    def right(self, board):
        pos=self.get_zero_position(board[0])
        matrix_pos=self.get_zero_matrix_position(pos)
        print(pos,matrix_pos)
        if matrix_pos[1]!=0:
            matrix=[]
            for i in range(len(board[1])):
                rm=[]
                for j in range(len(board[1][i])):
                    rm.append(board[1][i][j])
                matrix.append(rm)
         
            if len(matrix) <=matrix_pos[0]+1:
                return [0]
            if matrix_pos[1] ==0:
                return [0]
            moved_val=matrix[matrix_pos[0]][matrix_pos[1]-1]
            if moved_val==0:
                return matrix
            matrix[matrix_pos[0]][matrix_pos[1]]=moved_val
            matrix[matrix_pos[0]][matrix_pos[1]-1]=0
            listform=[item for sublist in matrix for item in sublist] #flattens board to an output
         #   print(matrix)
            del moved_val
            return [listform, matrix]
        else:
            return [0] 
            
    def left(self, board):
        pos=self.get_zero_position(self.board[0])
        matrix_pos=self.get_zero_matrix_position(pos)
        if matrix_pos[1] !=2:
            matrix=[]
            for i in range(len(board[1])):
                rm=[]
                for j in range(len(board[1][i])):
                    rm.append(board[1][i][j])
                matrix.append(rm)         
            if len(matrix) <3:
                return [0]
            if len(matrix[matrix_pos[0]]) <3:
                return [0]
            moved_val=matrix[matrix_pos[0]][matrix_pos[1]+1]
            if moved_val==0:
                return [0]
            matrix[matrix_pos[0]][matrix_pos[1]]=moved_val
            matrix[matrix_pos[0]][matrix_pos[1]+1]=0
            listform=[item for sublist in matrix for item in sublist] #flattens board to an output
            del moved_val
            return [listform, matrix]
        else:
            return [0]



    def manhattan(self): #defining the manhattan heuristic, which is going to be done in a sort of hard coded manner here
        #I am using the distance measure in the matrix
        htemp=0 
        k=0
        for i in range(len(self.board[1])):
            for j in range(len(self.board[1][i])):
                k = self.board[1][i][j] #gets value of matrix at row i collumn j
                cdif = abs((k%3) - j)%3 #diffence in postion of column #cant pass through the back, hence absolute value
                rdif = abs((int(k/3)-i)) #diffence in postion of rows 
                htemp+=cdif+rdif #manhattan measure is just row and column differnce summed, is admissible, see readme
        return int(htemp)
    def get_zero_position(self, in_list_board): #finds the zero in the list as it is slightly faster than itterating over nested lists
        list_form=in_list_board
        pos=0
        if type(list_form)==int:
            return pos
        for i in range(len(list_form)):
            if list_form[i]==0:
                pos=i
                break
        return pos
    def get_zero_matrix_position(self, listpos): #does algebra to return the zero's position in matrix from that in the list
        j=listpos%3
        i=int(listpos/3)
        #print([i,j])
        return [i,j]
class SearchStratagies:
    def __init__(self, method, goal):
        self.method=method
        self.output=[]
        self.end_condition=goal
    def breadth(self,state_board): #this is a single layer breadth search
        currstate=StateRep(state_board, 0)
        childstates=cp.deepcopy(currstate.child)
        out_moves=[]
        for i in range(len(childstates)):
            if childstates[i][0]==self.end_condition:
                out_moves.append([childstates[i][1], "End"])
                break
            else:
                out_moves.append(childstates[i][1]) 
            #outputs move leading to the child states
            self.output=out_moves
        return self.output 
    def depth(self, branch_number): #this is opening one note and going one layer down in depth
        childstates=self.state_to_expand.child.copy()
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

        

def bfs_iterate(state_board, goal_board, moves, state_path):
    for i in range(len(state_path)):
        if state_board==state_path[i]:
            #node_number=node_number-1
            return -1
    searchstate=SearchStratagies("bfs", goal_board)
    smoves=searchstate.breadth(state_board).copy()
    #print(smoves)
    for j in range(len(smoves)):
        moves.append(smoves)
    if len(moves[-1]) != 1:
        print(moves[-1])
    if len(moves[-1]) !=1 and moves[-1][-1]=="End": #checks if goal has been reached
            return 0
    else:
        return 1
    #so this takes in a current state to look at, a goal state to reach, an matrix of moves and a list of former states in the path
    #then it outputs a condition to tell us if the path has reached a terminus, the goal or keep going
    #moves gets all the allowed moves that take us to the children state of the node 

def bfs(board, goal_board):
    initial_state=StateRep(board, 0)
    child_states=cp.deepcopy(initial_state.child)
    moves=[]
    paths_moves=[]
    output=[]
    cost_of_path=0  
    max_depth=0
    node_number=0
    at_goal=False
    if initial_state.compare_state_to_board(goal_board)==True:
        moves.append(["at Goal"])
        at_goal=True
    val=bfs_iterate(board, goal_board, moves, [])
    if val==0:
        at_goal=True
        moves.append("Goal achived")
    node_number+=len(child_states)
    paths_moves=moves
    visited_states=[board] #stores all boards for visited states
    final_move_set=[] #output for final path
    states_to_visit=[]
    for i in range(len(child_states)):
        states_to_visit.append([[board], child_states[i]]) #sets up list of states that we are going to visit from a single node
    #print(moves)
    while at_goal==False:
        cs=[] #holds frontier to expand
        cost_of_path=cost_of_path+1 #set maxium path length to then refine below
        print("depth of path ", cost_of_path)
        #if(cost_of_path==2):
         #   at_goal=True
        if len(states_to_visit)==0:
            at_goal=True #if there are no other states in queue to visit, we have reached an end
        for i in range(len(states_to_visit)):
            #visit all states from the parent node
            #states to visit will hold the path of states to the parent node and then, we will flatten this to all arrays later down
            children=[]
            for j in range(len(states_to_visit[i][1])):
                children.append(states_to_visit[i][1][j][0][0]) #passes the list description of the board to the children array 
            previous_states=states_to_visit[i][0].copy()
            if type(previous_states)==str:
                continue
            #this is the list of children in this node and the states to prevent visiting the same node twice
            move=[]
            path_moves=[[el for el in row] for row in paths_moves[i]]
            if type(path_moves)==str:
                paths_moves[i]=[path_moves]
                path_moves=[path_moves]
            children_of_children=[]
            if len(children)==0:
                break
            #clears out the held moves to use as a dummy container 
            for j in range(len(children)):
                try: #catch undexpected
                    node_number+=1
                    current_board=children[j].copy()
                    if current_board==goal_board:
                        print("reached goal")
                        at_goal=True
                        break

                #now this expands the nodes on the frontier
                    val=bfs_iterate(current_board, goal_board, moves, visited_states)
                    if val==-1:
                        node_number=node_number-1
                        print("looped on child ", i, j)
                        continue
                    if val==0:
                        previous_states.append(current_board, goal_board)
                        for k in range(len(moves)):
                            p=path_moves.copy()
                            p.append(moves[k]) #captures the moves that were taken up to geting to the end
                            moves[k]=p.copy()
                        final_move_set=p.copy()
                        print("reached end")
                        at_goal==True
                        break
                    if val==1:
                        visited_states.append(current_board)
                        for k in range(len(moves)):
                            p=path_moves.copy()
                            p.append(moves[k])
                            moves[k]=p.copy()
                        previous_states.append(current_board)
                        c=StateRep(current_board, node_number)
                        children_of_children.append(c.child) #adds the children of the current node to later add to the stack
                except BaseException:
                    continue

            cs.append(children_of_children) #adds the new children to the frontier
            #then I will need to flatten the array slightly later
            paths_moves[i]=[[el for el in row] for row in moves]
        states_to_visit=[]
        tpaths=[[el for el in row] for row in paths_moves]
        paths_moves=[]
        for i in range(len(cs)): #flattening the frontier back into visit states
            for j in range(len(cs[i])):
                states_to_visit.append([tpaths[i][j], cs[i][j]])
                paths_moves.append(tpaths[i][j])
                
    
    #this next block is to select the shortest path and get deepest depth 
    for i in range(len(paths_moves)):
        if len(paths_moves[i]) > max_depth:
            max_depth=len(paths_moves[i])
    cost_of_path=len(final_move_set)
    #outout of the final results, minus ram and time which are calculated in the main body
    output.append(final_move_set)
    output.append(cost_of_path)
    output.append(node_number)
    output.append(cost_of_path)
    output.append(max_depth)
    return output


def dfs_iterate( board, goal_board, path, moves, branch_number):
    current_state=StateRep(board, 0)
    for i in range(len(path)):
        #check to see that the state has not been visited in this path
        if current_state.compare_state_to_board(path[i][0]):
            return 0 #need three cases as to move back up tree to search deeper
    searchstate=SearchStratagies("dfs", current_state, goal_board)
    moves.append(search_state.depth(branch_number)) #adds moves corresponding to the first unvisited branch of the tree
    if len(moves[-1]) != 1 and moves[-1][-1]=="End":
        return 0
    else:
        return 1

def dfs(board, goal_board):
    initial_state=StateRep(board,0)
    child_states=initial_state.child
    states_visited_board_and_child_number=[[0,0],[initial_state.parent[0].board[0],0]]
    exhausted=[False] #toggle to say if there are more children nodes to explore from this node
    at_goal=False
    moves=[]
    depth=0
    paths_moves=[] #moves taken when traversing a path 
    cost_of_path=0
    max_depth=0
    nodes=0
    current_state=StateRep(board,0)
    while at_goal==False:
            curr_child=states_visited_board_and_child_number[depth][-1][1]
            val=dfs_iterate(current_state.parent[0].board[0], goal_board, states_visited_board_and_child_number, moves, curr_child)
            nchild=len(current_state.child)
            if val==-1:
                if curr_child<nchild:
                    states_visited_board_and_child_number[depth][-1][1] +=1
                    paths_moves.append([])
                else:
                    exhausted[depth]=True
                    if max_depth<depth:
                        max_depth=depth
                    while exhausted[depth]==True:
                        moves.pop()
                        depth=depth-1
                        curr_board=states_visited_board_and_child_number[depth][-1][0]
                        curr_branch=states_visited_board_and_child_number[depth][-1][1]
                        current_state=StateRep(curr_board, 0) 
                        n_old_child=len(current_state.child)
            if val==0:
                node+=1
                depth+=1
                paths_moves[-1].append(moves[-1])
                if len(states_visited_board_and_child_number) <depth : #adds the first element at this depth 
                    states_visited_board_and_child_number.append([[curr_board, 0]])
                else: #adds an additional element at this depth
                    states_visited_board_and_child_number[depth].append([curr_board, 0])
                at_goal=True
            if val==1:
                node+=1
                depth+=1
                paths_moves[-1].append(moves[-1])
                curr_board=current_state.child[0][0][0]
                current_states=StateRep(curr_board, 0)
                if len(states_visited_board_and_child_number) <depth : #adds the first element at this depth 
                    states_visited_board_and_child_number.append([[curr_board, 0]])
                else: #adds an additional element at this depth
                    states_visited_board_and_child_number[depth].append([curr_board, 0])

    output=[]
    output.append(paths_moves[-1]) #gives path to goal
    output.append(depth) #cost=depth of path
    output.append(node)
    output.append(depth)
    output.append(max_depth)


#this is the old def, going from scratch 
def dfs_old(board, goal_board):
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
        #print(b.h)
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



