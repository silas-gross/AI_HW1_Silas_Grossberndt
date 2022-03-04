import copy as cp
import resource as rs
import sys
import time

class StateRep: #class to represent the state of the board
 
    def __init__(self, board, counter, move_given):
         #tracks the state by assigning a number to the state that is given by order in which the state is visited
        self.parent=[Board(board), counter]
        self.moves=move_given #moves to get to state
        tuple_board=tuple(l for l in self.parent[0].board[0])
        self.sn=hash(tuple_board) #hashing allows for faster lookup

    def child_nodes(self, board_val): #generates first order child nodes from a board
        #this expands the state parent 
        #makes it as a call to expand rather than a value to call so that way I can get the children as a staterep withotu loading all downstream children later
        children =[]
        bv=list(board_val) #protect against action changing value via pointer
        chboard=Board(bv)
        nextboard=[]
        #print(chboard.right(chboard.board), "right move")
        if chboard.up(chboard.board) !=[0] :
            nextboard=chboard.up(chboard.board)
            children.append(StateRep(nextboard,0, self.moves+["Up"]))
        if chboard.down(chboard.board) != [0]:
            nextboard=chboard.down(chboard.board)
            children.append(StateRep(nextboard,0, self.moves+["Down"]))
        if chboard.left(chboard.board) != [0]:
            nextboard=chboard.left(chboard.board)
            children.append(StateRep(nextboard,0, self.moves+["Left"]))
        if chboard.right(chboard.board) != [0]:
            nextboard=chboard.right(chboard.board)
            children.append(StateRep(nextboard,0, self.moves+["Right"]))
        return children
    def compare_state_to_board(self, board_to_compare): #compares the list form of the two boards
        listform=self.parent[0].board[0]
        if listform==board_to_compare:
            #print("matched")
            return True 
        else:
            return False
    def __hash__(self): #allows for easy comparison of states
        return self.sn
    def __eq__(self, obj): #not nescessary as it is doubled by compare to board, but it is useful to have definied potentially, faster if both are in satrep, but faster to match against a list then load a second staterep
        return self.sn==ob.sn
class Board: #class to give a board representation 
    
    def __init__(self, input_board):
        self.board=[input_board, self.convert_to_matrix(input_board)]
        #board contains representation of itself in list an matrix form to allow the zero position searcher to more quickly find a zero, but also makes the representation of the moves simpler to handle logically 
        self.zero_pos=self.get_zero_position(input_board) #gets the postion of the zero to give on which numbers the actions may act
        self.zero_matrix_pos=self.get_zero_matrix_position(self.zero_pos) #just a bit of algebra to get the zero postion in row/collumn form
    def convert_to_matrix(self, inp): #gives matrix from as it is easier to think about the action on the matrix
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
    #this is accomplished by first figuring out where zero is for this board
    #then, determines if this zero position allows the requested move to be taken
    #copys the array so as to not cause an issue 
    #moves the zero tile with the value needed
    #then returns the list form of the board
    def down(self, board):
        pos=self.get_zero_position(board[0])
        matrix_pos=self.get_zero_matrix_position(pos)
        if matrix_pos[0] !=2: #makes sure move can be taken
            moved_val=0
            matrix=[]
            for i in range(len(board[1])):
                rm=[]
                for j in range(len(board[1][i])):
                    rm.append(board[1][i][j])
                matrix.append(rm) 
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
            #print("changed", matrix)
            del moved_val
            return listform
        else:
            return [0]

    def up(self, board):
        pos=self.get_zero_position(board[0])
        matrix_pos=self.get_zero_matrix_position(pos)
        if matrix_pos[0] !=0: #makes sure move can be taken
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
                return [0]
            matrix[matrix_pos[0]][matrix_pos[1]]=moved_val
            matrix[matrix_pos[0]-1][matrix_pos[1]]=0
            listform=[item for sublist in matrix for item in sublist] #flattens board to an output
            del moved_val
            return listform
        else:
            return[0]

    def left(self, board):
        pos=self.get_zero_position(board[0])
        matrix_pos=self.get_zero_matrix_position(pos)
        #print(pos,matrix_pos)
        if matrix_pos[1]!=0: #makes sure move can be taken
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
                return [0]
            matrix[matrix_pos[0]][matrix_pos[1]]=moved_val
            matrix[matrix_pos[0]][matrix_pos[1]-1]=0
            listform=[item for sublist in matrix for item in sublist] #flattens board to an output
         #   print(matrix)
            del moved_val
            return listform
        else:
            return [0] 
    def right(self, board):
        pos=self.get_zero_position(board[0]) #get position of blank tile
        matrix_pos=self.get_zero_matrix_position(pos)
        #print(pos,matrix_pos)
        if matrix_pos[1]!=0: #makes sure we can take move
            matrix=[]
            for i in range(len(board[1])):
                rm=[]
                for j in range(len(board[1][i])):
                    rm.append(board[1][i][j])
                matrix.append(rm)
         
            if len(matrix) < matrix_pos[0]:
                return [0]
            if matrix_pos[1] ==2:
                return [0]
            moved_val=matrix[matrix_pos[0]][matrix_pos[1]+1] #holds value to move
            if moved_val==0:
                return [0] #makes sure we are not moving two zeros, this was an issue earlier
            matrix[matrix_pos[0]][matrix_pos[1]]=moved_val #moves value
            matrix[matrix_pos[0]][matrix_pos[1]+1]=0 #moves zero to new position
            listform=[item for sublist in matrix for item in sublist] #flattens board to an output
         #   print(matrix)
            del moved_val
            return listform
        else:
            return [0] 
            
    def manhattan(self): #defining the manhattan heuristic, which is going to be done in a sort of hard coded manner here
        #I am using the distance measure in the matrix
        #Not using an init version here so as to only calculate when needed, prevents bogging down of bfs and dfs 
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

def bfs(board, goal_board,bad_board):
    initial_state=StateRep(board,0, [])
    at_goal=False
    solution=[]
    max_depth=1
    node=0
    to_visit=[initial_state] #queue
    states_visited=set() #hash list
    while len(to_visit) >0: #if states are still in queue
        current=to_visit.pop(0) #removes first node in queue
        if current.sn in states_visited: #check if state has been visited before
            continue
        states_visited.add(current.sn) #adds record of visite state to hashlist
        node+=1
        if max_depth<len(current.moves):
            max_depth=len(current.moves) #increases max depth, should only ever be 1 more than depth
        if current.compare_state_to_board(goal_board): #checks if at goal
            solution=current.moves.copy() #gets path to goal
            at_goal=True
            break #if goal has been found, shortest path to goal has also been found for bfd
        elif current.compare_state_to_board(bad_board): #checks if solution was invalid
            solutions=list(current.moves)
            break #exit
        else:
            child= list(current.child_nodes(current.parent[0].board[0])) 
            #gets children
            for c in child:
                to_visit.append(c) #adds to queue in UDLR 
    if at_goal==False: #returns path that lead to knowing this was an invalid config
        output=[]
        solution=["Found board with odd transpiositions, board will not reach gal. Found test board at path: "]
        print("This state doesn't reach goal")
        output.append(solution)
        output.append(len(solution))
        output.append(node)
        output.append(len(solution))
        output.append(max_depth)
        return output

    output=[]
    output.append(solution) #gives path to goal
    output.append(len(solution)) #cost=depth of path
    output.append(node)
    output.append(len(solution))
    output.append(max_depth+1)
    return output




def dfs(board, goal_board, bad_board):
    initial_state=StateRep(board,0, [])
    at_goal=False 
    solution=[]
    max_depth=1
    node=0
    to_visit=[initial_state]
    states_visited=set()
    while len(to_visit) >0:
        current=to_visit.pop() #removes from queue in UDLR
        if current.sn in states_visited:
            states_visited.pop() #allows for searching of path properly to get the state via multiple path configs, so have to remove states from exhausted paths
            continue
        node+=1
        if max_depth<len(current.moves):
            max_depth=len(current.moves)+1
        if current.compare_state_to_board(goal_board):
            if len(current.moves)<len(solution) or solution==[]: #check if solution found is the shortest path to the solution
                solution=list(current.moves) #if solution found is shorter path than current solution, set new solution path to shorter path
            at_goal=True
            #no break here because we want to find all allowable solution paths
        elif current.compare_state_to_board(bad_board): #looks to see if config was invalid
            #this method is not the most efficient way of cehcking this but it is very unobtrusive to the main goal
            solutions=list(current.moves)
            break
        else:
            child= list(current.child_nodes(current.parent[0].board[0]))
            states_visited.add(current.sn) #adds hash
            child=child[::-1] #allows us to append to the stack un RLDU to be aple to pop off in proper order 
            for c in child:
               to_visit.append(c)
    if at_goal==False: #sets oputput if config was non-solvable
        output=[]
        solution=["Found board with odd transpiositions, board will not reach gal. Found test board at path: "]
        print("This state doesn't reach goal")
        output.append(solution)
        output.append(len(solution))
        output.append(node)
        output.append(len(solution))
        output.append(max_depth)
        return output

    output=[]
    output.append(solution) #gives path to goal
    output.append(len(solution)) #cost=depth of path
    output.append(node)
    output.append(len(solution))
    output.append(max_depth)
    return output

def ast(board, goal_board, bad_board):
    initial_state=StateRep(board,0, [])
    at_goal=False
    solution=[]
    cost_of_path=0
    max_depth=1 
    node=0
    to_visit={initial_state.parent[0].manhattan() : [initial_state]} #dictionary to handle priority queue
    states_visited=set() #faster lookup of hash
    highest_priority=initial_state.parent[0].manhattan() #sets initially to be modified later
    while len(to_visit) >0:
        if len(to_visit[highest_priority])==0: #checks to see if there are any states with the current priority
            to_visit.pop(highest_priority)
            priorities=list(to_visit.keys())
            priorities.sort()
            highest_priority=priorities[0]
        current=to_visit[highest_priority].pop() #opens deepest node at this priority
        if current.sn in states_visited: #checks to see if this state has been previously visited
            continue
        states_visited.add(current.sn) #adds the hash value to do faster lookup of previously visited states
        node+=1 #measures number of nodes visited
        if max_depth<len(current.moves): #increase max depth if necessary
            max_depth=len(current.moves)
        if current.compare_state_to_board(goal_board): #checks if at goal
            solution=list(current.moves)
            cost_of_path=highest_priority+1 #sets cost of path as f +1 to make the final step to the goal state
            at_goal=True
            break #A* should return optimal path, as shown in lecture, hence no need to expand further nodes
        elif current.compare_state_to_board(bad_board): #checks to see if the board has reached a bad config thus telling us that the original config will never reach goal
            solutions=list(current.moves)
            cost_of_path=highest_priority+1
            break
        else:
            child= [el for el in current.child_nodes(current.parent[0].board[0])]
            for c in child: #loads the children nodes and inserts into priority
                cost=c.parent[0].manhattan()+highest_priority+1
                if cost in to_visit: #if other nodes with this priority exist, add onto the frontier with this priority
                    #not sure if this should be in UDLR order, so just adding by deepest node to be processed first
                    to_visit[cost].append(c) 
                else:
                    to_visit[cost]=[c] #adds the node at the new priority
                    highest_priority=min(cost, highest_priority) #determines the new value of priority to visit
    if at_goal==False: #if board doesn't reach a valid configuration, give the path that lead to the bad board
        output=[]
        solution=["Found board with odd transpiositions, board will not reach gal. Found test board at path: "]
        print("This state doesn't reach goal")
        output.append(solution)
        output.append(len(solution))
        output.append(node)
        output.append(len(solution))
        output.append(max_depth)
        return output

    output=[]
    output.append(solution) #gives path to goal
    output.append(cost_of_path) 
    output.append(node)
    output.append(len(solution)) #depth of path is length of solution
    output.append(max_depth)
    return output

#take in arguments of the method and board
method=str(sys.argv[1])
boardin=sys.argv[2]
board=[]
#loads the board to a list form
for i in range(len(boardin)):
    if boardin[i]==',':
        continue
    else:
        board.append(int(boardin[i]))
start=time.time()
#this block tests to see if the entered board configuration is in the expected form
is_valid=True
if len(board) !=9:
    print(len(board))
    is_valid=False
else:    
    for i in range(9):
        is_valid=i in board
        if is_valid==False:
            print(i)
            break

#create output file in write mode
outfile=open("output.txt", "w")

goal_board=[0,1,2,3,4,5,6,7,8] #current goal, leaving variable as to make code later more generalizable

bad_board=[0,2,1,3,4,5,6,7,8] # this is a board with an odd # of transpositions.
#if this board is reached, that means that the intial board must also have had aodd number of transpositions and thus in unsolvable
out=[]

#process the input and begin running in mode requested
if is_valid==False:
    print("Invalid board configuration detected. Please check board configuration (should be a list of numbers 0-8 using all without repeats")
    quit()
elif method=="BFS" or method=="bfs" or method=="Bfs":
    print( "Solving board using breadth first search method")
    out=bfs(board, goal_board, bad_board)
elif method=="DFS" or method=="dfs" or method=="Dfs":
    print ("Solving board using depth first search method")
    out=dfs(board, goal_board, bad_board)
elif method=="A*" or method=="a*" or method=="Ast" or method=="ast" or method=="A *" or method=="a *" or method=="A star" or method=="a star":
    print( "Solving board using A* method with Manhattan Heuristic")
    out=ast(board, goal_board, bad_board)
else:
    print ("Method not recognized. \n Please enter method (bfs, dfs or ast). You entered: " +method)
    

end=time.time()
time_run=end-start #running time for code (aside from outputting)
out.append(time_run)
out.append(rs.getrusage(0).ru_maxrss)
#explicitly changes path into expected output form
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



