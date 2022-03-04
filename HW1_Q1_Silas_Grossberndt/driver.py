import copy as cp
import resource as rs
import sys
import time

class StateRep: #class to represent the state of the board
 
    def __init__(self, board, counter, move_given):
         #tracks the state by assigning a number to the state that is given by order in which the state is visited
        self.parent=[Board(board), counter]
        #self.child=self.child_nodes(self.parent[0].board[0])
        self.moves=move_given #moves to get to state
        tuple_board=tuple(l for l in self.parent[0].board[0])
        self.sn=hash(tuple_board)

    def child_nodes(self, board_val): #generates first order child nodes from a board
        #this expands the state parent and puts the children on the frontier to later be expanded
        children =[]
        bv=board_val.copy() #protect against action changing value via pointer
        chboard=Board(bv)
        nextboard=[]
        #print(chboard.right(chboard.board), "right move")
        if chboard.up(chboard.board) !=[0] :
            nextboard=chboard.up(chboard.board)
            children.append(StateRep(nextboard,0, self.moves+["Up"]))
        if chboard.down(chboard.board) != [0]:
            nextboard=chboard.down(chboard.board)
            children.append(StateRep(nextboard,0, self.moves+["Down"]))
        #print(chboard.board)
        if chboard.left(chboard.board) != [0]:
            nextboard=chboard.left(chboard.board)
            children.append(StateRep(nextboard,0, self.moves+["Left"]))

        if chboard.right(chboard.board) != [0]:
            nextboard=chboard.right(chboard.board)
            children.append(StateRep(nextboard,0, self.moves+["Right"]))

        #print(children, len(children))
        return children
    def compare_state_to_board(self, board_to_compare): #compares the list form of the two boards
        listform=self.parent[0].board[0]
        if listform==board_to_compare:
            #print("matched")
            return True 
        else:
            return False
    def __hash__(self):
        return self.sn
    def __eq__(self, obj):
        return self.sn==ob.sn
    def dfshash(self):
        m=self.moves.copy()
        m.append(self.parent[0].board[0])
        dtuple=tuple(tuple(l for l in a) for a in m)
        return hash(dtuple)
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
            return listform
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
        pos=self.get_zero_position(board[0])
        matrix_pos=self.get_zero_matrix_position(pos)
        #print(pos,matrix_pos)
        if matrix_pos[1]!=0:
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
            moved_val=matrix[matrix_pos[0]][matrix_pos[1]+1]
            if moved_val==0:
                return [0]
            matrix[matrix_pos[0]][matrix_pos[1]]=moved_val
            matrix[matrix_pos[0]][matrix_pos[1]+1]=0
            listform=[item for sublist in matrix for item in sublist] #flattens board to an output
         #   print(matrix)
            del moved_val
            return listform
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

def bfs(board, goal_board):
    initial_state=StateRep(board,0, [])
    at_goal=False
    solution=[]
    max_depth=1
    node=0
    to_visit=[initial_state]
    states_visited=set()
    while len(to_visit) >0:
        current=to_visit.pop(0)
        if current.sn in states_visited:
            continue
        states_visited.add(current.sn)
        node+=1
        if max_depth<len(current.moves):
            max_depth=len(current.moves)
        if current.compare_state_to_board(goal_board):
            solution=current.moves.copy() 
            at_goal=True
            break
        else:
            child= list(current.child_nodes(current.parent[0].board[0]))
            for c in child:
                to_visit.append(c)
    if at_goal==False:
        output=[]
        output.append(["Doesn't Reach Goal"])
        output.append(max_depth)
        output.append(node)
        output.append(max_depth)
        output.append(max_depth)
        print("State doesn't reach goal")
        return output

    output=[]
    output.append(solution) #gives path to goal
    output.append(len(solution)) #cost=depth of path
    output.append(node)
    output.append(len(solution))
    output.append(max_depth+1)
    return output




def dfs(board, goal_board):
    initial_state=StateRep(board,0, [])
    at_goal=False
    solution=[]
    max_depth=1
    node=0
    to_visit=[initial_state]
    states_visited=set()
    while len(to_visit) >0:
        current=to_visit.pop()
        if current.sn in states_visited:
            states_visited.pop()
            continue
        node+=1
        if max_depth<len(current.moves):
            max_depth=len(current.moves)+1
        if current.compare_state_to_board(goal_board):
            if len(current.moves)<len(solution) or solution==[]:
                solution=list(current.moves)
            at_goal=True
            print(len(to_visit))
        else:
            child= list(current.child_nodes(current.parent[0].board[0]))
            states_visited.add(current.sn)
            child=child[::-1]
            for c in child:
               to_visit.append(c)
    if at_goal==False:
        output=[]
        print("This state doesn't reach goal")
        output.append(["Doesn't Reach Goal"])
        output.append(max_depth)
        output.append(node)
        output.append(max_depth)
        output.append(max_depth)
        return output

    output=[]
    output.append(solution) #gives path to goal
    output.append(len(solution)) #cost=depth of path
    output.append(node)
    output.append(len(solution))
    output.append(max_depth)
    return output

def ast(board, goal_board):
    initial_state=StateRep(board,0, [])
    at_goal=False
    solution=[]
    cost_of_path=0
    max_depth=1
    node=0
    to_visit={initial_state.parent[0].h : [initial_state]}
    states_visited=set()
    highest_priority=initial_state.parent[0].h
    while len(to_visit) >0:
        if len(to_visit[highest_priority])==0:
            to_visit.pop(highest_priority)
            priorities=list(to_visit.keys())
            priorities.sort()
            highest_priority=priorities[0]
        current=to_visit[highest_priority].pop()
        if current.dfshash() in states_visited:
            continue
        states_visited.add(current.dfshash())
        node+=1
        if max_depth<len(current.moves):
            max_depth=len(current.moves)
        if current.compare_state_to_board(goal_board):
            solution=list(current.moves)
            cost_of_path=highest_priority+1
            at_goal=True
            break
        else:
            child= [el for el in current.child_nodes(current.parent[0].board[0])]
            for c in child:
                cost=c.parent[0].h+highest_priority+1
                if cost in to_visit:
                    to_visit[cost].append(c)
                else:
                    to_visit[cost]=[c]
                    highest_priority=min(cost, highest_priority)

    if at_goal==False:
        output=[]
        output.append(["Doesn't Reach Goal"])
        output.append(highest_priority)
        output.append(node)
        output.append(max_depth)
        output.append(max_depth+1)
        print("State doesn't reach goal")
        return output

    output=[]
    output.append(solution) #gives path to goal
    output.append(cost_of_path) #cost=depth of path
    output.append(node)
    output.append(len(solution))
    output.append(max_depth+1)
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



