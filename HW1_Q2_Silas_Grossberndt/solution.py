#Solving a cryptoarthematic problem 
#Goal is to get solution
#take input, set constraints, get solution

class constraints:
    def __init__(self, l):
        self.problem=dict(l)
    def assign_value(self, a):
        l=dict(self.problem)
        for l in letters.keys():
            i=0
            if type(letters[l]) != int:
                l=a[i]
                i+=1
        return l
    def check_if_valid(self,key_array):
        l=self.problem
        rhs=0
        lsh=0
        if len(k)==3:
            rhs=l[k[2]]+l[k[1]]
            lhs=l[k[0]]
        elif len(k)==2:
            rhs=l[k[1]]
            lhs=l[k[0]]
        else:
            rhs=1
            lhs=l[k[0]]
        if rhs==lhs or rhs==lhs+10:
            return True
        else:
            return False



    #take in input in three lines and pipe into three arrays
word1=input("Enter line 1> (e.g. SAVE) ")
word2=input("\n Enter line 2> (e.g. MORE) ")
word3=input("\n Enter line 2> (e.g. MONEY) ")
    
if len(word3) > max(len(word1), len(word2))+1 or len(word3) < max(len(word1), len(word2)):
    print("no solution under addition is possible for this puzzle", len(word3))
    quit()
    #then, the idea is that I will define a dictionary, where the key is the letter and it points to the value for the letter. Start with the letters as an array 
alphabet=["A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M", "N", 
            "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"]
letters=dict()
words=word1+word2+word3 
for letter in alphabet:
    is_in=letter in words
    if is_in:
        letters[letter]=range(10) #sets all possible values for the lettes
    #now we have a dictionary where the keys are the letters and the vaules are the values
    #next goal is to represent constraints 
    #should be that word1[i]+word2[i]=(word3[i] or word3[i]+10)
    #where the above are their corresponding values in the dictionary 
    
    #so we get max(len(word1), len(word2)) constraints + 1 constraint of each letter being different
  
    #easisest constraint comes from the fact that if word3 is longer than either word, the front value must be 1 
ls = list(letters.keys())
if len(word3) == max(len(word1), len(word2))+1:
    val=letters[str(word3[0])]
    letters[str(word3[0])]=1
    for l in ls:
        if l!= word3[0] and letters[l]==1:
            letters[l]=val #just to keep the values uniquely defined
    #additionally, we reject anything with over 10 letters as there is no solution based on definition of the problem
if len(ls) >10:
        print("Too many unique letters to find a solution")
        quit()
  
#best thing to do is to set up a search tree where the state is given by assigned values I guess?
#so this is really a CSP probably, but I feel like this may actually be the goal next time
#instead, assign a value to one of the letters, then assign more values until you reach a position that is not allowed
solution=dict()
c=constraints(letters)
is_solution==false
tried_sols=[]
for i in range(len(word3)): #largest of the arrays by requirement 
    l=[word3[i]]
    try:
        l.append(word2[i])
    except:
        pass
    try:
        l.append(word1[i])
    except:
        pass
    if len(l)==2 and l[0]!=l[1]:
        print("no solution possible")
        quit()
    for k in range(10): #this is going to be the slowest way of doing this I know
        for j in range(len(ls)):
            a=range(len(ls))
            solution.append(c.assign_values(a))
            if hash(solution[-1]) in tried_sols: 
                continue
            else:
                tried_sols.append(hash(solution[-1])) #makes sure not to try same solution twice
            is_good=c.check_if_valid(l)
            if is_good==False:
                solution.pop()
            h=a[j]
            a[j]+=k 
            a[j]=a[j]%10
            for m in range(len(ls)): #makes sure we don't assign the same value twice
                if m!=j and a[m]==a[j]:
                    a[m]=h
for j in range(len(word3)): #testing the solutions to see if they are valid for all of the contraints
    l=[word3[i]]
    try:
        l.append(word2[i])
    except:
        pass

    try:
        l.append(word1[i])
    except:
        pass
        
    for s in range(len(solution)):
        c=constraint(solution[s])
        t=c.is_valid(l)
        if t==False:
            solution.pop(s)
all_vals=dict(letters)
for k in all_vals.keys():
    all_vals[k]=[]
for s in solution:
    sk=s.keys()
    for k in sk:
        all_vals[k].append[s[k]]
for k in all_vals.keys():
    print(k, ": \" ", all_vals[k], " \" ")


                    
#going to use a DFS to search the state space 
#aiming to return all possible solutions to the state

        

