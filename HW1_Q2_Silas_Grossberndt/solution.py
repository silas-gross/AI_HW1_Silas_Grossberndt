#Solving a cryptoarthematic problem 
#Goal is to get solution
#take input, set constraints, get solution
class states:
    def __init__(self):
        a
class costraints:
    def __init__(self, letter1, letter2, letter3):
        self.problem={letter1:0, letter2:0, letter3:0}


def main():
    #take in input in three lines and pipe into three arrays
    word1=input("Enter line 1> (e.g. SAVE) ")
    word2=input("\n Enter line 2> (e.g. MORE) ")
    word3=input("\n Enter line 2> (e.g. MONEY) ")
    
    if len(word3) >= max(len(word1), len(word2))+1 or len(word3) < max(len(word1), len(word2):
        print("no solution under addition is possible for this puzzle")
        quit()
    #then, the idea is that I will define a dictionary, where the key is the letter and it points to the value for the letter. Start with the letters as an array 
    alphabet=["A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M", "N", 
            "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"]
    letters=dict()
    words=word1+word2+word3
    dummy_val=0 
    for letter in alphabet:
        is_in=letter in words
        if is_in:
            letters[letter]=dummy_val
            dummy_val+=1
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

    for i in range(len(word3)): #largest of the arrays by requirement 
        l.append(word3[i])
        try:
            l.append(word2[i])
        except:

        try:
            l.append(word1[i])
        except:

        if len(l)==2 and l[0]!=l[1]:
            print("no solution possible")
            quit()
        for
        

