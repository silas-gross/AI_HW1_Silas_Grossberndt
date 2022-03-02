This is the README file for Question 1 of Homework 1 for AI Spring 2022 at CUNY Graduate Center
*This project is written using Python v3.8.5*
#Overview
Broadly, this file aims to be able to take in an 8 puzzle and find the solution using any of Depth First Search, Breadth First Search or A\* algorithms.

The heuristic for the A\* is given by the Manhattan heuristic
#Details
To run the code, input should be in the form 
>python3 driver.py method state
where method is either dfs, bfs or ast (should be case independent)
and then the output will be given in a text file in the form (note the cost is =1/node outside of heuristic)

>path\_to\_goal: ['move1', 'move2', ...]
>cost\_of\_path: cost 
>nodes\_expanded: number\_of\_nodes
>search\_depth: length\_of\_search\_to\_path
>max\_search\_depth: deepest\_point\_of\_tree
>running\_time: in seconds
>max\_ram\_usage: in MB

#Goals Accomplished/Not accomplished
This work has accomplished all major goals and is safe against already solved input in the initial state
