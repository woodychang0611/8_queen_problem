import numpy as np
from  random import seed,randrange
from SearchAgent import SearchAgent,SearchMode
from  EightQueens import EightQueensProblem,EightQueensState

seed()
start_locations = np.zeros((8, 8))
start_locations[randrange(8)][randrange(8)]=1
start_state = EightQueensState(start_locations)

problem = EightQueensProblem()

#for _,state in problem.get_successors(start_state):
#    print(state)
#    print(state.is_fail())
#exit(0)

agent = SearchAgent(start_state, problem, SearchMode.A_Star)
solution_node = agent.search()
print(solution_node.state)

agent = SearchAgent(start_state, problem, SearchMode.Greedy_BFS)
solution_node = agent.search()
print(solution_node.state)

agent = SearchAgent(start_state, problem, SearchMode.UCS)
solution_node = agent.search()
print(solution_node.state)

agent = SearchAgent(start_state, problem, SearchMode.IDS)
solution_node = agent.search()
print(solution_node.state)

agent = SearchAgent(start_state, problem, SearchMode.RBFS)
solution_node = agent.search()
print(solution_node.state)

