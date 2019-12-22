import math
import random
import copy
import numpy as np


class EightQueensState:
    def __init__(self, queen_locations):
        self.queen_locations = queen_locations
        if (queen_locations.shape[0]!=queen_locations.shape[1]):
            raise Exception(f'Must be square shape, {queen_locations.shape} is not accpected!')
 
        self.chess_board_size = queen_locations.shape[0]
        if(self.chess_board_size!=8):
            raise Exception(f'chess_board_size must be 8, {queen_locations.chess_board_size} is not accpected!')
    def queen_count(self):
        return self.queen_locations.sum()
    def get_hash(self):
        count = int(0)
        sum = int(0)
        for i in self.queen_locations.flatten('C'):
            sum += i * math.pow(2,count)
            count += 1
        return int(sum)

    def is_fail(self):
        statistic = self.check_statistic()
        if (statistic[0]+statistic[1]==(self.chess_board_size*6-2)):
            return False
        else:
            return True
    #check queeen attack count of differet axis
    def check_statistic(self):
        statistic={}
        for i in range(9):
            statistic[i]=0
        for i in range(8):
        #Horizontal
            statistic[int(self.queen_locations[i].sum())]+=1
        #Vertical
            statistic[int(self.queen_locations[:,i].sum())]+=1        
        for i in range(-7,8):
        #Diagonal
            statistic[int(self.queen_locations.diagonal(i).sum())]+=1
            statistic[int(np.fliplr(self.queen_locations).diagonal(i).sum())]+=1
        return statistic

    def __str__(self):
        return (f'{self.queen_locations.__str__()} count: {self.queen_count()}')

    def __eq__(self, other):
        if(other == None):
            return False
        return np.array_equal(self.queen_locations, other.puzzle_locations)


class EightQueensProblem:
    def __init__ (self):
        pass
    def test_goal(self,state):
        if (state.queen_count() ==8 and not state.is_fail()):
            return True
        else:
            return False

    def cost(self,node):
        if (node.parent == None):
            return 1
        return node.parent.cost+1

    def heuristic(self,state):
        #return 0
        statistic = state.check_statistic()
        return 46-statistic[0]

    def test_fail(self,node):
        return node.state.is_fail()
    def get_successors(self,state):
        result=[]
        for action in range(8):
            new_state =EightQueensProblem.transition(state,action)
            if(new_state != None):
                result.append((action,new_state))
        return result  
    @staticmethod
    def transition(state: EightQueensState, action):
        new_state = copy.deepcopy(state)
        for y in range(8):
            if (1 not in new_state.queen_locations[y]):
                new_state.queen_locations[y][action] = 1
                return new_state
        raise Exception ('Cannot add new queen!')