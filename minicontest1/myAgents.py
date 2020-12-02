# myAgents.py
# ---------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

from game import Agent,Directions,Actions
from searchProblems import PositionSearchProblem
from util import Queue


def createAgents(num_pacmen, agent='MyAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]


def breadthFirstSearch(problem,direction):
    """add direction vector"""
    "*** YOUR CODE HERE ***"
    from game import Directions
    CLOSED = []
    OPEN = Queue()
    if (problem.isGoalState(problem.getStartState())):
        return Directions.STOP
    OPEN.push(((problem.getStartState(), []), []))  # (state...)path)

    while not OPEN.isEmpty():
        CurrentSta, Path = OPEN.pop()
        if problem.isGoalState(CurrentSta[0]):
            return Path  # goal
        elif CurrentSta[0] not in CLOSED:
            CLOSED.append(CurrentSta[0])
            for newSta in getSuccessors(problem,CurrentSta[0],direction):
                OPEN.push((newSta, Path + [newSta[1]]))


def getSuccessors(self, state,direction):
    """
    add direction vector
    """
    successors = []
    for action in direction:
        x,y = state
        dx, dy = Actions.directionToVector(action)
        nextx, nexty = int(x + dx), int(y + dy)
        if not self.walls[nextx][nexty]:
            nextState = (nextx, nexty)
            cost = self.costFn(nextState)
            successors.append( ( nextState, action, cost) )

    # Bookkeeping for display purposes
    self._expanded += 1 # DO NOT CHANGE
    if state not in self._visited:
        self._visited[state] = True
        self._visitedlist.append(state)

    return successors

class MyAgent(Agent):
    """
    Implementation of your agent.
    """
    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        # startPosition = gameState.getPacmanPosition(self.index)
        # food = gameState.getFood()
        # walls = gameState.getWalls()
        global agent_num
        agent_num = gameState.getNumAgents()

        #智能体的数量两个以下及两个
        if agent_num <= 2:
            if self.first == False:
                if(self.index%2==0):
                    self.direction_choice = self.direction_choice1
                else:
                    self.direction_choice = self.direction_choice2
                self.first = True

            problem = AnyFoodSearchProblem(gameState, self.index)
            return breadthFirstSearch(problem, self.direction_choice)
        #智能体的数量两个以上
        else:
            if self.first == False:
                if (self.index % 4 == 0):
                    self.direction_choice = self.direction_choice1
                elif (self.index % 4 == 1):
                    self.direction_choice = self.direction_choice2
                elif (self.index % 4 == 2):
                    self.direction_choice = self.direction_choice3
                else:
                    self.direction_choice = self.direction_choice4
                self.first = True

            problem = AnyFoodSearchProblem(gameState, self.index)
            return breadthFirstSearch(problem, self.direction_choice)

        #util.raiseNotDefined()

    def getAction(self, state):
        """
        Returns the next action the agent will take
        """
        global food_num
        global start
        if start==False:
            food_num = state.getNumFood()
            start = True
        if (len(self.path)==0):#此时需要再进行决策
            if finish == False:
                self.path = self.findPathToClosestDot(state)
                next = self.path[0]
                del self.path[0]
                return next
            else:
                return Directions.STOP#停止
        else:#此时只需要按照原来的决策运动
            next = self.path[0]
            del self.path[0]
            return next

        #return self.findPathToClosestDot(state)[0]
        #raise NotImplementedError()

    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """
        global target
        target = []
        global finish
        finish = False
        global start
        start = False
        self.path = []
        global agent_num
        #四种不同的方向顺序
        self.direction_choice1 = [Directions.NORTH, Directions.WEST, Directions.SOUTH, Directions.EAST]
        self.direction_choice2 = [Directions.SOUTH, Directions.EAST, Directions.NORTH, Directions.WEST]
        self.direction_choice3 = [Directions.EAST, Directions.NORTH, Directions.WEST, Directions.SOUTH]
        self.direction_choice4 = [Directions.WEST, Directions.NORTH, Directions.EAST, Directions.SOUTH]
        self.first = False

        #util.raiseNotDefined()

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """
    def __init__(self, gameState, agentIndex):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()
        self.position = gameState.getPacmanPosition(agentIndex)
        self.agentIndex = agentIndex
        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition(agentIndex)
        #self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE
        self.gameState = gameState

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        global target
        global finish
        global food_num
        x,y = state
        if state not in target and self.food[x][y]==True:
            target.append((x,y))
            if len(target)==food_num:
                finish = True
            return True
        return False
    #原本想更改代价函数，但是发现得不偿失
    def costFn(self,state):
        # x,y = state
        # if (x,y) in self.gameState.getPacmanPositions():
        #     return 4
        return 1
        #return self.food[x][y]
        #util.raiseNotDefined()

