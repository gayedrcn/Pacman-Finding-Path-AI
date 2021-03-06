# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and Pieter 
# Abbeel in Spring 2013.
# For more info, see http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""
#161180026-Ennur Gaye Dirican-BM455
import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
        Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    root_node = problem.getStartState()
    fringe = util.Stack() #fringe bir "yigin" veriyapisi olsun. sozdekodda "s" olarak tanimlanmistir.
    visitedNodes = set()  # gezilen dugumler dizisi
    startNode = (root_node, []) #baslangic dugumu. sozdekodda "v" olarak tanimlanmistir.

    fringe.push(startNode)

    if fringe.isEmpty():
        raise Exception('Arama basarisiz!')  # Sinir bos mu? Bos olmasi durumunda hata mesaji.

    while not fringe.isEmpty(): #stack bos degilken,
        node, actions = fringe.pop()
        endState = node

        if endState not in visitedNodes: #node kesfedildi olarak etiketlenmemisse, etiketini degistir.
            visitedNodes.add(endState)

            if problem.isGoalState(node):
                return actions
            else:
                successors = problem.getSuccessors(node) #successors gidilebilecek nodelarin listesini tutar.

                for successor in successors:#tum successorlari sinira iter.
                    child_node = actions + [successor[1]]
                    full_path = (successor[0], child_node) #Baslangic dugumunden cocuk dugumun hesaplama yolu
                    fringe.push(full_path)

    return actions
"""
DFS algoritmasinin kodlari icin "Pal,J., Chartterjee, D., Modak,S. (2019). 
DESIGNING OF SEARCH AGENTS USING PACMAN. sayfa 4"'deki sozdekod baz alinmistir.
"""



def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"
    fringe = util.Queue() #BFS kuyruk veriyapisini kullanir.
    visitedNodes = set() # gezilen dugumler dizisi
    fringe.push((problem.getStartState(), [], 0))
    while True:
        if fringe.isEmpty():
            raise Exception('Arama basarisiz!')  # Sinir bos mu? Bos olmasi durumunda hata mesaji.
        node, actions, pathCost = fringe.pop() #sinirdaki en yuzeysel dugumu secer.
        if node not in visitedNodes: #gezilmis dugumler
            visitedNodes.add(node)

            if problem.isGoalState(node):
                return actions
            else:
                successors = problem.getSuccessors(node) #successors gidilebilecek nodelarin listesini tutar.

                for successor in successors: #tum successorlari sinira iter.
                    child_node = actions + [successor[1]]
                    child_path = pathCost + successor[2]
                    full_path = (successor[0], child_node, child_path) #Baslangic dugumunden cocuk dugumun hesaplama yolu

                    fringe.push(full_path)

    return actions
"""
DFS algoritmasinin kodlari icin "Pal,J., Chartterjee, D., Modak,S. (2019). 
DESIGNING OF SEARCH AGENTS USING PACMAN. sayfa 6"'daki sozdekod baz alinmistir.
"""


def uniformCostSearch(problem):
    "Search the node of least total cost first. "

    "*** YOUR CODE HERE ***"
    fringe = util.PriorityQueue()
    visitedNodes = set() # gezilen dugumler dizisi
    fringe.push((problem.getStartState(), [], 0), 0)

    if fringe.isEmpty():
        raise Exception('Arama basarisiz!')  # Sinir bos mu? Bos olmasi durumunda hata mesaji.

    while not fringe.isEmpty():
        node, actions, pathCost = fringe.pop() #en dusuk maliyetli dugumu arar.

        if (node not in visitedNodes) :
            visitedNodes.add(node) #gezilmis dugumler

            if problem.isGoalState(node):
                return actions
            else:
                successors = problem.getSuccessors(node) #successors gidilebilecek nodelarin listesini tutar.

                for successor in successors: #tum successorlari sinira iter.
                    child_node = actions + [successor[1]]
                    child_path = pathCost + successor[2]
                    full_path = (successor[0], child_node, child_path) #Baslangic dugumunden cocuk dugumun hesaplama yolu

                    fringe.update(full_path, child_path)

    return actions
"""
UCS algoritmasinin kodlari icin "Pal,J., Chartterjee, D., Modak,S. (2019). 
DESIGNING OF SEARCH AGENTS USING PACMAN. sayfa 8"'deki sozdekod baz alinmistir.
"""

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    fringe = util.PriorityQueue()
    visitedNodes = set() # gezilen dugumler dizisi
    fringe.push((problem.getStartState(), [], 0), heuristic(problem.getStartState(), problem) + 0)

    if fringe.isEmpty():
        raise Exception('Arama basarisiz!')  # Sinir bos mu? Bos olmasi durumunda hata mesaji.

    while not fringe.isEmpty():
        element = fringe.pop()
        node = element[0]
        actions = element[1]
        pathCost = element[2]
        if problem.isGoalState(node):
            break #aranan dugum bulunduysa,
        else:
            if node not in visitedNodes:
                visitedNodes.add(node)  #yeni dugumleri ekler.
                successors = problem.getSuccessors(node) #successors gidilebilecek nodelarin listesini tutar.
                for successor in successors:
                    child_node = successor[0]
                    child_path = successor[1]
                    child_cost = successor[2]
                    total_path = actions + [child_path]  #Baslangic dugumunden cocuk dugumun hesaplama yolu
                    full_cost = pathCost + child_cost
                    fringe.push((child_node, total_path, full_cost), full_cost + heuristic(child_node, problem))

    return actions

"""
A* algoritmasinin kodlari icin "Pal,J., Chartterjee, D., Modak,S. (2019), 
DESIGNING OF SEARCH AGENTS USING PACMAN. sayfa 10"'daki sozdekod baz alinmistir.
"""
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

