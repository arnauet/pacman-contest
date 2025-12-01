# my_team.py
# ---------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

import random
import contest.util as util

from contest.capture_agents import CaptureAgent
from contest.game import Directions
from contest.util import nearest_point

#################
# Helpers #
#################


def null_heuristic(state, problem=None):
    return 0


def depth_first_search(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.get_start_state())
    print("Is the start a goal?", problem.is_goal_state(problem.get_start_state()))
    print("Start's successors:", problem.get_successors(problem.get_start_state()))
    """
    
    # track all visited states to avoid repeated points or doing circles
    explored_state = set()
    
    # use stack for lifo order
    frontier_dfs = util.Stack()
    
    # push initial state with empty path and zero cost    
    start_state = [problem.get_start_state(), [], 0]
    frontier_dfs.push(start_state)
       
    # loop until goal found and pacman won
    while (not frontier_dfs.is_empty()):
        
        # pop most recently added state from stack
        [current_state, path, cost] = frontier_dfs.pop()
        
        # only process if not visited before
        if (not current_state in explored_state): 
            explored_state.add(current_state)
            
            # check if we won
            if (problem.is_goal_state(current_state)):
                return path
                
            # get all possible next states
            successors = problem.get_successors(current_state)
            
            # process each child node
            for child in successors:
                [state, action, cost_state] = child
                
                # skip if already explored or in frontier
                if (not state in explored_state and not frontier_dfs.contains(child)):
                    
                    # we add new action to current path
                    new_path = path + [action]
                    
                    # we update the state
                    new_state = [state, new_path, cost_state]
                    
                    # push child to stack for later exploration
                    frontier_dfs.push(new_state)


def breadth_first_search(problem):
    """Search the shallowest nodes in the search tree first."""
    
    # set to track visited nodes
    explored_bfs = set()
    frontier_bfs = util.Queue()
    
    # we initialize state and empty path
    start_state = [problem.get_start_state(), [], 0]
    frontier_bfs.push(start_state)
        
    # we continue until the queue is empty 
    while (not frontier_bfs.is_empty()):
        
        # unqueue oldest state from front
        [current_state, path, cost] = frontier_bfs.pop()
        
        # we process only non explored states
        if (not current_state in explored_bfs):
            explored_bfs.add(current_state) 
            
            # return path if the goal is achieved
            if (problem.is_goal_state(current_state)):
                return path
            
            # get all neighbors of current state
            successors = problem.get_successors(current_state)
            
            # iterate for all successors
            for child in successors: 
                [state, action, cost_state] = child
                
                # add only new unexplored states
                if (not state in explored_bfs and not child in frontier_bfs.list):
                    new_path = path + [action]
                    new_state = [state, new_path, cost_state]
                    frontier_bfs.push(new_state)


def a_star_search(problem, heuristic=null_heuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    
    # set to track explored states
    a_explored = set()
    a_frontier = util.PriorityQueue()
    
    # p.queue follows f = g(n) + h(n)
    start = problem.get_start_state()
    start_node = [start, [], 0]
    a_frontier.push(start_node, 0)
    
    # loop until frontier is empty
    while (not a_frontier.is_empty()):
        
        # get the lowest cost from the queue and pop
        [current_state, path, cost] = a_frontier.pop()
        
        # check if the state has not been explored
        if (not current_state in a_explored):
            a_explored.add(current_state)
            
            # return if goal found
            if (problem.is_goal_state(current_state)):
                return path
            
            # get all successors
            successors = problem.get_successors(current_state)
            
            # for every successor we evaluate            
            for child in successors:
                [state, action, cost_state] = child 
                
                # we check if worth exploring or not                                
                if (not state in a_explored and not child in a_frontier.heap):
                    
                    # we add the current cost
                    new_cost = cost_state + cost
                    new_path = path + [action]
                    new_state = [state, new_path, new_cost]
                    
                    # we calculate the heuristic value: f(n) = g(n) + h(n)
                    h = new_cost + heuristic(new_state[0], problem)
                    a_frontier.push(new_state, h)


def bfs_to_goal(start, goal, walls):
    """
    Bfs adapted to find path between two positions on the map.
    Returns list of actions or empty list if no path found.
    """
    if start == goal:
        return []
    
    # set to track visited nodes
    explored_bfs = set()
    frontier_bfs = util.Queue()
    
    # map of actions to movements
    actions_map = {
        Directions.NORTH: (0, 1),
        Directions.SOUTH: (0, -1),
        Directions.EAST: (1, 0),
        Directions.WEST: (-1, 0),
    }
    
    # initialize with start position and empty path
    start_state = [start, []]
    frontier_bfs.push(start_state)
    
    # continue until queue is empty
    while not frontier_bfs.is_empty():
        [current_pos, path] = frontier_bfs.pop()
        
        # skip if already explored
        if current_pos in explored_bfs:
            continue
        
        explored_bfs.add(current_pos)
        
        # check if we reached the goal
        if current_pos == goal:
            return path
        
        # explore all possible directions
        for action, (dx, dy) in actions_map.items():
            next_pos = (int(current_pos[0] + dx), int(current_pos[1] + dy))
            
            # check boundaries
            if next_pos[0] < 0 or next_pos[0] >= walls.width:
                continue
            if next_pos[1] < 0 or next_pos[1] >= walls.height:
                continue
            # check walls
            if walls[next_pos[0]][next_pos[1]]:
                continue
            
            # add unexplored positions to frontier
            if next_pos not in explored_bfs:
                new_path = path + [action]
                frontier_bfs.push([next_pos, new_path])
    
    # no path found
    return []


def bfs_to_goal_avoiding(start, goal, walls, danger_positions):
    """
    Bfs that avoids certain positions (like ghost positions).
    Returns list of actions or empty list if no path found.
    """
    if start == goal:
        return []
    
    explored_bfs = set()
    frontier_bfs = util.Queue()
    
    actions_map = {
        Directions.NORTH: (0, 1),
        Directions.SOUTH: (0, -1),
        Directions.EAST: (1, 0),
        Directions.WEST: (-1, 0),
    }
    
    start_state = [start, []]
    frontier_bfs.push(start_state)
    
    while not frontier_bfs.is_empty():
        [current_pos, path] = frontier_bfs.pop()
        
        if current_pos in explored_bfs:
            continue
        
        explored_bfs.add(current_pos)
        
        if current_pos == goal:
            return path
        
        for action, (dx, dy) in actions_map.items():
            next_pos = (int(current_pos[0] + dx), int(current_pos[1] + dy))
            
            # check boundaries
            if next_pos[0] < 0 or next_pos[0] >= walls.width:
                continue
            if next_pos[1] < 0 or next_pos[1] >= walls.height:
                continue
            # check walls
            if walls[next_pos[0]][next_pos[1]]:
                continue
            # avoid danger positions
            if next_pos in danger_positions:
                continue
            
            if next_pos not in explored_bfs:
                new_path = path + [action]
                frontier_bfs.push([next_pos, new_path])
    
    #if its not a safe path found, try without avoiding dangers
    return bfs_to_goal(start, goal, walls)


#####################
# Team Coordination #
#####################

#We create a shared team state for coordination between agents

class TeamState:
    
    """
    Sheared state between team agents for coordination.
    """
    agent_targets = {}      # {agent_index: target_position}
    agent_positions = {}    # {agent_index: current_position}


def get_food_by_zone(food_list, walls, zone='top'):
    
    """
    We split the food into top and bottom zones to divide attack zones.
    Returns food in the specified zone for each of the agents.
    """
    if not food_list:
        return []
    
    mid_y = walls.height // 2
    
    if zone == 'top':
        return [f for f in food_list if f[1] >= mid_y]
    else:  # bottom
        return [f for f in food_list if f[1] < mid_y]


def get_agent_zone(index):
    
    """
    We assign a zone to each agent based on their index.
    Lower index -> top zone, higher index -> bottom zone.
    """
    
    
    if index % 2 == 0:
        return 'top'
    else:
        return 'bottom'


#################
# Team creation #
#################

"""
def create_team(first_index, second_index, is_red,
                first='OffensiveReflexAgent', second='DefensiveReflexAgent', num_training=0):
    """ """
    This function should return a list of two agents that will form the
    team, initialized using firstIndex and secondIndex as their agent
    index numbers.  isRed is True if the red team is being created, and
    will be False if the blue team is being created.

    """ """
    return [eval(first)(first_index), eval(second)(second_index)]
"""


def create_team(first_index, second_index, is_red,
                first='OffensiveReflexAgent', second='DefensiveReflexAgent', num_training=0):
    # DEBUG: both offensive class
    return [DefensiveReflexAgent(first_index), OffensiveReflexAgent(second_index)]

##########
# Agents #
##########

class ReflexCaptureAgent(CaptureAgent):
    """
    A base class for reflex agents that choose score-maximizing actions
    """

    def __init__(self, index, time_for_computing=.1):
        super().__init__(index, time_for_computing)
        self.start = None

    def register_initial_state(self, game_state):
        self.start = game_state.get_agent_position(self.index)
        CaptureAgent.register_initial_state(self, game_state)

    def choose_action(self, game_state):
        """
        Picks among the actions with the highest Q(s,a).
        """
        actions = game_state.get_legal_actions(self.index)
        values = [self.evaluate(game_state, a) for a in actions]

        max_value = max(values)
        best_actions = [a for a, v in zip(actions, values) if v == max_value]

        food_left = len(self.get_food(game_state).as_list())

        if food_left <= 2:
            best_dist = 9999
            best_action = None
            for action in actions:
                successor = self.get_successor(game_state, action)
                pos2 = successor.get_agent_position(self.index)
                dist = self.get_maze_distance(self.start, pos2)
                if dist < best_dist:
                    best_action = action
                    best_dist = dist
            return best_action

        return random.choice(best_actions)

    def get_successor(self, game_state, action):
        """
        Finds the next successor which is a grid position (location tuple).
        """
        successor = game_state.generate_successor(self.index, action)
        pos = successor.get_agent_state(self.index).get_position()
        if pos != nearest_point(pos):
            # only half a grid position was covered
            return successor.generate_successor(self.index, action)
        else:
            return successor

    def evaluate(self, game_state, action):
        """
        Computes a linear combination of features and feature weights
        """
        features = self.get_features(game_state, action)
        weights = self.get_weights(game_state, action)
        return features * weights

    def get_features(self, game_state, action):
        """
        Returns a counter of features for the state
        """
        features = util.Counter()
        successor = self.get_successor(game_state, action)
        features['successor_score'] = self.get_score(successor)
        return features

    def get_weights(self, game_state, action):
        """
        Normally, weights do not depend on the game state.  They can be either
        a counter or a dictionary.
        """
        return {'successor_score': 1.0}


class OffensiveReflexAgent(ReflexCaptureAgent):
    """
    A reflex agent that seeks food. This is an agent
    we give you to get an idea of what an offensive agent might look like,
    but it is by no means the best or only way to build an offensive agent.
    
    Uses BFS planning only for:
    - returning home when carrying food (plan mode: 'return_home')
    - going for capsules when ghosts are near (plan mode: 'capsule')
    
    Uses reflex behavior (features/weights) for normal food chasing.
    """
    
    #thresholds/limits  for planning decisions!!!
    
    CARRY_THRESHOLD = 3     #the food to caarry before returning home
    FOOD_LEFT_THRESHOLD = 6    #remainiing food to trigger return
    GHOST_DANGER_DIST = 3   #distance to consider ghost dangerous
    CAPSULE_TRIGGER_DIST = 6   #ghoost distance to trigger capsule plan
    
    def register_initial_state(self, game_state):
        
        """
        Initializes agent and pre-calculates boundary positions.
        Sets up planning attributes.
        """
        super().register_initial_state(game_state)
        self.boundary_positions = self._calculate_boundary(game_state)
        self.walls = game_state.get_walls()
        
        width = self.walls.width
        
        if self.red:
            self.enemy_boundary_x = width // 2  #boundary
        else:
            self.enemy_boundary_x = width // 2 - 1
        
        #our game state planning attributes
        self.plan = []             #list of actions to follow
        self.plan_mode = None    #return_home', 'capsule'..
        self.plan_target = None    #target position for current plan
        
        #coordination attributes
        self.my_zone = get_agent_zone(self.index)  # 'top' or 'bottom'
        
        team_indices = self.get_team(game_state)
        
        if self.index == team_indices[0]:
            self.my_zone = 'top'
        else:
            self.my_zone = 'bottom' #second agent created assigned bottom
            
        self.turns = 0
        #self.turns = 0
        
        #we dont lose one agent allways defending
        self.attack_turns = 80  #both agents attack during first mturns
               
        #team state
        TeamState.agent_positions[self.index] = self.start
        TeamState.agent_targets[self.index] = None
    
    def _calculate_boundary(self, game_state):
        """
        Calculates valid positions at the boundary of our territory.
        """
        walls = game_state.get_walls()
        width = walls.width
        height = walls.height
        
        #boundary depends on the team color
        if self.red:
            boundary_x = width // 2 - 1
        else:
            boundary_x = width // 2
        
        positions = []
        for y in range(height):
            if not walls[boundary_x][y]:
                positions.append((boundary_x, y))
        
        return positions
    
    def choose_action(self, game_state):
        
        """
        Main decision function. Uses planning for key moments,
        reflex behavior for normal food chasing.
        """
    
        self.turns += 1
    
        my_state = game_state.get_agent_state(self.index)
        my_pos = my_state.get_position()
        my_pos_int = (int(my_pos[0]), int(my_pos[1]))
    
        #update team state for coordination
        TeamState.agent_positions[self.index] = my_pos_int
    
        #get enemy info
        enemies = [game_state.get_agent_state(i) for i in self.get_opponents(game_state)]
        ghosts = [e for e in enemies if not e.is_pacman and e.get_position() is not None]
        dangerous_ghosts = [g for g in ghosts if g.scared_timer <= 2]
    
    
        #check if current plan is still valid or heuristics of the game has changed
        if self.plan:
            if self._is_plan_valid(game_state, my_state, my_pos_int):
                action = self.plan.pop(0)
                if action in game_state.get_legal_actions(self.index):
                    return action
            self._clear_plan()
    
        #check if we should enter return_home mode
        if self._should_return_home(game_state, my_state):
            return self._plan_return_home(game_state, my_pos_int, dangerous_ghosts)
    
        #check if we should go for a capsule
        if self._should_get_capsule(game_state, my_state, my_pos_int, dangerous_ghosts):
            return self._plan_capsule(game_state, my_pos_int)
    
        #default: use reflex behavior for food chasing
        return self._reflex_action(game_state)
    
    def _is_plan_valid(self, game_state, my_state, my_pos):
        """
        Checks if the current plan is still valid.
        """
        
        if self.plan_mode == 'return_home':
            #valid if still pacman and carrying food
            return my_state.is_pacman and my_state.num_carrying > 0
        
        elif self.plan_mode == 'capsule':
            #valid if capsule still exists and we can "make a play"
            capsules = self.get_capsules(game_state)
            return self.plan_target in capsules
        
        return False
    
    def _clear_plan(self):
        """
        Clears the current plan.
        """
        self.plan = []
        self.plan_mode = None
        self.plan_target = None
    
    def _should_return_home(self, game_state, my_state):
        """
        Checks if agent should return home to deposit food.
        """
        if not my_state.is_pacman:
            return False
        
        food_left = len(self.get_food(game_state).as_list())
        num_carrying = my_state.num_carrying
        
        #return if carrying enough food or few food left
        return (num_carrying >= self.CARRY_THRESHOLD )
    
    def _should_get_capsule(self, game_state, my_state, my_pos, dangerous_ghosts):
        """
        Checks if agent should go for a capsule.
        """
        capsules = self.get_capsules(game_state)
        
        if not capsules:
            return False
        
        if not my_state.is_pacman:
            return False
        
        #check if there's a dangerous ghost nearby
        if dangerous_ghosts:
            
            min_ghost_dist = min(self.get_maze_distance(my_pos, g.get_position()) for g in dangerous_ghosts)
            
            #go for capsule if ghost is close enough to be a threat
            return min_ghost_dist <= self.CAPSULE_TRIGGER_DIST
        
        return False
    
    def _plan_return_home(self, game_state, my_pos, dangerous_ghosts):
        
        """
        Creates a plan to return home and let the food.
        So we can cash back the points from the food.
        Uses BFS once to find path to closest boundary.
        """
        #find closest boundary position
        target = min(self.boundary_positions, key=lambda b: self.get_maze_distance(my_pos, b))
        
        #get danger positions to avoid
        danger_positions = self._get_danger_positions(dangerous_ghosts)
        
        #compute path using bfs (only once)
        path = bfs_to_goal_avoiding(my_pos, target, self.walls, danger_positions)
        
        if path:
            self.plan = path
            self.plan_mode = 'return_home'
            self.plan_target = target
            return self.plan.pop(0)
        
        #fallback to reflex if no path found
        return self._reflex_action(game_state)
    
    def _plan_capsule(self, game_state, my_pos):
        """
        Creates a plan to get a capsule.
        Uses BFS once to find path to closest capsule.
        """
        capsules = self.get_capsules(game_state)
        
        if not capsules:
            return self._reflex_action(game_state)
        
        #find closest capsule
        target = min(capsules, key=lambda c: self.get_maze_distance(my_pos, c))
        
        #compute path using bfs (only once)
        path = bfs_to_goal(my_pos, target, self.walls)
        
        if path:
            self.plan = path
            self.plan_mode = 'capsule'
            self.plan_target = target
            return self.plan.pop(0)
        
        #fallback to reflex if no path found
        return self._reflex_action(game_state)
    
    def _get_danger_positions(self, dangerous_ghosts):
        """
        Returns set of positions to avoid (near ghosts).
        """
        danger = set()
        for g in dangerous_ghosts:
            gpos = g.get_position()
            gpos = (int(gpos[0]), int(gpos[1]))
            # mark ghost position and adjacent cells as dangerous
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    danger.add((gpos[0] + dx, gpos[1] + dy))
        return danger
    
    def _reflex_action(self, game_state):
        
        """
        Uses reflex behavior (features/weights) to choose action.
        This is the default behavior for normal food chasing.
        """
        actions = game_state.get_legal_actions(self.index)
        values = [self.evaluate(game_state, a) for a in actions]
        
        max_value = max(values)
        best_actions = [a for a, v in zip(actions, values) if v == max_value]
        
        # DEBUGG
        my_pos = game_state.get_agent_state(self.index).get_position()
        print(f"Agent {self.index} at {my_pos}, is_pacman={game_state.get_agent_state(self.index).is_pacman}")
        
        for a, v in zip(actions, values):
            print(f"  {a}: {v:.1f}")
        
        if Directions.STOP in best_actions and len(best_actions) > 1:
            best_actions.remove(Directions.STOP)
        
        return random.choice(best_actions)
        
    def _dist_to_boundary(self, pos):
        
        return min(self.get_maze_distance(pos, b) for b in self.boundary_positions)
    
    def get_features(self, game_state, action):
        
        """
        Returns features for reflex evaluation.
        Zone-aware: prefers food in agent's assigned zone.
        """
        features = util.Counter()
        successor = self.get_successor(game_state, action)
        
        
        my_state = successor.get_agent_state(self.index)
        my_pos = my_state.get_position()
        
        food_list = self.get_food(successor).as_list()
        features['successor_score'] = -len(food_list)
        
        #we discourage not moving by penalizing it
        if action == Directions.STOP:
            features['stop'] = 1.0
        
        current_direction = game_state.get_agent_state(self.index).configuration.direction
        
        #there is not comming back from the same path, we dont encourage it
        
        if action == Directions.REVERSE.get(current_direction, None):
            features['reverse'] = 1.0
        
        #encouraging exploring the map
        
        if action == current_direction and action != Directions.STOP:
            features['forward'] = 2.5
        
        #if we are still ghost go around
        if not my_state.is_pacman:
            if self.red:
                dist_to_enemy = self.enemy_boundary_x - my_pos[0]
            else:
                dist_to_enemy = my_pos[0] - self.enemy_boundary_x
                
            #only on our side    
            if dist_to_enemy > 0:
                features['distance_to_enemy_side'] = float(dist_to_enemy)
                
                
        #distance to food - prefer food in my zone        
        if len(food_list) > 0:
            
            food_list = self.get_food(successor).as_list()
            
            #gotta get food from my side of the map first
            zone_food = get_food_by_zone(food_list, self.walls, self.my_zone)
            
            
            #if there is no food in my zone look all food
            if len(zone_food)>2:
                target_food = list(zone_food)
            else:
                target_food = list(food_list)
            
            
            #target_food = zone_food if zone_food else food_list
            
            #avoid food that teammate is targeting
            teammate_target = self._get_teammate_target()
            
            if teammate_target and teammate_target in target_food and len(target_food) > 1:
                target_food = [f for f in target_food if f != teammate_target]
        
            if target_food:
                min_food_dist = min(self.get_maze_distance(my_pos, food) for food in target_food)
                features['distance_to_food'] = float(min_food_dist)
            
                #upadte defensive agent for coordination purposes
                closest_food = min(target_food, key=lambda f: self.get_maze_distance(my_pos, f))
                TeamState.agent_targets[self.index] = closest_food
    
        #distance to capsules
        capsules = self.get_capsules(successor)
        if len(capsules) > 0:
            min_capsule_dist = min(self.get_maze_distance(my_pos, cap) for cap in capsules)
            features['distance_to_capsule'] = float(min_capsule_dist)
    
        #distance to home when carrying food
        if my_state.is_pacman and my_state.num_carrying > 0:
            
            dist_home = self._dist_to_boundary(my_pos)
            features['distance_to_home'] = float(dist_home)
            features['num_carrying'] = float(my_state.num_carrying)
            
        #dangerous ghosts annd enemies handling
        
        enemies = [successor.get_agent_state(i) for i in self.get_opponents(successor)]
        enemy_ghosts = [e for e in enemies if not e.is_pacman and e.get_position() is not None]
        dangerous_ghosts = [g for g in enemy_ghosts if g.scared_timer == 0]
        scared_ghosts = [g for g in enemy_ghosts if g.scared_timer > 0]
        
        if my_state.is_pacman and len(dangerous_ghosts) > 0:
            
            dists = [self.get_maze_distance(my_pos, g.get_position()) for g in dangerous_ghosts]
            min_ghost_dist = min(dists)
            
            #!"""" key variable as distance to "bravery" against ghosts
            if min_ghost_dist <= self.GHOST_DANGER_DIST:
            
                features['dangerous_ghost'] = 1.0 / (min_ghost_dist + 0.1)
        
        if my_state.is_pacman and len(scared_ghosts) > 0:
            
            dists = [self.get_maze_distance(my_pos, g.get_position()) for g in scared_ghosts]
            min_scared_dist = min(dists)
            features['scared_ghost'] = 1.0 / (min_scared_dist + 1.0)
            
        #this is to avoiod collision betweeen two agents
        teammate_pos = self._get_teammate_position()
        
        #we dont want them attacking at the same place
        if teammate_pos and my_state.is_pacman:
            
            #vertial distance penalitzation between the team
            dist_to_teammate = self.get_maze_distance(my_pos, teammate_pos)
            vertical_dist = abs(my_pos[1] - teammate_pos[1])
            
                                   
            if dist_to_teammate <= 1:
                features['too_close_teammate'] = 3.0
            
            if vertical_dist <= 2:
                
                features['too_close_teammate'] = float(3 - vertical_dist)
        
        return features
    
    def _get_teammate_target(self):
        
        """
        Returns the target that teammate is going for.
        """
        for idx, target in TeamState.agent_targets.items():
            if idx != self.index and target is not None:
                return target
        return None


    def _get_teammate_position(self):
        """
        Returns teammate's current position.
        """
        for idx, pos in TeamState.agent_positions.items():
            if idx != self.index:
                return pos
        return None
    
    
    def get_weights(self, game_state, action):
        
        """
        Reeturns weights for reflex evaluation.
         """
        weights = {
        
            'successor_score': 100.0,
            'distance_to_food': -25.0, #food and points its superimportant
            'distance_to_capsule': -40.0,
            'distance_to_home': -10.0, #im comming home for christmas
            'num_carrying': 5.0,
            'dangerous_ghost': -220.0,
            'scared_ghost': 350.0,
            'stop': -50.0,
            'forward': 10.0,
            'reverse': -15.0,
            'distance_to_enemy_side': -20.0,
            'too_close_teammate': -35,  #we avoid clustering of pacmans/ghosts
            }
    
        my_state = game_state.get_agent_state(self.index)
        
        #incentivize go enemy zone
        if not my_state.is_pacman:
            
            weights['distance_to_food'] = -8.0
            weights['distance_to_capsule'] = -10.0
            
        if not (my_state.is_pacman and my_state.num_carrying > 0):
            
            weights['distance_to_home'] = 0.0
            weights['num_carrying'] = 0.0
    
        return weights


class DefensiveReflexAgent(ReflexCaptureAgent):
    
    """
    Explanation: a refflex agent that keeps its side Pacman-free. Again,
    this is to give you an idea of what a defensive agent
    could be like.  It is not the best or only way to make
    such an agent.
    
    We use BFS for planning WHEN:
    - intercepting when invaders (plan mode: 'intercept')
    - returning Home when carrying food enough (plan mode: 'return_home')
    - 'patrolling' the boundary (plan mode: 'patrol')
    
    Uses reflex behavior for:
    - chasing nearby invaders
    - normal offensive actions (second attacker)
    """
    
    #thresholds to decide actions
    
    ATTACK_TURNS = 40   #turns to be aggressive at start
    CHASE_DISTANCE = 5     #distance to chase invaders directly
    INTERCEPT_DISTANCE = 12    #max distance for intercept planning
    CARRY_THRESHOLD = 3     #food to carry before returning home
    GHOST_DANGER_DIST = 4     #distance to consider ghost dangerous
    
    def register_initial_state(self, game_state):
        """
        Initializes agent and pre-calculates boundary positions.
        Sets up planning attributes.
        """
        super().register_initial_state(game_state)
        self.boundary_positions = self._calculate_boundary(game_state)
        self.walls = game_state.get_walls()
        self.turns = 0
        
        #planning attributes
        self.plan = []
        self.plan_mode = None  #initializa to none
        self.plan_target = None #initializa to none
        
        #coordination team attreibutes
        self.my_zone = 'bottom' if get_agent_zone(self.index) == 'top' else 'top'
        
        #ts as we seen before in offensive agent
        TeamState.agent_positions[self.index] = self.start
        TeamState.agent_targets[self.index] = None
    
    def _calculate_boundary(self, game_state):
        """
        Calculates valid positions at the boundary.
        """
        walls = game_state.get_walls()
        width = walls.width
        height = walls.height
        
        if self.red:
            boundary_x = width // 2 - 1
        else:
            boundary_x = width // 2
        
        positions = []
        for y in range(height):
            if not walls[boundary_x][y]:
                positions.append((boundary_x, y))
        
        return positions
        
    def choose_action(self, game_state):
        
        self.turns += 1
    
        my_state = game_state.get_agent_state(self.index)
        my_pos = my_state.get_position()
        my_pos_int = (int(my_pos[0]), int(my_pos[1]))
    
        #update team state for coordination as we seen before
        TeamState.agent_positions[self.index] = my_pos_int
    
        #detect invasors
        enemies = [game_state.get_agent_state(i) for i in self.get_opponents(game_state)]
        invaders = [e for e in enemies if e.is_pacman and e.get_position() is not None]
    
        #executing the main super-plan
        if self.plan:
            #patrol mode if no invasor
            if self.plan_mode == 'intercept' and len(invaders) == 0 and self.ATTACK_TURNS == self.turns:
                self._clear_plan()
            elif self.plan_mode != 'intercept' and len(invaders) > 0:
                 self._clear_plan() #avoid patrol and chase
            elif self.plan_mode == 'return_home' and not my_state.is_pacman:
                 self._clear_plan() 
            
            #kkeep the plan
            
            if self.plan:
                action = self.plan.pop(0)
                if action in game_state.get_legal_actions(self.index):
                    return action
                self._clear_plan() # Fallback si la acción no es legal

    
        
        #priority active defense
        if len(invaders) > 0:
            closest_dist = min(self.get_maze_distance(my_pos, i.get_position()) for i in invaders)
            
            #go collapse invader
            if closest_dist <= self.INTERCEPT_DISTANCE + 2: #more range
                return self._handle_invaders(game_state, my_state, my_pos_int, invaders)
    
        #cash food for points
        if my_state.is_pacman and my_state.num_carrying >= self.CARRY_THRESHOLD:
            return self._plan_return_home(game_state, my_pos_int)
    
        #patrol and attack handling
        if self.turns < self.ATTACK_TURNS:
            #choose action
            return self._choose_offensive_action(game_state, my_state, my_pos_int)
        else:
            #defend actively by planning a patrol
            return self._plan_patrol(game_state, my_pos_int)
            
        return self._defensive_reflex_action(game_state) #worst case scenario
        
    
    def _is_plan_valid(self, game_state, my_state, my_pos, invaders):
        """
        Checks if the current plan is still valid.
        """
        if self.plan_mode == 'return_home':
            return my_state.is_pacman and my_state.num_carrying > 0
        
        elif self.plan_mode == 'intercept':
            #valid if there are still invaders
            return len(invaders) > 0
        
        elif self.plan_mode == 'patrol':
            #valid if no invaders and not carrying food
            return len(invaders) == 0 and not (my_state.is_pacman and my_state.num_carrying > 0)
        
        return False
    
    def _clear_plan(self):
        """
        Clears the current plan.
        """
        self.plan = []
        self.plan_mode = None
        self.plan_target = None
    
    def _handle_invaders(self, game_state, my_state, my_pos, invaders):
        """
        Handles invaders based on their distance. To not compute overload with BFS 
        all the time.
        """
        
        #look foor the closest invader
        closest_invader = min(invaders, 
                              key=lambda i: self.get_maze_distance(my_pos, i.get_position()))
        inv_pos = closest_invader.get_position()
        inv_dist = self.get_maze_distance(my_pos, inv_pos)
        
        #if invader is very close, chase directly using reflex
        if inv_dist <= self.CHASE_DISTANCE:#should be 6
            return self._chase_invader(game_state, my_pos, inv_pos)
        
        #if invader is at medium distance, plan interception
        elif inv_dist <= self.INTERCEPT_DISTANCE: #should be 10
            return self._plan_intercept(game_state, my_pos, inv_pos)
        
        #invader is far soo we use reflex basic defensive behavior to not overload with BFS calls
        return self._defensive_reflex_action(game_state)
    
    def _chase_invader(self, game_state, my_pos, inv_pos):
        
        """
        Chases an invader direectly using reflex evaluation.
        No BFS planning here, just greedy chase to save time.
        """
        actions = game_state.get_legal_actions(self.index)
        actions = [a for a in actions if a != Directions.STOP]
        
        if not actions:
            return Directions.STOP
        
        #choose action that minimizes distance to invader
        best_action = None
        best_dist = float('inf')
        
        for action in actions:
            successor = self.get_successor(game_state, action)
            new_pos = successor.get_agent_state(self.index).get_position()
            dist = self.get_maze_distance(new_pos, inv_pos)
            
            if dist < best_dist:
                best_dist = dist
                best_action = action
        
        return best_action if best_action else random.choice(actions)
        
    
    #kind of like a policeman in the neirborhood
    def _plan_patrol(self, game_state, my_pos):
        """
        Plans a path to patrol the boundary positions around center map.
        The target alternaates between the top and bottom of the boundary of the teams.
        """
        
        #wee choose a patrol target (e.g., top/bottom boundary positions)
        boundary_pos = self._calculate_boundary(game_state)
        
        if not boundary_pos:
            return self._fallback_action(game_state)

        #we get top/bottom boundary points
        top_boundary = max(boundary_pos, key=lambda p: p[1])
        bottom_boundary = min(boundary_pos, key=lambda p: p[1])

        #alternate targets
        if self.plan_target == top_boundary:
            target = bottom_boundary
        else:
            target = top_boundary
            
        if my_pos == target: # If already at target, switch to the other
             target = bottom_boundary if target == top_boundary else top_boundary
        
        #we use bfs
        path = bfs_to_goal(my_pos, target, self.walls)
        
        if path:
            self.plan = path
            self.plan_mode = 'patrol'
            self.plan_target = target
            return self.plan.pop(0)
        
        return self._defensive_reflex_action(game_state) # Fallback to reflex
    
    def _plan_intercept(self, game_state, my_pos, inv_pos):
        
        """
        Plans an interception path to cut off the invader.
        Uses BFS once.
        """
        inv_pos_int = (int(inv_pos[0]), int(inv_pos[1]))
        
        # find best intercept point: boundary position closest to invader
        if self.boundary_positions:
            intercept_target = min(self.boundary_positions,
                                   key=lambda b: self.get_maze_distance(b, inv_pos_int))
        else:
            intercept_target = inv_pos_int
        
        #compute path using bfs (only once)
        path = bfs_to_goal(my_pos, intercept_target, self.walls)
        
        if path:
            self.plan = path
            self.plan_mode = 'intercept'
            self.plan_target = intercept_target
            return self.plan.pop(0)
        
        #fallback to direct chase behavioor
        return self._chase_invader(game_state, my_pos, inv_pos)
    
    def _plan_return_home(self, game_state, my_pos):
        
        """
        Another inteligent action designed that,
        Plans a path to return home and deposit food.
        """
        target = min(self.boundary_positions,
                     key=lambda b: self.get_maze_distance(my_pos, b))
        
        #get danger positions to dont lose score points oncee turning home
        
        #getting ghosts pos
        enemies = [game_state.get_agent_state(i) for i in self.get_opponents(game_state)]
        ghosts = [e for e in enemies if not e.is_pacman and e.get_position() is not None]
        
        
        dangerous_ghosts = [g for g in ghosts if g.scared_timer <= 2]
        danger_positions = self._get_danger_positions(dangerous_ghosts)
        
        #bfs our way out
        path = bfs_to_goal_avoiding(my_pos, target, self.walls, danger_positions)
        
        if path:
            self.plan = path
            self.plan_mode = 'return_home'
            self.plan_target = target
            return self.plan.pop(0)
        
        return self._fallback_action(game_state)
    
    def _choose_offensive_action(self, game_state, my_state, my_pos):
        
        """
        Offensive behavior when there are no invaders.
        Uses reflex evaluation to chase food.
        """
        #check for dangerous ghosts as we seen before
        
        enemies = [game_state.get_agent_state(i) for i in self.get_opponents(game_state)]
        ghosts = [e for e in enemies if not e.is_pacman and e.get_position() is not None]
        
        dangerous_ghosts = [g for g in ghosts if g.scared_timer <= 2]
        
        #if dangerous ghost nearby while pacman, be cautious
        if my_state.is_pacman and dangerous_ghosts:
            min_ghost_dist = min(self.get_maze_distance(my_pos, g.get_position()) 
                                 for g in dangerous_ghosts)
            if min_ghost_dist <= 3:
                return self._plan_return_home(game_state, my_pos)
        
        #use offensive reflex behavior wiht lower cost 
        return self._offensive_reflex_action(game_state)
    
    def _offensive_reflex_action(self, game_state):
        
        """
        Uses offensive features/weights for action selection.
        """
        actions = game_state.get_legal_actions(self.index)
        values = [self._evaluate_offensive(game_state, a) for a in actions]
        
        max_value = max(values)
        best_actions = [a for a, v in zip(actions, values) if v == max_value]
        
        return random.choice(best_actions)
    
    def _evaluate_offensive(self, game_state, action):
        
        """
        Evaluates action using offensive features.
        """
        features = self._get_offensive_features(game_state, action)
        weights = self._get_offensive_weights(game_state, action)
        return features * weights
    
    def _get_offensive_features(self, game_state, action):
        
        """
        Features for offensive behavior.
        """
        features = util.Counter()
        successor = self.get_successor(game_state, action)
        
        my_state = successor.get_agent_state(self.index)
        my_pos = my_state.get_position()
        
        food_list = self.get_food(successor).as_list()
        features['successor_score'] = -len(food_list)
        
        if action == Directions.STOP:
            features['stop'] = 1.0
        
        current_direction = game_state.get_agent_state(self.index).configuration.direction
        if action == Directions.REVERSE.get(current_direction, None):
            features['reverse'] = 1.0
        
        if action == current_direction and action != Directions.STOP:
            features['forward'] = 1.0
        
        if len(food_list) > 0:
            min_food_dist = min(self.get_maze_distance(my_pos, food) for food in food_list)
            features['distance_to_food'] = float(min_food_dist)
        
        #dangerous ghosts or enemies positions
        enemies = [successor.get_agent_state(i) for i in self.get_opponents(successor)]
        enemy_ghosts = [e for e in enemies if not e.is_pacman and e.get_position() is not None]
        dangerous_ghosts = [g for g in enemy_ghosts if g.scared_timer == 0]
        
        if my_state.is_pacman and len(dangerous_ghosts) > 0:
            dists = [self.get_maze_distance(my_pos, g.get_position()) for g in dangerous_ghosts]
            min_ghost_dist = min(dists)
            if min_ghost_dist <= 5:
                features['dangerous_ghost'] = 1.0 / (min_ghost_dist + 0.1)
        
        return features
        
    def _get_offensive_weights(self, game_state, action):
        
        #weights for offensive behavior
        return {
            'successor_score': 70.0,    #less than main agent
            'distance_to_food': -6.0,   #incentive food eating
            'dangerous_ghost': -180.0,
            'stop': -15.0,
            'forward': 4.0,
            'reverse': -6.0,
        }

    
    def _defensive_reflex_action(self, game_state):
        """
        Uses defensive features/weights for action selection.
        """
        actions = game_state.get_legal_actions(self.index)
        values = [self.evaluate(game_state, a) for a in actions]
        
        max_value = max(values)
        best_actions = [a for a, v in zip(actions, values) if v == max_value]
        
        return random.choice(best_actions)
    
    def _get_danger_positions(self, dangerous_ghosts):
        """
        Returns set of positions to avoid.
        """
        danger = set()
        for g in dangerous_ghosts:
            gpos = g.get_position()
            gpos = (int(gpos[0]), int(gpos[1]))
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    danger.add((gpos[0] + dx, gpos[1] + dy))
        return danger
    
    def _get_dist_to_boundary(self, pos):
        """
        Calculates distance to closest boundary position.
        """
        if not self.boundary_positions:
            return self.get_maze_distance(pos, self.start)
        return min(self.get_maze_distance(pos, bp) for bp in self.boundary_positions)
    
    def _fallback_action(self, game_state):
        """
        Fallback action when ourr plan has faailed (something
        else has happened in the game heuristics).
        """
        actions = game_state.get_legal_actions(self.index)
        actions = [a for a in actions if a != Directions.STOP]
        return random.choice(actions) if actions else Directions.STOP
    
    def get_features(self, game_state, action):
        
        """
        Features for defensive evaluation.
        """
        features = util.Counter()
        successor = self.get_successor(game_state, action)
        
        my_state = successor.get_agent_state(self.index)
        my_pos = my_state.get_position()
        
        #being on defense is good
        features['on_defense'] = 1
        if my_state.is_pacman:
            features['on_defense'] = 0
        
        #detect invaders
        enemies = [successor.get_agent_state(i) for i in self.get_opponents(successor)]
        invaders = [a for a in enemies if a.is_pacman and a.get_position() is not None]
        features['num_invaders'] = len(invaders)
        
        if len(invaders) > 0:
            dists = [self.get_maze_distance(my_pos, a.get_position()) for a in invaders]
            features['invader_distance'] = min(dists)
        
        #don't stop, little penalitzation for stopping
        if action == Directions.STOP:
            features['stop'] = 1
        
        #penalize reverse, to avoid getting back from the same path
        rev = Directions.REVERSE[game_state.get_agent_state(self.index).configuration.direction]
        if action == rev:
            features['reverse'] = 1
        
        return features
    
    def get_weights(self, game_state, action):
        
        my_state = game_state.get_agent_state(self.index)
        
        if self.turns < self.ATTACK_TURNS:
            on_defense_weight = 0
        else:
            on_defense_weight = 35
        
        """
        Weights for defensive evaluation.
        """
        return {
            'num_invaders': -1000,
            'on_defense': on_defense_weight,
            'invader_distance': -12,
            'stop': -1000,
            'reverse': -40
        }
