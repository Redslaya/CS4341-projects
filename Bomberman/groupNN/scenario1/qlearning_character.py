# This is necessary to find the main code
import sys
import operator
import random
import math
import numpy as np

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from entity import AIEntity
from entity import MovableEntity
from colorama import Fore, Back, Style, init
from sensed_world import SensedWorld
from events import Event
init(autoreset=True)

alpha = 0.3
gamma = 0.9
epsilon = 0.2

class QCharacter(CharacterEntity):

    def __init__(self, qtable, wm, wg, *args, **kwargs):
        super(QCharacter, self).__init__(*args, **kwargs)
        # Whether this character wants to place a bomb
        self.maybe_place_bomb = False
        # Debugging elements
        self.tiles = {}
        self.qtable = qtable
        # self.w = w  # weight of bomb feature
        self.wm = wm  # weight of monster distance feature
        self.wg = wg  # weight of goal distance
        # self.ww = ww  # weight of distance to closest wall
        # self.wcm = wcm  # weight of "are we moving closer to monster"
        # self.wcg = wcg  # weight of "are we moving closer to goal"+
        self.last_state = 0
        self.last_action = (0, 0)

        # return closest_bomb(), closest_monster((coords[0], coords[1]), wrld), monster_direction(coords, wrld), dist

    def do(self, wrld):
        dist_to_goal, dist_to_monster = calculate_features((self.x, self.y), wrld)


        actions = self.valid_moves(wrld)
        rel_actions = []
        for a in actions:
            rel_actions.append((a[0] - self.x, a[1] - self.y))

        #list of (rel_action, (features))
        Qs = []
        for rel in rel_actions:
            #calculate features of that new world
            Qs.append((rel, self.Q(wrld, rel)))

        best_q = -10000000000
        rel_move = None

        for q in Qs:
            if q[1] > best_q:
                best_q = q[1]
                rel_move = q[0]



        #update weights

        delta = (self.r(wrld, rel_move) + gamma * best_q) - self.Q(wrld, rel_move)

        self.wm = self.wm + alpha * delta * dist_to_monster

        self.wg = self.wg + alpha * delta * dist_to_goal

        self.yeet(rel_move[0], rel_move[1])



        # get the state
        # get the valid actions
        # test the valid actions
        # choose best action

        # yeet that action

    def yeet(self, x, y):
        self.move(x, y)
        print("WEIGHTS::::")
        print("MONST WEIGHT :::: ", self.wm)
        print("GOAL WEIGHT :::: ", self.wg)
        pass


    def Q(self, wrld, action):
        next_wrld = self.getNextWorld(wrld, action)

        c = next_wrld[0].me(self)
        print("CHARACTER:", c)

        if c is None:
            for event in next_wrld[1]:
                if event.tpe == Event.CHARACTER_FOUND_EXIT and event.character.name == self.name:
                    #print("WE CAN WIN!!!!!")
                    return 100
                elif event.tpe == Event.CHARACTER_KILLED_BY_MONSTER and event.character.name == self.name:
                    # print("WE CAN DIE!!!")
                    return -100
                else:  # Timed out??
                    return -1

        print(c)
        print(next_wrld)

        goal_dist, monst_dist = calculate_features((c.x, c.y), next_wrld[0])


        return (self.wg * goal_dist + self.wm * monst_dist)

    def choose_action(self, state, actions, wrld):
        if random.uniform(0, 1) < epsilon:
            return random.choice(actions)  # Pick an action randomly from set of valid actions
        else:
            move = self.select_best_move(state, actions, wrld)
            self.approximateQ(state, move, wrld)


    #taking in world, rel_action -- calculates new reward
    def getNextWorld(self, wrld, action):
        sim = SensedWorld.from_world(wrld)
        c = sim.me(self)  # finds our character in the simulated world
        # Are monsters moving?????
        # print("action:", action)
        c.move(action[0], action[1])  # moves character in simulated world
        sim = sim.next()  # updates simulated world
        c = sim[0].me(self)
        # sim[0] is world
        return sim

    # TODO not detecting wins or losses quite properly.
    def r(self, wrld, action):
        #print("REAL WORLD COORDINATES:", self.x, self.y)
        sim = SensedWorld.from_world(wrld)  # creates simulated world
        c = sim.me(self)  # finds our character in the simulated world
        # Are monsters moving?????
        #print("action:", action)
        c.move(action[0], action[1])  # moves character in simulated world
        sim = sim.next()  # updates simulated world

        #print(monster_tiles(sim[0]))
        c = sim[0].me(c)  # finds our character in the simulated world
        if c is None:
            for event in sim[1]:
                if event.tpe == Event.CHARACTER_FOUND_EXIT and event.character.name == self.name:
                    #print("WE CAN WIN!!!!!")
                    return 10
                elif event.tpe == Event.CHARACTER_KILLED_BY_MONSTER and event.character.name == self.name:
                    # print("WE CAN DIE!!!")
                    return -10
                else:  # Timed out??
                    return -5
        else:
            return 1

    # TODO this... closest_bomb & all other features should be normalized to 0,1
    def update_weights(self, delta, wrld, action):
        sim = SensedWorld.from_world(wrld)  # creates simulated world
        c = sim.me(self)  # finds our character in the simulated world
        if action != "bomb":
            c.move(action[0], action[1])  # moves character in simulated world
        else:
            c.place_bomb()  # places bomb if that's what we decided to do
        sim = sim.next()  # updates simulated world
        c = sim[0].me(c)  # finds our character in the simulated world


    # Closer to monster feature. Returns -1 if c is closer than self to monster, +1 if not, 0 if same
    def fcm(self, move, wrld):
        if move == "bomb":
            return -1
        selfdist = closest_monster((self.x, self.y), wrld)
        cdist = closest_monster((move[0] + self.x, move[1] + self.y), wrld)
        print("IN FCM")
        print("cdist: ", cdist)
        print("selfdist", selfdist)
        

        if cdist < selfdist:
            return -1
        if cdist > selfdist:
            return 1
        return 0


    # Closer to goal feature. Returns +1 if c is closer than self to goal, -1 if not, 0 if same
    def fcg(self, move, wrld):
        #print("Move: ", move)
        if move == "bomb":
            return 0
        selfdist = distance_to_exit((self.x, self.y), wrld)
        cdist = distance_to_exit((move[0] + self.x, move[1] + self.y), wrld)

        if cdist < selfdist:
            return -1
        if cdist > selfdist:
            return 1
        return 0

    def threatened(self, wrld):
        return True
        # TODO Add a check for bombs as well
        if closest_monster((self.x, self.y), wrld) <= 3:
            return True
        return False

    def valid_moves(self, wrld):
        moves = get_adjacent((self.x, self.y), wrld)
        final = []
        for m in moves:
            if not wrld.wall_at(m[0], m[1]):
                final.append(m)
            elif wrld.exitcell == (m[0], m[1]):
                return [m]
        return final

    # Resets styling for each cell. Prevents unexpected/inconsistent behavior that otherwise appears with coloring.

# ==================== STATIC METHODS ==================== #
# Many of our methods actually need to be static because they need to be applied to different world state objects.


# Calculates manhattan distance between the two sets of coordinates. These could be tuples, but whatever.
def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)


# Prioritizes downward
def cost_to(current, next):
    diff = (next[0] - current[0], next[1] - current[1])
    val = abs(diff[0]) + abs(diff[1])
    if val == 2:
        return 2
    else:
        return 1


# Returns a vector of values representing each feature.
# Vector structure: (bomb distance, monster distance, exit distance)
def calculate_features(coords, wrld):
    monster = closest_monster((coords[0], coords[1]), wrld)
    dist = distance_to_exit(coords, wrld)

    # TODO Add distance to wall??
    #return closest_bomb(coords, wrld), monster, dist, closest_wall(coords, wrld)
    return dist, monster
# ==================== FEATURES ==================== #
#   - Distance to closest bomb
#   - Distance to closest monster
#   - Distance to goal
#   - Distance to closest wall

# Returns an integer representing the Manhattan distance to the closest bomb.
def closest_bomb(coords, wrld):
    bombs = get_bombs(wrld)
    if len(bombs) == 0:
        return 0
    mindist = float('inf')
    for b in bombs:
        score = manhattan_distance(coords[0], coords[1], b[0], b[1])
        if score < mindist:
            mindist = score
        if mindist == 0:
           return 1
    return mindist


# Returns an integer representing the A* distance to the closest monster.
def closest_monster(coords, wrld):
    x = coords[0]
    y = coords[1]
    monsters = monster_tiles(wrld)
    p = float('inf')
    for m in monsters:
        distance = len(aStar((x, y), wrld, m, False))
        #print("Moster Dist:" , distance)
        if distance < p:
            p = distance
        if p == 0:
            p = 1
    return 1 / p


# Returns 1/(A* distance to exit)^2.
def distance_to_exit(coords, wrld):
    dist = (len(aStar(coords, wrld, wrld.exitcell)) ** 2)
    if dist < 1:
        return 1
    if dist == 0:
        dist = 1

    return 1 / dist

def closest_wall(coords, wrld):
    walls = get_walls(wrld)
    mindist = float('inf')
    for w in walls:
        dist = manhattan_distance(coords[0], coords[1], w[0], w[1])
        if dist < mindist:
            mindist = dist
        if mindist == 0:
            return 1
    return 1/mindist

# ========== END OF FEATURES ==========


# Returns a list of tiles which are occupied by at least 1 monster.
def monster_tiles(wrld):
    tiles = []
    for x in range(0, wrld.width()):
        for y in range(0, wrld.height()):
            if wrld.monsters_at(x, y):
                tiles.append((x, y))
    return tiles

def monster_direction(coords, wrld):
    x = coords[0]
    y = coords[1]
    monsters = monster_tiles(wrld)
    mcoords = 0, 0
    xval = 0
    yval = 0
    p = float('inf')
    for m in monsters:
        distance = len(aStar((x, y), wrld, m,False))
        if distance < p:
            p = distance
            mcoords = (m[0], m[1])
            xval = m[0] - x
            yval = m[1] - y

    return np.sign(xval), np.sign(yval)

# Returns a list of coordinates that contain bombs.
def get_bombs(wrld):
    bombs = []
    for x in range(0, wrld.width()):
        for y in range(0, wrld.height()):
            if wrld.bomb_at(x, y):
                bombs.append((x, y))
    return bombs

# Returns a list of coordinates in the world surrounding the current one.
# param current: An (x, y) point
def get_adjacent(current, wrld):
    # Returns a list of points in the form [(x1, y1), (x2, y2)]
    neighbors = []
    x = current[0]
    y = current[1]

    if x >= 1:
        if y >= 1:
            neighbors.append((x - 1, y - 1))  # top left
        neighbors.append((x - 1, y))  # middle left
        if y < wrld.height() - 1:
            neighbors.append((x - 1, y + 1))  # bottom left

    if y >= 1:
        neighbors.append((x, y - 1))  # top middle
    if y < wrld.height() - 1:
        neighbors.append((x, y + 1))  # bottom middle

    if x < wrld.width() - 1:
        if y >= 1:
            neighbors.append((x + 1, y - 1))  # top right
        neighbors.append((x + 1, y))  # middle right
        if y < wrld.height() - 1:
            neighbors.append((x + 1, y + 1))  # bottom right

    return neighbors

def get_walls(wrld):
    walls = []

    for x in range(0, wrld.width()):
        for y in range(0, wrld.height()):
            if wrld.wall_at(x, y):
                walls.append((x, y))
    return walls

def aStar(char, wrld, mapTo, toExit=True):
    # x = start[0]
    # y = start[1]
    # # print("SELFX: " + str(self.x))
    # # print("SELFY: " + str(self.y))
    # frontier = []
    # frontier.append(((x, y), 0))
    # came_from = {}
    # cost_so_far = {}
    # came_from[(x, y)] = None
    # cost_so_far[(x, y)] = 0

    # while not len(frontier) == 0:
    #     frontier.sort(key=lambda tup: tup[1])  # check that
    #     current = frontier.pop(0)
    #     if (current[0][0], current[0][1]) == goal:
    #         break
    #     for next in get_adjacent(current[0], wrld):
    #         #print(next)
    #         if wrld.wall_at(next[0], next[1]):
    #             cost_so_far[(next[0], next[1])] = 999
    #             new_cost = 1000
    #         else:
    #             new_cost = cost_to(current[0], next) + cost_so_far[current[0]]
    #             #new_cost = 1 + cost_so_far[current[0]]
    #         if next not in cost_so_far or new_cost < cost_so_far[next]:
    #             cost_so_far[next] = new_cost
    #             frontier.append((next, new_cost + manhattan_distance(next[0], next[1], goal[0], goal[1])))
    #             came_from[next] = current[0]


    # cursor = goal
    # path = []
    # while not cursor == (x, y):
    #     path.append(cursor)
    #     try:
    #         cursor = came_from[cursor]
    #     except KeyError:
    #         return [(0, 0)]
    # return path

        # print("Searching From " + str((char[0], char[1])))
    # print("Searching for " + str(mapTo))
    # A*
    #print("PATH FROM: ", char, " PATH TO: ", mapTo)

    frontier = []
    frontier.append(((char[0], char[1]), 0))
    came_from = {}
    cost_so_far = {}
    came_from[(char[0], char[1])] = None
    cost_so_far[(char[0], char[1])] = 0
    move = 1
    # print("charX " + str(char[0]))
    # print("charY " + str(char[1]))

    monsters = []

    for x in range(0, wrld.width()):
        for y in range(0, wrld.height()):
            if wrld.monsters_at(x, y):  # Finds all the monsters in the board
                monsters.append((x, y))
            if wrld.exit_at(x, y):  # Just in case exit is not where we expect it to be in the bottom right corner
                ex = (x, y)


    while not len(frontier) == 0:
        frontier.sort(key=lambda tup: tup[1])  # check that
        current = frontier.pop(0)
        if (current[0][0], current[0][1]) == mapTo:
            #print("HERE")
            break
        for next in get_adjacent(current[0], wrld):
            if wrld.wall_at(next[0], next[1]):
                    cost_so_far[(next[0], next[1])] = 999
                    new_cost = 1000
            
            elif (next[0], next[1]) in monsters and  toExit:
                    cost_so_far[(next[0], next[1])] = 99
                    new_cost = 100
                
            else:
                new_cost = cost_to(current[0], next) + cost_so_far[current[0]]
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                frontier.append((next, new_cost + manhattan_distance(next[0], next[1],mapTo[0], mapTo[1])))
                came_from[next] = current[0]

    # char.printOurWorld(wrld, cost_so_far)

    cursor = mapTo
    path = []
    while not cursor == (char[0], char[1]):
        move = cursor
        path.append(cursor)
        try:
            cursor = came_from[cursor]
        except KeyError:
            # char.move(0, 0)
            pass
            break
    #print("PATH: ", path)
    # print(path)

    if not len(path) == 0:
        move = path[len(path) - 1]

    # carries momentum? mayhaps not the best

    return path
    # char.move(move[0] - char[0], move[1] - char[1])