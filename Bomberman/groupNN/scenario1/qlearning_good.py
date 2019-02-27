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

    def __init__(self, qtable, w, *args, **kwargs):
        super(QCharacter, self).__init__(*args, **kwargs)
        # Whether this character wants to place a bomb
        self.maybe_place_bomb = False
        # Debugging elements
        self.tiles = {}
        self.qtable = qtable
        self.w = w  # weight of bomb feature
        # self.wm = wm  # weight of monster distance feature
        # self.wg = wg  # weight of goal distance
        # self.ww = ww  # weight of distance to closest wall
        # self.wcm = wcm  # weight of "are we moving closer to monster"
        # self.wcg = wcg  # weight of "are we moving closer to goal"+
        self.last_state = 0
        self.last_action = (0, 0)

    def do(self, wrld):
        print("REAL WORLD COORDINATES IN DO:", self.x, self.y)
        print("STATE: ", calculate_state((self.x, self.y), wrld))
        print("REWARD OF GOING RIGHT NEXT TO GOAL: ")
        # print(self.r(wrld, (1, 0)))
        # if not self.threatened(wrld):
        #     path = aStar((self.x, self.y), wrld, wrld.exitcell)  # (7, 18) usualy
        #     print("Path " + str(path))
        #     state = calculate_state((self.x, self.y), wrld)
        #     move = path[len(path) - 1]
        #     self.approximateQ(state, move, wrld)
        #     #print("move " + str(move))
        #     print(self.r(wrld, (move[0] - self.x, move[1] - self.y)))
        #     self.move(move[0] - self.x, move[1] - self.y)
        #     self.last_state_action = (state, move)
        #     pass
        # else:
        # If we are threatened, we do qlearning
        state = calculate_state((self.x, self.y), wrld)
        move = self.policy(state, wrld)
        # TODO Check that this is right.....
        #self.approximateQ(state, move, wrld)
        print(self.w)

        print("SELECTED MOVE FROM POLICY: ", move)
        self.approximateQ(state, move, wrld)
        self.last_action = move
        self.last_state = state
        print("UPDATE Q VALUE::: ", self.qtable[(state, move)])
        self.move(move[0], move[1])

    def policy(self, state, wrld):
        actions = self.valid_moves(wrld)
        maxval = float('-inf')
        bestmoves = []
        for a in actions:
            ra = (a[0] - self.x, a[1] - self.y)  # relative action
            keys = self.qtable.keys()
            if (state, ra) not in keys:
                self.qtable[(state, ra)] = 0
            value = self.qtable[(state, ra)]
            print("ACTION, VALUE: ", ra, value)
            if value > maxval:
                bestmoves.clear()
                maxval = value
                bestmoves.append(ra)
            if value == maxval:
                bestmoves.append(ra)
        m = random.choice(bestmoves)
        print("POLICY CHOSE:::::", m, "WITH VALUE::: ", maxval)
        return m

    def valid_moves(self, wrld):
        moves = get_adjacent((self.x, self.y), wrld)
        final = []
        for m in moves:
            if not wrld.wall_at(m[0], m[1]):
                final.append(m)
        return final

    def maxQ(self, state, wrld):
        if state == "exited":
            return 10
        if state == "died":
            return -10
        keys = self.qtable.keys()
        maxval = float('-inf')
        for k in keys:
            # print(k)
            # print(state)
            if state == k[0]:  # check if state part of key is the same as ours
                score = self.qtable[k]
                if score > maxval:
                    maxval = score
        return maxval

    def approximateQ(self, state, action, wrld):

        keys = self.qtable.keys()

        if (state, action) not in keys:
            self.qtable[(state, action)] = 0

        # Update weights of each feature

        # TODO FEATURES SHOULD BE EVALUATED !!!AFTER!!! TAKING SELECTED MOVE. So need an extra feature method for each.

        delta = (self.r(wrld, action) + gamma * self.maxQ(self.getNextState(wrld, action), wrld)) - self.qtable[(state, action)]

        #self.w = self.w + alpha * delta * distance_to_exit((self.x, self.y), wrld)

        # # Feature 1: distance to bomb
        # self.wb = self.wb + alpha * delta * closest_bomb((self.x, self.y), wrld)
        #
        # # Feature 2: distance to closest monster
        # self.wm = self.wm + alpha * delta * closest_monster((self.x, self.y), wrld)
        #
        # # Feature 3: distance to goal
        # self.wg = self.wg + alpha * delta * distance_to_exit((self.x, self.y), wrld)
        #
        # # Feature 4: distance to closest wall
        # self.ww = self.ww + alpha * delta * closest_wall((self.x, self.y), wrld)
        #
        # # Feature 5: does this take us closer to monster? -1 if so, +1 if not, 0 if same
        # wcm = self.fcm(action,wrld)
        # self.wcm = self.wcm + alpha * delta * self.fcm(action, wrld)
        # print("FCM: ", wcm)
        #
        # # Feature 6: does this take us closer to goal? +1 if so, -1 if not, 0 if same
        # wcg = self.fcg(action,wrld)
        # self.wcg = self.wcg + alpha * delta * self.fcg(action, wrld)
        # print("FCG: ", wcg)

        self.qtable[(state, action)] = self.r(wrld, action) + gamma * self.maxQ(state, wrld)
        self.w = self.w + alpha * delta * distance_to_exit((self.x, self.y), wrld)
        print(self.qtable[state, action])


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
            return 0
            # print("SIMULATED COORDINATES:", c.x, c.y)
            # state = calculate_state((c.x, c.y), sim[0])
            # keys = self.qtable.keys()
            # if (state, action) not in keys:
            #     self.qtable[(state, action)] = 0
            # return self.qtable[(state, action)]

# Returns a vector of values representing each feature.
# Vector structure: (bomb distance, monster distance, exit distance)
def calculate_state(coords, wrld):
    # monster = closest_monster((coords[0], coords[1]), wrld)
    dist = distance_to_exit(coords, wrld)

    # TODO Add distance to wall??
    #return closest_bomb(coords, wrld), monster, dist, closest_wall(coords, wrld)
    return dist


# Returns 1/(A* distance to exit)^2.
def distance_to_exit(coords, wrld):
    dist = (len(aStar(coords, wrld, wrld.exitcell)) ** 2)
    if dist < 1:
        return 1
    return 1/dist


def aStar(char, wrld, mapTo, toExit=True):
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
            # print("HERE")
            break
        for next in get_adjacent(current[0], wrld):
            if wrld.wall_at(next[0], next[1]):
                cost_so_far[(next[0], next[1])] = 999
                new_cost = 1000

            elif (next[0], next[1]) in monsters and toExit:
                cost_so_far[(next[0], next[1])] = 99
                new_cost = 100

            else:
                new_cost = cost_to(current[0], next) + cost_so_far[current[0]]
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                frontier.append((next, new_cost + manhattan_distance(next[0], next[1], mapTo[0], mapTo[1])))
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
    # print("PATH: ", path)
    # print(path)

    if not len(path) == 0:
        move = path[len(path) - 1]

    # carries momentum? mayhaps not the best

    return path
# char.move(move[0] - char[0], move[1] - char[1])


# Prioritizes downward
def cost_to(current, next):
    diff = (next[0] - current[0], next[1] - current[1])
    val = abs(diff[0]) + abs(diff[1])
    if val == 2:
        return 2
    else:
        return 1

# Calculates manhattan distance between the two sets of coordinates. These could be tuples, but whatever.
def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)


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
