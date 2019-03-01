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

    def __init__(self, wm, wg, wc, wcb, wbr, wxp, *args, **kwargs):
        super(QCharacter, self).__init__(*args, **kwargs)
        # Whether this character wants to place a bomb
        self.maybe_place_bomb = False
        # Debugging elements
        self.tiles = {}
        # self.w = w  # weight of bomb feature
        self.wm = wm  # weight of monster distance feature
        self.wg = wg  # weight of goal distance
        # self.ww = ww  # weight of wall distance
        self.wc = wc
        self.wcb = wcb
        self.wbr = wbr
        self.wxp = wxp
        # self.ww = ww  # weight of distance to closest wall
        # self.wcm = wcm  # weight of "are we moving closer to monster"
        # self.wcg = wcg  # weight of "are we moving closer to goal"+

        # return closest_bomb(), closest_monster((coords[0], coords[1]), wrld), monster_direction(coords, wrld), dist

    def do(self, wrld):
        print("THREATENED:: ", self.threatened(wrld))
        if not self.threatened(wrld):
            if len(wrld.bombs.values()) != 0:
                self.escape_bomb(wrld)
                return
            if len(wrld.explosions.values()) > 0:
                self.move(0, 0)
                pass
                return
            else:
                path = aStar((self.x, self.y), wrld, wrld.exitcell)
                move = path[len(path) - 1]
                if len(path) <= 1:
                    self.bomb_if_able(wrld)
                self.move(move[0] - self.x, move[1] - self.y)
                pass
                return

        # TODO !!!!!!!!!!!!!

        dist_to_goal, dist_to_monster, dist_to_corner, dist_to_bomb, bomb_range, explosion_dist = calculate_features((self.x, self.y), wrld)
        #dist_to_goal, dist_to_monster, dist_to_corner, = calculate_features((self.x, self.y), wrld)
        actions = self.valid_moves(wrld)

        rel_actions = []
        for a in actions:
            if a != "bomb":
                rel_actions.append((a[0] - self.x, a[1] - self.y))
            else:
                rel_actions.append("bomb")

        # list of (rel_action, (features))
        Qs = []
        for rel in rel_actions:
            # calculate features of that new world
            Qs.append((rel, self.Q(wrld, rel)))

        best_q = float("-inf")
        rel_move = []

        for q in Qs:
            if q[1] == best_q:
                rel_move.append(q[0])
            if q[1] > best_q:
                rel_move.clear()
                best_q = q[1]
                rel_move.append(q[0])

        move = random.choice(rel_move)

        # update weights

        print("MOVE::::: ", move)
        delta = (self.r(wrld, move) + gamma * best_q) - self.Q(wrld, move)

        print("DELTA::: ", delta)
        self.wm = self.wm + alpha * delta * dist_to_monster

        self.wg = self.wg + alpha * delta * dist_to_goal

        # self.ww = self.ww + alpha * delta * dist_to_wall

        self.wc = self.wc + alpha * delta * dist_to_corner

        self.wcb = self.wcb + alpha * delta * dist_to_bomb

        self.wbr = self.wbr + alpha * delta * bomb_range

        self.wxp = self.wxp + alpha * delta * explosion_dist

        # get the state
        # get the valid actions
        # test the valid actions
        # choose best action

        self.move(move[0], move[1])
        print("WEIGHTS::::")
        print("MONST WEIGHT :::: ", self.wm)
        print("GOAL WEIGHT :::: ", self.wg)
        pass

    def bomb_if_able(self, wrld):
        if len(wrld.bombs) < 1:
            for a in get_adjacent((self.x, self.y), wrld):
                if wrld.wall_at(a[0], a[1]):
                    self.place_bomb()
                    return

    def escape_bomb(self, wrld):
        if bomb_range((self.x, self.y), wrld) == 0:
            self.move(0, 0)
            pass
        else:
            adj = self.valid_moves(wrld)
            for a in adj:
                if bomb_range(a, wrld) == 0:
                    self.move(a[0] - self.x, a[1] - self.y)
                    pass


    def Q(self, wrld, action):
        next_wrld = self.getNextWorld(wrld, action)

        c = next_wrld[0].me(self)

        if c is None:
            for event in next_wrld[1]:
                if event.tpe == Event.CHARACTER_FOUND_EXIT and event.character.name == self.name:
                    # print("WE CAN WIN!!!!!")
                    return 100
                elif event.tpe == Event.CHARACTER_KILLED_BY_MONSTER and event.character.name == self.name:
                    # print("WE CAN DIE!!!")
                    return -100
                elif event.tpe == Event.BOMB_HIT_CHARACTER and event.character.name == self.name:
                    # print("WE CAN DIE!!!")
                    return -50
                else:  # Timed out??
                    return -1

        # goal_dist, monst_dist, corner_dist = calculate_features((c.x, c.y), next_wrld[0])
        goal_dist, monst_dist, corner_dist, bomb_dist, bomb_range, exp_dist = calculate_features((c.x, c.y), next_wrld[0])

        #return self.wg * goal_dist + self.wm * monst_dist + self.wc * corner_dist
        return self.wg * goal_dist + self.wm * monst_dist + self.wc * corner_dist + self.wcb * bomb_dist + self.wbr * bomb_range + self.wxp * exp_dist

    # taking in world, rel_action -- calculates new reward
    def getNextWorld(self, wrld, action):
        sim = SensedWorld.from_world(wrld)
        c = sim.me(self)  # finds our character in the simulated world
        # Are monsters moving?????
        # print("action:", action)
        if action == "bomb":
            c.place_bomb()
        else:
            c.move(action[0], action[1])  # moves character in simulated world
        sim = sim.next()  # updates simulated world
        c = sim[0].me(self)
        # sim[0] is world
        return sim

    # TODO not detecting wins or losses quite properly.
    def r(self, wrld, action):
        # print("REAL WORLD COORDINATES:", self.x, self.y)
        sim = SensedWorld.from_world(wrld)  # creates simulated world
        c = sim.me(self)  # finds our character in the simulated world
        # Are monsters moving?????
        # print("action:", action)
        if action == "bomb":
            c.place_bomb()
        else:
            c.move(action[0], action[1])  # moves character in simulated world
        sim = sim.next()  # updates simulated world

        # print(monster_tiles(sim[0]))
        c = sim[0].me(c)  # finds our character in the simulated world
        if c is None:
            for event in sim[1]:
                if event.tpe == Event.CHARACTER_FOUND_EXIT and event.character.name == self.name:
                    # print("WE CAN WIN!!!!!")
                    return 100
                elif event.tpe == Event.BOMB_HIT_CHARACTER and event.character.name == self.name:
                    # print("WE CAN DIE!!!")
                    return -50
                else:
                    return -100
        else:
            if (self.x, self.y) in get_corners(wrld):
                return -15
            return 0.1

    def threatened(self, wrld):
        # TODO Add a check for bombs as well
        dist = aStar_to_monster((self.x, self.y), wrld)

        if dist is not None and dist <= 3:
            return True
        return False

    def valid_moves(self, wrld):
        moves = get_adjacent((self.x, self.y), wrld)
        final = []
        bomb = None
        for m in moves:
            for b in wrld.bombs.values():
                bomb = b
            if not wrld.wall_at(m[0], m[1]) and not wrld.explosion_at(m[0], m[1]) and not (bomb_range((m[0], m[1]), wrld) != 0 and bomb is not None and bomb.timer <= 2):
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
    # wall = closest_wall(coords, wrld)
    corner = closest_corner(coords, wrld)
    bomb = closest_bomb(coords, wrld)
    br = bomb_range(coords, wrld)
    exp = closest_explosion(coords, wrld)
    # TODO Add distance to wall??
    #return closest_bomb(coords, wrld), monster, dist, closest_wall(coords, wrld)
    return dist, monster, corner , bomb, br, exp  # wall


# ==================== FEATURES ==================== #
#   - Distance to closest bomb
#   - Distance to closest monster
#   - Distance to goal
#   - Distance to closest wall


def bomb_range(coords, wrld):
    for b in wrld.bombs.values():
        print("BOMB:", b, b.x, b.y)
        if b.x - coords[0] == 0 or b.y - coords[1] == 0:
            print("TRUE!")
            return 1 / (1 + manhattan_distance(coords[0], coords[1], b.x, b.y) + abs(b.timer - wrld.bomb_time) ** 2)
    return 0

def closest_explosion(coords, wrld):
    mindist = float('inf')
    for e in wrld.explosions.values():
        dist = manhattan_distance(e.x, coords[0], e.y, coords[1])
        if dist < mindist:
            mindist = dist
        if dist == 0:
            return 1
    return 1 / mindist ** 2

# Returns an integer representing the A* distance to the closest monster.
def closest_monster(coords, wrld):
    x = coords[0]
    y = coords[1]
    monsters = monster_tiles(wrld)
    p = float('inf')
    for m in monsters:
        distance = len(aStar((x, y), wrld, m, False))
        # print("Moster Dist:" , distance)
        if distance < p:
            p = distance
        if p == 0:
            p = 1
    return 1 / p


def closest_monster_coords(coords, wrld):
    x = coords[0]
    y = coords[1]
    monsters = monster_tiles(wrld)
    p = float('inf')
    coords = None
    for m in monsters:
        distance = len(aStar((x, y), wrld, m, False))
        if distance < p:
            p = distance
            coords = m
    return coords


# aStar distance to closest monster
def aStar_to_monster(coords, wrld):
    x = coords[0]
    y = coords[1]
    monsters = monster_tiles(wrld)
    p = float('inf')
    for m in monsters:
        path = aStar((x, y), wrld, m, False)
        print("PATH: ", path)
        distance = len(path)
        if distance == 1 and path[0] == (m[0], m[1]):
            return None
        # print("Moster Dist:" , distance)
        if distance < p:
            p = distance
    return p

# aStar distance to closest bomb
def aStar_to_bomb(coords, wrld):
    x = coords[0]
    y = coords[1]
    bombs = wrld.bombs.values()
    p = float('inf')
    for b in bombs:
        distance = len(aStar((x, y), wrld, (b.x, b.y), False))
        # print("Moster Dist:" , distance)
        if distance < p:
            p = distance
    return 1 / p ** 2


# Returns 1/(A* distance to exit).
def distance_to_exit(coords, wrld):
    dist = (len(aStar(coords, wrld, wrld.exitcell)))
    if dist == 0:
        dist = 1

    return 1 / dist


# Returns aStar distance to exit
def aStar_to_exit(coords, wrld):
    dist = (len(aStar(coords, wrld, wrld.exitcell)))

    return dist


# Returns 1 / closestBomb
def closest_bomb(coords, wrld):
    bombs = wrld.bombs
    mindist = float('inf')
    for b in bombs.values():
        print("BOMB:", b)
        dist = len(aStar(coords, wrld, (b.x, b.y)))
        if dist < mindist:
            mindist = dist
        if dist == 0:
            return 1
    return 1 / mindist


def closest_wall(coords, wrld):
    walls = get_walls(wrld)
    mindist = float('inf')
    for w in walls:
        dist = manhattan_distance(coords[0], coords[1], w[0], w[1])
        if dist < mindist:
            mindist = dist
        if mindist == 0:
            return 0.5
    return 1 / ((mindist + 1) ** 3)


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
        distance = len(aStar((x, y), wrld, m, False))
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


# Returns a list of all the coordinates with 5 walls (or board edge) touching it.
# A corner looks like this:  X as in there are only 3 valid moves to move out of that space.
#                           0X
#                          XXX
def get_corners(wrld):
    corners = []
    for x in range(0, wrld.width()):
        for y in range(0, wrld.height()):
            moves = available_moves((x, y), wrld)
            if moves <= 3 and (x, y) != wrld.exitcell:
                corners.append((x, y))
    return corners


# Returns the A* distance to the closest corner
def closest_corner(coords, wrld):
    corners = get_corners(wrld)
    min_dist = float("inf")
    for c in corners:
        dist = len(aStar(coords, wrld, c))
        if dist < min_dist:
            min_dist = dist
    if min_dist == 0:
        return 1
    return 1 / min_dist


# Returns the number of valid moves from the given coordinate.
def available_moves(coords, wrld):
    adjacent = get_adjacent(coords, wrld)
    moves = 0
    for a in adjacent:
        if not wrld.wall_at(a[0], a[1]):
            moves += 1
    return moves


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
            if wrld.wall_at(next[0], next[1]) or wrld.explosion_at(next[0], next[1]):
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