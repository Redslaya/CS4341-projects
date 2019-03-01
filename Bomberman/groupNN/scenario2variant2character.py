# This is necessary to find the main code
import sys
import math
import random


sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back, Style, init
init(autoreset=True)


class scenario2variant2character(CharacterEntity):

    def __init__(self, *args, **kwargs):
        super(scenario2variant2character, self).__init__(*args, **kwargs)
        self.state = "SAFE"

    def do(self, wrld):
        self.reset_cells(wrld)
        monsters = []
        bombs = []
        explosions = []
        ex = wrld.exitcell
        for x in range(0, wrld.width()):
            for y in range(0, wrld.height()):
                if wrld.monsters_at(x, y):  # Finds all the monsters in the board
                    monsters.append((x, y))
                if wrld.bomb_at(x, y):
                    bombs.append((x, y))
                if wrld.explosion_at(x, y):
                    explosions.append((x,y))

        col_totals = [sum(x) for x in zip(*wrld.grid)]

        monster_cords = []
        for monster in monsters:
            monster_cords.append(inMonsterCoordinate(monster, self, (self.x, self.y), wrld))




        #find all coordinates that monsters can move to

        dx, dy = 0,0


        if sum(monster_cords) > 0:
            self.state = "DANGERZONE"

        print("CURRENT STATE::::::: ", self.state)

        # if not threatened go to goal directly
        #check in safe if in monster range
        if self.state == 'SAFE':

            dx, dy, pathlen, path = aStar(self, wrld, ex)

            if (self.x + dx, self.y + dy) is ex:
                self.yeet(dx, dy)
                pass

            if pathlen is 1:
                dx, dy, pathlenW, pathW = aStarNoWalls(self, wrld, ex)
                if wrld.wall_at(self.x + dx, self.y+dy):
                    if col_totals[self.y + dy] == 8:
                        self.place_bomb()
                        self.state = "BOMBING"
                        xDist, yDist, dangerCoords, desiredMove, danger = bombDistance(self.x, self.y, (self.x, self.y), wrld)
                        #calculate run away move from the position
                        moves = valid_moves(wrld, (self.x, self.y))
                        rel_moves = []
                        for move in moves:
                            rel_moves.append((move[0] - self.x, move[1] - self.y))
                        good_moves = list(desiredMove.intersection(set(rel_moves)))

                        m = random.choice(good_moves)
                        self.yeet(m[0], m[1])
                    else:
                        if wrld.wall_at(self.x + dx, self.y + dy):
                            moves = valid_moves(wrld, (self.x, self.y))
                            moves.sort(key=lambda x: x[1])
                            if not wrld.wall_at(moves[-1][0], moves[-1][1]):
                                dx, dy = (moves[-1][0] - self.x, moves[-1][1] - self.y)
                            elif len(bombs) == 0:
                                self.place_bomb()
                                self.state = "BOMBING"
                                dx, dy = 0,0
                        self.yeet(dx, dy)
                else:
                    self.yeet(dx, dy)



            else:
                self.yeet(dx, dy)
        elif self.state == "BOMBING":
            if len(bombs) == 0 and len(explosions) is not 0:
                self.yeet(0,0)
                # bomb exploding
            elif len(explosions) is 0 and len(bombs) == 0:
                self.state = "SAFE"
                dx, dy, moveLen, path = aStarNoWalls(self, wrld, ex)  # static a*
                self.yeet(dx, dy)
            else:
                xDist, yDist, dangerCoords, desiredMove, danger = bombDistance(self.x, self.y, bombs[0], wrld)
                if not danger:
                    dx, dy, moveLen, path = aStarNoWalls(self, wrld, ex)  # static a*
                    xDist, yDist, dangerCoords, desiredMove, danger2 = bombDistance(self.x + dx, self.y +dy, bombs[0], wrld)
                    if danger2:
                        self.yeet(0, 0)
                else:
                    moves = valid_moves(wrld, (self.x, self.y))
                    rel_moves = []
                    for move in moves:
                        rel_moves.append((move[0] - self.x, move[1] - self.y))
                    good_moves = list(desiredMove.intersection(set(rel_moves)))

                    m = random.choice(good_moves)
                    self.yeet(m[0], m[1])
                    # print("POSSIBLE MOVES\n", possible)

        elif self.state == "DANGERZONE":
            # make a preserving move
            good_move = random.choice(get_safe_moves(self, (self.x, self.y), wrld, monsters, bombs))
            self.place_bomb()
            self.state = "BOMBING"
            if len(bombs) == 0 and len(explosions) == 0:
                self.state = "SAFE"

            self.yeet(good_move[0], good_move[1])


        else:
            self.state = "SAFE"
            move = random.choice(valid_moves(wrld, (self.x, self.y)))
            self.yeet(move[0], move[1])




    def yeet(self, dx, dy):
        self.move(dx, dy)
        pass


    def calculateMove(self, wrld):
        xcoord = self.x
        ycoord = self.x

    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)

    # Prioritizes downward
    def cost_to(self, current, next):
        diff = (next[0] - current[0], next[1] - current[1])
        val = abs(diff[0]) + abs(diff[1])
        if val == 2:
            return 2
        else:
            return 1

    # Resets styling for each cell. Prevents unexpected/inconsistent behavior that otherwise appears with coloring.
    def reset_cells(self, wrld):
        for x in range(0, wrld.width()):
            for y in range(0, wrld.height()):
                self.set_cell_color(x, y, Fore.RESET + Back.RESET)

    def printOurWorld(self, wrld, cost_so_far):
        w, h = len(wrld.grid), len(wrld.grid[0])
        # print('\n\n')
        world = [[0 for x in range(w)] for y in range(h)]

        # for row in world:
        #     print(row)
        keys = cost_so_far.keys()

        for key in keys:
            # x,y = val[0][0], val[0][1]
            # print(x, y)
            # print("Set value at " + str(val[0][0]-1) + " , " + str(val[0][1] -1) + " to: " + str(val[1]) )

            k = str(key).split(',')

            world[key[1]][key[0]] = "{0:03d}".format(cost_so_far[key])
        world[self.y][self.x] = "X"
        #
        # for row in world:
        #     print(row)

    # Returns a list of tiles which are unsafe due to monsters.
    def monster_tiles(self, wrld):
        tiles = []
        for x in range(0, wrld.width()):
            for y in range(0, wrld.height()):
                if wrld.monsters_at(x, y):
                    for t in get_adjacent((x, y), wrld):
                        tiles.append(t)
                    tiles.append((x, y))
        # print("Monster tiles: ")
        # print(tiles)
        return tiles

    # ==================== FEATURES ==================== #
    #   - Distance to closest bomb
    #   - Distance to closest monster
    #   - 1 / (Distance to exit)^2

    # Returns an integer representing the Manhattan distance to the closest bomb.
    def closest_bomb(self):
        return 1

    # Returns an integer representing the Manhattan distance to the closest monster.
    def closest_monster(self):
        return 1

    # Returns the 1/(A* distance to exit)^2.
    def distance_to_exit(self):
        return 1


# Returns a list of coordinates in the world surrounding the current one.
# param current: An (x, y) point
def get_adjacent(current, wrld):
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



def distance_to_walls_should_place_bomb(wrld, char):
    (x,y) = (char.x, char.y)

    walls = []
    for i in range(0, wrld.width()):
        for j in range(0, wrld.height()):
            if wrld.wall_at(i, j):
                walls.append(((x,y), dist_to_wall((x,y), (i,j))))

    closest_coords = None
    closest_value = -float('inf')
    for wall, dist in walls:
        if dist > closest_value:
            closest_value = dist
            closest_coords = wall

    return False if closest_value < .5 else True


def dist_to_wall(pos, wall, wrld):
    dist = abs(pos[0] - wall[0]) + abs(pos[1] - wall[1])

    return 1 / dist

def distance_to_exit(coords, wrld):
    dist = (len(aStar(coords, wrld, wrld.exitcell)))
    if dist == 0:
        dist = 1

    return 1 / dist

def printFrontier(frontier):
    for val in frontier:
        print(val)


def aStarNoWalls(char, wrld, mapTo):
    char.reset_cells(wrld)
    frontier = []
    frontier.append(((char.x, char.y), 0))
    came_from = {}
    cost_so_far = {}
    came_from[(char.x, char.y)] = None
    cost_so_far[(char.x, char.y)] = 0
    move = 1
    # print("charX " + str(char.x))
    # print("charY " + str(char.y))

    while not len(frontier) == 0:
        frontier.sort(key=lambda tup: tup[1])  # check that
        current = frontier.pop(0)

        char.set_cell_color(current[0][0], current[0][1], Fore.RESET + Back.RESET)  # resets color
        if (current[0][0], current[0][1]) == mapTo:
            break
        for next in get_adjacent(current[0], wrld):
            new_cost = char.cost_to(current[0], next) + cost_so_far[current[0]]
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                if wrld.wall_at(next[0], next[1]):
                    cost_so_far[next] = cost_so_far[next] * 1.1

                frontier.append((next, new_cost + char.manhattan_distance(next[0], next[1], mapTo[0], mapTo[1])))
                came_from[next] = current[0]


    cursor = mapTo
    path = []
    while not cursor == (char.x, char.y):
        char.set_cell_color(cursor[0], cursor[1], Fore.RED + Back.GREEN)
        move = cursor
        path.append(cursor)
        try:
            cursor = came_from[cursor]
        except KeyError:
            char.move(0, 0)
            pass
            break


# print("PATH: ")
# print(path)

    if not len(path) == 0:
        move = path[len(path) - 1]

    # carries momentum? mayhaps not the best

    return move[0] - char.x, move[1] - char.y, len(path), path


def aStar(char, wrld, mapTo, toExit=True, ignoreWall=False):
    char.reset_cells(wrld)
    frontier = []
    frontier.append(((char.x, char.y), 0))
    came_from = {}
    cost_so_far = {}
    came_from[(char.x, char.y)] = None
    cost_so_far[(char.x, char.y)] = 0
    move = 1
    # print("charX " + str(char.x))
    # print("charY " + str(char.y))

    monsters = []
    ex = (7, 18)
    if toExit:
        # print("In here ")
        # Iterates through board to find monsters and exit
        for x in range(0, wrld.width()):
            for y in range(0, wrld.height()):
                if wrld.monsters_at(x, y):  # Finds all the monsters in the board
                    monsters.append((x, y))
                if wrld.exit_at(x, y):  # Just in case exit is not where we expect it to be in the bottom right corner
                    ex = (x, y)

        for t in monsters:
            char.set_cell_color(t[0], t[1], Fore.RED + Back.RED)

    while not len(frontier) == 0:
        frontier.sort(key=lambda tup: tup[1])  # check that
        current = frontier.pop(0)
        if toExit:
            char.set_cell_color(current[0][0], current[0][1], Fore.RESET + Back.RESET)  # resets color
        if (current[0][0], current[0][1]) == ex:
            break
        for next in get_adjacent(current[0], wrld):
            if wrld.wall_at(next[0], next[1]):
                cost_so_far[(next[0], next[1])] = 999
                new_cost = 1000
            elif (next[0], next[1]) in monsters:
                cost_so_far[(next[0], next[1])] = 99
                new_cost = 100
            else:
                new_cost = char.cost_to(current[0], next) + cost_so_far[current[0]]
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                frontier.append((next, new_cost + char.manhattan_distance(next[0], next[1], ex[0], ex[1])))
                came_from[next] = current[0]

    # char.printOurWorld(wrld, cost_so_far)

    cursor = mapTo
    path = []
    while not cursor == (char.x, char.y):
        if toExit:
            char.set_cell_color(cursor[0], cursor[1], Fore.RED + Back.GREEN)
        move = cursor
        path.append(cursor)
        try:
            cursor = came_from[cursor]
        except KeyError:
            char.move(0, 0)
            pass
            break
    # print("PATH: ")
    # print(path)

    if not len(path) == 0:
        move = path[len(path) - 1]

    # carries momentum? mayhaps not the best

    return move[0] - char.x, move[1] - char.y, len(path), path
    # char.move(move[0] - char.x, move[1] - char.y)



def bombDistance(x, y, bomb, world):
    # print("BOMB AT" + str(bomb))
    xDist = abs(bomb[0] - x)
    yDist = abs(bomb[1] - y)
    danger = False
    expl_range = world.expl_range
    dangerCoords = []
    for i in range(expl_range):
        dangerCoords.append((bomb[0] + i, bomb[1]))  # pos x dist coords
        dangerCoords.append((bomb[0] - i, bomb[1]))  # neg x dist
        dangerCoords.append((bomb[0], bomb[1] + i))  # pos y dist
        dangerCoords.append((bomb[0], bomb[1] - i))  # neg y dist

    dangerCoords = set(dangerCoords)
    if (x, y) in dangerCoords:
        danger = True
        # print("IN BAD SPOT")
    desiredMove = set([])
    if (danger):
        if x == bomb[0] and y == bomb[1]:  # standing on bomb
            desiredMove = set([(1, 1), (-1, -1), (-1, 1), (1, -1)])  # move any diagonal is preferred
        elif x == bomb[0] and yDist < expl_range:  # in x line and in range of blast
            desiredMove = set([(1, 1), (-1, -1), (0, 1), (0, -1), (-1, 1),
                               (1, -1)])  # move any diagonal or directly up or down to be out of x path
        elif y == bomb[1] and xDist < expl_range:  # in y line and in range of blast
            desiredMove = set([(1, 1), (-1, -1), (0, 1), (0, -1), (-1, 1),
                               (1, -1)])  # move any diagonal or directly left or right to be out of y path

    return xDist, yDist, dangerCoords, desiredMove, danger


def valid_moves(wrld, pos):
    moves = get_adjacent((pos[0], pos[1]), wrld)
    final = []
    for m in moves:
        if not wrld.wall_at(m[0], m[1]):
            final.append(m)
    return final


def inMonsterCoordinate(monster, player, playerPos, wrld):

    player.x = playerPos[0]
    player.y = playerPos[1]
    man = abs(monster[0] - playerPos[0]) + abs(monster[1]-playerPos[1])

    # x, y, plen, path = aStar(player, wrld, monster, False)

    if man <= 3 :
        return True
    else:
        return False


def get_safe_moves(char, player, wrld, monsters, bombs):
    moves = valid_moves(wrld, player)
    rel_moves = []
    for move in moves:
        rel_moves.append((move[0] - player[0], move[1] - player[1]))

    good_moves = []

    for monster in monsters:
        for move in rel_moves:
            if not inMonsterCoordinate(monster, char, (player[0] + move[0], player[1] + move[1]), wrld):
                good_moves.append(move)

    good_moves = list(set(good_moves))

    for move in good_moves:
        for bomb in bombs:
            xDist, yDist, dangerCoords, desiredMove, danger = bombDistance(player[0] + move[0], player[1] + move[1], bomb, wrld)
            if danger:
                index = good_moves.index(move)
                del good_moves[index]

    return list(set(good_moves))