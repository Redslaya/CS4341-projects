# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
from collections import defaultdict

STATE_MOVING = 0
STATE_DEFENSE = 1
STATE_BOMBING = 2
CARD = [(0,1),(0,-1),(1,0),(-1,0)]

class TestCharacter(CharacterEntity):

    def __init__(self, name, avatar, x, y):
        super().__init__(name, avatar, x, y)
        self.state = STATE_MOVING
        self.bomb = None
        self.bomb_radius = []
        self.expl_count = -1

    def estimate_distance(self, position, goal):
        return ((goal[0]-position[0])**2 + (goal[1]-position[1])**2)**(1/2)

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.insert(0, current)
        return total_path

    @staticmethod
    def is_valid_loc(nx, ny, wrld, walls=False):
        return 0 <= nx < wrld.width() and 0 <= ny < wrld.height() and (walls or not wrld.wall_at(nx, ny))

    def monster_radius(self, monsters, wrld):
        bad_spaces = set()
        for monster in monsters:
            for dx in range(-3, 3):
                for dy in range(-3, 3):
                    mx = monster[0]+dx
                    my = monster[1]+dy
                    if self.is_valid_loc(mx, my, wrld):
                        bad_spaces.add((mx, my))
        return bad_spaces

    def valid_neighbors(self, current, wrld, walls=False):
        result = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx = current[0] + dx
                ny = current[1] + dy
                if (dx != 0 or dy != 0) and self.is_valid_loc(nx, ny, wrld, walls):
                    result.append((nx, ny))
        return result

    def preserve(self, wrld, monsters):
        move = (self.x, self.y)
        best = 0
        for monster in monsters:
            best = min(best, self.estimate_distance((self.x, self.y), monster))
        for neighbor in self.valid_neighbors((self.x, self.y), wrld):
            if self.expl_count <= 0 and neighbor in self.bomb_radius:
                continue
            closest_monster = float("inf")
            for monster in monsters:
                dist = self.estimate_distance(neighbor, monster)
                closest_monster = min(dist, closest_monster)
            if closest_monster > best:
                move = neighbor
                best = closest_monster
        return move


    def move_bombing(self, wrld):
        print(self.target)
        print(self.x, self.y)
        for dirx,diry in CARD:
            tx, ty = self.target[0]+dirx, self.target[1]+diry
            print(tx, ty)
            path = self.a_star(wrld, (tx, ty))
            if path is not None:
                nx = path[1][0]
                ny = path[1][1]
                return nx, ny
        return None

    def place_bomb(self):
        super().place_bomb()
        self.bomb = (self.x, self.y)
        self.bomb_radius.append(self.bomb)
        for i in range(-1*self.expl_range, self.expl_range):
            self.bomb_radius.append((self.x+i, self.y))
            self.bomb_radius.append((self.x, self.y+i))
        self.expl_count = self.bomb_time

    def a_star(self, wrld, exit, walls=False):
        visited = set()
        frontier = [(self.x, self.y)]
        came_from = {}
        g_scores = defaultdict(lambda: float("inf"))
        g_scores[(self.x, self.y)] = 0
        f_scores = defaultdict(lambda: float("inf"))
        f_scores[(self.x, self.y)] = self.estimate_distance((self.x, self.y), exit)
        path = None

        while frontier:
            min_score, current = float("inf"), None
            for loc in frontier:
                if f_scores[loc] < min_score:
                    min_score = f_scores[loc]
                    current = loc
            frontier.remove(current)

            if current == exit:
                path = self.reconstruct_path(came_from, current)
                break

            visited.add(current)

            for neighbor in self.valid_neighbors(current, wrld, walls):
                if neighbor in visited:
                    continue

                tentative_gScore = g_scores[current] + 1

                if neighbor not in frontier:
                    frontier.append(neighbor)
                elif tentative_gScore >= g_scores[neighbor]:
                    continue

                came_from[neighbor] = current
                g_scores[neighbor] = tentative_gScore
                f_scores[neighbor] = g_scores[neighbor] + self.estimate_distance(neighbor, exit)

        return path

    def do(self, wrld):
        self.bomb_time = wrld.bomb_time
        self.expl_range = wrld.expl_range
        # Your code here
        w = wrld.width()
        h = wrld.height()
        exit = None
        monsters = []

        for x in range(w):
            for y in range(h):
                if wrld.exit_at(x, y):
                    exit = (x,y)
                if wrld.monsters_at(x, y):
                    monsters.append((x,y))

        monster_spaces = self.monster_radius(monsters, wrld)
        nx, ny = (self.x, self.y)
        print("===== STATE:", self.state, "=====")
        print("i'm at", self.x, self.y)
        if self.state == STATE_MOVING:
            path = self.a_star(wrld, exit)
            if path is None:
                print("trying walls true")
                path = self.a_star(wrld, exit, True)
                path_copy = path.copy()
                short_path = None
                while path_copy and short_path is None:
                    new_goal = path_copy.pop()
                    short_path = self.a_star(wrld, new_goal)
                    print("tried", new_goal, "got", short_path)

                if len(short_path) > 1:
                    print("in")
                    path = short_path

            nx = path[1][0]
            ny = path[1][1]
            if wrld.wall_at(nx, ny) and (nx-self.x)!=0 and (ny-self.y)!=0:
                self.state = STATE_BOMBING
                self.target = (nx, ny)
                nx, ny = self.move_bombing(wrld)
            elif wrld.wall_at(nx, ny) or (nx,ny) in monster_spaces:
                self.place_bomb()
                self.state = STATE_DEFENSE
                nx, ny = self.preserve(wrld, monsters)
            dx = nx-self.x
            dy = ny-self.y
            print("moving by", dx, dy)
            self.move(dx, dy)
        elif self.state == STATE_DEFENSE:
            self.expl_count -= 1
            nx, ny = self.preserve(wrld, monsters)
            dx = nx - self.x
            dy = ny - self.y
            print("moving by", dx, dy)
            self.move(dx, dy)
            if not wrld.bomb_at(self.bomb[0], self.bomb[1]) and \
                not wrld.explosion_at(self.bomb[0], self.bomb[1]):
                self.state = STATE_MOVING
                self.bomb = None
                self.bomb_radius = []
        elif self.state == STATE_BOMBING:
            nx, ny = -1, -1
            for dirx,diry in CARD:
                if (self.x+dirx, self.y+diry) == self.target:
                    self.place_bomb()
                    self.state = STATE_DEFENSE
                    nx, ny = self.preserve(wrld, monsters)
                    break
            if nx == -1 and ny == -1:
                nx, ny = self.move_bombing(wrld)
            dx = nx - self.x
            dy = ny - self.y
            print("moving by", dx, dy)
            self.move(dx, dy)


        return