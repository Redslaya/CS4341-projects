# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from scenario2variant3character import QCharacter

weights = []


w = open("weights_s2_v3.txt", "r")
for line in w.readlines():
    line = line.rstrip()
    if not line:
        break
    weights.append(float(line))
w.close()

# Create the game
random.seed() # TODO Change this if you want different random choices
g = Game.fromfile('map.txt')
g.add_monster(SelfPreservingMonster("selfpreserving", # name
                                    "S",              # avatar
                                    3, 9,             # position
                                    1                 # detection range
))

q = QCharacter(-100,
                100,
                -50,
               "Qlearn",  # name
               "Q",  # avatar
               0, 0  # position
               )
g.add_character(q)

# Run!
g.go()


