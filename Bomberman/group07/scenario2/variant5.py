# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from scenario2variant5character import QCharacter

weights = []

w = open("weights.txt", "r")
for line in w.readlines():
    line = line.rstrip()
    if not line:
        break
    weights.append(float(line))
w.close()

# Create the game
random.seed() # TODO Change this if you want different random choices
g = Game.fromfile('map.txt')
g.add_monster(StupidMonster("stupid", # name
                            "S",      # avatar
                            3, 5,     # position
))
g.add_monster(SelfPreservingMonster("aggressive", # name
                                    "A",          # avatar
                                    3, 13,        # position
                                    2             # detection range
))
q = QCharacter(-106.18527824504584,  # wm
               90.82070285421288,  # wg
               -59.1792971457871               ,  # ww
               -1.0396104369006423,  # weight distance to bomb
               -9.811175200382962,  # weight of "are we in the path of a bomb
               -10,  # weight of "are we gonna explode"
               # weights[4],  # wcm
               # weights[5],  # wcg
                               "Qlearn",  # name
                                "Q",  # avatar
                                0, 0  # position
                                )
g.add_character(q)

# Run!
g.go()

