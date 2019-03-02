# # TODO RECOMMENDED WEIGHTS:::
# -100.83230412067729
# 97.50308763796815
# -52.49691236203184


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
from scenario2variant4character import QCharacter

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
g.add_monster(SelfPreservingMonster("aggressive", # name
                                    "A",          # avatar
                                    3, 13,        # position
                                    2             # detection range
))

# TODO Add your character
q = QCharacter(-116.60590427878707,
                65.16728716363879,
                -84.83271283636117,  # wm
               # ww
               # weights[3],  # weight distance to bomb
               # weights[4],  # weight of "are we in the path of a bomb
               # weights[5],  # weight of "are we gonna explode"
               # weights[4],  # wcm
               # weights[5],  # wcg
               "Qlearn",  # name
               "Q",  # avatar
               0, 0  # position
               )
g.add_character(q)




# Run!
g.go()

