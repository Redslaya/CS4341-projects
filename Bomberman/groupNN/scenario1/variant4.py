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
from testcharacter import TestCharacter
from qlearning_character import QCharacter

# Create the game
for i in range(0, 20):
    random.seed(i)  # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')
    g.add_monster(SelfPreservingMonster("aggressive", # name
                                        "A",          # avatar
                                        3, 13,        # position
                                        2             # detection range
    ))

    # TODO Add your character
    q = QCharacter(-100,  # wm
                   100,  # wg
                   # weights[3],  # ww
                   # weights[4],  # wcm
                   # weights[5],  # wcg
                   "Qlearn",  # name
                   "Q",  # avatar
                   0, 0  # position
                   )

    g.add_character(q)

    # Run!
    g.go()
