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

# for i in range(0, 10):
    # Create the game
random.seed() # TODO Change this if you want different random choices
g = Game.fromfile('map.txt')
g.add_monster(SelfPreservingMonster("selfpreserving", # name
                                    "S",              # avatar
                                    3, 9,             # position
                                    1                 # detection range
                                    ))

# TODO Add your character
q = QCharacter(-100,  # wm
               100,  # wg
               -10,  # ww
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

#
# win = 0
# loss = 0
#
# with open('results.txt', 'r') as infile:
#     lines = infile.readlines()
#     for lin in lines:
#         if 'win' in lin:
#             win+= 1
#         else:
#             loss+= 1
#
# print("GAMES::", (win+loss))
# print("WIN:::", win/(win+loss))
# print("LOSS::", loss/(win+loss))
#
# with open('results.txt', 'w') as r:
#     r.write("")

