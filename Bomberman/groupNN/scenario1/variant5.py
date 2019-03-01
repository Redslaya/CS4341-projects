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
from testcharacter import TestCharacter
from qlearning_character import QCharacter

weights = []

for i in range(0, 20):
    w = open("weights.txt", "r")
    for line in w.readlines():
        line = line.rstrip()
        if not line:
            break
        weights.append(float(line))
    w.close()
    # Create the game
    random.seed(i) # TODO Change this if you want different random choices
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

    # TODO Add your character
    q = QCharacter(weights[0],  # wm
                   weights[1],  # wg
                   weights[2],  # ww
                   # weights[4],  # wcm
                   # weights[5],  # wcg
                   "Qlearn",  # name
                   "Q",  # avatar
                   0, 0  # position
                   )
    g.add_character(q)

    # Run!
    g.go()



win = 0
loss = 0

with open('results.txt', 'r') as infile:
    lines = infile.readlines()
    for lin in lines:
        if 'win' in lin:
            win+= 1
        else:
            loss+= 1

print("GAMES::", (win+loss))
print("WIN:::", win/(win+loss))
print("LOSS::", loss/(win+loss))

with open('results.txt', 'w') as r:
    r.write("")