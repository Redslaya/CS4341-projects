# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.stupid_monster import StupidMonster

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from scenario2variant5 import QCharacter

weights = []

for i in range(10):

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
                                3, 9      # position
    ))

    # TODO Add your character
    q = QCharacter(weights[0],  # wm
                   weights[1],  # wg
                   weights[2],  # ww
                   weights[3],  # weight distance to bomb
                   weights[4],  # weight of "are we in the path of a bomb
                   weights[5],  # weight of "are we gonna explode"
                   # weights[4],  # wcm
                   # weights[5],  # wcg
                   "Qlearn",  # name
                   "Q",  # avatar
                   0, 0  # position
                   )
    g.add_character(q)

    # Run!
    g.go()

    w = open("weights.txt", "w")
    weights[0] = q.wm
    weights[1] = q.wg
    weights[2] = q.wc
    weights[3] = q.wcb
    weights[4] = q.wbr
    weights[5] = q.wxp
    # weights[3] = q.ww
    # weights[4] = q.wcm
    # weights[5] = q.wcg
    for weight in weights:
        w.write(str(weight) + "\n")
    w.close()
    weights.clear()

