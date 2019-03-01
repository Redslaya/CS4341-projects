# This is necessary to find the main code
import sys
import random
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from scenario1character import QCharacter
from monsters.stupid_monster import StupidMonster

for i in range(50):
    # Create the game
    weights = []
    w = open("weights.txt", "r")
    for line in w.readlines():
        line = line.rstrip()
        if not line:
            break
        weights.append(float(line))
    w.close()

    # Create the game
    random.seed()  # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')

    g.add_monster(StupidMonster("stupid", # name
                                "S",      # avatar
                                3, 5,     # position
    ))
    q = QCharacter(weights[0],  # wm
                   weights[1],  # wg
                   # weights[2],  # ww
                   # weights[3],  # weight distance to explosion
                   # weights[4],  # weight of "are we in the center
                   #weights[5],  # weight of "are we gonna explode"
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
    weights[3] = q.wxp
    weights[4] = q.ws
    # weights[5] = q.wxp
    # weights[3] = q.ww
    # weights[4] = q.wcm
    # weights[5] = q.wcg
    for weight in weights:
        w.write(str(weight) + "\n")
    w.close()
    weights.clear()

win = 0
loss = 0
with open('results.txt' ,'r') as results:
    lines = results.readlines()

    for line in lines:
        if 'victory' in line:
            win += 1
        else:
            loss+= 1

print("WIN ::::: " , win/(win+loss))
print("LOSS ::::: " , loss/(win+loss))