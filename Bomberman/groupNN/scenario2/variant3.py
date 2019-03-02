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

for i in range(5):

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

    q = QCharacter(weights[0],
                   weights[1],
                   weights[2],
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
    # weights[3] = q.wcb
    # weights[4] = q.wbr
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
with open('results.txt', 'r') as infile:
    lines = infile.readlines()
    for line in lines:
        if 'victory' in line:
            win += 1
        else:
            loss += 1


print("GAMES::: ", win+loss)
print("WIN:::: ", win/(win+loss))
print("Loss:::: ", loss/(win+loss))