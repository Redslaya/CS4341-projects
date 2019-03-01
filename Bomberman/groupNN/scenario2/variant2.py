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
from scenario2variant2character import QCharacter

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

print("GAMES::: ", win + loss)
print("WIN:::: ", win / (win + loss))
print("Loss:::: ", loss / (win + loss))