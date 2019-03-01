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
from variant2character import QCharacter

# Create the game
for i in range(100):
    random.seed() # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')
    g.add_monster(StupidMonster("stupid", # name
                                "S",      # avatar
                                3, 9      # position
    ))

    # TODO Add your character
    q = QCharacter(-100,  # wm
                   100,  # wg
                   # weights[3],  # ww
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
    for line in lines:
        if 'victory' in line:
            win += 1
        else:
            loss += 1


print("GAMES::: ", win+loss)
print("WIN:::: ", win/(win+loss))
print("Loss:::: ", loss/(win+loss))