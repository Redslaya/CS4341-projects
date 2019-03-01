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


from hiEthan import TestCharacter

for i in range(100):
# Create the game
    random.seed() # TODO Change this if you want different random choices
    g = Game.fromfile('map.txt')
    g.add_monster(SelfPreservingMonster("aggressive",  # name
                                    "A",  # avatar
                                    3, 13,  # position
                                    2  # detection range
                                    ))


# TODO Add your character
    g.add_character(TestCharacter("me", # name
                                  "C",  # avatar
                                  0, 0  # position
    ))

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
# Create the game
