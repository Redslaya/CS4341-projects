# The sole purpose of this class is to be able to carry q-table over between iterations of the game.

# This is necessary to find the main code
import sys
import pickle
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
from game import Game

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from qlearning_character import QCharacter
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster
from qlearning_character import calculate_state


def save_obj(obj, name):
    with open('obj/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)


def load_obj(name):
    with open('obj/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)

weights = []
try:
    qtable = load_obj("qtb")
except:
    print("Couldnt load Q")
    qtable = {}
# qtable = {}

#TODO Qtable appending not updating sometimes???
for i in range(0, 20):
    w = open("weights.txt", "r")
    for line in w.readlines():
        line = line.rstrip()
        if not line:
            break
        weights.append(float(line))
    w.close()
    # Create the game
    g = Game.fromfile('map2.txt')

    # g.add_monster(StupidMonster("monster",  # name
    #                             "M",  # avatar
    #                             3, 9  # position
    #                             ))

    # g.add_monster(SelfPreservingMonster("monster",  # name
    #                                     "M",  # avatar
    #                                     0, 0,  # position
    #                                     2  # detection range
    #                                     ))

    # TODO Add your character
    q = QCharacter(qtable,   # starting q table
                        weights[0],  # wb
                        # weights[1],  # wm
                        # weights[2],  # wg
                        # weights[3],  # ww
                        # weights[4],  # wcm
                        # weights[5],  # wcg
                               "Qlearn",  # name
                                "Q",  # avatar
                                1, 0  # position
                                )
    g.add_character(q)
    # Run!
    g.go()
    print(g.world.events)
    print("G DONE::::::: " + str(g.done()))
    print((q.x, q.y))

    w = open("weights.txt", "w")
    weights[0] = q.w
    # weights[1] = q.wm
    # weights[2] = q.wg
    # weights[3] = q.ww
    # weights[4] = q.wcm
    # weights[5] = q.wcg
    for weight in weights:
        w.write(str(weight) + "\n")
    w.close()
    weights.clear()
    print(":::QTABLE:::")
    for q in qtable.keys():
        print(q, qtable[q])

save_obj(qtable, "qtb")