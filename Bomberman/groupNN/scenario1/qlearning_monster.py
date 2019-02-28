# The sole purpose of this class is to be able to carry q-table over between iterations of the game.

# This is necessary to find the main code
import sys
import pickle
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')
from events import Event

# Import necessary stuff
from game import Game

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from qlearning_character import QCharacter
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster


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


won = 0
lost = 0


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
    g = Game.fromfile('map.txt')

    # g.add_monster(StupidMonster("monster",  # name
    #                             "M",  # avatar
    #                             3, 9  # position
    #                             ))

    g.add_monster(SelfPreservingMonster("monster",  # name
                                        "M",  # avatar
                                        3, 9,  # position
                                        2  # detection range
                                        ))

    # TODO Add your character
    q = QCharacter(     weights[0],  # wm
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
    for e in g.world.events:
        print(e)
        if e.tpe == Event.CHARACTER_FOUND_EXIT and e.character.name == q.name:
            won += 1
    print((q.x, q.y))

    w = open("weights.txt", "w")
    weights[0] = q.wm
    weights[1] = q.wg
    weights[2] = q.ww
    # weights[3] = q.ww
    # weights[4] = q.wcm
    # weights[5] = q.wcg
    for weight in weights:
        w.write(str(weight) + "\n")
    w.close()
    weights.clear()


save_obj(qtable, "qtb")

print("WON: ", won)
print("LOST: ", lost)
print("TOTAL: ", won + lost)
print("PERCENT: ", won / (won + lost))