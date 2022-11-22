import move_robot
import read_plate

class state_machine:

    def __init__(self):
        self.states = {"drive", "cross_walk", "found_car"}
        self.currentState

    def state_change(self, state):
        if self.states.contains(state):
            self.currentState = state
        else:
            print("Not a state")

    # def run(self):
    #     while True: