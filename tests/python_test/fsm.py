#!/usr/bin/env python

class StateMachine:
    def __init__(self):
        self.state = 'state1'

    def state1_function(self):
        print("Executing State 1 Function")

    def state2_function(self):
        print("Executing State 2 Function")

    def state3_function(self):
        print("Executing State 3 Function")

    def state4_function(self):
        print("Executing State 4 Function")

    def transition(self, event):
        if self.state == 'state1':
            if event == 'event1':
                self.state = 'state2'
        elif self.state == 'state2':
            if event == 'event2':
                self.state = 'state3'
        elif self.state == 'state3':
            if event == 'event3':
                self.state = 'state4'
        elif self.state == 'state4':
            if event == 'event4':
                self.state = 'state1'

        # Execute function corresponding to the current state
        getattr(self, f'{self.state}_function')()

# Example usage:
if __name__ == "__main__":
    fsm = StateMachine()
    print("Current State:", fsm.state)

    fsm.transition('event1')
    print("Current State:", fsm.state)

    fsm.transition('event2')
    print("Current State:", fsm.state)

    fsm.transition('event3')
    print("Current State:", fsm.state)

    fsm.transition('event4')
    print("Current State:", fsm.state)



