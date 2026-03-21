import pygame
import time
#publish tro thrust_control
#msg type Float64MultiArraY
#inward thrust is -100
#outward thrust is 100
# """
#   0-----1
#  /       \
# /         \
# 5         2
# \         / 
#  \       /
#   4-----3
# """

#publisher
#the message, diff

#1. log msg, wait for next msg
#2. append time waited, newline
#3. repeat

#1. pub pwm, wait

class JoyStick:
    def __init__(self):
        self.startTime = int(time.time() * 1000)
        self.listening = False
        print("Initialized")

    def connect(self):
        pygame.init()
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        print(f"Detected {joystick_count} joysticks")
        if joystick_count > 0:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            print(f"Controller detected: {self.controller.get_name()}")
        else:
            print("No controller found.")
    
    def listen(self):
        self.listening = not self.listening
        print(f"Listening: {self.listening}")

        while self.listening:

            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    self.pubButtonPress(event.button)

            left_axis_x = self.controller.get_axis(0)
            left_axis_y = self.controller.get_axis(1)
            right_axis_x = self.controller.get_axis(2)
            right_axis_y = self.controller.get_axis(3)
            self.sticks = [[left_axis_x, left_axis_y], [right_axis_x, right_axis_y]]

            deadband = 0.1
            for i in {0,1}:
                if abs(self.sticks[i][1]) < deadband: self.sticks[i][1] = 0;
                if abs(self.sticks[i][0]) < deadband: self.sticks[i][0] = 0;

            for stick in self.sticks:
                if stick[0] != 0 or stick[1] != 0:
                    self.pubStickMove()
                    break
        
    def pubStickMove(self):
        # self.sticks[][]
        # [left_x, left_y]
        # [right_x, right_y]
        # y: -1 full up 1 full down
        # x: -1 full left 1 full right
        
        print(f"LX: {self.sticks[0][0]} LY: {self.sticks[0][1]} RX: {self.sticks[0][0]} RY: {self.sticks[1][1]}")        


    def pubButtonPress(self, button):
        print(f"Button Pressed: {button}")

def main(args=None):
    joystick = JoyStick()
    joystick.connect()
    joystick.listen()