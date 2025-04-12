import pygame
import math
import time
pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
joysticks = []

for i in range(joystick_count):
    joystick = pygame.joystick.Joystick(i)
    joystick.init()
    joysticks.append(joystick)
    print(f"Joystick {i}: {joystick.get_name()}")

def truncate(number):
    factor = 10.0 ** 2
    return math.trunc(number * factor) / factor

running = True
while running:
    axes = [truncate(joystick.get_axis(i)) for i in range(joystick.get_numaxes())]
    print(axes)
    for event in pygame.event.get():
        
        buttons = [joystick.get_button(i) for i in range(10)]
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYAXISMOTION:
            print(f"Joystick {event.joy} axis {event.axis} motion: {event.value}")
        elif event.type == pygame.JOYBALLMOTION:
            print(f"Joystick {event.joy} ball {event.ball} motion: {event.rel}")
        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"Joystick {event.joy} button {event.button} down")
        elif event.type == pygame.JOYBUTTONUP:
            print(f"Joystick {event.joy} button {event.button} up")
        elif event.type == pygame.JOYHATMOTION:
            print(f"Joystick {event.joy} hat {event.hat} motion: {event.value}")
    time.sleep(0.01)
pygame.quit()