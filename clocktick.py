import pygame
import time

pygame.init()

gameDisplay = pygame.display.set_mode((800,600))
clock = pygame.time.Clock()

crashed = False


counter = 1
start_time = time.perf_counter()
while not crashed:
    current_time = time.perf_counter()
    elapsed_time = current_time - start_time
    print(elapsed_time)
    for event in pygame.event.get():
        if(elapsed_time > 10.0):
            print("CRASHED")
            crashed = True
            pygame.quit()
    
    pygame.display.update()
    print(counter)
    counter += 1
    clock.tick(10) # will be 10 in the next run 