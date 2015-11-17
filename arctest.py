import pygame
from random import choice, random
from math import pi, cos, sin, atan2

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
pygame.init()
size = (1024, 768)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("PID test")
done = False
screen.fill(BLACK)

ball = 512, 384#choice(range(512)), choice(range(768))# 
goal = 1024, 384

def angle_to(point_1, point_2):
    return atan2(point_1[1] - point_2[1],
                 point_1[0] - point_2[0])

class Agent():
    def __init__(self, loc, theta, speed, rotation):
        self.loc = loc
        self.initial_theta = self.theta = theta
        self.speed = speed
        self.rotation = rotation
        self.path = [tuple(self.loc)]

    def __str__(self):
        return "rotation:{}\ninitial theta:{}\nfinal theta:{}\nfinal location:{}\n".format(self.rotation, self.initial_theta, self.theta, self.loc)

    def rotate_to(self, angle):
        self.theta = angle

    def rotate_by(self, angle):
        self.theta += angle

    def advance(self):
        self.theta = (-angle_to(goal, ball) + 2*angle_to(self.loc, ball))
        self.loc = (self.loc[0] + self.speed * cos(self.theta),
                    self.loc[1] + self.speed * sin(self.theta))
        self.path.append(tuple(self.loc))

    def draw(self, screen, color):
        pygame.draw.lines(screen, color, False, self.path)
        pygame.draw.line(screen, BLUE, self.path[0], ball)

start_loc = (choice(range(512)), choice(range(768)))
agents = [Agent((choice(range(512)), choice(range(768))), random() * 2 * pi, 20, random()*0.5-0.25) for n in range(100)]


while not done:
    # --- Main event loop
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    
    pygame.draw.line(screen, GREEN, ball, (1024, 384))
    for agent in agents:
        matches = []
        agent.advance()
        if (abs(sin(agent.theta)) < 0.1 and
            abs(agent.loc[0] - 512) < 20 and
            abs(agent.loc[1] - 384) < 20):
            matches.append(agent)
        else:
            agent.draw(screen, WHITE)
    for agent in matches:
        agent.draw(screen, RED)
    pygame.display.flip()
pygame.quit()
