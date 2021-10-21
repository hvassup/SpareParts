import sys
import pygame

from simulation.visualization.visualizer import Visualizer

offset = 400
scale = 4
def scale_point(x, y):
    return offset + (x) * scale, offset + (y) * scale

def scale_size(w, h):
    return w * scale, h * scale

class PyGameVisualizer(Visualizer):
    def __init__(self):
        self.a = 0
        pygame.init()
        pygame.display.init()
        self.surface = pygame.display.set_mode((900, 700))

    def clear(self):
        self.surface.fill((0, 0, 0))
    
    def show(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN and event.key == 32: # Press space
                print('Hello')
        pygame.display.update()

    def draw_text(self, msg, point):
        font1 = pygame.font.SysFont('chalkduster.ttf', 16)
        img = font1.render(msg, True, (255, 255, 255))
        self.surface.blit(img, point)
        
    def draw_point(self, x, y, col=(255, 255, 255), size=2):
        pygame.draw.circle(self.surface, col, scale_point(x, y), size)
    
    def draw_points(self, points, col, originX=0, originY=0, size=2):
        for x, y in points:
            self.draw_point(originX + x, originY + y, col, size)
    
    def draw_rectangle(self, color, pos, size):
        rect = scale_point(*pos) + scale_size(*size)
        pygame.draw.rect(self.surface, color, rect)
    
    def draw_rectangles(self, rectangles, col=(255, 255, 255)):
        for pos, size in rectangles:
            self.draw_rectangle(col, pos, size)
    
    def draw_line(self, x1, y1, x2, y2, col):
        pygame.draw.line(self.surface, col, scale_point(x1, y1), scale_point(x2, y2))