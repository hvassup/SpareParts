import sys
import pygame

from simulation.visualization.visualizer import Visualizer

offset = 300
scale = 200
def scale_point(x, y):
    return offset + (x) * scale, offset + (y) * scale

def scale_size(w, h):
    return w * scale, h * scale

class PyGameVisualizer(Visualizer):
    def __init__(self):
        self.a = 0
        pygame.init()
        pygame.display.init()
        self.surface = pygame.display.set_mode((900, 600))

    def draw_text(self, msg, point):
        font1 = pygame.font.SysFont('chalkduster.ttf', 16)
        img = font1.render(msg, True, (255, 255, 255))
        self.surface.blit(img, point)
        
    def draw_point(self, x, y, col=(255, 255, 255), size=2):
        pygame.draw.circle(self.surface, col, scale_point(x, y), size)

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
    
    def draw_rect(self, color, pos, size):
        rect = scale_point(*pos) + scale_size(*size)
        pygame.draw.rect(self.surface, color, rect)
    
    def visualize_danger_spots(self, spots):
        for pos, size in spots:
            self.draw_rect((255, 0, 0), pos, size)

    def visualize_safe_zone(self, pos, size):
        self.draw_rect((0, 255, 0), pos, size)
    
    def draw_line_to_point(self, x1, y1, x2, y2):
        pygame.draw.line(self.surface, (255, 0, 255), scale_point(x1, y1), scale_point(x2, y2))
    
    def draw_points(self, points, col, originX=0, originY=0, size=2):
        for x, y in points:
            self.draw_point(originX + x, originY + y, col, size)
        
    def visualize_world(self, world, height, width):
        self.draw_rect((0, 0, 255), (-width/2, -height/2), (width, height))
