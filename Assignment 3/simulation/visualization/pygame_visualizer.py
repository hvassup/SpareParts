import sys

import pygame

offset = 400
scale = 400


def scale_point(x, y):
    return offset + (x) * scale, offset + (y) * scale


def scale_size(w, h):
    return w * scale, h * scale


class PyGameVisualizer:
    def __init__(self):
        self.a = 0
        pygame.init()
        pygame.display.init()
        self.surface = pygame.display.set_mode((900, 700))

    def clear(self):
        self.surface.fill((255, 255, 255))

    def show(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN and event.key == 32:  # Press space
                print('Hello')
        pygame.display.update()

    def draw_text(self, msg, point):
        font1 = pygame.font.SysFont('chalkduster.ttf', 16)
        img = font1.render(msg, True, (0, 0, 0))
        self.surface.blit(img, point)

    def draw_point(self, x, y, col=(0, 0, 0), size=0.01):
        pygame.draw.circle(self.surface, col, scale_point(x, y), size * scale)

    def draw_points(self, points, col, originX=0, originY=0, size=0.01):
        for x, y in points:
            self.draw_point(originX + x, originY + y, col, size)

    def draw_rectangle(self, pos, size, color):
        rect = scale_point(*pos) + scale_size(*size)
        pygame.draw.rect(self.surface, color, rect)

    def draw_rectangles(self, rectangles, col=(0, 0, 0)):
        for pos, size in rectangles:
            self.draw_rectangle(pos, size, col)

    def draw_line(self, x1, y1, x2, y2, col):
        pygame.draw.line(self.surface, col, scale_point(x1, y1), scale_point(x2, y2))
