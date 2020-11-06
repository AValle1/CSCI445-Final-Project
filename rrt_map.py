from PIL import Image, ImageDraw
import numpy as np
import math


class Map:
    def __init__(self, file_name):
        self.img = Image.open(file_name)
        self.width, self.height = self.img.size
        self.draw = ImageDraw.Draw(self.img)

    def has_obstacle(self, x, y):
        return np.array(self.img.getpixel((x, y))).sum() != 0

    def draw_line(self, pos1, pos2, color, width=2):
        self.draw.line((pos1[0], pos1[1], pos2[0], pos2[1]), fill=(color[0],color[1],color[2],255), width=width)

    def save(self, file_name):
        self.img.save(file_name)
