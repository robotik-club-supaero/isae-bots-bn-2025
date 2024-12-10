import config as cf
import vars as vr
import utils as u
import tools as t
import pygame as pg
from math import cos, sin, radians

class Rectangle:
    def __init__(self, topleft, width, height, angle=0):
        self.id = u.getNewId()

        self.topleft = topleft
        self.angle = radians(angle)
        self.w, self.h = width, height

        self.p1 = self.topleft
        self.p2 = t.Vadd(self.topleft, [self.w * cos(self.angle), self.w * sin(self.angle)])
        self.p3 = t.Vadd(self.p2, [-self.h * sin(self.angle), self.h * cos(self.angle)])
        self.p4 = t.Vadd(self.topleft, [-self.h * sin(self.angle), self.h * cos(self.angle)])
        self.points = [self.p1, self.p2, self.p3, self.p4]

    def update(self):
        self.draw()

    def getSegs(self):
        return [t.makeSeg(self.points[i % len(self.points)], self.points[(i + 1) % len(self.points)]) for i in range(len(self.points))]

    def draw(self):
        pg.draw.polygon(vr.window, (150, 150, 150), self.points)
        pg.draw.polygon(vr.window, (100, 100, 100), self.points, width=3)

class Shape:
    def __init__(self, points):
        self.id = u.getNewId()

        self.points = points

    def update(self):
        self.draw()

    def getSegs(self):
        return [t.makeSeg(self.points[i % len(self.points)], self.points[(i + 1) % len(self.points)]) for i in range(len(self.points))]

    def draw(self):
        pg.draw.polygon(vr.window, (150, 150, 150), self.points)
        pg.draw.polygon(vr.window, (100, 100, 100), self.points, width=3)
