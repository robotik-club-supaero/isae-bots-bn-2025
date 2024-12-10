import vars as vr
import config as cf
import pygame as pg
from tools import *
from random import randint

def Text(msg, coord, size, color):  # blit to the screen a text
    TextColor = pg.Color(color) # set the color of the text
    font = pg.font.Font("./pixel.ttf", size)  # set the font
    return vr.window.blit(font.render(msg, True, TextColor), coord)  # return and blit the text on the screen

def getInputs():
    keys = pg.key.get_pressed()
    vr.inputs["SPACE"] = True if keys[pg.K_SPACE] else False
    vr.inputs["R"] = True if keys[pg.K_r] else False
    vr.inputs["G"] = True if keys[pg.K_g] else False
    vr.inputs["B"] = True if keys[pg.K_b] else False
    vr.inputs["N"] = True if keys[pg.K_n] else False

    vr.inputs["UP"] = True if keys[pg.K_UP] else False
    vr.inputs["DOWN"] = True if keys[pg.K_DOWN] else False
    vr.inputs["RIGHT"] = True if keys[pg.K_RIGHT] else False
    vr.inputs["LEFT"] = True if keys[pg.K_LEFT] else False

def isInWindow(coord):
    if 0 <= coord[0] <= vr.win_width:
        if 0 <= coord[1] <= vr.win_height:
            return True
    return False

def keepInWindow(coord, delta=0):
    x, y = coord
    return [max(delta, min(vr.win_width - delta, x)), max(delta, min(vr.win_height - delta, y))]

def makeSeg(a, b):
    return lambda t: (b[0] + (t - 1) * (b[0] - a[0]), b[1] + (t - 1) * (b[1] - a[1]))

def cross_product(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]

def drawSeg(seg, color=(20, 20, 100), width=4):
    pg.draw.line(vr.window, color, seg(0), seg(1), width)

def getNewId():
    vr.id += 1
    return vr.id

def getRndCoord(delta=100):
    return [randint(delta, vr.win_width - delta), randint(delta, vr.win_height - delta)]

def getClosestPoint(points, a):
    n_min = None
    point = None
    for p in points:
        new_norm = distance(a, p)
        if n_min is None or new_norm < n_min:
            n_min = new_norm
            point = p
    return point
