# Global variables used by the game
import config as cf
import pygame as pg

window_size = cf.window_default_size
win_width, win_height = window_size[0], window_size[1]
middle = (win_width // 2, win_height // 2)

window = None
clock = None

running = False

# In game
inputs = {}
fps = cf.fps
dt = 1/fps

cursor = (0, 0)
info_txt = ""
id = 0

robot = None
robots = []
geo = []

back_grids = [pg.transform.scale(pg.image.load("./rsc/Grid.png"), cf.window_default_size),
              pg.transform.scale(pg.image.load("./rsc/Grid_jolie.png"), cf.window_default_size)]
