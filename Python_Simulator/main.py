import pygame as pg
from math import cos, sin, pi, tan
import tools as t
import utils as u
import vars as vr
import config as cf
import time
from Robot import Robot
from Geometry import Rectangle, Shape
from random import randint
from ZoneInterdites import PosToCoord, coordToPos

def init():

    pg.init()
    pg.display.set_caption(cf.game_name)

    # screen initialisation
    if not cf.fullscreen:
        vr.window = pg.display.set_mode(vr.window_size)
    else:
        vr.window = pg.display.set_mode((0, 0), pg.FULLSCREEN)
        vr.window_size = vr.window.get_size()

    vr.clock = pg.time.Clock()

    vr.robot = Robot([30, 150], 25, team='T1')
    vr.robots.append(vr.robot)
    #vr.robots.append(Robot(vr.middle, 25, angle=-180, team='T2'))

    vr.geo.append(Shape([(260, 0), (940, 0), (940, 3), (940, 80), (780, 80), (780, 180), (420, 180), (420, 80), (260, 80)]))

    return

def main():
    init()

    vr.running = True

    frames_fps, t_fps = 0, time.time() - 1

    while vr.running:

        vr.clock.tick(cf.fps)

        frames_fps += 1
        vr.fps = frames_fps/(time.time() - t_fps)
        vr.dt = 1 / vr.fps if vr.fps != 0 else 0.1
        if frames_fps > 1000:
            frames_fps, t_fps = 0, time.time()

        for event in pg.event.get():
            if event.type == pg.QUIT:
                vr.running = False
            elif event.type == pg.MOUSEBUTTONDOWN:
                print("Cursor : ", pg.mouse.get_pos())

        # Main Loop #
        pre_update()
        if vr.fps > cf.fps * cf.fps_treshold:
            u.getInputs()
            update()
        post_update()
        # --------- #

    return

def update():
    vr.cursor = pg.mouse.get_pos()

    for g in vr.geo:
        g.update()

    for r in vr.robots:
        r.update()

    return

def pre_update():
    vr.window.fill('black')

    vr.window.blit(vr.back_grids[1], [0, 0])

    u.Text("fps : " + str(round(vr.fps, 1)), (10, vr.win_height - 24), 14, 'orange')
    u.Text("info : " + str(vr.info_txt), (10, vr.win_height - 48), 14, 'orange')
    return

def post_update():
    for i in range(len(vr.zones) - 1):
        topleft = PosToCoord(vr.zones[i])
        pg.draw.rect(vr.window, 'orange', [topleft, [cf.cell_x, cf.cell_y]], 5)

    pg.display.update()
    return

if __name__ == "__main__":
    main()