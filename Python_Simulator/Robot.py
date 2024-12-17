import re
import time
import config as cf
import vars as vr
import utils as u
import tools as t
import pygame as pg
from math import cos, sin, pi, radians, atan2, asin
from Pathfinding import Pathfinding
from ZoneInterdites import getZonesInterdites

class Robot:
    def __init__(self, coord, radius, angle=0, team='default', target=None):
        self.id, self.team = u.getNewId(), team
        self.radius = radius
        self.coord, self.speed = coord, 0
        self.angle = radians(angle)
        self.eye = t.Vcl(1, self.coord, self.radius * 0.8, [cos(self.angle), sin(self.angle)])

        self.color = 'blue'

        self.vision_right = RayZo(self.eye, 8, self.angle, 25, angle_offset=13)
        self.vision_left = RayZo(self.eye, 8, self.angle, 25, angle_offset=-13)

        self.hitbox = [t.Vadd(self.coord, [-self.radius, -self.radius]),
                       t.Vadd(self.coord, [self.radius, -self.radius]),
                       t.Vadd(self.coord, [self.radius, self.radius]),
                       t.Vadd(self.coord, [-self.radius, self.radius])]

        self.target = target
        self.state = 'INIT'
        self.t_start = time.time()

    def MachineEtat(self):
        # Le cerveaux

        if time.time() - self.t_start >= cf.global_duration:
            self.state = 'END'
            return None

        tirette = False # En réalité on mesure l'etat de la tirette (ici déjà tiré)
        if self.state == 'INIT':
            if tirette is False: self.state = 'STOP'
            else: self.state = 'INIT'
        elif self.state == 'MVT':
            vr.zones = getZonesInterdites(self.getVision(), self.coord, self.angle)
            path, target_reached = Pathfinding(self.coord, self.target, vr.zones) # A* (to make)
            if target_reached:
                self.state = 'STOP'
                self.target = None
            else:
                # path = BezierCurve()
                self.Asservissement(path[0]) # move vers le target selon le path
        elif self.state == 'STOP':
            if self.target is not None:
                self.state = 'MVT'
        elif self.state == 'END':
            pass
        else:
            pass

    def Asservissement(self, target: [int | float, int | float]) -> None:
        # Go to target in straight line

        target_dir = t.normalise(t.Vdiff(target, self.coord))
        rob_dir = [cos(self.angle), sin(self.angle)]

        # Compute Delta Angle : Delta_angle < 0 => Need to Turn RIGHT | Delta_angle > 0 => Need to Turn LEFT
        scalar = t.scalar(target_dir, rob_dir)
        cross = t.cross_product(target_dir, rob_dir)
        if scalar >= 0: delta_angle = asin(cross / (t.norm(target_dir) * t.norm(rob_dir)))
        elif cross <= 0: delta_angle = -pi - asin(t.cross_product(target_dir, rob_dir) / (t.norm(target_dir) * t.norm(rob_dir)))
        else: delta_angle = pi - asin(t.cross_product(target_dir, rob_dir) / (t.norm(target_dir) * t.norm(rob_dir)))

        # Speed
        speed = 2

        self.speed = speed
        self.angle += -delta_angle/4

        self.angle = pi if self.angle < -pi else self.angle
        self.angle = -pi if self.angle > pi else self.angle

        self.coord = u.keepInWindow(t.Vcl(1, self.coord, self.speed, [cos(self.angle), sin(self.angle)]), delta=self.radius)
        self.eye = t.Vcl(1, self.coord, self.radius * 0.8, [cos(self.angle), sin(self.angle)])
        self.hitbox = [t.Vadd(self.coord, [-self.radius, -self.radius]),
                       t.Vadd(self.coord, [self.radius, -self.radius]),
                       t.Vadd(self.coord, [self.radius, self.radius]),
                       t.Vadd(self.coord, [-self.radius, self.radius])]

    def update(self):
        self.target = vr.cursor

        self.MachineEtat()

        self.vision_right.update(self.eye, self.angle, id=self.id)
        self.vision_left.update(self.eye, self.angle, id=self.id)

        self.draw()

    def getHitbox(self):
        return [t.makeSeg(self.hitbox[i % len(self.hitbox)], self.hitbox[(i+1) % len(self.hitbox)]) for i in range(len(self.hitbox))]

    def getVision(self) -> tuple[list[float], list[float]]:
        return self.vision_left.getVision(), self.vision_right.getVision()

    def draw(self):
        pg.draw.circle(vr.window, self.color, self.coord, self.radius)
        pg.draw.circle(vr.window, 'black', self.eye, 5)

def AlgoMovement(robot, algo_controled=False):
    speed = 0
    dangle = 0

    if robot.team != 'T1':
        return speed, dangle

    target = vr.cursor
    dir_target = t.normalise(t.Vdiff(target, robot.coord))
    r_dir = [cos(robot.angle), sin(robot.angle)]

    u.drawSeg(t.makeSeg(robot.coord, target)) #
    u.drawSeg(t.makeSeg(robot.coord, t.Vcl(1, robot.coord, robot.radius * 3, r_dir)), color='green', width=6)

    scalar = t.scalar(dir_target, r_dir)
    cross = t.cross_product(dir_target, r_dir)
    if scalar >= 0:
        delta_angle = asin(cross / (t.norm(dir_target) * t.norm(r_dir)))
    else:
        if cross <= 0:
            delta_angle = -pi - asin(t.cross_product(dir_target, r_dir) / (t.norm(dir_target) * t.norm(r_dir)))
        else:
            delta_angle = pi - asin(t.cross_product(dir_target, r_dir) / (t.norm(dir_target) * t.norm(r_dir)))
    # Delta_angle < 0 => Need to Turn RIGHT | Delta_angle > 0 => Need to Turn LEFT

    Max_Turn_Speed, Move_Speed = min(pi/4, abs(delta_angle)) if algo_controled else pi/4, 2
    def TurnRight(amount=0.1):
        return Max_Turn_Speed * amount
    def TurnLeft(amount=0.1):
        return -Max_Turn_Speed * amount
    def Forward():
        return Move_Speed
    def Backward():
        return -Move_Speed
    def Stop():
        return 0

    if algo_controled:

        vision_right, vision_left = robot.getVision()

    else: # Keyboard Control
        if vr.inputs['RIGHT']:
            dangle = TurnRight()
        elif vr.inputs['LEFT']:
            dangle = TurnLeft()
        if vr.inputs['UP']:
            speed = Forward()
        elif vr.inputs['DOWN']:
            speed = Backward()
        else:
            speed = Stop()

    return speed, dangle

class Ray:
    def __init__(self, source, angle):
        self.id = u.getNewId()

        self.anchor = source
        self.angle = angle

        self.hit_point = self.ray(self.getRayLengthToBorder())

    def ray(self, length):
        return t.Vcl(1, self.anchor, length, [cos(self.angle), sin(self.angle)])

    def getRayLengthToBorder(self):
        Lx, Ly = vr.win_width, vr.win_height
        if abs(self.angle) < pi/2:
            Lx = abs(vr.win_width - self.anchor[0]) / cos(self.angle)
        elif abs(self.angle) > pi/2:
            Lx = abs(self.anchor[0]) / cos(self.angle)
        if 0 < self.angle:
            Ly = abs(vr.win_height - self.anchor[1]) / sin(self.angle)
        elif self.angle < 0:
            Ly = abs(self.anchor[1]) / sin(self.angle)
        return min(abs(Lx), abs(Ly)) - 10

    def update(self, anchor=None, angle=None, ignore_id=None):
        if angle is not None:
            self.anchor = anchor
        if angle is not None:
            self.angle = angle
            da = abs(abs(self.angle) - pi)
            self.angle = pi - da if self.angle < -pi else self.angle
            self.angle = da - pi if self.angle > pi else self.angle

        hit_border_at = self.ray(self.getRayLengthToBorder())
        potential_hit_points = [hit_border_at]
        for g in vr.geo:
            for gseg in g.getSegs():  # (to optimise, repetition)
                touch, point = t.isIntersecting(t.makeSeg(self.anchor, hit_border_at), gseg)
                if touch:
                    potential_hit_points.append(point)
        for r in vr.robots:
            if r.id != ignore_id:
                for rseg in r.getHitbox(): # (to optimise, repetition)
                    touch, point = t.isIntersecting(t.makeSeg(self.anchor, hit_border_at), rseg)
                    if touch:
                        potential_hit_points.append(point)

        self.hit_point = u.getClosestPoint(potential_hit_points, self.anchor)
        self.draw()

    def draw(self):
        pg.draw.line(vr.window, 'yellow', self.anchor, self.hit_point)
        pg.draw.circle(vr.window, 'red', self.hit_point, 5)

class RayZo:
    def __init__(self, source, nb_rays, angle_0, delta_angle, angle_offset=0):

        self.source = source
        self.nb_rays = nb_rays
        self.angle_0, self.delta_angle, self.angle_offset = radians(angle_0), radians(delta_angle), radians(angle_offset)
        self.rays = [Ray(self.source, self.angle_0) for i in range(self.nb_rays)]

    def getVision(self):
        return [round(t.distance(self.source, ray.hit_point), 1) for ray in self.rays]

    def update(self, source=None, angle=None, id=None):
        if source is not None:
            self.source = source
        if angle is not None:
            self.angle_0 = angle + self.angle_offset

        dists = self.getVision()
        for n, ray in enumerate(self.rays):
            ray.update(self.source, self.angle_0 + ((n - (self.nb_rays - 1) / 2) / (self.nb_rays - 1)) * self.delta_angle, ignore_id=id)
            u.Text(str(dists[n]), ray.hit_point, 14, 'white')
