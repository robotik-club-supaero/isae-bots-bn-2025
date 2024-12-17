import vars as vr
import config as cf
from math import cos, sin, radians, degrees, pi
from tools import distance, norm, Vadd

def getZonesInterdites(vision_r_l, coord, orientation):
    vision_l, vision_r = vision_r_l
    vision = {'L7': vision_l[0], 'L6': vision_l[1], 'L5': vision_l[2], 'L4': vision_l[3],
              'L3': vision_l[4], 'L2': vision_l[5], 'L1': vision_l[6], 'L0': vision_l[7],
              'R7': vision_r[7], 'R6': vision_r[6], 'R5': vision_r[5], 'R4': vision_r[4],
              'R3': vision_r[3], 'R2': vision_r[2], 'R1': vision_r[1], 'R0': vision_r[0]}

    ANGLE = [-25.5, -21.9, -18.4, -14.8, -11.2, -7.6, -4.0, -0.5, 0.5, 4.0, 7.6, 11.2, 14.8, 18.4, 21.9, 25.5]

    LEFT = ['L7', 'L6', 'L5', 'L4', 'L3', 'L2', 'L1', 'L0']
    RIGHT = ['R0', 'R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7']

    DangerDist = 300
    pointDangerous = []
    for i, key in enumerate(LEFT + RIGHT):
        d = vision[key]
        if d < DangerDist:
            angle = (orientation + radians(ANGLE[i]))
            da = abs(abs(angle) - pi)
            angle = pi - da if angle < -pi else angle
            angle = da - pi if angle > pi else angle
            pointDangerous.append([cos(angle) * d, sin(angle) * d])

    dpoints = []
    for i in range(len(pointDangerous) - 1):
        dpoints.append(pointDangerous[i])

        dx = pointDangerous[i+1][0] - pointDangerous[i][0]
        dy = pointDangerous[i + 1][1] - pointDangerous[i][1]

        if any([dx > 20, dy > 20]):
            p1 = [pointDangerous[i][0] + dx, pointDangerous[i][1]]
            p2 = [pointDangerous[i][0], pointDangerous[i][1] + dy]

            if distance(coord, p1) <= distance(p2, coord):
                p_intermediaire = p1
            else:
                p_intermediaire = p2
            dpoints.append(p_intermediaire[:])

    pos = []
    for point in dpoints:
        pos.append(coordToPos(Vadd(coord, point)))

    all_pos = []
    for i in range(len(pos)-1):
        bres_ = bresenham(pos[i][0], pos[i][1], pos[i+1][0], pos[i+1][1])
        print(bres_)
        all_pos = all_pos + bres_

    return all_pos


def coordToPos(coord):
    x, y = coord
    return int((x / vr.win_width) * (cf.nx - 1)), int((y / vr.win_height) * (cf.ny - 1))

def PosToCoord(pos):
    px, py = pos
    return (px + 0.5) * cf.cell_x, (py + 0.5) * cf.cell_y

def bresenham(x1, y1, x2, y2):
    pixels = []  # Liste pour stocker les pixels à dessiner
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        pixels.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    return pixels
