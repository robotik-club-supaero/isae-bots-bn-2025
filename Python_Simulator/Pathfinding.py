from tools import *

# A* to make
def Pathfinding(coordi, target, zones_interdites) -> tuple[list[list[float]], bool]:
    d = distance(coordi, target)
    return [target], True if distance(coordi, target) < 10 else False
