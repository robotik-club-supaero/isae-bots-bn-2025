# Some useful functions

def isIntersecting(seg1, seg2):
    p, r = seg1(0), Vdiff(seg1(1), seg1(0))
    q, s = seg2(0), Vdiff(seg2(1), seg2(0))

    rxs = cross_product(r, s)

    if rxs != 0.: # May intersect
        t = cross_product(Vdiff(q, p), s) / rxs
        u = cross_product(Vdiff(q, p), r) / rxs
        if 0 <= t <= 1 and 0 <= u <= 1: # Verify validity
            return True, seg1(t)
        return False, None
    elif cross_product(Vdiff(q, p), r) == 0.: # Collinear
        dt = scalar(s, r) / scalar(r, r)
        if scalar(s, r) / scalar(r, r) <= 1:
            return True, seg2(1/2) if norm(Vdiff(seg1(1), seg1(0))) > norm(Vdiff(seg2(1), seg2(0))) else seg1(1/2)
    else: # Not intersecting
        return False, None

def s(x):
    return 1 if x >= 0 else -1

def inv(x, explode=1e-9):
    return 1/x if x != 0. else explode

def Vmult(v, coef):
    return [vi * coef for vi in v]

def Vadd(v1, v2):
    return [v1[i] + v2[i] for i in range(min(len(v1), len(v2)))]

def Vdiff(v1, v2):
    return [v1[i] - v2[i] for i in range(min(len(v1), len(v2)))]

def Vcl(l1, v1, l2, v2):
    return [l1 * v1[i] + l2 * v2[i] for i in range(min(len(v1), len(v2)))]

def VectsSum(vectors):
    return [sum([vectors[i][u] for i in range(len(vectors))]) for u in range(min([len(v) for v in vectors]))]

def norm(v):
    return sum([vi ** 2 for vi in v]) ** 0.5

def distance(a, b):
    return norm(Vdiff(b, a))

def scalar(v1, v2, rounded=False):
    sc = sum([v1[i] * v2[i] for i in range(min(len(v1), len(v2)))])
    return sc if rounded is False else round(sc)

def vectorial(v1, v2):
    return [v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]]

def normalise(v):
    return Vmult(v, inv(norm(v)))

def moy(vector):
    return sum(vector) / len(vector) if len(vector) != 0 else 0

def ControlValue(value, a, b):
    if value < a:
        return a
    elif value > b:
        return b
    else:
        return value

def makeSeg(a, b):
    return lambda t: (b[0] + (t - 1) * (b[0] - a[0]), b[1] + (t - 1) * (b[1] - a[1]))

def cross_product(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]
