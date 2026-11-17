import math
from pathlib import Path

import cv2
import numpy as np

from field import ARUCO_SIZE, BLOCKER, BUILDINGS, EDGES, FIELD, ISLAND_CUBES, NODES, POSES, tag_pose
from vision import marker


def coords(values):
    return ' '.join(f'{v:.7g}' for v in values)


def box(name, x, y, w, d, h, color):
    return f'''
Solid {{
  name "{name}"
  translation {coords((x, y, h/2))}
  children [ Shape {{
    appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }}
    geometry DEF {name} Box {{ size {coords((w, d, h))} }}
  }} ]
  boundingObject USE {name}
}}
'''


def texture(name, corners, light=0):
    points = ', '.join(coords(p) for p in corners)
    return f'''
Shape {{
  appearance PBRAppearance {{
    baseColorMap ImageTexture {{ url ["{name}.png"] filtering 0 }}
    emissiveColor {light} {light} {light}
    roughness 1
    metalness 0
  }}
  geometry IndexedFaceSet {{
    coord Coordinate {{ point [ {points} ] }}
    texCoord TextureCoordinate {{ point [ 0 0, 1 0, 1 1, 0 1 ] }}
    coordIndex [ 0 1 2 3 -1 ]
  }}
}}
'''


def paint_floor(directory):
    scale = 260
    w, h = FIELD
    rows, cols = round(h*scale), round(w*scale)
    rng = np.random.default_rng(9)
    greens = np.clip(np.array([44, 89, 46]) + rng.normal(0, 10, (rows, cols, 1)), 0, 255)
    floor = greens.astype(np.uint8)
    def px(p):
        return (round((p[0]+w/2)*scale), round((h/2-p[1])*scale))
    # Paint the central road and both side branches as in the photos.
    lines = [(NODES[a], NODES[b]) for a, b in EDGES]
    lines.extend([((-.12, 1.05), (-.12, 1.72)), ((-.12, -1.40), (-.12, -1.72))])
    for thickness, color in ((.40, (244, 244, 232)), (.37, (46, 44, 44))):
        for a, b in lines:
            thick = .49 if {a, b} == {(1.29, 1.05), (1.29, .77)} else thickness
            cv2.line(floor, px(a), px(b), color, round(thick*scale), cv2.LINE_AA)
    for i, (x, y, yaw) in POSES.items():
        ux, uy = math.cos(yaw), math.sin(yaw)
        cx, cy = x - .21*ux, y - .21*uy
        for k in range(-2, 3):
            a = cx - .067*k*uy, cy + .067*k*ux
            b = a[0]+.085*ux, a[1]+.085*uy
            cv2.line(floor, px(a), px(b), (230, 230, 230), 9, cv2.LINE_AA)
        p = px((x+.09*ux, y+.09*uy))
        cv2.rectangle(floor, (p[0]-16, p[1]-16), (p[0]+16, p[1]+16), (142, 74, 36), -1)
        cv2.putText(floor, 'P', (p[0]-10, p[1]+10), cv2.FONT_HERSHEY_SIMPLEX,
                    .75, (255, 255, 255), 2, cv2.LINE_AA)
    assert cv2.imwrite(str(directory / 'floor.png'), cv2.resize(floor, (1024, 1024)))


def make_world(directory: Path, start, jitter, blocked):
    w, h = (a/2 for a in FIELD)
    paint_floor(directory)
    world = '''#VRML_SIM R2025a utf8
EXTERNPROTO "CourierTurtle.proto"
WorldInfo {
  basicTimeStep 20
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties { material1 "tire" material2 "default" coulombFriction [ 1 ] }
    ContactProperties { material1 "caster" material2 "default" coulombFriction [ .001 ] }
  ]
}
Viewpoint { orientation -.38311045 .38311045 .84050745 1.74367812 position 0 -3.65 4.20 }
Background { skyColor [ .70 .76 .82 ] }
DirectionalLight { direction -.2 .35 -1 intensity 1 castShadows TRUE }
'''
    world += box('floor', 0, 0, 2*w, 2*h, .03, '.19 .35 .16').replace('0 0 0.015', '0 0 -0.015')
    world += texture('floor', ((-w,-h,.001),(w,-h,.001),(w,h,.001),(-w,h,.001)))
    walls = ((-w-.025,0,.05,2*h+.1), (w+.025,0,.05,2*h+.1),
             (0,-h-.025,2*w,.05), (0,h+.025,2*w,.05))
    for i, args in enumerate(walls):
        world += box(f'wall{i}', *args, .23, '.59 .63 .66')
    for i, args in enumerate(BUILDINGS):
        world += box(f'building{i}', *args, .27, '.72 .10 .085' if i < 4 else '.07 .32 .62')
    for i, (x,y,s) in enumerate(ISLAND_CUBES):
        world += box(f'cube{i}', x,y,s,s,s, '.93 .72 .13')
    if blocked:
        world += box('blocking_cube', *BLOCKER, BLOCKER[2], '.97 .72 .08')
    for i in POSES:
        side = 256
        sign = np.full((side, side), 255, np.uint8)
        sign[56:-56, 56:-56] = marker(i)
        assert cv2.imwrite(str(directory / f'tag{i}.png'), sign)
        x, y, yaw = tag_pose(i)
        x -= .003*math.cos(yaw)
        y -= .003*math.sin(yaw)
        # The plane's front points down the bay toward the approaching camera.
        rx, ry = math.sin(yaw), -math.cos(yaw)
        a = ARUCO_SIZE*128/144
        z = .145
        corners = [(x+s*rx, y+s*ry, z+b) for s,b in
                   ((-a,-a),(a,-a),(a,a),(-a,a))]
        world += texture(f'tag{i}', corners)
    x,y,yaw = POSES[start]
    world += f'CourierTurtle {{ translation {coords((x,y,.001))} rotation 0 0 1 {yaw+jitter:.7g} }}\n'
    path = directory / 'city.wbt'
    path.write_text(world)
    return path
