import numpy as np

def rad2deg(rad):
    return rad * 180.0/np.pi

def deg2rad(deg):
    return deg * np.pi / 180.0


def Rot2Euler(Rot):
    return np.array([ np.arctan2(Rot[2][1],Rot[2][2]),
                      np.arctan2(-Rot[2][0],np.sqrt(Rot[2][1]**2 + Rot[2][2]**2)),
                      np.arctan2(Rot[1][0],Rot[0][0])
                    ])

def Rot_x(a):
    return np.array([   [1,   0     ,    0     ],
                        [0,np.cos(a),-np.sin(a)],
                        [0,np.sin(a), np.cos(a)]
                     ], dtype=float
                     )

def Rot_y(b):
    return np.array([   [np.cos(b)  ,   0     ,    np.sin(b)     ],
                        [   0       ,   1     ,       0          ],
                        [-np.sin(b) ,   0     ,    np.cos(b)     ]
                     ], dtype=float
                     )

def Rot_z(g):
    return np.array([   [np.cos(g)  ,    -np.sin(g)     ,    0    ],
                        [np.sin(g)  ,     np.cos(g)     ,    0    ],
                        [    0      ,       0           ,    0    ]
                     ], dtype=float
                     )

def Rot_zyx(*args):
    """
        Rotation matrix
            (a)lpha: float, (x-axis)
            (b)beta: float, (y-axis)
            (g)amma: float, (z-axis)
    """
    if len(args) == 1:
        a,b,g = args[0]
    else:
        a = args[0]
        b = args[1]
        g = args[2]

    ca = np.cos(a)
    sa = np.sin(a)

    cb = np.cos(b)
    sb = np.sin(b)

    cg = np.cos(g)
    sg = np.sin(g)

    return np.array(
                    [
                        [cb * cg,   cg * sa * sb - ca * sg    ,    ca * cg * sb + sa * sg     ],
                        [cb * sg,   ca * cg      + sa *sb * sg,   -cg * sa      + ca * sb * sg],
                        [-sb    ,   cb * sa                   ,    ca * cb                    ]
                    ], dtype=float
                    )
