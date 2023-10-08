import numpy as np
from numpy import sin, cos, pi
import cv2

# Rotation matrix by x-axis
def rxmat(a):
    return np.array([
        [1,0,0],
        [0,cos(a),-sin(a)],
        [0,sin(a),cos(a)]
    ])

# Rotation matrix by y-axis
def rymat(a):
    return np.array([
        [cos(a),0,sin(a)],
        [0,1,0],
        [-sin(a),0,cos(a)]
    ])

# Rotation matrix by z-axis
def rzmat(a):
    return np.array([
        [cos(a),-sin(a),0],
        [sin(a),cos(a),0],
        [0,0,1]
    ])

# Projection 3D point (object) to 2D point (screen)
def project(x, y, z):
    fx, fy = 100, 100
    cx = W/2
    cy = H/2
    I = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    p = np.array([x, y, z])
    out = I @ p
    out = out / out[2]
    return out[0], out[1]

# Screen width, height
W, H = 1000,1000

# Rotation angle of whole donut
phix = 0.0
phiz = 0.0

while True:
    # Create a black frame
    frame = np.zeros((H, W, 3), np.uint8)

    R1 = 60  # cross section radius
    R2 = 200 # donut radius
    lighting_direction = np.array([0, 1, -5])
    lighting_direction = lighting_direction / np.linalg.norm(lighting_direction)
    for py in np.linspace(0, 2*pi, 60):
        cen = np.array([R2*cos(py), 0, R2*sin(py)])
        for pz in np.linspace(0, 2*pi, 20):
            point = cen + np.array([R1*cos(pz), R1*sin(pz), 0]) @ rymat(py)
            # Rotation
            point = point @ rxmat(phix) @ rzmat(phiz)
            # Position
            point = point + np.array([0,0,300])
            # Iluminance
            N = np.array([[cos(pz), sin(pz), 0]]) @ rymat(py) @ rxmat(phix) @ rzmat(phiz) 
            N = N / np.linalg.norm(lighting_direction)
            L = np.dot(N[0], lighting_direction)
            if L > 0:
                # Project and draw to screen
                x, y = project(*point)
                cv2.circle(frame, (int(x), int(y)), int(L * 1.5), (255,255,255), -1)

    # Show frame
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Change rotation
    phix += 0.05
    phiz += 0.05