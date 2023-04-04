# https://github.com/Vlad12344/Robotic_arm_Object_Detection/blob/master/Utils.py
import numpy as np

# Logitech C270 
imgSize = (1280, 720)
fx = 840.03948975
fy = 837.31976318
intrinsic = np.array([[fx,           0.0,         335.28379881],
                      [  0.0,         fy,  239.29821099],
                      [  0.0,           0.0,           1.0       ]])

# The object is in a plane with z=0
def pixelTo3D(pixel, cameraPosition):
    u, v = pixel
    tx, ty, tz = cameraPosition
    
    K = intrinsic
    
    # Coordenadas normalizadas del punto en el plano de la imagen
    cx = imgSize[0] / 2
    cy = imgSize[1] / 2
    x_norm = np.array([(u - cx) / fx, (v - cy) / fy, 1])

    # Matriz de parámetros extrínsecos de la cámara
    R = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])

    t = np.array([[tx], [ty], [tz]])  # Vector de traslación

    # Combinación de la matriz de proyección y los parámetros extrínsecos
    RT = np.hstack((R, t))
    P = np.dot(K, RT)
    print(P)
    print(t)

    # Punto en coordenadas homogéneas
    x_hom = np.hstack((x_norm, 1))

    # Punto en coordenadas del mundo real
    X_hom = np.dot(np.linalg.inv(P), x_hom)
    X = X_hom[:3] / X_hom[3]

    # Imprimir resultado
    print("El punto ({}, {}) en la imagen corresponde a la posición ({}, {}, {}) en el mundo real.".format(u, v, X[0], X[1], X[2]))

pixelTo3D((0,0), (0,0,1))