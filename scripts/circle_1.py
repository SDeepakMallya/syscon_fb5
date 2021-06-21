#!/usr/bin/env python3

import pandas as pd
import numpy as np
import math

def linear_approx(time, pos_x, pos_y, heading):
    #returns velocities
    t = np.array(time)
    x_path = np.array(pos_x)
    y_path = np.array(pos_y)
    thetas = np.array(heading)
    
    dt = np.ediff1d(t)
    dx = np.ediff1d(x_path)
    dy = np.ediff1d(y_path)
    dtheta = np.ediff1d(thetas)
    ds = np.sqrt(dx * dx + dy * dy)
    v = np.average(ds/dt)
    w = np.average(d_theta/dt)

    return v, w


def circum_center(pos_x, pos_y, num_iterations = 50, eps = 1e-04):
    #returns center and radius
    n = len(pos_x)
    cen_x = []
    cen_y = []
    for _ in range(num_iterations):
        id1 = np.random.choice(range(n//3), 1)
        id2 = np.random.choice(range(n//3, 2 * n//3), 1)
        id3 = np.random.choice(range(2 * n//3, n), 1)
        x_a, y_a = (pos_x[id1] + pos_x[id2])/2, (pos_y[id1] + pos_y[id2])/2
        x_b, y_b = (pos_x[id2] + pos_x[id3])/2, (pos_y[id2] + pos_y[id3])/2
        
        if abs(pos_y[id1] -  pos_y[id2]) <= eps:
            cen_x.append(x_a)
            m2 = (pos_x[id3] - pos_x[id2])/(pos_y[id2] - pos_y[id3])
            cen_y.append(m2 * x_a + y_b - m2 * x_b)
        
        elif abs(pos_y[id2] -  pos_y[id3]) <= eps:
            cen_x.append(x_b)
            m1 = (pos_x[id1] - pos_x[id2])/(pos_y[id2] - pos_y[id1])
            cen_y.append(m1 * x_b + y_a - m1 * x_a)

        else:
            m1 = (pos_x[id1] - pos_x[id2])/(pos_y[id2] - pos_y[id1]) 
            m2 = (pos_x[id3] - pos_x[id2])/(pos_y[id2] - pos_y[id3]) 
            cen_x.append((y_a - y_b - m1  * x_a + m2 * x_b) / (m2 - m1))
            cen_y.append(m1 * cen_x[-1] + y_a - m1 * x_a)


    x = float(sum(cen_x)/num_iterations)
    y = float(sum(cen_y)/num_iterations) 
    pos_x = np.array(pos_x)
    pos_y = np.array(pos_y)
    r = np.average(np.sqrt((pos_x - x) ** 2 + (pos_y - y) ** 2))

    return x, y, r

def get_velocities(cen, rad, pos_1, pos_2, duration):

    ang1 = math.atan2(pos_1[1] - cen[1], pos_1[0] - cen[0]) % (2 * math.pi)
    ang2 = math.atan2(pos_2[1] - cen[1], pos_2[0] - cen[0]) % (2 * math.pi)

    w = (ang2 - ang1) / duration
    v = rad * w

    return v, w


def jacobian(xs, ys, beta, batch_size = 10):

    jacob = -np.ones((batch_size, 3))
    den = np.sqrt((xs - beta[0]) ** 2 + (ys - beta[1]) ** 2)
    jacob[:, 0] = (beta[0] - xs)/den
    jacob[:, 1] = (beta[1] - ys)/den
    
    return jacob



def gauss_newton(pos_x, pos_y, eps = 1e-04, batch_size = 10):

    # t = np.array(time)
    x_path = np.array(pos_x)
    y_path = np.array(pos_y)
    # thetas = np.array(heading)
    n = len(pos_x)

    beta = circum_center(x_path, y_path)
    err = np.inf

    while err > eps:
        ids = np.random.choice(range(n), batch_size)
        xs = x_path[ids]
        ys = y_path[ids]

        jacob = jacobian(xs, ys, beta)
        batch_err = np.sqrt((xs - beta[0]) ** 2 + (ys - beta[1]) ** 2) - beta[2]
        update = np.matmul(np.linalg.pinv(jacob), batch_err)

        err = max(abs(update))
        beta -= update
        print (beta, err)
        
    return beta


def circle_test_gen(cen, rad, num_points, err = 1e-03):

    thetas = np.random.random(num_points) * 2 * math.pi
    pos_x = rad * np.cos(thetas) + cen[0] + np.random.random(num_points) * err
    pos_y = rad * np.sin(thetas) + cen[1] + np.random.random(num_points) * err

    return pos_x, pos_y




