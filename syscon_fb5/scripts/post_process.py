#!usr/bin/env python3

import circle_1 as circ
import glob, os
import math
import numpy as np
import matplotlib.pyplot as plt
import rospkg
dirname = rospkg.RosPack().get_path('syscon_fb5')

def is_stationary(x_path, y_path, tol = 0.05):
    x_path = np.array(x_path.copy())
    y_path = np.array(y_path.copy())

    dx = np.ediff1d(x_path)
    dy = np.ediff1d(y_path)
    ds = np.sqrt(dx * dx + dy * dy)

    return (sum(ds) < tol)

def wheel_velocities(vel, ang_vel, wheel_base=0.18):
    right_vel = vel + wheel_base*ang_vel/2
    left_vel = 2*vel - right_vel
    return left_vel, right_vel

def read_file(filename):
    """
    Read file and generate path trajectory
    Expects file to be in the current folder.
    """
    time = []
    x_pos = []
    y_pos = []
    heading = []

    with open(filename, 'r') as f:
        row = f.readline()
        while row != '':
            content = row.split(', ')
            time.append(float(content[0]))
            x_pos.append(float(content[1]))
            y_pos.append(float(content[2]))
            heading.append(float(content[3]))

            row = f.readline()
    return time, x_pos, y_pos, heading

def process_data(dir_name):
    """
    Generates data for dhinkachika
    """
    # os.chdir(dir_path)
    pwm_left_forward = []
    pwm_left_backward = []
    pwm_right_forward = []
    pwm_right_backward = []
    low_forward_pwm = 0
    low_backward_pwm = 0
    i = 0 
    for filename in glob.glob(dir_name + '/calibration_files/pos_1[2-6][0-9]*.csv'):
        print("Processing " + filename)
        i += 1
        print(i)
        name = filename.split('/')[-1].split('.')[0].split('_')
        # name_split = filename.split('_')
        pwm = int(name[1])
        motion = name[2]
        time, x_pos, y_pos, heading = read_file(filename)
        duration = time[-1] - time[0]
        init_pos = (x_pos[0], y_pos[0])
        fin_pos = (x_pos[-1], y_pos[-1])
        mid_pos = (x_pos[len(x_pos)//2], y_pos[len(y_pos)//2])

        if not is_stationary(x_pos, y_pos):
            cen_x, cen_y, rad = circ.gauss_newton(x_pos, y_pos)
            cen = (cen_x, cen_y)
            vel, ang_vel = circ.get_velocities(cen, rad, init_pos, fin_pos, mid_pos, duration, motion)
            left_vel, right_vel = wheel_velocities(vel, ang_vel)

            if motion == 'forward':
                pwm_left_forward.append([pwm, left_vel])
                pwm_right_forward.append([pwm, right_vel])

            elif motion == 'backward':
                pwm_left_backward.append([-pwm, left_vel])
                pwm_right_backward.append([-pwm, right_vel])
        else:
            if motion == 'forward' and pwm > low_forward_pwm:
                low_forward_pwm = pwm
            if motion == 'backward' and pwm > low_backward_pwm:
                low_backward_pwm = pwm

    return low_forward_pwm, low_backward_pwm, pwm_left_forward, pwm_right_forward, pwm_left_backward, pwm_right_backward


def regress(data):

    X = np.array([[c[1], c[1] ** 2, 1] for c in data])
    Y = np.array([c[0] for c in data])
    W = np.matmul(np.linalg.pinv(X), Y)
    plt.scatter(X[:,0], Y)
    thetas = np.linspace(min(X[:,0]), max(X[:,0]), 100)
    X_pred = np.array([[c, c ** 2, 1] for c in thetas])
    Y_pred = np.matmul(X_pred, W)
    plt.plot(thetas, Y_pred, 'r')
    # plt.show()
    return W

if __name__ == '__main__':

    # filename = 'pos_140_forward_2.csv'
    # time, x_pos, y_pos, heading = read_file(dir_path + filename)

    # cen_x, cen_y, rad = circ.circum_center(x_pos, y_pos, 100)
    # print("Circle info: ", cen_x, cen_y, rad)

    # thetas = np.linspace(0, 2*math.pi, 100)
    # point_x = cen_x + rad*np.cos(thetas)
    # point_y = cen_y + rad*np.sin(thetas)


    # duration = time[-1] - time[0]
    # init_pos = (x_pos[0], y_pos[0])
    # fin_pos = (x_pos[-1], y_pos[-1])
    # cen = (cen_x, cen_y)
    # vel, ang_vel = circ.get_velocities(cen, rad, init_pos, fin_pos, duration)
    # print('Circum-center', vel, ang_vel, duration)
    # vel, ang_vel = circ.linear_approx(time, x_pos, y_pos, heading)
    # print(vel, ang_vel)
    # left_vel, right_vel = wheel_velocities(vel, ang_vel)


    # # cen_x, cen_y, rad = circ.gauss_newton(x_pos, y_pos)
    # # print("Circle info: ", cen_x, cen_y, rad)

    # thetas = np.linspace(0, 2*math.pi, 100)
    # point_x1 = cen_x + rad*np.cos(thetas)
    # point_y1 = cen_y + rad*np.sin(thetas)



    # pwm_forward, pwm_backward = process_data(dir_path)
    # print(pwm_forward[150], pwm_backward[150])



    # plt.scatter(x_pos, y_pos, color='g')
    # plt.plot(point_x, point_y, color='b')
    # plt.plot(point_x1, point_y1, color='r')
    # plt.show()

    # plt.plot(time)
    # plt.show()

    processed_data = process_data(dirname)
    # print(processed_data[0])
    labels = ['low_forward_pwm', 'low_backward_pwm', 'pwm_left_forward', 'pwm_right_forward', 'pwm_left_backward', 'pwm_right_backward']
    with open(dirname + '/calibration_bot1.yaml', 'w') as f:
        f.write("{}:{}\n".format(labels[0],processed_data[0]))
        f.write("{}:{}\n".format(labels[1],processed_data[1]))            


        for i in range(2,len(labels)):
            weights = regress(processed_data[i])
            f.write("{}:{}\n".format(labels[i]," ".join(list(map(str,weights)))))

    



