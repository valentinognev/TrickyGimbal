import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def gimbal_trajectory():
    gamma = np.deg2rad(20)   # [deg]

    elevation1 = np.deg2rad(-20)  # [rad] elevation angle with Z - phi
    azimuth1 = np.deg2rad(40)  # [rad] azimuth angle in XY plane - curlphi
    elevation2 = np.deg2rad(30)  # [rad] elevation angle with Z - phi
    azimuth2 = np.deg2rad(70)  # [rad] azimuth angle in XY plane - curlphi

    rot_elev1 = R.from_euler('y', np.rad2deg(elevation1), degrees=True).as_matrix()
    rot_az1 = R.from_euler('z', np.rad2deg(azimuth1), degrees=True).as_matrix()

    rot_elev2 = R.from_euler('y', np.rad2deg(elevation2), degrees=True).as_matrix()
    rot_az2 = R.from_euler('z', np.rad2deg(azimuth2), degrees=True).as_matrix()

    rot1 = rot_az1 @ rot_elev1
    rot2 = rot_az2 @ rot_elev2

    rinit = np.array([0, 0, 1])
    r1 = rot1 @ rinit
    r2 = rot2 @ rinit

    r1r2axis = np.cross(r1, r2)
    r1r2axis /= np.linalg.norm(r1r2axis)
    theta = np.arccos(np.dot(r1, r2))
    N = 10
    dtheta = theta / N

    rotdtheta = R.from_rotvec(r1r2axis * dtheta).as_matrix()

    curr = r1
    elevation = []
    azimuth = []
    alpha1 = []
    alpha2 = []

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([0, r1[0]], [0, r1[1]], [0, r1[2]], 'rx-')
    ax.plot([0, r2[0]], [0, r2[1]], [0, r2[2]], 'mx-')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.grid(True)

    for i in range(N):
        elevation_i = np.arccos(np.dot(curr, np.array([0, 0, 1])))
        azimuth_i = np.arctan2(curr[1], curr[0])
        alpha2_i = 2 * np.arcsin(np.sin(elevation_i / 2) / np.sin(gamma))
        eta = np.arccos((np.sin(alpha2_i / 2) - np.sin(gamma) * np.sin(elevation_i / 2)) / (np.cos(elevation_i / 2) * np.cos(gamma)))
        alpha1_i = np.pi - eta - azimuth_i

        curr_2 = np.array([
            (-np.cos(alpha1_i)) * np.cos(gamma) * np.sin(gamma) - ((-np.cos(alpha1_i)) * np.cos(alpha2_i) * np.cos(gamma) - np.sin(alpha1_i) * np.sin(alpha2_i)) * np.sin(gamma),
            np.cos(gamma) * np.sin(alpha1_i) * np.sin(gamma) - (np.cos(alpha2_i) * np.cos(gamma) * np.sin(alpha1_i) - np.cos(alpha1_i) * np.sin(alpha2_i)) * np.sin(gamma),
            np.cos(gamma)**2 + np.cos(alpha2_i) * np.sin(gamma)**2
        ])

        ax.plot([0, curr[0]], [0, curr[1]], [0, curr[2]], 'b-o')
        plot_gimbal(alpha1_i, alpha2_i, gamma, ax)

        elevation.append(elevation_i)
        azimuth.append(azimuth_i)
        alpha1.append(alpha1_i)
        alpha2.append(alpha2_i)

        print(f'elevation {elevation_i:.6f} , azimuth {azimuth_i:.6f} , alpha1 {alpha1_i:.6f} , alpha2 {alpha2_i:.6f}')
        curr = rotdtheta @ curr
    #ax.axis('equal')
    # Create cubic bounding box to simulate equal aspect ratio
    max_range = 2
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() 
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() 
    Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() 
    # Comment or uncomment following both lines to test the fake bounding box:
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')

    #
    plt.figure()
    plt.plot(elevation, 'x-')
    plt.plot(azimuth, 'x-')
    plt.show()

def plot_gimbal(alpha1, alpha2, gamma, ax):
    rot1_gamma = R.from_euler('y', np.rad2deg(gamma), degrees=True).as_matrix()
    rrod1 = np.array([-1, 0, 0])
    rot_alpha1 = lambda alpha1: R.from_euler('z', np.rad2deg(alpha1), degrees=True).as_matrix()
    rrod2 = np.array([1, 0, 0])
    rot_alpha2 = lambda alpha2: R.from_euler('z', np.rad2deg(alpha2), degrees=True).as_matrix()
    rrod_cam = np.array([-1, 0, 0])
    rot_cam = R.from_euler('y', np.rad2deg(np.pi / 2 - gamma), degrees=True).as_matrix()

    plot_coord_system([0, 0, 0], np.eye(3), 0.1, ax)

    rod1rot = rot_alpha1(np.pi - alpha1) @ rot1_gamma
    rod1tip = rod1rot @ rrod1
    rod1 = np.array([[0, 0, 0], rod1tip])
    ax.plot(rod1[:, 0], rod1[:, 1], rod1[:, 2])
    plot_coord_system(rod1tip, rod1rot, 0.1, ax)

    rod2rot = rod1rot @ rot_alpha2(alpha2)
    rod2tip = rod2rot @ rrod2
    rod2 = np.array([[0, 0, 0], rod2tip]) + rod1[1, :]
    ax.plot(rod2[:, 0], rod2[:, 1], rod2[:, 2])
    plot_coord_system(rod2[1, :], rod2rot, 0.1, ax)

    rod_camrot = rod2rot @ rot_cam
    rod_camtips = rod_camrot @ rrod_cam
    rod_cam = np.array([[0, 0, 0], rod_camtips]) + rod2[1, :]
    ax.plot(rod_cam[:, 0], rod_cam[:, 1], rod_cam[:, 2])
    plot_coord_system(rod_cam[1, :], rod_camrot, 0.1, ax)

    fff = np.arccos(np.dot(rod_camtips, [0, 0, 1]) / np.linalg.norm(rod_camtips) / np.linalg.norm([0, 0, 1]))
    phi_ = np.arctan2(rod_camtips[1], rod_camtips[0])
    # print(phi_)

def plot_coord_system(pos, rot_mat, scale, ax):
    points = np.array([
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ]) * scale
    points = (rot_mat @ points.T).T + np.array(pos)
    origin = points[0]
    dirX = points[1] - points[0]
    dirY = points[2] - points[0]
    dirZ = points[3] - points[0]
    ax.quiver(origin[0], origin[1], origin[2], dirX[0], dirX[1], dirX[2], linewidth=2, color='b')
    ax.quiver(origin[0], origin[1], origin[2], dirY[0], dirY[1], dirY[2], linewidth=2, color='r')
    ax.quiver(origin[0], origin[1], origin[2], dirZ[0], dirZ[1], dirZ[2], linewidth=2, color='m')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

gimbal_trajectory()
