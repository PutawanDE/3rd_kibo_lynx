import math
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

pos = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]

def eulerto_quaternion(roll: float, pitch: float, yaw: float, **kwargs):
    #https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
    """Convert an Euler angle to a quaternion.

    Args:
        roll (float): The roll (rotation around x-axis) angle in radians.
        pitch (float): The pitch (rotation around y-axis) angle in radians.
        yaw (float): The yaw (rotation around z-axis) angle in radians.

    Returns:
        ndarray: The orientation in quaternion [x,y,z,w] format
    """

    if (kwargs.get("degree")):
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return np.array([qx, qy, qz, qw])

def rotate_vector_by_quaternion(v: Tuple[float, float, float], q: Quaternion):
    # https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    u = q[:3]
    s = q[3]

    return 2.0 * np.dot(u, v) * u + np.dot((s*s - np.dot(u, u)), v) + 2.0*s * np.cross(u, v)

def points_from(pmin: pos, pmax: pos):
    xmin, ymin, zmin = pmin
    xmax, ymax, zmax = pmax

    """
    return point from 
    .      →      .

    ↑             ↓

    .      ←      .

    and change to next layer 
    """
    return [
        (xmin, ymin, zmin), 
        (xmax, ymin, zmin),
        (xmax, ymin, zmax),
        (xmin, ymin, zmax),

        (xmin, ymax, zmin),
        (xmax, ymax, zmin),
        (xmax, ymax, zmax),
        (xmin, ymax, zmax)
    ]

def plot_scatter(ax: plt.Axes, point: np.ndarray, **kwargs):
    xdata = np.array(point[:,0])
    ydata = np.array(point[:,1])
    zdata = np.array(point[:,2])

    size = 4
    color = "black"
   
    if (kwargs.get("size")):
        size = kwargs.get("size")
    
    if (kwargs.get("color")):
        color = kwargs.get("color")
    
    if (kwargs.get("text")):
        # label first scatter point, hacky way
        ax.text(xdata[0], ydata[0], zdata[0], kwargs.get("text"))

    ax.scatter(xdata, ydata, zdata, color=color, marker="o", s=size)

def plot_filled_rect(ax: plt.Axes, P: np.ndarray, color: str, **kwargs):
    # verts is just a collection of points in each side
    verts = [[P[0], P[1], P[2], P[3]], 
        [P[4], P[5], P[6], P[7]],
        [P[0], P[1], P[5], P[4]],
        [P[3], P[2], P[6], P[7]],
        [P[0], P[3], P[7], P[4]],
        [P[1], P[2], P[6], P[5]]
    ]
    
    plot_scatter(ax, P)
    ax.add_collection(Poly3DCollection(verts, facecolor=color, alpha=0.1, linewidths=1, edgecolor="r"))

def main():
    """
    Vector&Quaternion experiment
    V = (-1, 0, 0)
    NAV = (0, -1, 0)
    #q = eulerto_quaternion(0, 90, 0, degree=True)
    q = np.array([0, 0, 0.707, 0.707])
    V_ = rotate_vector_by_quaternion(NAV, q)
    print(q)
    print(V, V_)
    """

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.invert_zaxis()

    kiz = np.array(points_from([10.3, -10.2, 4.32], [11.55, -6.4, 5.57]))
    koz1 = np.array(points_from([9.8673, -9.18813, 3.81957], [10.7673, -8.28813, 4.81957]))
    koz2 = np.array(points_from([9.8585, -9.4500, 4.82063], [12.0085, -8.5000, 4.87063]))
    koz3 = np.array(points_from([11.1067, -9.44819, 4.87385], [12.0067, -8.89819, 5.87385]))
    plot_filled_rect(ax, kiz, "cyan")
    plot_filled_rect(ax, koz1, "red")
    plot_filled_rect(ax, koz2, "red")
    plot_filled_rect(ax, koz3, "red")

    # starting point
    plot_scatter(ax, np.array([(10.76150, -6.88490, 5.31647)]), color="red", size=24)

    # Astrobee Vector 
    #ax.quiver(10.76150, -6.88490, 5.31647, V[0], V[1], V[2], color="red", length=0.2)
    #ax.quiver(10.76150, -6.88490, 5.31647, NAV[0], NAV[1], NAV[2], color="green", length=0.2)
    #ax.quiver(10.76150, -6.88490, 5.31647, V_[0], V_[1], V_[2], length=0.2)
    
    # point-A
    plot_scatter(ax, np.array([(10.71000, -7.70000, 4.48000)]), color="blue", size=24, text="A")

    # point 2
    plot_scatter(ax, np.array([(11.27460, -9.92284, 5.29881)]), color="blue", size=24, text="2")

    # goal point
    plot_scatter(ax, np.array([(11.27460, -7.89178, 4.96538)]), color="gold", size=24, text="goal")

    plt.show()
    return

if __name__ == "__main__":
    main()
