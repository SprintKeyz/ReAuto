# final
import math
import matplotlib.pyplot as plt
from dataclasses import dataclass


def inject(waypoints, spacing=0.5):
    new_waypoints = []

    for i in range(len(waypoints) - 1):
        start_point = waypoints[i]
        end_point = waypoints[i+1]

        # calculate the vector between start and end points
        vector = [end_point[0]-start_point[0], end_point[1]-start_point[1]]
        mag = math.sqrt(vector[0]**2 + vector[1]**2)

        # calculate number of points that fit in the line segment
        num_points_that_fit = math.ceil(mag / spacing)

        # calculate the new vector with the desired spacing
        new_vector = [vector[0]/num_points_that_fit,
                      vector[1]/num_points_that_fit]

        # loop over the number of points and add new points to the list
        for j in range(num_points_that_fit):
            new_x = start_point[0] + j*new_vector[0]
            new_y = start_point[1] + j*new_vector[1]
            new_waypoints.append([new_x, new_y])

    # add the last point in the path to the new_points array
    new_waypoints.append(waypoints[-1])

    return new_waypoints


def smoother(path):
    weight_smooth = 0.81
    weight_data = 1 - weight_smooth
    tolerance = 0.001

    # copy array
    new_path = [point[:] for point in path]

    change = tolerance

    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path)-1):
            for j in range(len(path[i])):
                aux = new_path[i][j]
                new_path[i][j] += weight_data * (path[i][j] - new_path[i][j]) + weight_smooth * (
                    new_path[i-1][j] + new_path[i+1][j] - (2.0 * new_path[i][j]))
                change += abs(aux - new_path[i][j])

    return new_path


def make_path(waypoints):
    new_waypoints = inject(waypoints, spacing=3.0)
    return smoother(new_waypoints)


# graph it out
path = [[0, 0], [25, 6], [40, 9], [80, 2], [100, 1.7]]
new_path = make_path(path)

plt.plot([p[0] for p in path], [p[1]
         for p in path], linestyle='--', marker='o', color='r')
plt.plot([p[0] for p in new_path], [p[1]
         for p in new_path], linestyle='--', marker='o', color='b')
plt.show()

# calc distances
dists = []
for i in range(len(new_path)-1):
    dists.append(math.sqrt((new_path[i+1][0]-new_path[i][0])**2 +
                           (new_path[i+1][1]-new_path[i][1])**2))


@dataclass
class Waypoint:
    x: float
    y: float
    curvature: float = 0.0
    velocity: float = 0.0
    dist_to_point: float = 0.0


Waypoints = []


# calculate the distance to the current waypoint and store it

for i in range(len(new_path)-1):
    dist = math.sqrt((new_path[i+1][0]-new_path[i][0])**2 +
                     (new_path[i+1][1]-new_path[i][1])**2)

    dist_prev_point = 0.0
    if i > 0:
        dist_prev_point = Waypoints[i-1].dist_to_point

    Waypoints.append(Waypoint(
        new_path[i][0], new_path[i][1], 0.0, 0.0, dist_prev_point + dist))

# calculate the curvature of the path at each waypoint
# this is done to slow down around sharp turns

'''
We can calculate the
curvature at a point by finding the radius of the circle that intersects the point and the two
points on either side of it. Curvature is then just 1/radius.
'''

for i in range(1, len(Waypoints)-1):
    # fix an edge case divide by zero error
    if Waypoints[i-1].x == Waypoints[i].x:
        Waypoints[i-1].x += 0.0001

    k1 = 0.5 * (Waypoints[i-1].x**2 + Waypoints[i-1].y**2 - Waypoints[i].x **
                2 - Waypoints[i].y**2)/(Waypoints[i-1].x - Waypoints[i].x)

    k2 = (Waypoints[i-1].y - Waypoints[i].y) / \
        (Waypoints[i-1].x - Waypoints[i].x)
        
    b = 0.5 * (Waypoints[i].x**2 - 2 * Waypoints[i].x * k1 + Waypoints[i].y**2 - Waypoints[i+1].x**2 + 2 * Waypoints[i+1].x * k1 - Waypoints[i+1].y**2) / \
        (Waypoints[i+1].x * k2 - Waypoints[i+1].y +
         Waypoints[i].y - Waypoints[i].x * k2)

    a = k1 - k2 * b

    radius = math.sqrt((Waypoints[i-1].x - a)**2 + (Waypoints[i-1].y - b)**2)
    curavuture = 1/radius

    Waypoints[i].curvature = curavuture

max_velocity = 100000.0
turn_speed_k = 2.0  # 1 to 5
final_velocity = 0.0  # stop at the end!
decel_limit = 500  # m/s^2 - I know this is a bit high, but it's just for testing

Waypoints[-1].velocity = final_velocity

# calculate the velocity at each waypoint
# work backwards in the path from len-1 to 1 (including 1)

for i in range(len(Waypoints)-2, 0, -1):
    start = Waypoints[i]
    end = Waypoints[i-1]

    vel = turn_speed_k / start.curvature

    if vel > max_velocity:
        vel = max_velocity

    # dist from last point
    dist = start.dist_to_point - end.dist_to_point

    # max velocity that can be achieved in the distance (respecting acceleration)
    max_vel = math.sqrt(start.velocity**2 + (2 * decel_limit * dist))

    new_vel = min(vel, max_vel)

    end.velocity = new_vel

# print out the results
for i in range(len(Waypoints)):
    print(
        f'Waypoint {i}: x={Waypoints[i].x}, y={Waypoints[i].y}, curvature={Waypoints[i].curvature}, velocity={Waypoints[i].velocity}')

# the code to follow the path