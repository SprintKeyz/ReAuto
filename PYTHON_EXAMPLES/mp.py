import matplotlib.pyplot as plt
import math

# define our simulation varaibles
setpoint = 10.0  # in
t_step = 0.01

v_max = 5.0  # in/s
a_max = 12.0  # in/s^2

# our motors have first order velocity dynamics, meaning it is not an advantage
# to use s-curve profiles. Therefore, we'll use trapezoidal profiles.

# the profle uses the following equations:
# x(t) = x0 + v0*t + 0.5*a*t^2, where x(t) is the position at time t
# v(t) = v0 + a*t, where v(t) is the velocity at time t


def generate_profile(max_v, time_to_max_v, dt, setpoint):
    '''
    max_v: the max velocity of the profile
    time_to_max_v: the time it takes to go from 0 to max_v
    dt: timestep (s)
    setpoint: target at which velocity is 0
    '''

    t_rec = [0.0]
    x_rec = [0.0]
    v_rec = [0.0]
    a_rec = [0.0]

    a = max_v / time_to_max_v
    time_at_max_v = setpoint / max_v - time_to_max_v

    if max_v * time_to_max_v > setpoint:
        time_to_max_v = math.sqrt(setpoint / a)
        time_from_max_v = time_to_max_v
        time_total = 2.0 * time_to_max_v
        profile_max_v = a * time_to_max_v

    else:
        time_from_max_v = time_to_max_v + time_at_max_v
        time_total = time_from_max_v + time_to_max_v
        profile_max_v = max_v

    while t_rec[-1] < time_total:
        t = t_rec[-1] + dt
        t_rec.append(t)

        if t < time_to_max_v:
            # accelerate to max v
            a_rec.append(a)
            v_rec.append(a*t)

        elif t < time_from_max_v:
            a_rec.append(0.0)
            v_rec.append(profile_max_v)

        elif t < time_total:
            decel_time = t - time_from_max_v
            a_rec.append(-a)
            v_rec.append(profile_max_v - a * decel_time)

        else:
            a_rec.append(0.0)
            v_rec.append(0.0)

        x_rec.append(x_rec[-1] + v_rec[-1] * dt)

    return t_rec, x_rec, v_rec, a_rec


# calculate the time it woudl take to reach max velocity
time_to_max_v = v_max / a_max

profile = generate_profile(v_max, time_to_max_v, t_step, setpoint)

# graph the profile
plt.plot(profile[0], profile[1])
plt.plot(profile[0], profile[2])
plt.plot(profile[0], profile[3])

plt.legend(['Position', 'Velocity', 'Acceleration'])
plt.xlabel('Time (s)')

plt.suptitle('Trapezoidal Motion Profile')
plt.title(
    'Max velocity: {}m/s, Acceleration: {}m/$s^2$, Target: {}in'.format(v_max, a_max, setpoint))

plt.show()