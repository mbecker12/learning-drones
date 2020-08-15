def verlet_get_delta_x(velocity, current_acceleration, delta_t):
    dv = velocity * delta_t
    da = 0.5 * current_acceleration * delta_t * delta_t
    delta_x = velocity * delta_t + 0.5 * current_acceleration * delta_t * delta_t
    return delta_x


def verlet_get_delta_v(last_acceleration, current_acceleration, delta_t):
    delta_v = 0.5 * (last_acceleration + current_acceleration) * delta_t
    return delta_v
