import numpy as np
from numpy.core.fromnumeric import shape

# A C-Spline is a np array of the follwing structure:
# 
# t_1, q_1, ... q_n, qd_1, qd_n
# ...
# t_m, q_1, ... q_n, qd_1, qd_n
#

CSpline = np.ndarray

def sample_cspline_segment(p0, p1, time):
    t = (time - p0[0]) / (p1[0] - p0[0])

    n_q = (p0.shape[0] - 1) // 2
    scale = p1[0] - p0[0]

    h_00 = 2*t**3 - 3*t**2 + 1
    h_10 = t**3 - 2*t**2 + t
    h_01 = -2*t**3 + 3*t**2
    h_11 = t**3 - t**2
    return p0[1:n_q + 1] * h_00 \
         + p0[1 + n_q:] * h_10 * scale \
         + p1[1:n_q + 1] * h_01 \
         + p1[1 + n_q:] * h_11 * scale

def sample_cspline(cspline: CSpline, time: float):
    if time <= 0:
        return cspline[0, 1:]
    elif time >= cspline[-1, 0]:
        return cspline[-1, 1:]
    
    idx = 1
    while cspline[idx, 0] < time:
        idx += 1
    
    return sample_cspline_segment(cspline[idx - 1], cspline[idx], time)

def resample_spline(cspline: CSpline, samples: int):
    times = np.linspace(cspline[0, 0], cspline[-1, 0], samples, endpoint=True)

    out = []

    idx = 1
    for t in times:
        while cspline[idx, 0] < t:
            idx += 1
        
        out.append(np.hstack(([t], sample_cspline_segment(cspline[idx - 1], 
                                                          cspline[idx], t))))

    return np.vstack(out)

def rescale_cspline(cspline: CSpline, vel_limits, margin=0.0):
    sampled = resample_spline(cspline, cspline.shape[0] * 3)

    n_q = (cspline.shape[1] - 1) // 2
    vel_max = np.max(np.abs(sampled[:, n_q+1:]), axis=0)

    max_vel_frac = np.max(vel_max / (vel_limits * (1.0 - margin)))

    out_spline = cspline.copy()
    out_spline[:, 0] /= max_vel_frac
    out_spline[:, n_q + 1:] /= max_vel_frac

    return out_spline

