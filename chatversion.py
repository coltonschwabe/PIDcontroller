import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

def KLT_transfer(Kp,L,T):
    numer = [Kp]
    deno = [T,1]
    
    # modeling the time delay
    delay_approx = ctrl.pade(L,3)
    delay_num, delay_den = delay_approx
    
    system_without_delay = ctrl.TransferFunction(numer, deno)
    delay_tf = ctrl.TransferFunction(delay_num, delay_den)
    system_with_delay = ctrl.series(system_without_delay, delay_tf)
    
    return system_with_delay
    
def PI_loopfilter(Kp,Ti):
    Ki = Kp/Ti
    return ctrl.TransferFunction([Kp, Ki], [1, 0]) # (sKp + Ki)/s
    
Kp = -0.000912
L = 1.5
T = 110.5

K = 0.15 / Kp + (0.35 - L*T / ((L + T)*(L + T)))*(T / (Kp * L))
Ti = 0.35 * L + 6.7*L*T*T / (T*T + 2*L*T + 10*L*L)

controller = PI_loopfilter(K, Ti)
temp_system = KLT_transfer(Kp, L, T)
closed_loop = ctrl.feedback(controller * temp_system, 1, -1) #system1, system2, feedback or feedforward
#time, response = ctrl.step_response(closed_loop, np.linspace(0,60,500))
times = np.linspace(0, 60*4, 500*4)
dt = times[1] - times[0]
vals = np.concatenate((np.full(500, 19.50), np.full(500, 20.5), np.full(500, 19.5), np.full(500, 20.5)))

state = 0

response = []

integral = 0

currentresp = 0 # current response

for i in range(0, len(times)-1):
    error = vals[i] - currentresp
    integral += error * dt
    PIDval = K/Ti * integral + K * error

    t, r, s = ctrl.forced_response(temp_system, [times[i], times[i + 1]], [PIDval, PIDval], X0=state, return_x = True) #time, response
    currentresp = np.squeeze(r[-1])
    response.append(currentresp)

    state = np.squeeze(s[:,-1])

plt.plot(times[:-1], response)
plt.plot(times, vals)
plt.show()