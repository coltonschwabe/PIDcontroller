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
vals = np.concatenate((np.full(500, 13.50), np.full(500, 26.5), np.full(500, 13.5), np.full(500, 26.5)))

state = 0

response = []
response2 = []

integral = 0
integralTerm = 0

currentresp = 0 # current response

#without anti windup
#max PID val is currently 24669.827997126748
max = 150000

def sat(val, m):
    if (val > m): return m
    elif (val < -1*m): return -1*m

    return val

for i in range(0, len(times)-1):
    error = vals[i] - currentresp
    
    PIDval = K/Ti * integral + K * error

    PIDvalsat = sat(PIDval, max) # saturates output

    t, r, s = ctrl.forced_response(temp_system, [times[i], times[i + 1]], [PIDvalsat, PIDvalsat], X0=state, return_x = True) #time, response
    currentresp = np.squeeze(r[-1])
    response.append(currentresp)

    integral += error * dt

    state = np.squeeze(s[:,-1])

state = 0
currentresp = 0
for i in range(0, len(times)-1):
    error = vals[i] - currentresp
    
    PIDval_unsat = integralTerm + K * error
    PIDvalsat = sat(PIDval_unsat, max) # saturates output

    t, r, s = ctrl.forced_response(temp_system, [times[i], times[i + 1]], [PIDvalsat, PIDvalsat], X0=state, return_x = True) #time, response
    currentresp = np.squeeze(r[-1])
    response2.append(currentresp)

    integralTerm += (K/Ti * error + (1/(Ti)) * (PIDvalsat - PIDval_unsat)) * dt

    state = np.squeeze(s[:,-1])


plt.plot(times[:-1], response, label = 'Without Anti-windup')
plt.plot(times[:-1], response2, label = 'With Anti-windup')
plt.plot(times, vals, label = 'Setpoint')
plt.legend()
plt.show()