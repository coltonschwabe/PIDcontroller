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
vals = np.concatenate((np.full(500, 11.50), np.full(500, 28.5), np.full(500, 11.5), np.full(500, 28.5)))

#without anti windup
#max PID val is currently 24669.827997126748
max = 150000

def sat(val, m):
    if (val > m): return m
    elif (val < -1*m): return -1*m

    return val



for j in range(1, 21):
    response = []
    integralTerm = 0
    state = 0
    currentresp = 0
    Tt = Ti*(10/j)/25
    
    for i in range(0, len(times)-1):
        error = vals[i] - currentresp
        
        PIDval_unsat = integralTerm + K * error
        PIDvalsat = sat(PIDval_unsat, max) # saturates output

        t, r, s = ctrl.forced_response(temp_system, [times[i], times[i + 1]], [PIDvalsat, PIDvalsat], X0=state, return_x = True) #time, response
        currentresp = np.squeeze(r[-1])
        response.append(currentresp)

        integralTerm += (K/Ti * error + (1/(Tt)) * (PIDvalsat - PIDval_unsat)) * dt

        state = np.squeeze(s[:,-1])

    plt.plot(times[:-1], response, label = str(j))
    

plt.plot(times, vals, label = 'Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Temperature (C)')
plt.legend()
plt.savefig('antiwindupsim.png')
plt.show()
