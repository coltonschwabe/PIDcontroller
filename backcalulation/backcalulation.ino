// Potential better anti-windup

// Calculate the error
  float T = temp();
  float error = setpoint - T;
// Calculate integral term
  float tau = 0.026;

  // Calculate derivative term
  float derivative = (error - prev_error) / tau;

  // Compute the control output
  control_output = Kp * error + integral + Kd * derivative;

  // tracking time constant
  float Tt = xyz;

  // "undwinds" if output is saturated
  if (control_output > output_max) {
    integral += (Ki * error + (outputmax - control_output) / Tt) * tau

    control_output = output_max;
  } else if (control_output < output_min)  {
    integral += (Ki * error + (outputmax - control_output) / Tt) * tau

    control_output = output_min;
  } else {
    integral += Ki * error * tau;
  }

  prev_error = error;