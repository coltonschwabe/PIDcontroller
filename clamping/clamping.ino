// Potential better anti-windup

// Calculate the error
  float T = temp();
  float error = setpoint - T;
// Calculate integral term
  float tau = 0.026;
  integral += error * tau;

  // Anti-windup: Clamp the integral term to avoid overflow
  if (integral > output_max / Ki) integral = output_max / Ki;
  if (integral < output_min / Ki) integral = output_min / Ki;

  // Calculate derivative term
  float derivative = (error - prev_error) / tau;

  // Compute the control output
  control_output = Kp * error + Ki * integral + Kd * derivative;

  // Clamp the output to actuator limits
  if (control_output > output_max) control_output = output_max;
  if (control_output < output_min) control_output = output_min;

  prev_error = error;