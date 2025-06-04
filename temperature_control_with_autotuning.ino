#include <SPI.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <math.h>

SPISettings settings(2000000,MSBFIRST,SPI_MODE1);
LiquidCrystal lcd(7,8,2,3,4,5);

void setup() {
  SPI.begin();
  SPI.beginTransaction(settings);
  Serial.begin(2000000);

  lcd.begin(8,2);

  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(A1,INPUT);
  digitalWrite(10,HIGH);

  delay(1);
  //Write DAC control register
  PORTB &= B11111011;
  SPI.transfer(B00000100);
  SPI.transfer16(0b0000000011100101);
  PORTB |= B00000100;
  delay(10);
  DACwrite(0);
}

//Read ADC input and average
float ADCread(){
  float x = 0.0;
  for(int j=0; j<1000; j++){
    PORTB &= B11111101; //Writes pin 9 low
    x += SPI.transfer16(0);
    PORTB |= B00000010; //Writes pin 9 high
  }
  x /= 1000.0;
  return x;
}

//Write 16 bit integer to DAC
void DACwrite(int n){
  /* check if control register is correct */
  PORTB &= B11111011;
  SPI.transfer(0b1100);
  SPI.transfer16(0);
  PORTB |= B00000100;

  PORTB &= B11111011;
  SPI.transfer(0);
  byte control = SPI.transfer16(0); 
  PORTB |= B00000100;

  if (control != 0b11100101)
  {
    //Write DAC control register
    PORTB &= B11111011;
    SPI.transfer(B00000100);
    SPI.transfer16(0b0000000011100101);
    PORTB |= B00000100;
  }

  PORTB &= B11111011;
  SPI.transfer(B00000011);
  SPI.transfer16(n);
  PORTB |= B00000100;
}

//Read thermistor temperature
float temp(){
  float x = ADCread();
  float y = 5.0*x/65535.0;      
  float RT = (10000.0*y)/(5.0-y);
  float R0 = 10000.0;
  float T0 = 25 + 273.15;
  float B = float((EEPROM.read(12) << 8) + EEPROM.read(13)); // read beta from eeprom. Need to update beta using terminal software.
  float rinf = R0*exp(-B/T0);
  float T = B/log(RT/rinf) - 273.15;
  return T;
}

//Read current to TEC (or heater)
float current(){
  return 1000.0*(2.0*analogRead(A0)/1023.0 - 1);
}

//Global counters
float LCD_counter = 0.0;
float current_sum = 0.0;
int manual_step = -1;

//Global timers
unsigned long t0 = 0;
unsigned long t1 = 0;

//Global state
char state = '0';

//Global PID error variables
float sum_error = 0.0;
float integralTerm = 0.0;
float previous_error = 0.0;

//Global autotuning variables
int atState = 0;
float Iu = 0.0;
float Iy = 0.0;
long d1 = 0;
long d2 = 0;
float h = 0.2; //hysteresis
float setTemp = 20.0;
long workingOutput = 0;
bool withinH = true; // within hysteresis
unsigned long tOn = millis();
unsigned long tOff = tOn;

//Global parameters (sent over serial)
unsigned long parameters[10];

//ensures value stays within upper and lower limits
long sat(long val, long upperlimit, long lowerlimit) {
  if (val >= upperlimit) return upperlimit;
  else if (val <= lowerlimit) return lowerlimit;

  return val;
}

void loop() {

//*********Turn output off if switch is in "off" position*********
  boolean circuit_on = digitalRead(A1);
  if(circuit_on == false){
    DACwrite(0);
    lcd.setCursor(0,0);
    lcd.print("  OFF   ");
    lcd.setCursor(0,1);
    lcd.print(String(temp(),1)+" C  ");
    
    if(Serial.available() >= 48){
      state = 'A';
    }
  }

//*********Normal PID operation*********
  if((state == '0' || state == '1') && circuit_on == true){
    // Read PID constants from EEPROM
    float Kp = float(((unsigned long)EEPROM.read(0) << 16) + ((unsigned long)EEPROM.read(1) << 8) + (unsigned long)EEPROM.read(2));
    float Ki = float(((unsigned long)EEPROM.read(3) << 16) + ((unsigned long)EEPROM.read(4) << 8) + (unsigned long)EEPROM.read(5));
    float Kd = float(((unsigned long)EEPROM.read(6) << 16) + ((unsigned long)EEPROM.read(7) << 8) + (unsigned long)EEPROM.read(8));

    float Ti = Kp / Ki;
    float Td = Kd / Kp;
    float Tt = (Ti + Td)/2.0;

    //Read setpoint from EEPROM
    float setpoint = float((EEPROM.read(9) << 8) + EEPROM.read(10))/100.0;

    // Read gain sign from EEPROM
    int sign0 = EEPROM.read(11);
    int sign;
    if(sign0 == 1){
      sign = -1;
    }
    if(sign0 == 2){
      sign = 1;
    }

    //Read upper and lower limits from EEPROM
    float lower_limit = float((EEPROM.read(14) << 8) + EEPROM.read(15))/1000.0;
    float upper_limit = float((EEPROM.read(16) << 8) + EEPROM.read(17))/1000.0;

    
    float tau = 0.026;
    
    //Calculate error and error sum (for integrator)
    float T = temp();
    float error = setpoint - T;
    //sum_error += error;

    //Find DAC output from PID equation
    long DAC_integer = sign*round(Kp*error + integralTerm + (Kd/tau)*(error-previous_error));
    long DAC_integer_sat = sat(DAC_integer, round(32767.0*upper_limit), round(-32768.0*lower_limit));

    integralTerm += (Ki * error + (1/Tt) * (DAC_integer_sat - DAC_integer)) * tau;

    DACwrite(DAC_integer_sat);
    
    //Stall the loop if the temperature is too low or high for the diode
    while(T < 15 || T > 40){
      DACwrite(0);
      lcd.setCursor(0,0);
      lcd.print(" DANGER ");
      lcd.setCursor(0,1);
      lcd.print(String(T,1)+" C  ");
      T = temp();
      //Break out of loop if terminal program sends serial data
      if(Serial.available() >= 48){
        state = 'A';
        break;
      }
      //Break out of loop if the board is switched off
      if(digitalRead(A1) == false){
        break;
      }
    }
    
    previous_error = error;

    //Write to error monitor
    float error_monitor = error;
    if(error > 1.0){
      error_monitor = 1.0;
    }
    if(error < -1.0){
      error_monitor = -1.0;
    } 
    byte monitor_out = round(0.5*(error_monitor + 1.0)*255.0);   
    analogWrite(6,monitor_out);

    // Write to LCD
    float current_reading = current();
    current_sum += current_reading;
    LCD_counter += 1.0;
    if(LCD_counter < 50){
      delayMicroseconds(5400);
    }
    if(LCD_counter >= 50){
      int I_lcd = round(current_sum/LCD_counter);
      lcd.setCursor(0,0);
      if(I_lcd == 1000){
        lcd.print(">1A     ");
      }
      else if(I_lcd == -1000){
        lcd.print("<-1A    ");
      }
      else{
        lcd.print(String(I_lcd)+" mA  ");
      }
      lcd.setCursor(0,1);
      lcd.print(String(T,1)+" C  ");
      LCD_counter = 0.0;
      current_sum = 0.0;
    }

    //Log lock data
    if(state == '1'){
      unsigned long t = millis();
      if(t-t1 > 1000){
        Serial.print(T,4);
        Serial.print(",");
        Serial.print(DAC_integer);
        Serial.print(",");
        Serial.println(current_reading);
        t1 = millis();
      }
    }

    //Process serial data in
    if(Serial.available() >= 48){
      state = 'A';
    }
  }

//*********Read EEPROM*********
  if(state == '2'){
    unsigned long Kp = ((unsigned long)EEPROM.read(0) << 16) + ((unsigned long)EEPROM.read(1) << 8) + (unsigned long)EEPROM.read(2);
    unsigned long Ki = ((unsigned long)EEPROM.read(3) << 16) + ((unsigned long)EEPROM.read(4) << 8) + (unsigned long)EEPROM.read(5);
    unsigned long Kd = ((unsigned long)EEPROM.read(6) << 16) + ((unsigned long)EEPROM.read(7) << 8) + (unsigned long)EEPROM.read(8);

    float setpoint = float((EEPROM.read(9) << 8) + EEPROM.read(10))/100.0;

    byte sign0 = EEPROM.read(11);
    int sign;
    if(sign0 == 1){
      sign = -1;
    }
    if(sign0 == 2){
      sign = 1;
    }

    unsigned int beta = ((unsigned int)EEPROM.read(12) << 8) + (unsigned int)EEPROM.read(13);
    float lower_limit = float((EEPROM.read(14) << 8) + EEPROM.read(15))/10.0;
    float upper_limit = float((EEPROM.read(16) << 8) + EEPROM.read(17))/10.0;

    Serial.print(Kp);
    Serial.print(",");
    Serial.print(Ki);
    Serial.print(",");
    Serial.print(Kd);
    Serial.print(",");
    Serial.print(setpoint,2);
    Serial.print(",");
    Serial.print(sign);
    Serial.print(",");
    Serial.print(beta);
    Serial.print(",");
    Serial.print(lower_limit);
    Serial.print(",");
    Serial.println(upper_limit);
    
    state = '0';
  }

//*********Update setpoint in EEPROM*********
  if(state == '3'){
    EEPROM.write(9,(parameters[3] >> 8));
    EEPROM.write(10,parameters[3]);
    state = '0';
  }

//*********Save PID parameters in EEPROM*********
  if(state == '4'){
    for(byte j=0; j<=2; j++){
      EEPROM.write(3*j,(parameters[j] >> 16));
      EEPROM.write(3*j+1,(parameters[j] >> 8));
      EEPROM.write(3*j+2,parameters[j]);
    }
    EEPROM.write(9,(parameters[3] >> 8));
    EEPROM.write(10,parameters[3]);
    state = '7';
  }

//*********Save config parameters in EEPROM*********
  if(state == '5'){
    EEPROM.write(11,parameters[6]);
    EEPROM.write(12,parameters[7] >> 8);
    EEPROM.write(13,parameters[7]);
    EEPROM.write(14,parameters[8] >> 8);
    EEPROM.write(15,parameters[8]);
    EEPROM.write(16,parameters[9] >> 8);
    EEPROM.write(17,parameters[9]);
    state = '7';
  }

//*********Constant output*********
  if(state == '6' && circuit_on == true){
    int sign;
    if(parameters[6] == 1){
      sign = -1;
    }
    if(parameters[6] == 2){
      sign = 1;
    }
     
    float percentage = float(parameters[4])/1000.0;
    int DAC_integer = sign*round(32767.0*percentage);
    DACwrite(DAC_integer);
   
    //Log data until a serial command is received
    while(Serial.available() < 48){
      unsigned long t = millis();
      
      current_sum += current();
      LCD_counter += 1.0;
      
      if(t-t1 > 1000){
        float T = temp();

        int I_lcd = round(current_sum/LCD_counter);
        lcd.setCursor(0,0);
        if(I_lcd == 1000){
          lcd.print(">1A     ");
        }
        else if(I_lcd == -1000){
          lcd.print("<-1A    ");
        }
        else{
          lcd.print(String(I_lcd)+" mA  ");
        }
        lcd.setCursor(0,1);
        lcd.print(String(T,1)+" C  ");
        
        Serial.print(T,4);
        Serial.print(",");
        Serial.print(DAC_integer);
        Serial.print(",");
        Serial.println(I_lcd);
        t1 = millis();
      }
      current_sum = 0.0;
      LCD_counter = 0.0;
    }
    state = 'A';
  }
  
//*********Output off*********
  if(state == '7'){
    //Set DAC to 0V
    DACwrite(0);

    //Average readings for LCD
    while(Serial.available() < 48){
      unsigned long t = millis();
      current_sum += current();
      LCD_counter += 1.0;
      
      if(t-t1 > 1000){
        float T = temp();
        int I_lcd = round(current_sum/LCD_counter);
        lcd.setCursor(0,0);
        lcd.print(String(I_lcd)+" mA  ");
        lcd.setCursor(0,1);
        lcd.print(String(T,1)+" C  ");
        t1 = millis();
      }
      current_sum = 0.0;
      LCD_counter = 0.0;
    }
    state = 'A';
  }

//*********Tuning pulse*********
  if(state == '8' && circuit_on == true){
    float sign;
    if(parameters[6] == 1){
      sign = -1.0;
    }
    if(parameters[6] == 2){
      sign = 1.0;
    }
    int DAC_integer;
    float percentage = sign*float(parameters[4])/1000.0;
    unsigned long t = millis();
    
    if(t-t0 <= 1000){
      DAC_integer = 0;
    }
    if(t-t0 > 1000){
      DAC_integer = round(32767.0*percentage);
      DACwrite(DAC_integer);
    }

    float T = temp();     
    
    // Write to LCD
    float current_reading = current();
    current_sum += current_reading;
    LCD_counter += 1.0;
    if(LCD_counter < 50){
      delayMicroseconds(5400);
    }
    if(LCD_counter >= 50){
      int I_lcd = round(current_sum/LCD_counter);
      lcd.setCursor(0,0);
      if(I_lcd == 1000){
        lcd.print(">1A     ");
      }
      else if(I_lcd == -1000){
        lcd.print("<-1A    ");
      }
      else{
        lcd.print(String(I_lcd)+" mA  ");
      }
      lcd.setCursor(0,1);
      lcd.print(String(T,1)+" C  ");
      LCD_counter = 0.0;
      current_sum = 0.0;
    }
    if(t-t1 > 100){  
      Serial.print(T,4);
      Serial.print(",");
      Serial.print(DAC_integer);
      Serial.print(",");
      Serial.println(current_reading);
      t1 = millis();
    }
    if(Serial.available() >= 48){
      state = 'A';
    }
  }

//*********Manual tuning*********
  if(state == '9' && circuit_on == true){
    float Kp = float(parameters[0]);
    float Ki = float(parameters[1]);
    float Kd = float(parameters[2]);
    float setpoint = float(parameters[3])/100.0;
    float amp = float(parameters[4])/100.0;
    unsigned long period = parameters[5]*100;

    float Ti = Kp / Ki;
    float Td = Kd / Kp;
    float Tt = (Ti + Td) / 2.0;

    int sign;
    if(parameters[6] == 1){
      sign = -1;
    }
    if(parameters[6] == 2){
      sign = 1;
    }

    float set = setpoint + manual_step*amp;

    unsigned long t = millis();
    if(t-t0 > period){
      manual_step = -manual_step;
      t0 = millis();
    }

    float tau = 0.028;
    
    //Calculate error and error sum (for integrator)
    float T = temp();
    float error = set - T;
    //sum_error += error;

    //Find DAC output from PID equation
    long DAC_integer = sign*round(Kp*error + integralTerm + (Kd/tau)*(error-previous_error));
    long DAC_integer_sat = sat(DAC_integer, 32767, -32768);

    integralTerm += (Ki * error + (1/Tt) * (DAC_integer_sat - DAC_integer)) * tau;

    DACwrite(DAC_integer_sat);
    
    previous_error = error;

    float current_reading = current();
    current_sum += current_reading;
    LCD_counter += 1.0;
    if(t-t1 > 1000){
      Serial.print(T,4);
      Serial.print(",");
      Serial.print(DAC_integer);
      Serial.print(",");
      Serial.println(current_reading);

      int I_lcd = round(current_sum/LCD_counter);
      lcd.setCursor(0,0);
      lcd.print(String(I_lcd)+" mA  ");
      lcd.setCursor(0,1);
      lcd.print(String(T,1)+" C  ");

      current_sum = 0.0;
      LCD_counter = 0.0;
      t1 = millis();
    }
    else{
      delayMicroseconds(7090);
    }

    if(Serial.available() >= 48){
      state = 'A';
    }
  }

//*********Run through autotuning************
  if(state == 'T' && circuit_on == true) {
    float dt = 0.028; // this is referred to as tau in other sections of the code
    float T = temp();

    float uOn = workingOutput - d1;
    float uOff = workingOutput + d2;

    unsigned long t = millis();

    if (atState == 0) { // PID control to bring to equilibrium
      if (setTemp == 20.0) { //changes working point temp to current temp
        setTemp = T;
      }

      float Kp = float(parameters[0]);
      float Ki = float(parameters[1]);
      float Kd = float(parameters[2]);

      float Ti = Kp / Ki;
      float Td = Kd / Kp;
      float Tt = (Ti + Td) / 2.0;

      int sign;
      if(parameters[6] == 1){
        sign = -1;
      }
      if(parameters[6] == 2){
        sign = 1;
      }

      //Calculate error and error sum (for integrator)
      float error = setTemp - T;

      //Find DAC output from PID equation
      long DAC_integer = sign*round(Kp*error + integralTerm + (Kd/dt)*(error-previous_error));
      long DAC_integer_sat = sat(DAC_integer, 32767, -32768);

      integralTerm += (Ki * error + (1/Tt) * (DAC_integer_sat - DAC_integer)) * dt;

      DACwrite(DAC_integer_sat);
    
      previous_error = error;

      if (abs(setTemp - T) < h) { //within temp boundaries
        if (abs(workingOutput - DAC_integer_sat) < 250) {
          if (t - t0) > 5000 { //steady for 5 seconds
            atState += 1; //move to autotuning 

            if (workingOutput < 0) {
              d1 = (32768 + workingOutput) / 2;
              d2 = 2 * d1;
            } else {
              d2 = (32767 - workingOutput) / 2;
              d1 = 2 * d2;
            }

            //if (d1 == 0) { // whichs means they both are 0 (im not sure what would happen)
          }
        } else {
          workingOutput = DAC_integer_sat;
          t0 = millis();
        }
      }

    } else if (atState == 1) { //moving out of hysteresis
      if (abs(T - setTemp) < h) {
        DACwrite(uOff);     
      } else { // outside of hysteresis, move to tOn
        withinH = false;
        atState += 1;
        tOn = millis();
      }
    } else if (atState == 2) { //tOn
      if (abs(T - setTemp) < h || !withinH) {
        if (!withinH && abs(T - setTemp) < h) {
          withinH = true;
        }

        DACwrite(uOn);
        Iy += (T - setTemp) * dt;
        Iu += (uOn - workingOutput) * dt;
      } else { // outside of hysteresis, move to tOff
        withinH = false;
        atState += 1;
        tOn = t - tOn; // sets tOn to delta t interval
        tOff = millis();
      }
    } else if (atState == 3) { //tOff
      if (abs(T - setTemp) < h || !withinH) {
        if (!withinH && abs(T - setTemp) < h) {
          withinH = true;
        }

        DACwrite(uOff);
        Iy += (T - setTemp) * dt;
        Iu += (uOff - workingOutput) * dt;
      } else { // outside of hysteresis, calculate
        withinH = false;
        atState += 1;
        tOff = t - tOff; // sets tOff to delta t interval
      }
    } else if (atState == 4) { //calculate
      float found_Kp = Iy / Iu;
      float gamma = ((float) max(d1, d2)) / min(d1, d2);
      float rho = ((float) max(tOn, tOff)) / min(tOn, tOff);
      float tau = (gamma - rho) / ((gamma - 1)*(0.35 * rho + 0.65));
      float L_over_T = tau / (1 - tau);
      float found_T = tOn / log((h / abs(found_Kp) - d2 + exp(L_over_T) * (d1 + d2)) / (d1 - h / abs(found_Kp)));
      float found_L = found_T * L_over_T;
    }

    float current_reading = current(); //reads current and displays on LCD
    current_sum += current_reading;
    LCD_counter += 1.0;
    if(t-t1 > 1000){
      Serial.print(T,4);
      Serial.print(",");
      Serial.print(DAC_integer);
      Serial.print(",");
      Serial.println(current_reading);

      int I_lcd = round(current_sum/LCD_counter);
      lcd.setCursor(0,0);
      lcd.print(String(I_lcd)+" mA  ");
      lcd.setCursor(0,1);
      lcd.print(String(T,1)+" C  ");

      current_sum = 0.0;
      LCD_counter = 0.0;
      t1 = millis();
    }
    else{
      delayMicroseconds(7090);
    }

    if(Serial.available() >= 48){ //checks if new data avaiable 
      state = 'A';
    }
  }

//*********Read and sort serial data*********
  if(state == 'A'){
    state = Serial.read(); //first character is state

    //Read serial and store in parameters
    byte index[] = {6,6,6,3,3,4,0,3,3,3}; // groups rest of chars into groups of (7, 7, 7, 4, 4, 5, 1, 4, 4, 4) which completes the other 47
    char serial_data[8];
    for(byte i=0; i<10; i++){
      for(int j=0; j<=index[i]; j++){
        serial_data[j] = Serial.read();
      }
      parameters[i] = atol(serial_data); //converts "1234" into 1234 for example and stores in parameters
      memset(serial_data,0,sizeof(serial_data)); //clears array to make every value null
    }
    
    //Clear serial buffer
    while(Serial.available()>0){
      byte x = Serial.read();
    }

    lcd.clear();
    
    //Zero the DAC for the AMIGO pulse
    if(state == '8' && circuit_on == true){
      DACwrite(0);
    }
    LCD_counter = 0.0;
    current_sum = 0.0;
    previous_error = 0.0;
    sum_error = 0.0;
    integralTerm = 0.0;
    t0 = millis();
    t1 = millis();
  }

}
