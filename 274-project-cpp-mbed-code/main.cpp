#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"


// to change each time I run this:
// horizon, dt
#define N 4
#define horizon 9
#define NUM_INPUTS 14 + N * (horizon - 2) * 4
#define NUM_OUTPUTS 15
float dt = 0.4;

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); //initialize the motor shield with a period of 24000 ticks or ~10kHZ
Ticker currentLoop;

// Variables for q1
float current1;
float current_des1 = 0;
float prev_current_des1 = 0;
float current_int1 = 0;
float th1;
float dth1;
float duty_cycle1;
float angle1_init;
float th1_des; 
float dth1_des; 
float tau1_ff;

// Variables for q2
float current2;
float current_des2 = 0;
float prev_current_des2 = 0;
float current_int2 = 0;
float th2;
float dth2;
float duty_cycle2;
float angle2_init;
float th2_des; 
float dth2_des;
float tau2_ff;

float t_start; 

// Fixed kinematic parameters
const float l_OA=.113; 
const float l_AB=.116; 

// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float impedance_control_period_us = 1000.0f;  // 1kHz impedance control loop
float start_period, traj_period, end_period;

// Control parameters
float current_Kp;         
float current_Ki;           
float current_int_max = 3.0f;       
float duty_max1;
float duty_max2;       
float K_1; 
float K_2; 
float D_1; 
float D_2; 

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction

float inputs[NUM_INPUTS];


void update(){
    //take encoder measurements
    th1 = encoderA.getPulses() *PULSE_TO_RAD + angle1_init;       
    th2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;       
    current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    dth1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity 
    current2 = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f)-15.0f);           // measure current
    dth2 = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity         
}
void CurrentLoop() {
    // Current control interrupt function

    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
    update(); 

    float err_c1 = current_des1 - current1;                                             // current errror
    current_int1 += err_c1;                                                             // integrate error
    current_int1 = fmaxf( fminf(current_int1, current_int_max), -current_int_max);      // anti-windup
    float ff1 = R*current_des1 + k_t*dth1;                                         // feedforward terms
    duty_cycle1 = (ff1 + current_Kp*err_c1 + current_Ki*current_int1)/supply_voltage;   // PI current controller
    float absDuty1 = abs(duty_cycle1);
    if (absDuty1 > duty_max1) {
        duty_cycle1 *= duty_max1 / absDuty1;
        absDuty1 = duty_max1;
    }    
    if (duty_cycle1 < 0) { // backwards
        motorShield.motorAWrite(absDuty1, 1);
    } else { // forwards
        motorShield.motorAWrite(absDuty1, 0);
    }             
    prev_current_des1 = current_des1; 
    
    float err_c2 = current_des2 - current2;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max), -current_int_max);      // anti-windup   
    float ff2 = R*current_des2 + k_t*dth2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    float absDuty2 = abs(duty_cycle2);
    if (absDuty2 > duty_max2) {
        duty_cycle2 *= duty_max2 / absDuty2;
        absDuty2 = duty_max2;
    }    
    if (duty_cycle2 < 0) { // backwards
        motorShield.motorBWrite(absDuty2, 1);
    } else { // forwards
        motorShield.motorBWrite(absDuty2, 0);
    }             
    prev_current_des2 = current_des2; 
    
}

float evaluate_poly(int start_ind){
    float t_live = t - start_period - t_start; 
    float t_mod = fmod(t_live, dt);

    float increment = t_live / dt;
    int increment_step = int(increment);
    int increment_step_4 = 4 * increment_step;
    float val = 0.0; 
    int exp = N-1;
    for (int i=0; i<N; i++){
        val = val + inputs[start_ind+i+increment_step_4]*pow(t_mod, exp); 
        exp = exp-1; 
    }
    return val;
}

void updateParams() {
        th1_des  = evaluate_poly(13);
        dth1_des  = evaluate_poly(13 + N * (horizon - 2) * 1); 
        th2_des = -evaluate_poly(13 + N * (horizon - 2)  * 2);   
        dth2_des = evaluate_poly(13 + N * (horizon - 2) * 3);
        // tau1_ff = evaluate_poly(13 + N * (horizon - 2) * 4);
        // tau2_ff = evaluate_poly(13 + N * (horizon - 2) * 5);      
}

void setup() {
        // Get inputs from MATLAB          
        update(); 
        // Setup experiment
        t.reset();
        t.start();
        encoderA.reset();
        encoderB.reset();
        encoderC.reset();
        encoderD.reset();

        motorShield.motorAWrite(0, 0); //turn motor A off
        motorShield.motorBWrite(0, 0); //turn motor B off
        // Attach current loop interrupt
        currentLoop.attach_us(CurrentLoop,current_control_period_us);
        
        start_period  = inputs[0];    // First buffer time, before trajectory
        traj_period   = inputs[1];    // Trajectory time/length
        end_period    = inputs[2];    // Second buffer time, after trajectory

        angle1_init   = inputs[3];    // Initial angle for q1 (rad)
        angle2_init   = inputs[4];    // Initial angle for q2 (rad)

        duty_max1     = inputs[9];   // Maximum duty factor
        duty_max2     = inputs[10]; 

        current_Kp    = inputs[11]; 
        current_Ki    = inputs[12]; 

        th1_des       = angle1_init;  
        th2_des       = angle2_init; 
        dth1_des      = 0.0; 
        dth2_des      = 0.0; 

}
void setGains(){
    // Set gains based on buffer and traj times
    if (t < start_period) {
        if (K_1 > 0 || K_2 > 0) {
            K_2 = 1; // for joint space control, set this to 1; for Cartesian space control, set this to 50
            D_1 = 1; // for joint space control, set this to 1; for Cartesian space control, set this to 50
            D_2 = 1;  // for joint space control, set this to 0.1; for Cartesian space control, set this to                     }
    }}
    else if (t < start_period + traj_period) {
        K_1  = inputs[5];    // Foot stiffness N/m
        K_2  = inputs[6];    // Foot stiffness N/m
        D_1  = inputs[7];    // Foot stiffness N/m
        D_2  = inputs[8];    // Foot damping N/(m/s)
    }
}
void setCurrents(){
    // calculate error
    float e_1 = th1_des-th1;
    float e_2 = th2_des-th2;
    float de_1 = dth1_des - dth1;
    float de_2 = dth2_des - dth2;

    // calculate desired torques
    float T1 = K_1 * e_1 + D_1 * de_1 + tau1_ff;

    float T2 = K_2 * e_2 + D_2 * de_2 + tau2_ff;
                
    // Set desired currents             
    current_des1 = T1/k_t;            
    current_des2 = T2/k_t;     
}
void sendData(){   
    // Form output to send to MATLAB     
    float output_data[NUM_OUTPUTS];
    // current time
    output_data[0] = t.read();
    // motor 1 state
    output_data[1] = th1;
    output_data[2] = dth1;  
    output_data[3] = current1;
    output_data[4] = current_des1;
    output_data[5] = duty_cycle1;
    // motor 2 state
    output_data[6] = th2;
    output_data[7] = dth2;
    output_data[8] = current2;
    output_data[9] = current_des2;
    output_data[10]= duty_cycle2;
    
    output_data[11] = th1_des; 
    output_data[12] = dth1_des; 
    output_data[13] = th2_des; 
    output_data[14] = dth2_des; 
    
    // Send data to MATLAB
    server.sendData(output_data,NUM_OUTPUTS);
}
void cleanUp(){
    // Cleanup after experiment
        pc.printf("experiment over");
        server.setExperimentComplete();
        currentLoop.detach();
        motorShield.motorAWrite(0, 0); //turn motor A off
        motorShield.motorBWrite(0, 0); //turn motor B off
}

int main(void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    while(1){
        // If there are new inputs, this code will run
        if (server.getParams(inputs,NUM_INPUTS)) {     
            setup();    
            pc.printf("started experiment");
            t_start = t.read(); 
            while( t.read()-t_start < start_period + traj_period + end_period) {  
                updateParams(); 
                setGains(); 
                setCurrents(); 
                sendData(); 
                wait_us(impedance_control_period_us);   
            } //end for loop
            cleanUp(); 
        } // end if
    } // end while
} // end main

