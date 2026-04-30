#include "mbed.h"
#include <cmath>
#include <cstdio>

PwmOut buck_pwm(PB_13); 
AnalogIn Vbatt(PA_7);         
AnalogIn Vout(PA_6);          
AnalogIn Current_sense(PC_4); 
DigitalOut status_led(LED1); 

// --- I2C & INA219 SETUP ---
I2C i2c(PA_10, PA_9);         
const int INA219_ADDR = (0x40 << 1); 

void ina219_init() {
    char config_data[3] = {0x00, 0x39, 0x9F};
    i2c.write(INA219_ADDR, config_data, 3);
    
    char cal_data[3] = {0x05, 0x10, 0x00}; 
    i2c.write(INA219_ADDR, cal_data, 3);
}

float ina219_readBusVoltage() {
    char reg = 0x02;
    char data[2];
    i2c.write(INA219_ADDR, &reg, 1, true); 
    i2c.read(INA219_ADDR, data, 2);
    uint16_t value = (data[0] << 8) | data[1]; 
    value >>= 3; 
    return value * 0.004f; 
}

float ina219_readCurrent() {
    char reg = 0x04;
    char data[2];
    i2c.write(INA219_ADDR, &reg, 1, true); 
    i2c.read(INA219_ADDR, data, 2);
    int16_t value = (data[0] << 8) | data[1];
    return value * 0.0001f; 
}

// --- STATE MACHINE ---
enum SystemState {
    SYSTEM_OFF,
    SYSTEM_PROBE,
    MPPT_CC_MODE,
    MPPT_CV_MODE
};
SystemState current_state = SYSTEM_OFF;

// --- CONTROL PARAMETERS ---
float CHARGING_CURRENT = 0.1f;  
const float CURRENT_DEAD_ZONE = 0.02f; 
const float VOLTAGE_DEAD_ZONE = 0.05f;    

// PI CONTROL TUNING
const float Kp_I = 0.01f;     
const float Ki_I = 0.005f;    
const float Kp_V = 0.05f;     
const float Ki_V = 0.0005f;   

// --- MPPT P&O PARAMETERS ---
float prev_power = 0.0f;
bool increasing_current = true;
const float PANEL_UVLO_VOLTAGE = 10.0f; 
const float MAX_CURRENT = 3.0f;
// VARIABLE STEP SIZE CONSTANTS
const float MPPT_K_STEP = 0.2f;    
const float MPPT_MAX_STEP = 0.15f; 
const float MPPT_MIN_STEP = 0.01f; 
// ------- VARIABLES ---------
float duty_cycle = 0.0f;
float integral_sum = 0.0f;    
float Vbatt_V = 0.0f;         
float Vout_V = 0.0f;          
float Iout_A = 0.0f;
int loop_counter = 0;
const float ALPHA = 0.1f;     

// ---------------- UART ----------------
BufferedSerial hm10(PC_10, PC_11, 9600); 
BufferedSerial pc(USBTX, USBRX, 9600);

int main() {
    hm10.set_blocking(false);
    hm10.write("Buck MPPT Controller Ready. Autonomous Mode Active.\r\n", 53);
    
    i2c.frequency(400000); 
    ina219_init();
    
    buck_pwm.period_us(20); 
    buck_pwm.write(1.0f); 
 
    while (true) {
        
        while(hm10.readable()) { char c; hm10.read(&c, 1); }

        // ---------- 1. SENSOR READING WITH EMWA FILTER ----------
        float instant_vbatt_adc = Vbatt.read();
        float instant_vbatt = instant_vbatt_adc * 3.3f * 8.5f; 
        Vbatt_V = (ALPHA * instant_vbatt) + ((1.0f - ALPHA) * Vbatt_V); 

        float instant_vout_adc = Vout.read();
        float instant_vout = instant_vout_adc * 3.3f * 8.5f; 
        Vout_V = (ALPHA * instant_vout) + ((1.0f - ALPHA) * Vout_V); 
        
        float instant_i_adc = Current_sense.read();
        float instant_current = instant_i_adc * 3.3f * 1.0f; 
        Iout_A = (ALPHA * instant_current) + ((1.0f - ALPHA) * Iout_A);

        // ---------- 2. STATE MACHINE ----------
        switch (current_state) {
            
            case SYSTEM_OFF:
            {
                buck_pwm.write(1.0f); 
                duty_cycle = 0.0f;
                integral_sum = 0.0f;
                CHARGING_CURRENT = 0.1f; 

                // Autonomous Wake-Up Routine (Check every 5 seconds)
                if (loop_counter % 5000 == 0) {
                    float panel_v = ina219_readBusVoltage();
                    if (panel_v > Vbatt_V + 1.5f) {
                        int panel_mV = (int)(panel_v * 1000.0f); 
                        char msg[100];                         
                        sprintf(msg, "Sunlight detected (%d.%03d V). Probing for power...\r\n", 
                                panel_mV / 1000, abs(panel_mV % 1000));
                        hm10.write(msg, strlen(msg));            
                        current_state = SYSTEM_PROBE;
                    }
                }
                break;
            }

            case SYSTEM_PROBE:
            {
                buck_pwm.write(0.95f); 
                thread_sleep_for(10); 
                
                float load_voltage = ina219_readBusVoltage();
                
                if (load_voltage > 14.0f) {
                    integral_sum = (Vbatt_V / load_voltage) - 0.02f; 
                    if (integral_sum < 0.0f) integral_sum = 0.0f;
                    
                    hm10.write("Probe Passed! Entering MPPT CC Mode.\r\n", 38);
                    current_state = MPPT_CC_MODE;
                } else {
                    buck_pwm.write(1.0f); 
                    hm10.write("Ghost Voltage GO OUTSIDE! System powering off.\r\n", 48);
                    current_state = SYSTEM_OFF; 
                }
                break;
            }
                
            case MPPT_CC_MODE:
            {
                if (loop_counter % 100 == 0) {
                    float panel_v = ina219_readBusVoltage();
                    
                    // Live undervoltage tracking
                    if (panel_v < Vbatt_V + 0.2f) {
                        hm10.write("Panel voltage too low. System powering off.\r\n", 45);
                        current_state = SYSTEM_OFF;
                        break; 
                    }

                    // Transition to CV Mode
                    if (Vbatt_V >= 16.8f) {
                        hm10.write("Target Voltage Reached. Entering CV Mode.\r\n", 43);
                        current_state = MPPT_CV_MODE;
                        break;
                    }

                    // --- STANDARD MPPT TRACKING ---
                    float panel_i = ina219_readCurrent();
                    float current_power = panel_v * panel_i;
                    float delta_P = current_power - prev_power;
                    
                    float dynamic_step = fabs(delta_P) * MPPT_K_STEP;
                    if (dynamic_step > MPPT_MAX_STEP) dynamic_step = MPPT_MAX_STEP; 
                    if (dynamic_step < MPPT_MIN_STEP) dynamic_step = MPPT_MIN_STEP;
                    
                    if (delta_P < -0.05f) { 
                        increasing_current = !increasing_current;
                    }
                    
                    if (increasing_current) {
                        CHARGING_CURRENT += dynamic_step;
                    } else {
                        CHARGING_CURRENT -= dynamic_step;
                    }
                    
                    if (CHARGING_CURRENT > MAX_CURRENT) CHARGING_CURRENT = MAX_CURRENT; 
                    if (CHARGING_CURRENT < 0.1f) CHARGING_CURRENT = 0.1f; 
                    
                    prev_power = current_power;
                }

                // --- INNER LOOP: PI CONSTANT CURRENT MATH ---
                float error = CHARGING_CURRENT - Iout_A;
                if (fabs(error) < CURRENT_DEAD_ZONE) {
                    error = 0.0f; 
                }

                integral_sum += (error * Ki_I);
                if (integral_sum > 0.95f) integral_sum = 0.95f;
                if (integral_sum < 0.0f) integral_sum = 0.0f;

                duty_cycle = (error * Kp_I) + integral_sum;

                if (duty_cycle > 0.97f) duty_cycle = 0.97f;
                if (duty_cycle < 0.03f) duty_cycle = 0.03f;

                buck_pwm.write(1.0f - duty_cycle); 
                break;
            }
                
            case MPPT_CV_MODE:
            {
                if (loop_counter % 100 == 0) {
                    float panel_v = ina219_readBusVoltage();
                    
                    // Live undervoltage tracking
                    if (panel_v < Vbatt_V + 0.2f) {
                        hm10.write("Panel voltage too low. System powering off.\r\n", 45);
                        current_state = SYSTEM_OFF;
                        break; 
                    }
                }

                // Hysteresis fallback to CC mode
                if (Vbatt_V < 16.0f) {
                    hm10.write("Battery voltage dropped. Return to CC charging.\r\n", 49);
                    current_state = MPPT_CC_MODE;
                    break;
                }

                float error = 16.8f - Vout_V;
                
                // Apply Dead Zone
                if (fabs(error) < VOLTAGE_DEAD_ZONE) {
                    error = 0.0f;
                }

                // Tail Current Cutoff
                if (Iout_A < 0.15f)  { 
                    hm10.write("Battery FULL. System powering off.\r\n", 36);
                    current_state = SYSTEM_OFF;
                    break; 
                } 

                // --- INNER LOOP: PI CONSTANT VOLTAGE MATH ---
                integral_sum += (error * Ki_V);
                
                // Anti-windup 
                if (integral_sum > 0.95f) integral_sum = 0.95f;
                if (integral_sum < 0.0f) integral_sum = 0.0f;

                duty_cycle = (error * Kp_V) + integral_sum;

                // Hardware safety clamps
                if (duty_cycle > 0.97f) duty_cycle = 0.97f;
                if (duty_cycle < 0.03f) duty_cycle = 0.03f;

                buck_pwm.write(1.0f - duty_cycle); 
                break;
            }
        }

        // ---------- 3. STATUS----------
        if (current_state == MPPT_CC_MODE) {
            status_led = 1; 
        } else {
            if ((loop_counter % 500) == 0){
                status_led = !status_led; 
            }
        }

        // ---------- 4. AUTO TELEMETRY STREAM (Every 1.5 Seconds) ----------
        if ((loop_counter % 1500) == 0) {
            int state_num = current_state;
            float p_v = ina219_readBusVoltage();
            float p_i = ina219_readCurrent();
            
            // Convert floats to integer millivolts/milliamps
            int pv_mV    = (int)(p_v * 1000.0f);
            int pi_mA    = (int)(p_i * 1000.0f);
            int vout_mV  = (int)(Vout_V * 1000.0f);
            int vbatt_mV = (int)(Vbatt_V * 1000.0f);
            int iout_mA  = (int)(Iout_A * 1000.0f);
            
            char t_msg[128];
            // Format: STATE, Vpanel, Ipanel, Vout, Vbatt, Iout, Duty
            sprintf(t_msg, "%d,%d.%03d,%d.%03d,%d.%03d,%d.%03d,%d.%03d,%d\r\n", 
                    state_num, 
                    pv_mV / 1000, abs(pv_mV % 1000),
                    pi_mA / 1000, abs(pi_mA % 1000),
                    vout_mV / 1000, abs(vout_mV % 1000),
                    vbatt_mV / 1000, abs(vbatt_mV % 1000),
                    iout_mA / 1000, abs(iout_mA % 1000),
                    (int)(duty_cycle * 100.0f));
                    
            hm10.write(t_msg, strlen(t_msg));
            pc.write(t_msg, strlen(t_msg));
        }

        thread_sleep_for(1); 
        // Loop counter for modulus operation
        loop_counter++;
    }
}