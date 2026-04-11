/* PIN Definition - Updated for Teensy 4.1 + BLE Bridge */

// CAN bus
#define CAN_TX_PIN 3        
#define CAN_RX_PIN 4        

// Motor control
#define BW_PIN 16           
#define HOME_SWITCH_PIN 1   
#define FW_PIN 15           
#define CONTROL_MODE_PIN 17 
#define PWM_PIN 6           
#define DIR_PIN 27          
#define ENABLE_PIN 5        
#define FW_SWITCH_PIN 23    
#define BW_SWITCH_PIN 22    

// ESCON Feedback
#define ESCON_AN1 31        
#define ESCON_AN2 21        
#define ESCON_BREAK 28 

// TEMPERATURE Feedback
#define FEEDBACK_TEMP 18        

// led pin
#define LED 13              // Note: This will flicker when Bluetooth data arrives

// AS5045 Encoder (MOVED TO AVOID SPI SLAVE CONFLICT)
#define AS5045_CLK_PIN 14  
#define AS5045_CS_PIN 20   
#define AS5045_DATA_PIN 1   // Moved from 12 to 1

// SPI pin map for Straingauge torque (MOVED TO AVOID SPI SLAVE CONFLICT)
#define TORQUE_CLK_PIN 19   // Moved from 13 to 19
#define TORQUE_CS_PIN 9    
#define TORQUE_DATA_PIN 0   // Moved from 11 to 0