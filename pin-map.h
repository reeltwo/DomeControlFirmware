////////////////////////////////////////////
// CONTROL BOARD PIN OUT
////////////////////////////////////////////
// Only change if you using a different PCB
////////////////////////////////////////////

#ifdef __AVR_ATmega2560__
// AVR Mega platform
#define PWM_INPUT_PIN           2    // PWM Input Pin (Stealth or other PWM based dome control)
#define PWM_OUTPUT_PIN          3    // PWM Output Pin (PWM Syren controller) - Not supported yet

#define PIN_ENCODER_A           4    // Rotary encoder pin A
#define PIN_ENCODER_B           5    // Rotary encoder pin B

#define BUTTON_IN               8    // Rotary encoder button SW1
#define BUTTON_LEFT             9    // Rotary encoder button SW2
#define BUTTON_UP               12   // Rotary encoder button SW3
#define BUTTON_RIGHT            11   // Rotary encoder button SW4
#define BUTTON_DOWN             10   // Rotary encoder button SW5

#define DOUT1_PIN               22	 // Digital Output pin 1
#define DOUT2_PIN               23   // Digital Output pin 2
#define DOUT3_PIN               24   // Digital Output pin 3
#define DOUT4_PIN               25   // Digital Output pin 4
#define DOUT5_PIN               26   // Digital Output pin 5
#define DOUT6_PIN               27   // Digital Output pin 6
#define DOUT7_PIN               28   // Digital Output pin 7
#define DOUT8_PIN               29   // Digital Output pin 8

#define SENSOR1_PIN             A0   // Optical Sensor Input Pin 1
#define SENSOR2_PIN             A1   // Optical Sensor Input Pin 2
#define SENSOR3_PIN             A2   // Optical Sensor Input Pin 3
#define SENSOR4_PIN             A3   // Optical Sensor Input Pin 4
#define SENSOR5_PIN             A4   // Optical Sensor Input Pin 5
#define SENSOR6_PIN             A5   // Optical Sensor Input Pin 6
#define SENSOR7_PIN             A6   // Optical Sensor Input Pin 7
#define SENSOR8_PIN             A7   // Optical Sensor Input Pin 8
#define SENSOR9_PIN             A8   // Optical Sensor Input Pin 9
#define SENSOROUT_PIN           A9   // Analog Sensor Output Pin

#define COMMAND_SERIAL          Serial3  // Use HardwareSerial3 for Serial commands
#define COMMAND_SERIAL_READ     Serial3  // Use HardwareSerial3 for Serial commands
#define COMMAND_SERIAL_WRITE    Serial3  // Use HardwareSerial3 for Serial commands

#elif defined(ROAM_A_DOME_COMPACT_PCB)

#define PWM_INPUT_PIN           19   // PWM Input Pin (Stealth or other PWM based dome control)
#define PWM_OUTPUT_PIN          18   // PWM Output Pin (PWM Syren controller) - Not supported yet

#define DOUT1_PIN               33   // Digital Output pin 1
#define DOUT2_PIN               25   // Digital Output pin 2
#define DOUT3_PIN               26   // Digital Output pin 3
#define DOUT4_PIN               27   // Digital Output pin 4
#define DOUT5_PIN               23   // Digital Output pin 5

#define RXD1_PIN                34   // Dome Sensor Ring Input Pin
#define TXD1_PIN                0    // unused
//#define RXD2_PIN                17   // Syren input from droid controller (Shadow/Padawan)
#define RXD2_PIN                32   // Syren input from droid controller (Shadow/Padawan)
#define TXD2_PIN                16   // Syren output from Roam-a-dome
//#define RXD3_PIN                32   // Command Serial input (receive)
#define RXD3_PIN                17   // Command Serial input (receive)
#define TXD3_PIN                4    // Command Serial output (send)

#define EXPAND_INT_PIN          13   // GPIO Expander interrupt pin
#define PPMIN_RC_PIN            14   // PPM RC Input Pin

#define STATUSLED_PIN           5    // LED Status

#define COMMAND_SERIAL          Serial2  // Use HardwareSerial2 for Serial commands
#define COMMAND_SERIAL_READ     driveSerial
#define COMMAND_SERIAL_WRITE    Serial2  // Use HardwareSerial2 for Serial commands

#elif defined(ROAM_A_DOME_FULLSIZE_PCB)

#define PWM_INPUT_PIN           19   // PWM Input Pin (Stealth or other PWM based dome control)
#define PWM_OUTPUT_PIN          18   // PWM Output Pin (PWM Syren controller) - Not supported yet

#define PIN_ENCODER_A           36   // Rotary encoder pin A
#define PIN_ENCODER_B           39   // Rotary encoder pin B

#define DOUT4_PIN               33   // Digital Output pin 4
#define DOUT5_PIN               25   // Digital Output pin 5
#define DOUT6_PIN               26   // Digital Output pin 6
#define DOUT7_PIN               27   // Digital Output pin 7
#define DOUT8_PIN               23   // Digital Output pin 8

#define RXD1_PIN                34	 // Dome Sensor Ring Input Pin
#define TXD1_PIN                0	 // unused
#define RXD2_PIN                17	 // Syren input from droid controller (Shadow/Padawan)
#define TXD2_PIN                16   // Syren output from Roam-a-dome
#define RXD3_PIN                32   // Command Serial input (receive)
#define TXD3_PIN                4    // Command Serial output (send)

#define EXPAND_INT_PIN          13   // GPIO Expander interrupt pin
#define PPMIN_RC_PIN            14   // PPM RC Input Pin

///////////////////////////////////
// GPIO Expander Pins

#define USE_I2C_GPIO_EXPANDER
#define GPIO_PIN_BASE           200  // Pin number base for GPIO pins

// Pin numbers greater equal GPIO_PIN_BASE sent to GPIO expander
#define BUTTON_IN               GPIO_PIN_BASE+0   // Rotary encoder button SW1
#define BUTTON_LEFT             GPIO_PIN_BASE+1   // Rotary encoder button SW2
#define BUTTON_UP               GPIO_PIN_BASE+2   // Rotary encoder button SW3
#define BUTTON_RIGHT            GPIO_PIN_BASE+3   // Rotary encoder button SW4
#define BUTTON_DOWN             GPIO_PIN_BASE+4   // Rotary encoder button SW5

#define DOUT1_PIN          		GPIO_PIN_BASE+5   // Digital Output pin 1
#define DOUT2_PIN          		GPIO_PIN_BASE+6   // Digital Output pin 2
#define DOUT3_PIN				GPIO_PIN_BASE+7   // Digital Output pin 3

#define COMMAND_SERIAL          Serial2  // Use HardwareSerial2 for Serial commands
#define COMMAND_SERIAL_READ     driveSerial
#define COMMAND_SERIAL_WRITE    Serial2  // Use HardwareSerial2 for Serial commands

#elif defined(LILYGO_MINI32)

#define PWM_INPUT_PIN           36   // PWM Input Pin (Stealth or other PWM based dome control)

#ifdef USE_LCD_SCREEN
 #define PIN_ENCODER_A          2    // Rotary encoder pin A
 #define PIN_ENCODER_B          4    // Rotary encoder pin A

 #define BUTTON_IN              33   // Rotary encoder button SW1
 #define BUTTON_LEFT            25   // Rotary encoder button SW2
 #define BUTTON_UP              26   // Rotary encoder button SW3
 #define BUTTON_RIGHT           27   // Rotary encoder button SW4
 #define BUTTON_DOWN            23   // Rotary encoder button SW5
#endif

#define RXD1_PIN                34   // Dome Sensor Ring Input Pin
#define TXD1_PIN                39   // unused
#define RXD2_PIN                17   // Syren input from droid controller (Shadow/Padawan)
#define TXD2_PIN                16   // Syren output from Roam-a-dome
#define RXD3_PIN                32   // Command Serial input (receive)
#define TXD3_PIN                0    // Command Serial output (send)

#define DOUT1_PIN          		5
#define DOUT2_PIN          		13
#define DOUT3_PIN				14
#define DOUT4_PIN				18
#define DOUT5_PIN				19

#define COMMAND_SERIAL          Serial2  // Use HardwareSerial2 for Serial commands
#define COMMAND_SERIAL_READ     driveSerial
#define COMMAND_SERIAL_WRITE    Serial2  // Use HardwareSerial2 for Serial commands

#endif
