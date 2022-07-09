////////////////////////////////////////////
// CONTROL BOARD PIN OUT
////////////////////////////////////////////
// Only change if you using a different PCB
////////////////////////////////////////////

#ifdef __AVR_ATmega2560__
// AVR Mega platform
#define PWM_INPUT_PIN           2
#define PWM_OUTPUT_PIN          3

#define PIN_ENCODER_A           4
#define PIN_ENCODER_B           5

#define BUTTON_IN               8
#define BUTTON_LEFT             9
#define BUTTON_UP               12
#define BUTTON_RIGHT            11
#define BUTTON_DOWN             10

#define DOUT1_PIN               22
#define DOUT2_PIN               23
#define DOUT3_PIN               24
#define DOUT4_PIN               25
#define DOUT5_PIN               26
#define DOUT6_PIN               27
#define DOUT7_PIN               28
#define DOUT8_PIN               29

#define SENSOR1_PIN             A0
#define SENSOR2_PIN             A1
#define SENSOR3_PIN             A2
#define SENSOR4_PIN             A3
#define SENSOR5_PIN             A4
#define SENSOR6_PIN             A5
#define SENSOR7_PIN             A6
#define SENSOR8_PIN             A7
#define SENSOR9_PIN             A8
#define SENSOROUT_PIN           A9

#define MARC_SERIAL             Serial3

#elif defined(AMIDALA_AUTOMATION_PCB)

#define PWM_INPUT_PIN           19
#define PWM_OUTPUT_PIN          18

#define PIN_ENCODER_A           36
#define PIN_ENCODER_B           39

#define DOUT4_PIN               33
#define DOUT5_PIN               25
#define DOUT6_PIN               26
#define DOUT7_PIN               27
#define DOUT8_PIN               23

#define RXD1_PIN                34
#define RXD2_PIN                17
#define TXD2_PIN                16
#define RXD3_PIN                32
#define TXD3_PIN                4

#define EXPAND_INT_PIN          13
#define PPMIN_RC_PIN            14

///////////////////////////////////
// GPIO Expander Pins

#define USE_I2C_GPIO_EXPANDER
#define GPIO_PIN_BASE           200

// Pin numbers greater equal GPIO_PIN_BASE sent to GPIO expander
#define BUTTON_IN               GPIO_PIN_BASE+0   /* SW1 */
#define BUTTON_LEFT             GPIO_PIN_BASE+1   /* SW2 */
#define BUTTON_UP               GPIO_PIN_BASE+2   /* SW3 */
#define BUTTON_RIGHT            GPIO_PIN_BASE+3   /* SW4 */
#define BUTTON_DOWN             GPIO_PIN_BASE+4   /* SW5 */

#define DOUT1_PIN          		GPIO_PIN_BASE+5
#define DOUT2_PIN          		GPIO_PIN_BASE+6
#define DOUT3_PIN				GPIO_PIN_BASE+7

#elif defined(LILYGO_MINI32)

#define PIN_ENCODER_A           2
#define PIN_ENCODER_B           4

#define BUTTON_IN               33   /* SW1 */
#define BUTTON_LEFT             25   /* SW2 */
#define BUTTON_UP               26   /* SW3 */
#define BUTTON_RIGHT            27   /* SW4 */
#define BUTTON_DOWN             23   /* SW5 */

#define RXD1_PIN                34
#define RXD2_PIN                17
#define TXD2_PIN                16
#define RXD3_PIN                32
#define TXD3_PIN                0

#define DOUT1_PIN          		5
#define DOUT2_PIN          		13
#define DOUT3_PIN				14
#define DOUT4_PIN				18
#define DOUT5_PIN				19

#endif
