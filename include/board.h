/*
 * Author: Augustin.B
 * Date  : 02.04.2022
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <avr/io.h>

// Spindle control input, comes from controller running GRBL,
// We convert the read value to an RPM target value for the velocity controller.
#define SPINDLE_IN_PORT     PORTA
#define SPINDLE_IN_PIN      2

// Spindle tachometer input,
// Takes a single channel squre signal with the frequency of spindle rotation.
// This is used as feedback for the velocity controllers.
#define TACHO_IN_PORT       PORTA
#define TACHO_IN_PIN        3

// Power Supply Voltage Measurement,
// Connected to power supply through a 1:10 voltage divider.
// Used to monitor the power supply volate and issue warnings to user through a panel LED.
// This pin is used as analog input.
#define PWR_MEAS_PORT       PORTD
#define PWR_MEAS_PIN        4

// TMC2209 UART, with UART1
// These are connected to a single wire uart line down to the TMC2209 drivers on the main GRBL controller.
// Default driver addresses are X = 0, Y = 1, Z = 2, Dual (X) = 3.
// Main GRBL controller still controls the enable pin of these drivers which are disabled by default, pull down to enable.
#define DRV_TX_PORT         PORTC
#define DRV_TX_PIN          4
#define DRV_RX_PORT         PORTC
#define DRV_RX_PIN          5

// Serial -> USB communication, with UART0
// Used either with arduino every, or with UART to USB dongle.
#define SER2USB_TX_PORT     PORTB
#define SER2USB_TX_PIN      4
#define SER2USB_RX_PORT     PORTB
#define SER2USB_RX_PIN      5

// Spindle control ouputs, goes from tthe coprocessor board to the spindle driver.
// Using a simple DC driver with a PWM and Direction inputs.
#define SPINDLE_PWM_PORT    PORTA
#define SPINDLE_PWM_PIN     0
#define SPINDLE_DIR_PORT    PORTF
#define SPINDLE_DIR_PIN     5

// Endstop inputs, used to automatically test and tune sensorless stall detection on the TMC2209s.
// As with GRBL, the DUAL (X) axis shares its endtop line with the Z axis.
#define ENDSTOP_X_PORT      PORTC
#define ENDSTOP_X_PIN       6
#define ENDSTOP_Y_PORT      PORTB
#define ENDSTOP_Y_PIN       2
#define ENDSTOP_ZD_PORT     PORTF
#define ENDSTOP_ZD_PIN      4

// General purpose header, for panel LEDs, or other prototyping purposes.
#define GP_D7_PORT          PORTA
#define GP_D7_PIN           1
#define GP_D8_PORT          PORTE
#define GP_D8_PIN           3
#define GP_D9_PORT          PORTB
#define GP_D9_PIN           0
#define GP_D10_PORT         PORTB
#define GP_D10_PIN          1

// Builtin LED
#define LEDBUILTIN_PORT     PORTE
#define LEDBUILTIN_PIN      2

#endif /* BOARD_H_ */