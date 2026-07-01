/*
 * Board defaults for the DW3000 Arduino port.
 *
 * Define any DW3000_PIN_* macro before including dw3000.h, or pass it as a
 * compiler flag, to override these defaults for a custom board.
 */

#ifndef DW3000_BOARD_CONFIG_H_
#define DW3000_BOARD_CONFIG_H_

#if defined(ARDUINO_ARCH_NRF52)

#ifndef DW3000_PIN_RST
#define DW3000_PIN_RST 14
#endif

#ifndef DW3000_PIN_IRQ
#define DW3000_PIN_IRQ 12
#endif

#ifndef DW3000_PIN_CS
#define DW3000_PIN_CS 7
#endif

#ifndef DW3000_PIN_SCK
#define DW3000_PIN_SCK 5
#endif

#ifndef DW3000_PIN_MOSI
#define DW3000_PIN_MOSI 9
#endif

#ifndef DW3000_PIN_MISO
#define DW3000_PIN_MISO 10
#endif

#else

#ifndef DW3000_PIN_RST
#define DW3000_PIN_RST 27
#endif

#ifndef DW3000_PIN_IRQ
#define DW3000_PIN_IRQ 34
#endif

#ifndef DW3000_PIN_CS
#define DW3000_PIN_CS 4
#endif

#endif

#endif /* DW3000_BOARD_CONFIG_H_ */
