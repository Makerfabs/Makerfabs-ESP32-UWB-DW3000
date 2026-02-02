#pragma once

// Default pins of the NRF52 microcontroller used to connect to the UWB DWM3001 module


/* Interface  SPI utilisé dans DWM300CPK (SPIM3) */
#ifndef DW3000_SPI_INSTANCE
#define DW3000_SPI_INSTANCE 3
#endif

/* Signal d'horloge SPI. */
#ifndef DW3000_SPI_SCK_PIN
#define DW3000_SPI_SCK_PIN 8
#endif

/* Master out Slave in (data from MCU to DW3000) */
#ifndef DW3000_SPI_MOSI_PIN
#define DW3000_SPI_MOSI_PIN 9
#endif

/*Master in Sliave out (data from DW3000 to MCU)  */
#ifndef DW3000_SPI_MISO_PIN
#define DW3000_SPI_MISO_PIN 10
#endif

/* Sélection de la puce SPI */
#ifndef DW3000_CS_PIN
#define DW3000_CS_PIN 7
#endif

/* RESET */
#ifndef DW3000_RST_PIN
#define DW3000_RST_PIN 11
#endif

/* Interruptions */
#ifndef DW3000_IRQ_PIN
#define DW3000_IRQ_PIN 12
#endif

/* wake-up */
#ifndef DW3000_WAKE_PIN
#define DW3000_WAKE_PIN 13
#endif

/* Optional GPIOs for external PA/LNA enables (set to -1 if unused). */
/*
#ifndef DW3000_LNA_PIN
#define DW3000_LNA_PIN -1
#endif

#ifndef DW3000_PA_PIN
#define DW3000_PA_PIN -1
#endif
*/
