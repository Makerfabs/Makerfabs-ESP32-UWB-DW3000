#pragma once

/*
J'ai utilisé la même logique et les mêmes formules que pour le DWM1001 afin de trouver et d'identifier l'identifiant de la carte.

1- La commande « board get dw mac » reconstruit l'adresse MAC 64 bits du module.

2- La commande « board get cpu id » permet de déduire l'identifiant unique du microcontrôleur.

3- La commande « board get shoprt address » crée une adresse courte de 16 bits à partir des 16 derniers bits de l'adresse MAC.
 */

#include <stdint.h>

#if __has_include("nrf.h")
#include "nrf.h"
#elif __has_include("nrf52.h")
#include "nrf52.h"
#else
#error "nrf.h header is required to read device identity registers."
#endif

static inline uint64_t board_get_dw_mac()
{
    /* Same formula used by  DWM100x_id library. */
    uint64_t high = (uint64_t)(NRF_FICR->DEVICEADDR[1] & 0xFFFF);
    uint64_t low = NRF_FICR->DEVICEADDR[0];
    return (high << 32) | low;
}

static inline uint64_t board_get_cpu_id()
{
    return ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
}

static inline uint16_t board_get_short_address()
{
    return (uint16_t)(board_get_dw_mac() & 0xFFFF);
}

