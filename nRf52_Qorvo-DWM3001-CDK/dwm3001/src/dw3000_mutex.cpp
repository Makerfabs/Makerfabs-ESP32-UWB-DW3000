/*
Ce module garantit que les accès critiques (SPI, registres) ne sont pas interrompus par une 
IRQ DW3000 ou une autre interruption Cortex-M, tout en restaurant exactement l'état initial 
une fois la section critique terminée.
---------------------------------------------------------------------------------------
 Cette version utilise PRIMASK au lieu du mutex RTOS. 
 L’ancienne version (portENTER_CRITICAL) bloquait seulement le scheduler RTOS,
 mais ne garantissait pas le blocage complet des interruptions matérielles.
 Le DW3000 nécessite un timing très précis : une seule interruption peut
 perturber la lecture des timestamps et fausser le calcul du ToF.
 PRIMASK désactive toutes les interruptions du CPU via __disable_irq(),
 puis restaure exactement l’état précédent avec __enable_irq().
 Cela garantit une section critique totalement atomique et fiable.
 PRIMASK = registre CPU qui contrôle l’activation globale des interruptions.
 */



/*! ----------------------------------------------------------------------------
 * @file    deca_mutex.c
 * @brief   IRQ interface / mutex implementation
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <dw3000_device_api.h>
#include <dw3000_port.h>

static const decaIrqStatus_t kPrimaskFlag = (decaIrqStatus_t)(1UL << 31);
// ---------------------------------------------------------------------------
//
// NB: The purpose of this file is to provide for microprocessor interrupt enable/disable, this is used for
//     controlling mutual exclusion from critical sections in the code where interrupts and background
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines in the
//     deca_irq.h include file to map these calls transparently to the target system.  Alternatively the
//     appropriate code may be embedded in the functions provided below.
//
//     This mutex dependent on HW port.
//	   If HW port uses EXT_IRQ line to receive ready/busy status from DW1000 then mutex should use this signal
//     If HW port not use EXT_IRQ line (i.e. SW polling) then no necessary for decamutex(on/off)
//
//	   For critical section use this mutex instead
//	   __save_intstate()
//     __restore_intstate()
// ---------------------------------------------------------------------------


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a critical section
 * It returns the irq state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
decaIrqStatus_t decamutexon(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    decaIrqStatus_t s = port_GetEXT_IRQStatus();

    if(s) {
        port_DisableEXT_IRQ(); //disable the external interrupt line
    }

    if (primask == 0)
    {
        s |= kPrimaskFlag;
    }

    return s ;   // return state before disable, value is used to re-enable in decamutexoff call
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s)        // put a function here that re-enables the interrupt at the end of the critical section
{
    decaIrqStatus_t irqMaskState = (s & ~kPrimaskFlag);

    if(irqMaskState) { //need to check the port state as we can't use level sensitive interrupt on the STM ARM
        port_EnableEXT_IRQ();
    }

    if(s & kPrimaskFlag)
    {
        __enable_irq();
    }
}
