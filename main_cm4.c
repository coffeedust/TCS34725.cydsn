/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "tcs34725.h"
#include <stdio.h>

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    tcs34725_tcs34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
    mUART_Start();

    if (tcs34725_begin()) {
        mUART_PutString("Found sensor\n");
    } else {
        mUART_PutString("No TCS34725 found ... check your connections\n");
        while (1);
    }
    Cy_GPIO_Write(pLedWhite_PORT, pLedWhite_NUM, 1);
    for(;;)
    {
      uint16_t r, g, b, c, colorTemp, lux;
      char str[255];

      tcs34725_getRawData(&r, &g, &b, &c);
      // colorTemp = tcs.calculateColorTemperature(r, g, b);
      colorTemp = tcs34725_calculateColorTemperature_dn40(r, g, b, c);
      lux = tcs34725_calculateLux(r, g, b);

      sprintf(str, "Color Temp: %d K - ", colorTemp);
      mUART_PutString(str);
      sprintf(str, "Lux: %d - ", lux);
      mUART_PutString(str);
      sprintf(str, "R: %d ", r);
      mUART_PutString(str);
      sprintf(str, "G: %d ", g);
      mUART_PutString(str);
      sprintf(str, "B: %d ", b);
      mUART_PutString(str);
      sprintf(str, "C: %d ", c);
      mUART_PutString(str);
      mUART_PutString("\n");
    }
}

/* [] END OF FILE */
