
#include "stm32f4xx.h"

#include "tm_stm32f4_fonts.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_disco.h"
#include <stdio.h>

int main(void) {
char str[15];
int a;

SystemInit();

TM_DELAY_Init();

TM_ILI9341_Init();
/*PA0 */
TM_ADC_Init(ADC1, ADC_Channel_0);

TM_ILI9341_Rotate(TM_ILI9341_Orientation_Landscape_2);

while (1) {
a=TM_ADC_Read(ADC1, ADC_Channel_0);

if (5000>a && 3200<a ) {

TM_ILI9341_Fill(ILI9341_COLOR_RED);

} else if (3200>a&& 2400<a ) {

TM_ILI9341_Fill(ILI9341_COLOR_BLUE2);
}
else if (2400>a && 1600<a ) {

TM_ILI9341_Fill(ILI9341_COLOR_BLUE);

}
else if (1600>a && 800<a ) {
TM_ILI9341_Fill(ILI9341_COLOR_GREEN);

}
else if (800>a && 0<a ) {

TM_ILI9341_Fill(ILI9341_COLOR_GREEN2);

}
sprintf(str, "%4d",a);

TM_ILI9341_Puts(130, 175,str, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);
TM_ILI9341_Puts(80, 150,"ADC UYGULAMASI", &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);
TM_ILI9341_Puts(70, 200,"www.roboturka.com", &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_WHITE);

Delayms(100);

}
}