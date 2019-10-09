#ifndef __HX711_H
#define __HX711_H

#include "main.h"



extern void Init_HX711pin(void);
extern uint32_t HX711_Read(void);
extern void Get_Maopi(void);
extern uint32_t Get_Weight(void);
extern float HX711_Filter(void) ;
extern uint32_t HX711_Buffer;
extern uint32_t Weight_Maopi;
extern uint32_t Weight_Shiwu;
extern uint8_t Flag_Error;

#endif

