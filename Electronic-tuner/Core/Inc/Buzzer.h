#ifndef __Buzzer_H
#define __Buzzer_H 

#include "stm32g4xx_hal.h"
//------------------------------------------------
//|                Private macro				 				 |
//------------------------------------------------
extern TIM_HandleTypeDef htim15;
//------------------------------------------------
//|               Public subroutine							 |
//------------------------------------------------
void Buzzer_PlayFrequency(uint16_t frequency);
void Buzzer_Tweet(float Buzzer_Hz);
//------------------------------------------------
//|                     Audio						      	 |
//------------------------------------------------
#define Si_0    31   //NOTE_B0
#define Do_1    33   //NOTE_C1
#define Do_1_1  35   //NOTE_CS1
#define Re_1    37   //NOTE_D1
#define Re_1_1  39   //NOTE_DS1
#define Mi_1    41   //NOTE_E1
#define Fa_1    44   //NOTE_F1
#define Fa_1_1  46   //NOTE_FS1
#define So_1    49   //NOTE_G1
#define So_1_1  52   //NOTE_GS1
#define La_1    55   //NOTE_A1
#define La_1_1  58   //NOTE_AS1
#define Si_1    62   //NOTE_B1
#define Do_2    65   //NOTE_C2
#define Do_2_1  69   //NOTE_CS2
#define Re_2    73   //NOTE_D2
#define Re_2_1  78   //NOTE_DS2
#define Mi_2    82   //NOTE_E2
#define Fa_2    87   //NOTE_F2
#define Fa_2_1  93   //NOTE_FS2
#define So_2    98   //NOTE_G2
#define So_2_1  104  //NOTE_GS2
#define La_2    110  //NOTE_A2
#define La_2_1  117  //NOTE_AS2
#define Si_2    123  //NOTE_B2
#define Do_3    131  //NOTE_C3
#define Do_3_1  139  //NOTE_CS3
#define Re_3    147  //NOTE_D3
#define Re_3_1  156  //NOTE_DS3
#define Mi_3    165  //NOTE_E3
#define Fa_3    175  //NOTE_F3
#define Fa_3_1  185  //NOTE_FS3
#define So_3    196  //NOTE_G3
#define So_3_1  208  //NOTE_GS3
#define La_3    220  //NOTE_A3
#define La_3_1  233  //NOTE_AS3
#define Si_3    247  //NOTE_B3
#define Do_4    262  //NOTE_C4
#define Do_4_1  277  //NOTE_CS4
#define Re_4    294  //NOTE_D4
#define Re_4_1  311  //NOTE_DS4
#define Mi_4    330  //NOTE_E4
#define Fa_4    349  //NOTE_F4
#define Fa_4_1  370  //NOTE_FS4
#define So_4    392  //NOTE_G4
#define So_4_1  415  //NOTE_GS4
#define La_4    440  //NOTE_A4
#define La_4_1  466  //NOTE_AS4
#define Si_4    494  //NOTE_B4
#define Do_5    523  //NOTE_C5
#define Do_5_1  554  //NOTE_CS5
#define Re_5    587  //NOTE_D5
#define Re_5_1  622  //NOTE_DS5
#define Mi_5    659  //NOTE_E5
#define Fa_5    698  //NOTE_F5
#define Fa_5_1  740  //NOTE_FS5
#define So_5    784  //NOTE_G5
#define So_5_1  831  //NOTE_GS5
#define La_5    880  //NOTE_A5
#define L1_5_1  932  //NOTE_AS5
#define Si_5    988  //NOTE_B5
#define Do_6    1047 //NOTE_C6
#define Do_6_1  1109 //NOTE_CS6
#define Re_6    1175 //NOTE_D6
#define Re_6_1  1245 //NOTE_DS6
#define Mi_6    1319 //NOTE_E6
#define Fa_6    1397 //NOTE_F6
#define Fa_6_1  1480 //NOTE_FS6
#define So_6    1568 //NOTE_G6
#define So_6_1  1661 //NOTE_GS6
#define La_6    1760 //NOTE_A6
#define La_6_1  1865 //NOTE_AS6
#define Si_6    1976 //NOTE_B6
#define Do_7    2093 //NOTE_C7
#define Do_7_1  2217 //NOTE_CS7
#define Re_7    2349 //NOTE_D7
#define Re_7_1  2489 //NOTE_DS7
#define Mi_7    2637 //NOTE_E7
#define Fa_7    2794 //NOTE_F7
#define Fa_7_1  2960 //NOTE_FS7
#define So_7    3136 //NOTE_G7
#define So_7_1  3322 //NOTE_GS7
#define La_7    3520 //NOTE_A7
#define La_7_1  3729 //NOTE_AS7
#define Si_7    3951 //NOTE_B7
#define Do_8    4186 //NOTE_C8
#define Do_8_1  4435 //NOTE_CS8
#define Re_8    4699 //NOTE_D8
#define Re_8_1  4978 //NOTE_DS8

#endif
