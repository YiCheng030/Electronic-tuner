#include "Buzzer.h"

void Buzzer_PlayFrequency(uint16_t frequency){
    uint32_t pulse = (170000000 / frequency) / 2;
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pulse);
}

void Buzzer_Tweet(float Buzzer_Hz){
    Buzzer_PlayFrequency(Buzzer_Hz);
    HAL_Delay(100);
}