/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "fonts.h"
#include "GFX_FUNCTIONS.h"
#include "st7735.h"
#include "Buzzer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFTN 2048
#define FFTN_half FFTN/2

#define WDataQuantity FFTN/8
#define WDataQuantity2 WDataQuantity*2
#define WDataQuantity3 WDataQuantity*3
#define WDataQuantity4 WDataQuantity*4

#define Fs 8000.0
#define PI 3.14159265359
#define bin(x) Data_real[x] * Data_real[x] + Data_imag[x] * Data_imag[x]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint16_t M_of_N_FFT = 0;

int16_t DMA_dual_Data[FFTN * 2 - 10]; // DMA Read dual channel data
uint16_t Background_count = 6000;
float Data_real[FFTN]; //real
float Data_imag[FFTN]; //imag
float W_real[WDataQuantity + 1] = {};
float W_imag[WDataQuantity + 1] = {};
float freq, Cents;
uint8_t Octave, Finish = 0, Background_flag1, Background_flag2;
int8_t schedule, Old_schedule = 0, e = 0;
float ADCBattery, Old_ADCBattery;

float C[8]  = {16.352, 32.704, 65.408, 130.816, 261.632, 523.264, 1046.528, 2093.056};
float CU[8] = {17.324, 34.649, 69.297, 138.595, 277.189, 554.379, 1108.758, 2217.515};
float D[8]  = {18.354, 36.709, 73.418, 146.836, 293.672, 587.344, 1174.688, 2349.376};
float DU[8] = {19.446, 38.892, 77.784, 155.567, 311.135, 622.269, 1244.538, 2489.076};
float E[8]  = {20.602, 41.204, 82.409, 164.818, 329.636, 659.271, 1318.542, 2637.084};
float F[8]  = {21.827, 43.655, 87.309, 174.618, 349.237, 698.473, 1396.947, 2793.893};
float FU[8] = {23.125, 46.250, 92.501, 185.002, 370.003, 740.007, 1480.013, 2960.027};
float G[8]  = {24.500, 49.001, 98.001, 196.002, 392.005, 784.010, 1568.019, 3136.039};
float GU[8] = {25.957, 51.914, 103.829, 207.657, 415.315, 830.629, 1661.258, 3322.517};
float A[8]  = {27.501, 55.001, 110.003, 220.005, 440.010, 880.021, 1760.042, 3520.084};
float AU[8] = {29.136, 58.272, 116.544, 233.087, 466.175, 932.350, 1864.699, 3729.398};
float B[8]  = {30.868, 61.737, 123.474, 246.947, 493.895, 987.790, 1975.580, 3951.160};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_rx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f); // 調用串列輸出printf
char* intprint(int num); // 將整數轉換成字符串
char* floatprint(float num); // 將浮點數轉換成字符串
void Interface(void); // LCD載入初始背景
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); // timer4 內部計數器 (設定1ms)，用來計數閒置時間
void I2S_DMA_Read(); // I2S DMA聲音數據讀取，並確認是否閒置
void Init_FFT(int N_of_FFT); // 初始化傅立葉變換的相關變量
void ChangeSeat(float DataInput[FFTN]); // 變址
void W_tabular(void); // 快速傅立葉變換的W旋轉因子的建表
void FFT(void); // 實現快速傅立葉變換的運算，使用蝶形運算（Butterfly diagram）
void Close_FFT(void); // FFT結束後清空聲音資料
void Hz(void); // 尋找FFT結果的第一個峰值，並計算聲音頻率
bool InRange(float LowAudio, float AudioOne, float AudioTwo, float HighAudio); // 確認聲音頻率是否在設定值內
void name_of_pitch(void); // 確認聲音頻率對應哪個音名、計算音分差，判斷調音是否完成並把資料顯示在螢幕上
void Tuner_Interfaz(void); // 進度條顯示
int ADCBattery_Read(void); // 讀取ADC
void BatteryShow(void); // 估算電池電壓
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief 調用串列輸出printf
  * @note 
**/
int fputc(int ch, FILE *f){
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
  * @brief 將整數轉換成字符串
  * @note 
**/
char* intprint(int num){
    static char buffer[8]; // 可以容納一個32位整數
    snprintf(buffer, sizeof(buffer), "%d", num);
    return buffer;
}

/**
  * @brief 將浮點數轉換成字符串
  * @note 
**/
char* floatprint(float num){
    static char buffer[8]; // 可以容納一些小數
    snprintf(buffer, sizeof(buffer), "%.2f", num); // %.2f 保留小數後2位
    return buffer;
}

/**
  * @brief timer4 內部計數器 (設定1ms)，用來計數閒置時間
  * @param Background_flag1 閒置旗標1，用於確認是否閒置
  * @param Background_flag2 閒置旗標2，用於開關累加閒置計數
  * @param Background_count 累加閒置計數時間(1ms + 1)
  * @note 
**/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(TIM4 == htim -> Instance){
    if(Background_flag1 == 1 && Background_flag2 == 0)
      Background_count++;
  }
}

/**
  * @brief LCD載入初始背景
  * @note 
**/
void Interface(void){
  ST7735_FillScreen(ST7735_PU); // 背景
  fillCircle(80, 110, 90, ST7735_BLACK); // 圓
  fillTriangle(0, 80, 40, 80, 0, 30, ST7735_PU); // 左三角
  fillTriangle(160, 80, 120, 80, 160, 30, ST7735_PU); // 右三角
  fillTriangle(75, 20, 85, 20, 80, 25, ST7735_GREEN); // 上三角
  drawLine(26, 40, 27, 41, ST7735_WHITE);
  drawLine(41, 30, 42, 31, ST7735_WHITE);
  drawLine(60, 23, 60, 24, ST7735_WHITE);
  drawLine(100, 22, 100, 23, ST7735_WHITE);
  drawLine(118, 29, 117, 30, ST7735_WHITE);
  drawLine(132, 39, 131, 40, ST7735_WHITE);
  drawRoundRect(70, 2, 20, 10, 1,ST7735_WHITE);
  fillRoundRect(90, 4, 3, 6, 0, ST7735_WHITE);
  ST7735_WriteString(2, 60, "-50", Font_7x10, ST7735_WHITE, ST7735_PU);
  ST7735_WriteString(136, 60, "+50", Font_7x10, ST7735_WHITE, ST7735_PU);
  ST7735_WriteString(3, 3, "--", Font_16x26, ST7735_WHITE, ST7735_PU);
  ST7735_WriteString(35, 5, "-", Font_11x18, ST7735_WHITE, ST7735_PU);
  ST7735_WriteString(120, 3, "-", Font_16x26, ST7735_WHITE, ST7735_PU);
  ST7735_WriteString(140, 5, "-", Font_11x18, ST7735_WHITE, ST7735_PU);
}

/**
  * @brief I2S DMA聲音數據讀取，並確認是否閒置
  * @param count1 雙聲道計數器
  * @param count2 單聲道計數器
  * @param BigData 大數值計數器
  * @param BigDataValve 大數值閥值
  * @param DMA_dual_Data 雙聲道聲音數據讀取陣列 (FFTN * 2)
  * @param Data_real 聲音數據實部
  * @param Data_imag 聲音數據虛部
  * @param Background_flag1 閒置旗標1，用於確認是否閒置
  * @param Background_flag2 閒置旗標2，用於開關累加閒置計數
  * @param Background_count 累加閒置計數時間(1ms + 1)
  * @note 
**/
void I2S_DMA_Read(void){
  uint16_t count1, count2 = 0, BigData = 0, BigDataValve = 150;
  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)DMA_dual_Data, FFTN * 2);
  for(count1 = 0; count1 < FFTN * 2; count1++){
    if(DMA_dual_Data[count1] != 0){
      Data_real[count2] = (float)DMA_dual_Data[count1];
      Data_imag[count2] = 0.0;
      if(Data_real[count2] > BigDataValve)
        BigData++;
      count2++;
    }
  }
  if(BigData >= 75){
    Background_flag1 = 0; // 閒置模式關閉
    Background_flag2 = 0; // 閒置計數旗標關閉
    Background_count = 0; // 閒置計數歸零
  }
  else{
    Background_flag1 = 1; // 閒置模式開啟
    if(Background_count >= 4000 && Background_flag2 == 0){  // 閒置計數 >= 4000 FFT停止計算
      ST7735_WriteString(3, 3, "--", Font_16x26, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(35, 5, "-", Font_11x18, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(120, 3, "-", Font_16x26, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(140, 5, "-", Font_11x18, ST7735_WHITE, ST7735_PU);
      fillTriangle(24, 51, 35, 42, 62, 100, ST7735_BLACK); //左4
      fillTriangle(37, 41, 48, 35, 66, 100, ST7735_BLACK); //左3
      fillTriangle(50, 34, 61, 31, 71, 100, ST7735_BLACK); //左2
      fillTriangle(63, 30, 74, 28, 75, 100, ST7735_BLACK); //左1
      fillTriangle(76, 27, 84, 27, 80, 120, ST7735_WHITE); //中
      fillTriangle(86, 28, 97, 30, 85, 100, ST7735_BLACK); //右1
      fillTriangle(99, 31, 110, 34, 89, 100, ST7735_BLACK); //右2
      fillTriangle(112, 35, 123, 41, 94, 100, ST7735_BLACK); //右3
      fillTriangle(125, 42, 136, 51, 98, 100, ST7735_BLACK); //右4
      Background_flag2 = 1; // 閒置計數旗標開啟
      Background_count = 0; // 閒置計數歸零
      Old_schedule = 0;
    }
  }
}

/**
  * @brief 初始化傅立葉變換的相關變量
  * @param N_of_FFT 傅立葉變換大小
  * @param count 迴圈計數器
  * @param temp_N_FFT 臨時存儲計算中的值
  * @param M_of_N_FFT 計算 FFT 運算所需的迭代次數
  * @note 
**/
void Init_FFT(int N_of_FFT){
  uint16_t count = 0;
  uint16_t temp_N_FFT = 1;
  M_of_N_FFT = 0;
  
  for(count = 0; temp_N_FFT < N_of_FFT; count++){
    temp_N_FFT = 2 * temp_N_FFT;
    M_of_N_FFT++;
  }
}

/**
  * @brief 聲音資料進行重新排列，採用雷德演算法（Rader's algorithm）
  * @param nextValue 用於在計算 k 的下一個倒位序的值
  * @param nextM 用於控制迴圈
  * @param i 迴圈計數器
  * @param k 迴圈計數器
  * @param j 迴圈計數器
  * @param temp 用於暫存數據進行交換時的臨時變數
  * @param DMADataSize 聲音資料
  * @param DataInput 將自然順序變成倒位序後存放的陣列
  * @note 
**/
void ChangeSeat(float DataInput[FFTN]){
  int16_t nextValue, nextM, i, k, j = 0;
  float temp;

  nextValue = FFTN / 2;       // 變址運算，把自然順序變成倒位序
  nextM = FFTN - 1;

  for(i = 0; i < nextM; i++){
    if(i < j){                       // 如果 i < j ,即進行變址
      temp = DataInput[j];
      DataInput[j] = DataInput[i];
      DataInput[i] = temp;
    }
    k = nextValue;                   // 求 j 的下一個倒位序
    while(k <= j){                   // 如果 k <= j ,表示j的最高位為 1
      j = j - k;                     // 把最高位變成 0
      k = k / 2;                     // k / 2，比較次高位，依次類推，逐個比較，直到某個位為 0
    }
    j = j + k;                       // 把 0 改為 1
  }
}

/**
  * @brief 快速傅立葉變換的W旋轉因子的建表
  * @param angle 用於控制傅立葉變換的迭代次數
  * @param B 表示迭代步長的一半
  * @param J 控制計算角度
  * @param M_of_N_FFT FFT 運算所需的迭代次數，log2(DataSize)
  * @note
**/
void W_tabular(void){
  uint16_t B, J;
  double angle;
  B = 1 << (M_of_N_FFT - 1);

  for(J = 0; J < B / 4 + 1; J++){
    angle = (double)J / B;
    W_real[J] =  cos(angle * PI);
    W_imag[J] = -sin(angle * PI);
  }
}

/**
  * @brief 實現快速傅立葉變換的運算，使用蝶形運算（Butterfly diagram）
  * @param L 用於控制傅立葉變換的迭代次數
  * @param B 表示迭代步長的一半，用於蝶形運算中
  * @param J 用於迴圈操作，控制計算角度和遍歷子問題
  * @param n 用於迴圈操作，遍歷數據進行蝶形運算
  * @param step 表示步長，2 的 L 次方
  * @param k 表示 K 的 B（步長的一半）處的位置
  * @param W_Rotary_Amplitude 計算W旋轉幅度
  * @param W_Rotary_Position 計算W旋轉位置
  * @param W_count 對應W表中的位置
  * @param W_real 代表蝶形運算中的旋轉因子的實部
  * @param W_imag 代表蝶形運算中的旋轉因子的虛部
  * @param Temp_XX_real 用於暫存旋轉因子實部的計算結果
  * @param Temp_XX_imag 用於暫存旋轉因子虛部的計算結果
  * @param M_of_N_FFT FFT 運算所需的迭代次數，log2(DataSize)
  * @note
**/
void FFT(void){
  uint16_t L = 0, B = 0, J = 0, n = 0, W_Rotary_Amplitude, W_Rotary_Position;
  int16_t step = 0, k = 0, W_count;
  float W_realAfter, W_imagAfter, Temp_XX_real, Temp_XX_imag;
  ChangeSeat(Data_real); // 變址
  for(L = 1; L <= M_of_N_FFT; L++){
    step = 1 << L; // 2 ^ L  
    B = step >> 1; // B = 2 ^ ( L - 1 )
    W_Rotary_Amplitude = 1 << (M_of_N_FFT - L); // 計算W旋轉幅度 // = 2 ^ (總跌代次數 - 目前跌代次數)
    W_Rotary_Position = 0; // W旋轉位置歸零
    W_count = 0;
    for(J = 0; J < B; J++){
/*--------------------取得對應的W--------------------*/
      if(W_Rotary_Position < WDataQuantity){         // 0 ~ (WDataQuantity - 1)
        W_realAfter =  W_real[W_count];
        W_imagAfter =  W_imag[W_count];
        if(W_count + W_Rotary_Amplitude <= WDataQuantity)
          W_count += W_Rotary_Amplitude;
      }
      else if(W_Rotary_Position < (WDataQuantity2)){   // (WDataQuantity - 1) ~ 0
        W_realAfter = -W_imag[W_count];
        W_imagAfter = -W_real[W_count];
        if(W_count != 0)
          W_count -= W_Rotary_Amplitude;
      }
      else if(W_Rotary_Position < WDataQuantity3){   // 0 ~ (WDataQuantity - 1)
        W_realAfter =  W_imag[W_count];
        W_imagAfter = -W_real[W_count];
        if(W_count + W_Rotary_Amplitude <= WDataQuantity)
          W_count += W_Rotary_Amplitude;
      }
      else{                                          // (WDataQuantity - 1) ~ 0
        W_realAfter = -W_real[W_count];
        W_imagAfter =  W_imag[W_count];
        if(W_count != 0)
          W_count -= W_Rotary_Amplitude;
      }
/*--------------------FFT計算--------------------*/
      for(n = J; n < FFTN; n = n + step){
        k = n + B;
        Temp_XX_real = Data_real[k] * W_realAfter - Data_imag[k] * W_imagAfter;
        Temp_XX_imag = Data_real[k] * W_imagAfter + Data_imag[k] * W_realAfter;

        Data_real[k] = Data_real[n] - Temp_XX_real;
        Data_imag[k] = Data_imag[n] - Temp_XX_imag;

        Data_real[n] = Data_real[n] + Temp_XX_real;
        Data_imag[n] = Data_imag[n] + Temp_XX_imag;
      }
      W_Rotary_Position = W_Rotary_Position + W_Rotary_Amplitude; // 計算下一次W旋轉位置
    }
  }
}

/**
  * @brief FFT結束後清空聲音資料
  * @note 
**/
void Close_FFT(void){
  int16_t count;
  Finish = 0;
  for(count = 0; count < FFTN; count++){
    Data_real[count] = 0;
    Data_imag[count] = 0.0;
  }
}

/**
  * @brief 尋找FFT結果的第一個峰值，並計算聲音頻率
  * @param bin FFT結果
  * @param binmax bin陣列中最大值
  * @param bin_count 計數變數
  * @param peak_count 計數變數
  * @param error 找尋peakmax_point + error個點，向後尋找更大的值
  * @param peakmax 儲存第一峰值的值，向後找到更大的值並替代原本的值
  * @param binvalve 以最大值binmax的binvalve倍尋找主頻率
  * @param point 設定迴圈變數
  * @param peak_point 儲存第一峰值的點，向後找到更大的值並替代原本的點
  * @param peak 最終第一個峰值的點
  * @param freq 計算出的聲音頻率
  * @note 
**/
void Hz(void){
  uint16_t bin_countmin = 13, bin_countmax = 130, bin_count, peak_count;
  uint16_t peak, peak_point;
  uint8_t error = 10;
  float binvalve = 0.3;
  float binmax = 0, peakmax = 0;
  for(bin_count = bin_countmin; bin_count <= FFTN_half; bin_count++){
    if(bin(bin_count) > binmax)
      binmax = bin(bin_count);
  }
  for(bin_count = bin_countmin; bin_count <= bin_countmax; bin_count++){
    if(bin(bin_count) >= binmax * binvalve && bin(bin_count) >= bin(bin_count - 1)){
      for(peak_count = 0; peak_count <= error; peak_count++){
        if(bin(bin_count + peak_count) > peakmax){ 
          peakmax = bin(bin_count + peak_count);
          peak_point = peak_count;
          peak = bin_count + peak_point;
          break;
        }
      }
      if(peak != 0)
        break;
    }
  }
  freq = (Fs / FFTN * peak);
}

/**
  * @brief 確認聲音頻率是否在設定值內
  * @param LowAudio 前八度音
  * @param AudioOne 目前八度音
  * @param AudioTwo 目前八度音
  * @param HighAudio 後八度音
  * @param Al 計算目前音名與前音名頻率的中間值
  * @param Ah 計算目前音名與後音名頻率的中間值
  * @note 用於確認目前聲音頻率在哪個八度音區間
  * @retval true or false
**/
bool InRange(float LowAudio, float AudioOne, float AudioTwo, float HighAudio){
  float Al, Ah;
  Al = AudioOne - ((AudioOne - LowAudio) / 2.0);
  Ah = AudioTwo + ((HighAudio - AudioTwo) / 2.0);
  if((freq > Al) && (freq < Ah))
    return true;
  else
    return false;
}

/**
  * @brief 確認聲音頻率對應哪個音名、計算音分差，判斷調音是否完成並把資料顯示在螢幕上
  * @param freq 聲音頻率
  * @param Octave 八度音
  * @param Cents 音分
  * @param Finish 完成旗標
**/
void name_of_pitch(void){
  if(InRange(C[0], C[0], B[0], C[1]))
    Octave = 0;
  else if(InRange(B[0], C[1], B[1], C[2]))
    Octave = 1;
  else if(InRange(B[1], C[2], B[2], C[3]))
    Octave = 2;
  else if(InRange(B[2], C[3], B[3], C[4]))
    Octave = 3;
  else if(InRange(B[3], C[4], B[4], C[5]))
    Octave = 4;
  else if(InRange(B[4], C[5], B[5], C[6]))
    Octave = 5;
  else if(InRange(B[5], C[6], B[6], C[7]))
    Octave = 6;
  else if(InRange(B[6], C[7], B[7], B[7]))
    Octave = 7;
/*-----------------------------------------------------------------------------------------------------*/
  if(freq >= C[0] && freq < FU[2]){
    Cents = 1200 * log2(E[2] / freq);
    if(Octave == 2 && Cents <= 25 && Cents >= -25){
      Finish = 1;
      ST7735_WriteString(120, 3, "E", Font_16x26, ST7735_GREEN, ST7735_PU);
      ST7735_WriteString(140, 7, "2", Font_11x18, ST7735_GREEN, ST7735_PU);
    }
    else{
      ST7735_WriteString(120, 3, "E", Font_16x26, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(140, 7, "2", Font_11x18, ST7735_WHITE, ST7735_PU);
    }
  }
  else if(freq >= FU[2] && freq < C[3]){
    Cents = 1200 * log2(A[2] / freq);
    if(Octave == 2 && Cents <= 25 && Cents >= -25){
      Finish = 1;
      ST7735_WriteString(120, 3, "A", Font_16x26, ST7735_GREEN, ST7735_PU);
      ST7735_WriteString(140, 7, "2", Font_11x18, ST7735_GREEN, ST7735_PU);
    }
    else{
      ST7735_WriteString(120, 3, "A", Font_16x26, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(140, 7, "2", Font_11x18, ST7735_WHITE, ST7735_PU);
    }
  }
  else if(freq >= C[3] && freq < F[3]){
    Cents = 1200 * log2(D[3] / freq);
    if(Octave == 3 && Cents <= 25 && Cents >= -25){
      Finish = 1;
      ST7735_WriteString(120, 3, "D", Font_16x26, ST7735_GREEN, ST7735_PU);
      ST7735_WriteString(140, 7, "3", Font_11x18, ST7735_GREEN, ST7735_PU);
    }
    else{
      ST7735_WriteString(120, 3, "D", Font_16x26, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(140, 7, "3", Font_11x18, ST7735_WHITE, ST7735_PU);
    }
  }
  else if(freq >= F[3] && freq < A[3]){
    Cents = 1200 * log2(G[3] / freq);
    if(Octave == 3 && Cents <= 25 && Cents >= -25){
      Finish = 1;
      ST7735_WriteString(120, 3, "G", Font_16x26, ST7735_GREEN, ST7735_PU);
      ST7735_WriteString(140, 7, "3", Font_11x18, ST7735_GREEN, ST7735_PU);
    }
    else{
      ST7735_WriteString(120, 3, "G", Font_16x26, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(140, 7, "3", Font_11x18, ST7735_WHITE, ST7735_PU);
    }
  }
  else if(freq >= A[3] && freq < D[4]){
    Cents = 1200 * log2(B[3] / freq);
    if(Octave == 3 && Cents <= 25 && Cents >= -25){
      Finish = 1;
      ST7735_WriteString(120, 3, "B", Font_16x26, ST7735_GREEN, ST7735_PU);
      ST7735_WriteString(140, 7, "3", Font_11x18, ST7735_GREEN, ST7735_PU);
    }
    else{
      ST7735_WriteString(120, 3, "B", Font_16x26, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(140, 7, "3", Font_11x18, ST7735_WHITE, ST7735_PU);
    }
  }
  else if(freq >= D[4] && freq < B[7]){
    Cents = 1200 * log2(E[4] / freq);
    if(Octave == 4 && Cents <= 25 && Cents >= -25){
      Finish = 1;
      ST7735_WriteString(120, 3, "E", Font_16x26, ST7735_GREEN, ST7735_PU);
      ST7735_WriteString(140, 7, "4", Font_11x18, ST7735_GREEN, ST7735_PU);
    }
    else{
      ST7735_WriteString(120, 3, "E", Font_16x26, ST7735_WHITE, ST7735_PU);
      ST7735_WriteString(140, 7, "4", Font_11x18, ST7735_WHITE, ST7735_PU);
    }
  }
/*-----------------------------------------------------------------------------------------------------*/
  if(InRange(B[Octave - 1], C[Octave], C[Octave], CU[Octave])) //C
    ST7735_WriteString(3, 3, " C", Font_16x26, ST7735_WHITE, ST7735_PU);
  else if(InRange(C[Octave], CU[Octave], CU[Octave], D[Octave])) //C#
    ST7735_WriteString(3, 3, "C#", Font_16x26, ST7735_WHITE, ST7735_PU);
  else if(InRange(CU[Octave], D[Octave], D[Octave], DU[Octave])){ //D
    if(Finish == 1)
      ST7735_WriteString(3, 3, " D", Font_16x26, ST7735_GREEN, ST7735_PU);
    else
      ST7735_WriteString(3, 3, " D", Font_16x26, ST7735_WHITE, ST7735_PU);
  }
  else if(InRange(D[Octave], DU[Octave], DU[Octave], E[Octave])) //D#
    ST7735_WriteString(3, 3, "D#", Font_16x26, ST7735_WHITE, ST7735_PU);
  else if(InRange(DU[Octave], E[Octave], E[Octave], F[Octave])){  //E
    if(Finish == 1)
      ST7735_WriteString(3, 3, " E", Font_16x26, ST7735_GREEN, ST7735_PU);
    else
      ST7735_WriteString(3, 3, " E", Font_16x26, ST7735_WHITE, ST7735_PU);
  }
  else if(InRange(E[Octave], F[Octave], F[Octave], FU[Octave]))  //F
    ST7735_WriteString(3, 3, " F", Font_16x26, ST7735_WHITE, ST7735_PU);
  else if(InRange(F[Octave], FU[Octave], FU[Octave], G[Octave]))//F#
    ST7735_WriteString(3, 3, "F#", Font_16x26, ST7735_WHITE, ST7735_PU);
  else if(InRange(FU[Octave], G[Octave], G[Octave], GU[Octave])){ //G
    if(Finish == 1)
      ST7735_WriteString(3, 3, " G", Font_16x26, ST7735_GREEN, ST7735_PU);
    else
      ST7735_WriteString(3, 3, " G", Font_16x26, ST7735_WHITE, ST7735_PU);
  }
  else if(InRange(G[Octave], GU[Octave], GU[Octave], A[Octave])) //G#
    ST7735_WriteString(3, 3, "G#", Font_16x26, ST7735_WHITE, ST7735_PU);
  else if(InRange(GU[Octave], A[Octave], A[Octave], B[Octave])){  //A
    if(Finish == 1)
      ST7735_WriteString(3, 3, " A", Font_16x26, ST7735_GREEN, ST7735_PU);
    else
      ST7735_WriteString(3, 3, " A", Font_16x26, ST7735_WHITE, ST7735_PU);
  }
  else if(InRange(A[Octave], AU[Octave], AU[Octave], B[Octave])) //A#
    ST7735_WriteString(3, 3, "A#", Font_16x26, ST7735_WHITE, ST7735_PU);
  else if(InRange(AU[Octave], B[Octave], B[Octave], C[Octave + 1])){ //B
    if(Finish == 1)
      ST7735_WriteString(3, 3, " B", Font_16x26, ST7735_GREEN, ST7735_PU);
    else
      ST7735_WriteString(3, 3, " B", Font_16x26, ST7735_WHITE, ST7735_PU);
  }
/*-----------------------------------------------------------------------------------------------------*/
  if(Octave == 0)
    ST7735_WriteString(35, 7, "0", Font_11x18, ST7735_WHITE, ST7735_PU);
  else if(Octave == 1)
    ST7735_WriteString(35, 7, "1", Font_11x18, ST7735_WHITE, ST7735_PU);
  else if(Octave == 2){
    if(Finish == 1)
      ST7735_WriteString(35, 7, "2", Font_11x18, ST7735_GREEN, ST7735_PU);
    else
      ST7735_WriteString(35, 7, "2", Font_11x18, ST7735_WHITE, ST7735_PU);
  }
  else if(Octave == 3){
    if(Finish == 1)
      ST7735_WriteString(35, 7, "3", Font_11x18, ST7735_GREEN, ST7735_PU);
    else
      ST7735_WriteString(35, 7, "3", Font_11x18, ST7735_WHITE, ST7735_PU);
  }
  else if(Octave == 4){
    if(Finish == 1)
      ST7735_WriteString(35, 7, "4", Font_11x18, ST7735_GREEN, ST7735_PU);
    else
      ST7735_WriteString(35, 7, "4", Font_11x18, ST7735_WHITE, ST7735_PU);
  }
  else if(Octave == 5)
    ST7735_WriteString(35, 7, "5", Font_11x18, ST7735_WHITE, ST7735_PU);
}

/**
  * @brief 進度條顯示
  * @param count 計數變數
  * @param back_flag 進度條前後旗標
  * @param Cents 音分
  * @param schedule 進度條
  * @param Old_schedule 上一次進度條
  * @param Octave 八度音
  * @param Finish 完成旗標
**/
void Tuner_Interfaz(void){
  int8_t count;
  uint8_t back_flag;
  if(Cents < -50.0)
    schedule = -4;
  else if(Cents >= -50.0 && Cents < -42.0)
    schedule = -3;
  else if(Cents >= -42.0 && Cents < -34.0)
    schedule = -2;
  else if(Cents >= -34.0 && Cents < -25.0)
    schedule = -1;
  else if((Cents >= -25.0 && Cents <= 25.0) || Finish == 1)
    schedule = 0;
  else if(Cents > 25.0 && Cents <= 34.0)
    schedule = 1;
  else if(Cents > 34.0 && Cents <= 42.0)
    schedule = 2;
  else if(Cents > 42.0 && Cents <= 50.0)
    schedule = 3;
  else if(Cents > 50.0)
    schedule = 4;

  if(Old_schedule > schedule)
    back_flag = 1;
  else if(Old_schedule < schedule)
    back_flag = 0;
  else
    back_flag = 2;

  if(back_flag == 0){
    for(count = Old_schedule; count <= schedule; count++){
      switch(count){
        case -4:
          fillTriangle(24, 51, 35, 42, 62, 100, ST7735_BLACK); //左4
          break;
        case -3:
          fillTriangle(37, 41, 48, 35, 66, 100, ST7735_BLACK); //左3
          break;
        case -2:
          fillTriangle(50, 34, 61, 31, 71, 100, ST7735_BLACK); //左2
          break;
        case -1:
          fillTriangle(63, 30, 74, 28, 75, 100, ST7735_BLACK); //左1
          break;
        case 0:
          if(Finish == 1){
            fillTriangle(76, 27, 84, 27, 80, 120, ST7735_GREEN); //中
            Buzzer_Tweet(La_4);
          }
          else
            fillTriangle(76, 27, 84, 27, 80, 120, ST7735_WHITE); //中
          break;
        case 1:
          fillTriangle(86, 28, 97, 30, 85, 100, ST7735_WHITE); //右1
          break;
        case 2:
          fillTriangle(99, 31, 110, 34, 89, 100, ST7735_WHITE); //右2
          break;
        case 3:
          fillTriangle(112, 35, 123, 41, 94, 100, ST7735_WHITE); //右3
          break;
        case 4:
          fillTriangle(125, 42, 136, 51, 98, 100, ST7735_WHITE); //右4
          break;
      }
    }
  }

  else if(back_flag == 1){
    for(count = Old_schedule; count >= schedule; count--){
      switch(count){
        case -4:
          fillTriangle(24, 51, 35, 42, 62, 100, ST7735_WHITE); //左4
          break;
        case -3:
          fillTriangle(37, 41, 48, 35, 66, 100, ST7735_WHITE); //左3
          break;
        case -2:
          fillTriangle(50, 34, 61, 31, 71, 100, ST7735_WHITE); //左2
          break;
        case -1:
          fillTriangle(63, 30, 74, 28, 75, 100, ST7735_WHITE); //左1
          break;
        case 0:
          if(Finish == 1){
            fillTriangle(76, 27, 84, 27, 80, 120, ST7735_GREEN); //中
            Buzzer_Tweet(La_4);
          }
          else
            fillTriangle(76, 27, 84, 27, 80, 120, ST7735_WHITE); //中
          break;
        case 1:
          fillTriangle(86, 28, 97, 30, 85, 100, ST7735_BLACK); //右1
          break;
        case 2:
          fillTriangle(99, 31, 110, 34, 89, 100, ST7735_BLACK); //右2
          break;
        case 3:
          fillTriangle(112, 35, 123, 41, 94, 100, ST7735_BLACK); //右3
          break;
        case 4:
          fillTriangle(125, 42, 136, 51, 98, 100, ST7735_BLACK); //右4
          break;
      }
    }
  }
  Old_schedule = schedule;
}

/**
  * @brief 讀取ADC
  * @retval ADC or 0
**/
int ADCBattery_Read(void){
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 100);
  if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
    return HAL_ADC_GetValue(&hadc1);
  }
  return 0;
}

/**
  * @brief 估算電池電壓
  * @param count 計數變數
  * @param Battery_flag 電池顯示旗標
  * @param ADCBattery 估算電池電壓
  * @param Old_ADCBattery 上一次估算電池電壓
**/
void BatteryShow(void){
  int8_t count;
  uint8_t Battery_flag;
  ADCBattery = (ADCBattery_Read() * (3.3 / 4096) * 2.1);
  if(ADCBattery > 3 && ADCBattery <= 3.3)
    ADCBattery = 1;
  else if(ADCBattery > 3.3 && ADCBattery <= 3.6)
    ADCBattery = 2;
  else if(ADCBattery > 3.6 && ADCBattery <= 3.9)
    ADCBattery = 3;
  else if(ADCBattery > 3.9 && ADCBattery <= 4.2)
    ADCBattery = 4;

  if(Old_ADCBattery > ADCBattery)
    Battery_flag = 1;
  else
    Battery_flag = 0;

  if(Battery_flag == 0){
    for(count = Old_ADCBattery; count <= ADCBattery; count++){
      switch(count){
        case 1:
          fillRoundRect(71, 3, 4, 8, 0, ST7735_GREEN);
          break;
        case 2:
          fillRoundRect(76, 3, 4, 8, 0, ST7735_GREEN);
          break;
        case 3:
          fillRoundRect(81, 3, 4, 8, 0, ST7735_GREEN);
          break;
        case 4:
          fillRoundRect(86, 3, 4, 8, 0, ST7735_GREEN);
          break;
      }
    }
  }

  else if(Battery_flag == 1){
    for(count = Old_ADCBattery; count >= ADCBattery; count--){
      switch(count){
        case 1:
          fillRoundRect(71, 3, 4, 8, 0, ST7735_BLACK);
          break;
        case 2:
          fillRoundRect(76, 3, 4, 8, 0, ST7735_BLACK);
          break;
        case 3:
          fillRoundRect(81, 3, 4, 8, 0, ST7735_BLACK);
          break;
        case 4:
          fillRoundRect(86, 3, 4, 8, 0, ST7735_BLACK);
          break;
      }
    }
  }
  Old_ADCBattery = ADCBattery;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2S_Init(&hi2s3); // 啟動I2S
  HAL_DMA_Init(&hdma_spi3_rx); // 啟動SPI3 DMA(I2S3)
  HAL_TIM_Base_Start_IT(&htim4); // 啟動timer4(內部計數器)
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // 啟動timer15 PWM 通道1(蜂鳴器)
  ST7735_Init(); // LCD螢幕初始化
  Interface(); // LCD載入初始背景
  Init_FFT(FFTN); // 初始化傅立葉變換的相關變量
  // HAL_Delay(1000);
  W_tabular();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    I2S_DMA_Read(); // I2S DMA聲音數據讀取，並確認是否閒置
    BatteryShow(); // 估算電池電壓
    if(Background_flag1 == 0){ // 閒置模式關閉
      FFT(); // 實現快速傅立葉變換的運算，使用蝶形運算（Butterfly diagram）
      Hz(); // 尋找FFT結果的第一個峰值，並計算聲音頻率
      name_of_pitch(); // 確認聲音頻率對應哪個音名、計算音分差，判斷調音是否完成並把資料顯示在螢幕上
      Tuner_Interfaz(); // 進度條顯示
    }
    Close_FFT(); // FFT結束後清空聲音資料
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|CS_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RET_GPIO_Port, RET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 CS_Pin DC_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|CS_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RET_Pin */
  GPIO_InitStruct.Pin = RET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RET_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */