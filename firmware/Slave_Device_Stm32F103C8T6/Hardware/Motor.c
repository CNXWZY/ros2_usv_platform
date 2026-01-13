#include "motor.h"

// 实例化全局 PID 变量（初始参数需要根据实际电机调整）
PID_Controller PID_A = {0, 2.5f, 0.4f, 0.1f, 0, 0, 0};
PID_Controller PID_B = {0, 2.5f, 0.4f, 0.1f, 0, 0, 0};

/**
  * @brief  所有硬件初始化（PWM、方向 IO、编码器接口、定时中断、串口）
  */
void Motor_All_Init(void) {
    // 1. 开启外设时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 2. PWM 引脚 (PA0, PA1) 配置为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 方向控制引脚 (PA2, PA3, PA4, PA5) 配置为推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. 串口 USART1 引脚 (PA9-TX, PA10-RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 接收端设为上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 5. 编码器引脚 (PA6/7, PB6/7) 配置为上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // 电机 A 编码器
    GPIO_Init(GPIOB, &GPIO_InitStructure); // 电机 B 编码器

    // 6. 配置 TIM2 生成 PWM (频率 = 72MHz / 72 / 100 = 10kHz)
    TIM_TimeBaseInitTypeDef TIM_Base;
    TIM_Base.TIM_Period = 100 - 1; 
    TIM_Base.TIM_Prescaler = 72 - 1; 
    TIM_Base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_Base);

    TIM_OCInitTypeDef TIM_OC;
    TIM_OCStructInit(&TIM_OC);
    TIM_OC.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM2, &TIM_OC); // 电机 A
    TIM_OC2Init(TIM2, &TIM_OC); // 电机 B
    TIM_Cmd(TIM2, ENABLE);

    // 7. 配置硬件编码器模式 (TIM3, TIM4)
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM3, ENABLE);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM4, ENABLE);

    // 8. 配置 TIM1 用于 10ms PID 采样中断
    TIM_Base.TIM_Period = 100 - 1; 
    TIM_Base.TIM_Prescaler = 7200 - 1; // 72MHz / 7200 / 100 = 100Hz (10ms)
    TIM_TimeBaseInit(TIM1, &TIM_Base);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    // 9. 配置 USART1 (115200 波特率)
    USART_InitTypeDef USART_InitStr;
    USART_InitStr.USART_BaudRate = 115200;
    USART_InitStr.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStr.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStr.USART_Parity = USART_Parity_No;
    USART_InitStr.USART_StopBits = USART_StopBits_1;
    USART_InitStr.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStr);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启接收中断
    USART_Cmd(USART1, ENABLE);

    // 10. 配置中断优先级 (NVIC)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStr;
    NVIC_InitStr.NVIC_IRQChannel = TIM1_UP_IRQn;   // PID 定时器中断
    NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = 0; // 最高优先级
    NVIC_InitStr.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStr);

    NVIC_InitStr.NVIC_IRQChannel = USART1_IRQn;    // 串口接收中断
    NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_Init(&NVIC_InitStr);
}

// 电机 A 控制 (-100 到 100)
void MotorA_SetSpeed(int16_t duty) {
    if (duty >= 0) {
        GPIO_SetBits(GPIOA, GPIO_Pin_2); GPIO_ResetBits(GPIOA, GPIO_Pin_3); // AIN1=1, AIN2=0
        TIM_SetCompare1(TIM2, duty > 100 ? 100 : duty);
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_2); GPIO_SetBits(GPIOA, GPIO_Pin_3); // AIN1=0, AIN2=1
        TIM_SetCompare1(TIM2, duty < -100 ? 100 : -duty);
    }
}

// 电机 B 控制 (-100 到 100)
void MotorB_SetSpeed(int16_t duty) {
    if (duty >= 0) {
        GPIO_SetBits(GPIOA, GPIO_Pin_4); GPIO_ResetBits(GPIOA, GPIO_Pin_5); // BIN1=1, BIN2=0
        TIM_SetCompare2(TIM2, duty > 100 ? 100 : duty);
    } else {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4); GPIO_SetBits(GPIOA, GPIO_Pin_5); // BIN1=0, BIN2=1
        TIM_SetCompare2(TIM2, duty < -100 ? 100 : -duty);
    }
}

// 获取编码器计数并清零（以便下次计算增量）
int16_t Encoder_GetA(void) { int16_t temp = (int16_t)TIM_GetCounter(TIM3); TIM_SetCounter(TIM3, 0); return temp; }
int16_t Encoder_GetB(void) { int16_t temp = (int16_t)TIM_GetCounter(TIM4); TIM_SetCounter(TIM4, 0); return temp; }

// PID 核心算法实现
float PID_Update(PID_Controller *p, float actual) {
    p->Error = p->Target - actual;  // 计算偏差
    p->Integral += p->Error;        // 积分累加
    // 积分限幅
    if (p->Integral > 100) p->Integral = 100;
    if (p->Integral < -100) p->Integral = -100;
    // 位置式 PID 公式
    float output = (p->Kp * p->Error) + (p->Ki * p->Integral) + (p->Kd * (p->Error - p->LastError));
    p->LastError = p->Error;        // 更新上次偏差
    // 输出限幅
    if (output > 100) output = 100;
    if (output < -100) output = -100;
    return output;
}
