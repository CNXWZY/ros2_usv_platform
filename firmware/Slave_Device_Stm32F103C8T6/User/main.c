#include "motor.h"

/**
  * @brief  串口 1 接收中断服务函数 (接收 ROS 2 下发的控制指令)
  * 协议：0xAA (头) + SpeedA (有符号8位) + SpeedB (有符号8位) + 0x55 (尾)
  */
void USART1_IRQHandler(void) {
    static uint8_t RxState = 0;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART1);
        
        if (RxState == 0 && data == 0xAA) RxState = 1;
        else if (RxState == 1) { PID_A.Target = (int8_t)data; RxState = 2; } 
        else if (RxState == 2) { PID_B.Target = (int8_t)data; RxState = 3; }
        else if (RxState == 3 && data == 0x55) { RxState = 0; }
        else RxState = 0;

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/**
  * @brief  TIM1 中断服务函数 (10ms 周期执行：PID 控制 + 数据上传)
  */
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        // 1. 获取反馈（编码器脉冲数）
        int16_t speedA = Encoder_GetA();
        int16_t speedB = Encoder_GetB();

        // 2. PID 计算并执行输出
        MotorA_SetSpeed((int16_t)PID_Update(&PID_A, (float)speedA));
        MotorB_SetSpeed((int16_t)PID_Update(&PID_B, (float)speedB));

        // 3. 将原始数据通过串口发送给 ROS 2
        // 协议：0xBB + SpeedA_H + SpeedA_L + SpeedB_H + SpeedB_L + 0x66
        USART_SendData(USART1, 0xBB);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, (uint8_t)(speedA >> 8));
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, (uint8_t)(speedA & 0xFF));
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, (uint8_t)(speedB >> 8));
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, (uint8_t)(speedB & 0xFF));
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, 0x66);

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

int main(void) {
    // 系统初始化
    Motor_All_Init();

    while (1) {
        // 后台任务可以在这里执行，PID 逻辑在定时器中断中全自动运行
    }
}
