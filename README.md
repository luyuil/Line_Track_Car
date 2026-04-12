# 以下是小车的引脚配置
采用stm32fc8t6最小系统板
## OLED
SCL    B8
SDA    B9
## MPU6050
SCL    B10
SDA    B11
## 按键
1      A1         往下/+
2      A2         往上/-
3      C15        确定/选中
4      C14        放回/取消选中
## LED
led1   B1
led2   B3
## 蜂鸣器
蜂鸣器  B0
## PWM
pwma   A2
pwmb   A3
## 编码器
左     A6 & A7    TIM3
右     B6 & B7    TIM4
## 电机   TIM2
左     A4 & A5
右     B4 & B5
## 红外循迹
B12
B13
B14
B15
A8
A9
A10
A11
## 定时器  TIM1