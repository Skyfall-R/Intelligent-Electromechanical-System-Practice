/**
 * @file drv8833.c
 * @brief DRV8833双H桥电机驱动器控制实现
 */

#include "drv8833.h"
#include "tim.h"

/** @brief IN1引脚的定时器 */
#define IN1_TIM &htim2
/** @brief IN2引脚的定时器 */
#define IN2_TIM &htim2
/** @brief IN3引脚的定时器 */
#define IN3_TIM &htim2
/** @brief IN4引脚的定时器 */
#define IN4_TIM &htim2
/** @brief IN5引脚的定时器 */
#define IN5_TIM &htim3
/** @brief IN6引脚的定时器 */
#define IN6_TIM &htim3
/** @brief IN7引脚的定时器 */
#define IN7_TIM &htim3
/** @brief IN8引脚的定时器 */
#define IN8_TIM &htim3


/** @brief IN1引脚的定时器通道 */
#define CH1 TIM_CHANNEL_1
/** @brief IN2引脚的定时器通道 */
#define CH2 TIM_CHANNEL_2
/** @brief IN3引脚的定时器通道 */
#define CH3 TIM_CHANNEL_3
/** @brief IN4引脚的定时器通道 */
#define CH4 TIM_CHANNEL_4



/** @brief 最大速度值 */
#define MAX_SPEED 100


/** @brief 当前衰减模式 */
static DecayMode currentDecayMode_1 = SLOW_DECAY;
static DecayMode currentDecayMode_2 = SLOW_DECAY;
static DecayMode currentDecayMode_3 = SLOW_DECAY;
static DecayMode currentDecayMode_4 = SLOW_DECAY;

/**
 * @brief 设置IN1引脚的PWM占空比
 * @param duty 占空比值
 */

static inline void __SetIn1PWM(uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(IN1_TIM, CH1, duty);
}


/**
 * @brief 设置IN2引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn2PWM(uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(IN2_TIM, CH2, duty);
}


/**
 * @brief 设置IN3引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn3PWM(uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(IN3_TIM, CH3, duty);
}


/**
 * @brief 设置IN4引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn4PWM(uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(IN4_TIM, CH4, duty);
}


/**
 * @brief 设置IN5引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn5PWM(uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(IN5_TIM, CH1, duty);
}


/**
 * @brief 设置IN6引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn6PWM(uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(IN6_TIM, CH2, duty);
}


/**
 * @brief 设置IN7引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn7PWM(uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(IN7_TIM, CH3, duty);
}


/**
 * @brief 设置IN8引脚的PWM占空比
 * @param duty 占空比值
 */
static inline void __SetIn8PWM(uint8_t duty)
{
    __HAL_TIM_SET_COMPARE(IN8_TIM, CH4, duty);
}


/**
 * @brief 初始化DRV8833
 */
void DRV8833_Init(void)
{
    HAL_TIM_PWM_Start(IN1_TIM, CH1);
    HAL_TIM_PWM_Start(IN2_TIM, CH2);
    HAL_TIM_PWM_Start(IN3_TIM, CH3);
    HAL_TIM_PWM_Start(IN4_TIM, CH4);
    HAL_TIM_PWM_Start(IN5_TIM, CH1);
    HAL_TIM_PWM_Start(IN6_TIM, CH2);
    HAL_TIM_PWM_Start(IN7_TIM, CH3);
    HAL_TIM_PWM_Start(IN8_TIM, CH4);
}

/**
 * @brief 设置衰减模式
 * @param mode 衰减模式
 */
void DRV8833_SetDecayMode(DecayMode mode)
{
    currentDecayMode_1 = mode;
}

/**
 * @brief 控制电机前进
 * @param speed 速度值（0-100）
 */
void DRV8833_1_Forward(uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    if (currentDecayMode_1 == FAST_DECAY) {
        __SetIn1PWM(speed);
        __SetIn2PWM(0);
    } else {
        __SetIn1PWM(MAX_SPEED);
        __SetIn2PWM(MAX_SPEED - speed);
    }
}

/**
 * @brief 控制电机前进
 * @param speed 速度值（0-100）
 */
void DRV8833_2_Forward(uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    if (currentDecayMode_2 == FAST_DECAY) {
        __SetIn3PWM(speed);
        __SetIn4PWM(0);
    } else {
        __SetIn3PWM(MAX_SPEED);
        __SetIn4PWM(MAX_SPEED - speed);
    }
}
    /**
     * @brief 控制电机前进
     * @param speed 速度值（0-100）
     */

	void DRV8833_3_Forward(uint8_t speed)
    {
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;

        if (currentDecayMode_3 == FAST_DECAY) {
            __SetIn5PWM(speed);
            __SetIn6PWM(0);
        } else {
            __SetIn5PWM(MAX_SPEED);
            __SetIn6PWM(MAX_SPEED - speed);
        }
    }
        /**
         * @brief 控制电机前进
         * @param speed 速度值（0-100）
         */
        void DRV8833_4_Forward(uint8_t speed)
        {
            if (speed > MAX_SPEED)
                speed = MAX_SPEED;

            if (currentDecayMode_4 == FAST_DECAY) {
                __SetIn7PWM(speed);
                __SetIn8PWM(0);
            } else {
                __SetIn7PWM(MAX_SPEED);
                __SetIn8PWM(MAX_SPEED - speed);
            }
        }
/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void DRV8833_1_Backward(uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    if (currentDecayMode_1 == FAST_DECAY) {
        __SetIn1PWM(0);
        __SetIn2PWM(speed);
    } else {
        __SetIn1PWM(MAX_SPEED - speed);
        __SetIn2PWM(MAX_SPEED);
    }
}

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void DRV8833_2_Backward(uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    if (currentDecayMode_2 == FAST_DECAY) {
        __SetIn3PWM(0);
        __SetIn4PWM(speed);
    } else {
        __SetIn3PWM(MAX_SPEED - speed);
        __SetIn4PWM(MAX_SPEED);
    }
}

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void DRV8833_3_Backward(uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    if (currentDecayMode_3 == FAST_DECAY) {
        __SetIn5PWM(0);
        __SetIn6PWM(speed);
    } else {
        __SetIn5PWM(MAX_SPEED - speed);
        __SetIn6PWM(MAX_SPEED);
    }
}

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void DRV8833_4_Backward(uint8_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    if (currentDecayMode_4 == FAST_DECAY) {
        __SetIn7PWM(0);
        __SetIn8PWM(speed);
    } else {
        __SetIn7PWM(MAX_SPEED - speed);
        __SetIn8PWM(MAX_SPEED);
    }
}

/**
 * @brief 电机刹车
 */
void DRV8833_1_Brake(void)
{
    __SetIn1PWM(MAX_SPEED);
    __SetIn2PWM(MAX_SPEED);
}

/**
 * @brief 电机刹车
 */
void DRV8833_2_Brake(void)
{
    __SetIn3PWM(MAX_SPEED);
    __SetIn4PWM(MAX_SPEED);
}

/**
 * @brief 电机刹车
 */
void DRV8833_3_Brake(void)
{
    __SetIn5PWM(MAX_SPEED);
    __SetIn6PWM(MAX_SPEED);
}

/**
 * @brief 电机刹车
 */
void DRV8833_4_Brake(void)
{
    __SetIn7PWM(MAX_SPEED);
    __SetIn8PWM(MAX_SPEED);
}

/**
 * @brief 电机滑行
 */
void DRV8833_1_Coast(void)
{
    __SetIn1PWM(0);
    __SetIn2PWM(0);
}

/**
 * @brief 电机滑行
 */
void DRV8833_2_Coast(void)
{
    __SetIn3PWM(0);
    __SetIn4PWM(0);
}

/**
 * @brief 电机滑行
 */
void DRV8833_3_Coast(void)
{
    __SetIn5PWM(0);
    __SetIn6PWM(0);
}

/**
 * @brief 电机滑行
 */
void DRV8833_4_Coast(void)
{
    __SetIn7PWM(0);
    __SetIn8PWM(0);
}
