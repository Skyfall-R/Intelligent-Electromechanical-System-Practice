/**
  ******************************************************************************
  * @file    motion.h
  * @brief   小车运动模块 —— 对外只认数字动作码
  *          协议见 docs/02-调试协议.md  v3 §5
  *
  *  设计要点
  *   · 对外接口只有一个数字：Motion_Set(code)。状态机、上位机、视觉都用同一套码
  *   · 四轮方向集中在 WHEEL_DIR[] 一张表里。加动作 = 加一行，不再复制粘贴闭环代码
  *   · 闭环沿用原 main.c 的做法：每 tick 让编码器计数向 MOTION_TARGET 靠，
  *     占空比每次 ±MOTION_STEP。目标值和步长可被上位机改，默认就是原来的 11 / 1
  *   · Motion_Tick() 不阻塞、不调 HAL_Delay，由主循环按节拍调用
  ******************************************************************************
  */
#ifndef __MOTION_H__
#define __MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================= 动作码 ========================= *
 * 这几个数字就是主控与外界通讯的全部运动语汇。
 * 改动这里必须同步改 docs/02-调试协议.md §5 和 motion.c 里的 WHEEL_DIR[]。
 */
typedef enum {
    MOTION_BRAKE   = 0,   /* 00 刹车 */
    MOTION_FORWARD = 1,   /* 01 直行 */
    MOTION_LEFT    = 2,   /* 02 左移（横向平移，车头朝向不变） */
    MOTION_RIGHT   = 3,   /* 03 右移 */
    MOTION_TURN_L  = 4,   /* 04 左转（原地逆时针） */
    MOTION_TURN_R  = 5,   /* 05 右转（原地顺时针） */
    MOTION_CODE_COUNT
} motion_code_t;

/* ========================= 轮子编号 ========================= */
enum {
    WHEEL_LF = 0,   /* 左前  编码器 TIM1  电机 DRV8833_1 */
    WHEEL_LB,       /* 左后  编码器 TIM5  电机 DRV8833_2 */
    WHEEL_RF,       /* 右前  编码器 TIM4  电机 DRV8833_3 */
    WHEEL_RB,       /* 右后  编码器 TIM8  电机 DRV8833_4 */
    WHEEL_COUNT
};

/* ========================= 节拍与默认整定值 ========================= */
#define MOTION_TICK_MS      12    /* Motion_Tick 的调用周期，和原 HAL_Delay(12) 一致 */
#define MOTION_TARGET_DEF   11    /* 每 tick 期望的编码器计数，原 main.c 里写死的 11 */
#define MOTION_STEP_DEF      1    /* 占空比每 tick 的调整量，原 main.c 里的 ++/-- */
#define MOTION_DUTY_MAX    100    /* DRV8833_x_Forward/Backward 的入参上限 */

/* ========================= API ========================= */

/** 初始化：启动四路编码器、四路 PWM 归零、当前动作置 00 刹车 */
void Motion_Init(void);

/** 设动作码。非法码一律当 00 刹车处理，并返回 0 */
int Motion_Set(uint8_t code);

/** 当前动作码 */
uint8_t Motion_Get(void);

/** 每 MOTION_TICK_MS 调一次：读编码器、跑闭环、驱动电机。不阻塞 */
void Motion_Tick(void);

/** 动作码是否合法 */
int Motion_CodeValid(int code);

/** 动作码的中文名，日志用。非法码返回 "?" */
const char *Motion_CodeName(uint8_t code);

/** 上一 tick 四轮实测计数（已折算成「期望方向为正」），遥测用 */
void Motion_GetCount(int16_t out[WHEEL_COUNT]);

/** 当前四轮 PWM 占空比 0~100，遥测用 */
void Motion_GetDuty(uint8_t out[WHEEL_COUNT]);

/** 闭环目标计数 / 占空比步长。上位机参数 SPD_TARGET、SPD_STEP 走这两个口 */
void    Motion_SetTarget(int16_t counts);
int16_t Motion_GetTarget(void);
void    Motion_SetStep(uint8_t step);
uint8_t Motion_GetStep(void);

#ifdef __cplusplus
}
#endif
#endif /* __MOTION_H__ */
