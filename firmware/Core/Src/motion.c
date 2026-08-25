/**
  ******************************************************************************
  * @file    motion.c
  * @brief   小车运动模块实现
  *
  *  原 main.c 里 mode==0/1/2/3 四大块、每块 65 行几乎相同的闭环代码，
  *  在这里被压成一张 6×4 的方向表 + 一份通用的单轮闭环。
  *  加一个动作 = 在表里加一行，不用再复制粘贴。
  ******************************************************************************
  */
#include "motion.h"
#include "tim.h"
#include "drv8833.h"

/* ========================= 四轮方向表 ========================= *
 *  +1 = 该轮正转   -1 = 反转   0 = 刹车
 *
 *  ⚠ 这张表依赖麦克纳姆轮的辊子朝向（X 型 / O 型）和实际装配，
 *    必须在真车上逐条验证，验证办法见 docs/02-调试协议.md §5.3。
 *
 *  两条已有旁证（来自同学原 main.c，已在真车上跑过）：
 *    · 原 mode 1「直行」 = 四轮全 Forward          → 与本表 01 一致
 *    · 原 mode 2         = (+, -, -, +)            → 与本表 03「右移」一致
 *  另：原 mode 3 = (+, -, +, -)，不对应任何标准麦轮基本动作，
 *    看着像未完成的斜向平移，本表没有采纳。
 */
static const int8_t WHEEL_DIR[MOTION_CODE_COUNT][WHEEL_COUNT] = {
    /*                        左前  左后  右前  右后 */
    /* 00 刹车 */          {    0,    0,    0,    0 },
    /* 01 直行 */          {   +1,   +1,   +1,   +1 },
    /* 02 左移 */          {   -1,   +1,   +1,   -1 },
    /* 03 右移 */          {   +1,   -1,   -1,   +1 },
    /* 04 左转 */          {   -1,   -1,   +1,   +1 },
    /* 05 右转 */          {   +1,   +1,   -1,   -1 },
};

static const char *CODE_NAME[MOTION_CODE_COUNT] = {
    "刹车", "直行", "左移", "右移", "左转", "右转"
};

/* ========================= 轮子 ↔ 外设映射 ========================= *
 *  取自同学原 main.c 的注释与变量名：
 *    htim1 左前轮速度 / htim5 左后 / htim4 右前 / htim8 右后
 *    DRV8833_1 左前 / _2 左后 / _3 右前 / _4 右后
 */
static TIM_HandleTypeDef *const ENC_TIM[WHEEL_COUNT] = {
    &htim1, &htim5, &htim4, &htim8
};

typedef void (*drive_fn)(uint8_t);
typedef void (*brake_fn)(void);

static const drive_fn FWD[WHEEL_COUNT] = {
    DRV8833_1_Forward, DRV8833_2_Forward, DRV8833_3_Forward, DRV8833_4_Forward
};
static const drive_fn BWD[WHEEL_COUNT] = {
    DRV8833_1_Backward, DRV8833_2_Backward, DRV8833_3_Backward, DRV8833_4_Backward
};
static const brake_fn BRK[WHEEL_COUNT] = {
    DRV8833_1_Brake, DRV8833_2_Brake, DRV8833_3_Brake, DRV8833_4_Brake
};

/* ========================= 内部状态 ========================= */
static uint8_t  s_code    = MOTION_BRAKE;
static uint8_t  s_duty[WHEEL_COUNT];        /* 当前占空比 0~100 */
static int16_t  s_count[WHEEL_COUNT];       /* 上一 tick 的实测计数（期望方向为正） */
static int16_t  s_target  = MOTION_TARGET_DEF;
static uint8_t  s_step    = MOTION_STEP_DEF;

/* ========================= 工具 ========================= */

int Motion_CodeValid(int code)
{
    return (code >= 0 && code < (int)MOTION_CODE_COUNT);
}

const char *Motion_CodeName(uint8_t code)
{
    return Motion_CodeValid(code) ? CODE_NAME[code] : "?";
}

/** 读一路编码器并清零。ARR=65535，强转 int16 就直接拿到有符号增量，
 *  不用再写 `count > 65525 || count == 0` 这种绕圈判断 */
static int16_t read_enc(int i)
{
    int16_t d = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM[i]);
    __HAL_TIM_SET_COUNTER(ENC_TIM[i], 0);
    return d;
}

static void apply(int i, int8_t dir, uint8_t duty)
{
    if (dir > 0)      FWD[i](duty);
    else if (dir < 0) BWD[i](duty);
    else              BRK[i]();
}

/* ========================= API ========================= */

void Motion_Init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   /* 左前 */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);   /* 左后 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   /* 右前 */
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);   /* 右后 */

    DRV8833_Init();

    for (int i = 0; i < WHEEL_COUNT; i++) {
        s_duty[i]  = 0;
        s_count[i] = 0;
        __HAL_TIM_SET_COUNTER(ENC_TIM[i], 0);
        BRK[i]();
    }
    s_code   = MOTION_BRAKE;
    s_target = MOTION_TARGET_DEF;
    s_step   = MOTION_STEP_DEF;
}

int Motion_Set(uint8_t code)
{
    if (!Motion_CodeValid(code)) {
        code = MOTION_BRAKE;
    }
    if (code == s_code) {
        return (code != MOTION_BRAKE);
    }

    /* 换动作时占空比归零再重新起步。
     * 原代码 mode 是启动时定死的、运行中不会变，所以没这个问题；
     * 现在动作随时会切，100% 占空下直接反向会有很大的电流冲击。 */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        s_duty[i] = 0;
        BRK[i]();
    }
    s_code = code;
    return 1;
}

uint8_t Motion_Get(void) { return s_code; }

void Motion_Tick(void)
{
    const int8_t *dir = WHEEL_DIR[s_code];

    for (int i = 0; i < WHEEL_COUNT; i++)
    {
        int16_t raw = read_enc(i);

        if (dir[i] == 0) {                 /* 刹车：不跑闭环 */
            s_count[i] = raw;
            s_duty[i]  = 0;
            BRK[i]();
            continue;
        }

        /* 折算到「期望方向为正」，正反转共用同一段判断 */
        int16_t meas = (dir[i] > 0) ? raw : (int16_t)(-raw);
        s_count[i] = meas;

        if (meas < s_target) {
            uint16_t d = (uint16_t)(s_duty[i] + s_step);
            s_duty[i] = (d > MOTION_DUTY_MAX) ? MOTION_DUTY_MAX : (uint8_t)d;
        } else if (meas > s_target) {
            s_duty[i] = (s_duty[i] > s_step) ? (uint8_t)(s_duty[i] - s_step) : 0;
        }
        /* meas == s_target 时保持不动，和原代码一致 */

        apply(i, dir[i], s_duty[i]);
    }
}

void Motion_GetCount(int16_t out[WHEEL_COUNT])
{
    for (int i = 0; i < WHEEL_COUNT; i++) out[i] = s_count[i];
}

void Motion_GetDuty(uint8_t out[WHEEL_COUNT])
{
    for (int i = 0; i < WHEEL_COUNT; i++) out[i] = s_duty[i];
}

void Motion_SetTarget(int16_t counts)
{
    if (counts < 1)   counts = 1;
    if (counts > 500) counts = 500;
    s_target = counts;
}

int16_t Motion_GetTarget(void) { return s_target; }

void Motion_SetStep(uint8_t step)
{
    s_step = (step < 1) ? 1 : ((step > 20) ? 20 : step);
}

uint8_t Motion_GetStep(void) { return s_step; }
