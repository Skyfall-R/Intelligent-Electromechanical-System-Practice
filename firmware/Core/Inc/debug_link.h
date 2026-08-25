/**
  ******************************************************************************
  * @file    debug_link.h
  * @brief   主控 ↔ 上位机 调试链路（蓝牙 HC-05）
  *          协议见 算法开发/02-调试协议.md  v2
  *
  *  设计要点
  *   · 与具体串口无关：Init 时传 UART 句柄，PD5/PD6(USART2) 或 PC12/PD2(UART5) 都能用
  *   · 中断里只入环形缓冲，解析全在主循环，绝不阻塞
  *   · 发送用 Transmit_IT，忙就跳过这一帧，绝不阻塞
  ******************************************************************************
  */
#ifndef __DEBUG_LINK_H__
#define __DEBUG_LINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ========================= 参数表 ========================= *
 * 增删参数只改这里和 debug_link.c 里的 s_params[]，两处顺序必须一致
 */
typedef enum {
    P_KP_V = 0,      /* 轮速环 P   ×1000 */
    P_KI_V,          /* 轮速环 I   ×1000 */
    P_KD_V,          /* 轮速环 D   ×1000 */
    P_KP_W,          /* 航向环 P   ×1000 */
    P_KI_W,          /* 航向环 I   ×1000 */
    P_KD_W,          /* 航向环 D   ×1000 */
    P_KP_VIS,        /* 视觉伺服 P ×1000 */
    P_KD_VIS,        /* 视觉伺服 D ×1000 */
    P_V_MAX,         /* 速度限幅   mm/s   */
    P_VY_MAX,        /* 横移限幅   mm/s   */
    P_W_MAX,         /* 角速度限幅 ×100 °/s */
    P_V_CRUISE,      /* 巡航速度   mm/s   */
    P_V_NEAR,        /* 近距段速度 mm/s   */
    P_D_NEAR_MM,     /* 远/近分界（车体中心基准） */
    P_D_BLIND_MM,    /* 拨轮遮挡盲区 */
    P_ALIGN_TOL_C,   /* 对准阈值 ×100 度 */
    P_WALL_TRIG_MM,  /* 碰壁换行触发距离 */
    P_ROW_PITCH_MM,  /* 弓字形行距 */
    P_WHEEL_L_MM,    /* 麦轮逆运动学的 L（半轴距+半轮距） */
    P_WHEEL_R_MM,    /* 轮半径 ×100 mm（Φ75 → 3750） */
    P_ENC_PPR,       /* 编码器一圈脉冲数（四倍频后） */
    P_KNOB_SPD,      /* 拨轮舵机工作转速 */
    P_TELEM_HZ,      /* 遥测频率 Hz 1~50 */
    P_COUNT
} dbg_param_id_t;

/* ========================= 遥测数据源 ========================= *
 * 主控每个遥测周期把当前状态填进来，DebugLink 负责打包发送
 */
typedef struct {
    int16_t  v_lf, v_lb, v_rf, v_rb;  /* 四轮实测速度 mm/s  左前/左后/右前/右后 */
    int16_t  yaw_c;                   /* IMU 航向 ×100 度 */

    uint8_t  vis_class;               /* 0无 1红方块 2黄圆柱 3目标区 */
    int16_t  vis_bear_c;              /* 视觉方位角 ×100 度，左负右正 */
    uint16_t vis_dist;                /* mm，65535 = 无效 */
    uint8_t  vis_conf;                /* 0~100 */
    uint8_t  vision_lost;             /* 0/1 */

    uint16_t tof[3];                  /* ToF ①②③  mm，0 = 无效/超量程 */

    uint16_t intake;                  /* 入料口累计计数 */
    uint8_t  state;                   /* 任务状态 1~10 */
    uint8_t  n_col;                   /* 已收数量 */
    uint8_t  fault;                   /* 故障位，见 02-调试协议.md §8 */

    int16_t  cmd_vx, cmd_vy, cmd_w_c; /* 当前实际下发的三自由度指令 */
    int16_t  srv[3];                  /* 舵机当前值 0导轨 1拨轮 2推杆 */
} dbg_telem_t;

/* ========================= 运行模式 ========================= */
typedef enum {
    DBG_AUTO   = 0,   /* 状态机接管 */
    DBG_MANUAL = 1,   /* 上位机遥控，状态机暂停 */
    DBG_ESTOP  = 2    /* 急停，必须收到 A 或 S 才能退出 */
} dbg_mode_t;

/* ========================= 上位机下发的控制量 ========================= */
typedef struct {
    dbg_mode_t mode;

    int16_t  vx, vy, w_c;        /* 遥控三自由度。MANUAL 之外无意义 */

    int16_t  servo[3];           /* 舵机目标值 0导轨 1拨轮 2推杆 */
    uint8_t  servo_new;          /* bit0~2：对应舵机有新命令，主控处理完自行清零 */

    uint8_t  force_state;        /* 1~10 = 上位机要求切到该状态；0 = 无请求，主控处理完清零 */
    uint8_t  vis_mode;           /* 0~3 = 上位机指定的视觉模式；0xFF = 无请求 */
} dbg_ctrl_t;

/* ========================= 故障位 ========================= */
#define DBG_FAULT_STALL      (1u << 0)  /* 电机堵转 */
#define DBG_FAULT_VISION     (1u << 1)  /* 视觉失效 */
#define DBG_FAULT_LOWBAT     (1u << 2)  /* 电池低压 */
#define DBG_FAULT_RC_TIMEOUT (1u << 3)  /* 遥控心跳超时 */
#define DBG_FAULT_LINK       (1u << 4)  /* 蓝牙链路错误 */

/* ========================= API ========================= */

/** 初始化。huart 传蓝牙所在的串口句柄 */
void DebugLink_Init(UART_HandleTypeDef *huart);

/** 在 HAL_UART_RxCpltCallback 里、判断是本串口后调用。只入缓冲，极快 */
void DebugLink_OnRxByte(void);

/** 在 HAL_UART_ErrorCallback 里、判断是本串口后调用 */
void DebugLink_OnError(void);

/** 主循环每圈调用：解析缓冲区里的字节、处理命令、检查心跳超时 */
void DebugLink_Poll(void);

/** 遥测节拍里调用（默认 50 ms），把当前状态发出去 */
void DebugLink_SendTelem(const dbg_telem_t *t);

/** 取上位机下发的控制量（只读） */
const dbg_ctrl_t *DebugLink_Ctrl(void);

/** 取参数当前值 */
int32_t DebugLink_Param(dbg_param_id_t id);

/** 发一条日志行给上位机。text 不得含 , * \r \n，长度 ≤ 100 */
void DebugLink_Log(const char *text);

/** 链路统计，调试用 */
typedef struct {
    uint32_t rx_bytes, rx_drop, uart_err;
    uint32_t line_ok, line_bad_crc, line_bad_fmt, line_ovf;
    uint32_t tx_frames, tx_drop;
} dbg_stat_t;
const dbg_stat_t *DebugLink_Stat(void);

#ifdef __cplusplus
}
#endif
#endif /* __DEBUG_LINK_H__ */
