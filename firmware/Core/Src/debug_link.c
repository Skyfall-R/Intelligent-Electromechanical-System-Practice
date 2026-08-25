/**
  ******************************************************************************
  * @file    debug_link.c
  * @brief   主控 ↔ 上位机 调试链路实现
  *          协议见 算法开发/02-调试协议.md  v2
  ******************************************************************************
  */
#include "debug_link.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ===================== 配置 ===================== */
#define RXBUF_SIZE      256      /* 必须是 2 的幂 */
#define LINE_MAX        160
#define TXBUF_SIZE      192
#define RC_TIMEOUT_MS   200      /* 遥控心跳超时 */
#define MAX_FIELDS      28

/* ===================== 内部状态 ===================== */
static UART_HandleTypeDef *s_uart = NULL;

/* 接收环形缓冲：中断只写 head，主循环只读 tail，一写一读天然无锁 */
static volatile uint8_t  s_rxbuf[RXBUF_SIZE];
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;
static uint8_t           s_rx_byte;          /* HAL 中断的落点 */

static char     s_line[LINE_MAX];
static uint16_t s_line_len = 0;

static char     s_tx[TXBUF_SIZE];

static dbg_ctrl_t s_ctrl;
static dbg_stat_t s_stat;
static uint32_t   s_last_rc_ms = 0;          /* 上次收到 S 或 H 的时刻 */
static uint8_t    s_link_fault = 0;

/* ---- 参数表：顺序必须和 debug_link.h 里的枚举一致 ---- */
typedef struct { const char *name; int32_t val; int32_t lo; int32_t hi; } param_t;

static param_t s_params[P_COUNT] = {
    /*  name              初值      下限      上限   */
    { "KP_V",            1000,        0,   100000 },
    { "KI_V",              50,        0,   100000 },
    { "KD_V",               0,        0,   100000 },
    { "KP_W",            1000,        0,   100000 },
    { "KI_W",               0,        0,   100000 },
    { "KD_W",               0,        0,   100000 },
    { "KP_VIS",          1000,        0,   100000 },
    { "KD_VIS",             0,        0,   100000 },
    { "V_MAX",            600,        0,     2000 },
    { "VY_MAX",           600,        0,     2000 },
    { "W_MAX",          18000,        0,    36000 },
    { "V_CRUISE",         200,        0,     2000 },
    { "V_NEAR",           120,        0,     2000 },
    { "D_NEAR_MM",        490,        0,     5000 },
    { "D_BLIND_MM",       326,        0,     5000 },
    { "ALIGN_TOL_C",      100,        1,     6000 },
    { "WALL_TRIG_MM",     300,       50,     4000 },
    { "ROW_PITCH_MM",    1000,      100,     3000 },
    { "WHEEL_L_MM",       150,       10,     1000 },
    { "WHEEL_R_MM",      3750,      500,    20000 },
    { "ENC_PPR",         1560,        1,   200000 },
    { "KNOB_SPD",         500,    -1000,     1000 },
    { "TELEM_HZ",          20,        1,       50 },
};

/* ===================== 小工具 ===================== */

static uint8_t line_xor(const char *s, uint16_t n)
{
    uint8_t c = 0;
    for (uint16_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
    return c;
}

static int hex2(const char *p, uint8_t *out)
{
    uint8_t v = 0;
    for (int i = 0; i < 2; i++) {
        char c = p[i];
        v <<= 4;
        if      (c >= '0' && c <= '9') v |= (uint8_t)(c - '0');
        else if (c >= 'A' && c <= 'F') v |= (uint8_t)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') v |= (uint8_t)(c - 'a' + 10);
        else return 0;
    }
    *out = v;
    return 1;
}

/** 把整行发出去（自动补校验和 \r\n）。忙就丢，绝不阻塞 */
static void send_line(const char *body, uint16_t n)
{
    if (!s_uart) return;
    if (s_uart->gState != HAL_UART_STATE_READY) { s_stat.tx_drop++; return; }
    if (n + 5 >= TXBUF_SIZE) { s_stat.tx_drop++; return; }

    memcpy(s_tx, body, n);
    uint8_t ck = line_xor(s_tx, n);
    int m = snprintf(s_tx + n, TXBUF_SIZE - n, "*%02X\r\n", ck);
    if (m <= 0) { s_stat.tx_drop++; return; }

    if (HAL_UART_Transmit_IT(s_uart, (uint8_t *)s_tx, (uint16_t)(n + m)) == HAL_OK)
        s_stat.tx_frames++;
    else
        s_stat.tx_drop++;
}

void DebugLink_Log(const char *text)
{
    char b[128];
    int n = snprintf(b, sizeof(b), "L,%.100s", text);
    if (n > 0) send_line(b, (uint16_t)n);
}

static void send_param(dbg_param_id_t id)
{
    char b[64];
    int n = snprintf(b, sizeof(b), "K,%s,%ld",
                     s_params[id].name, (long)s_params[id].val);
    if (n > 0) send_line(b, (uint16_t)n);
}

static void send_version(void)
{
    char b[64];
    int n = snprintf(b, sizeof(b), "V,0.1.0,2,%lu",
                     (unsigned long)HAL_RCC_GetSysClockFreq());
    if (n > 0) send_line(b, (uint16_t)n);
}

static void set_mode(dbg_mode_t m)
{
    if (s_ctrl.mode == m) return;
    s_ctrl.mode = m;
    if (m != DBG_MANUAL) { s_ctrl.vx = s_ctrl.vy = s_ctrl.w_c = 0; }
    DebugLink_Log(m == DBG_AUTO   ? "MODE AUTO"
                : m == DBG_MANUAL ? "MODE MANUAL"
                                  : "MODE ESTOP");
}

/* ===================== 命令处理 ===================== */

/** 就地把 line 按 ',' 切成字段，返回字段数 */
static int split(char *line, char *f[], int maxf)
{
    int n = 0;
    f[n++] = line;
    for (char *p = line; *p && n < maxf; p++) {
        if (*p == ',') { *p = '\0'; f[n++] = p + 1; }
    }
    return n;
}

static long clampf(long v, long lo, long hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void handle_line(char *line, uint16_t len)
{
    (void)len;
    char *f[MAX_FIELDS];
    int   nf = split(line, f, MAX_FIELDS);
    char  c  = f[0][0];

    switch (c)
    {
    /* ---- S,vx,vy,w  三自由度遥控 ---- */
    case 'S': {
        if (nf != 4) { s_stat.line_bad_fmt++; DebugLink_Log("BAD FMT S"); return; }
        long vx = strtol(f[1], NULL, 10);
        long vy = strtol(f[2], NULL, 10);
        long w  = strtol(f[3], NULL, 10);
        /* 越界整行丢弃，不钳到边界——数值离谱本身说明数据有问题 */
        if (vx < -600 || vx > 600 || vy < -600 || vy > 600 || w < -18000 || w > 18000) {
            s_stat.line_bad_fmt++; DebugLink_Log("BAD RANGE"); return;
        }
        if (s_ctrl.mode != DBG_MANUAL) set_mode(DBG_MANUAL);
        s_ctrl.vx  = (int16_t)vx;
        s_ctrl.vy  = (int16_t)vy;
        s_ctrl.w_c = (int16_t)w;
        s_last_rc_ms = HAL_GetTick();
        break;
    }

    /* ---- H  心跳 ---- */
    case 'H':
        s_last_rc_ms = HAL_GetTick();
        break;

    /* ---- J,id,val  舵机 ---- */
    case 'J': {
        if (nf != 3) { s_stat.line_bad_fmt++; DebugLink_Log("BAD FMT J"); return; }
        long id = strtol(f[1], NULL, 10);
        long v  = strtol(f[2], NULL, 10);
        if (id < 0 || id > 2 || v < -32768 || v > 32767) {
            s_stat.line_bad_fmt++; DebugLink_Log("BAD RANGE"); return;
        }
        if (s_ctrl.mode != DBG_MANUAL) { DebugLink_Log("J IGNORED AUTO"); return; }
        s_ctrl.servo[id] = (int16_t)v;
        s_ctrl.servo_new |= (uint8_t)(1u << id);
        break;
    }

    /* ---- P,NAME,value  设参数 ---- */
    case 'P': {
        if (nf != 3) { s_stat.line_bad_fmt++; DebugLink_Log("BAD FMT P"); return; }
        for (int i = 0; i < P_COUNT; i++) {
            if (strcmp(f[1], s_params[i].name) == 0) {
                long v = strtol(f[2], NULL, 10);
                s_params[i].val = (int32_t)clampf(v, s_params[i].lo, s_params[i].hi);
                send_param((dbg_param_id_t)i);   /* 无论成败都回读当前实际值 */
                return;
            }
        }
        DebugLink_Log("NO SUCH PARAM");
        break;
    }

    /* ---- G,state  强制切状态 ---- */
    case 'G': {
        if (nf != 2) { s_stat.line_bad_fmt++; return; }
        long st = strtol(f[1], NULL, 10);
        if (st < 1 || st > 10) { s_stat.line_bad_fmt++; DebugLink_Log("BAD RANGE"); return; }
        s_ctrl.force_state = (uint8_t)st;
        break;
    }

    /* ---- M,0~3  视觉模式字 ---- */
    case 'M': {
        if (nf != 2) { s_stat.line_bad_fmt++; return; }
        long m = strtol(f[1], NULL, 10);
        if (m < 0 || m > 3) { s_stat.line_bad_fmt++; DebugLink_Log("BAD RANGE"); return; }
        s_ctrl.vis_mode = (uint8_t)m;
        break;
    }

    /* ---- Q  请求同步：回 V + 全部 K ---- */
    case 'Q':
        send_version();
        for (int i = 0; i < P_COUNT; i++) send_param((dbg_param_id_t)i);
        break;

    /* ---- E  急停 ---- */
    case 'E':
        set_mode(DBG_ESTOP);
        s_ctrl.servo[0] = s_ctrl.servo[1] = s_ctrl.servo[2] = 0;
        s_ctrl.servo_new = 0x07;
        break;

    /* ---- A  回自动 ---- */
    case 'A':
        set_mode(DBG_AUTO);
        break;

    default:
        s_stat.line_bad_fmt++;
        break;
    }
}

/** 收到一整行（不含 \n），校验后分发 */
static void on_line(char *line, uint16_t len)
{
    if (len < 4) { s_stat.line_bad_fmt++; return; }      /* 最短 "E*45" */

    /* 找末尾的 *XX */
    if (line[len - 3] != '*') { s_stat.line_bad_fmt++; return; }
    uint8_t want;
    if (!hex2(&line[len - 2], &want)) { s_stat.line_bad_fmt++; return; }

    uint16_t body = (uint16_t)(len - 3);
    if (line_xor(line, body) != want) {
        s_stat.line_bad_crc++;                            /* 静默丢弃，不回日志免得刷屏 */
        return;
    }
    line[body] = '\0';
    s_stat.line_ok++;
    handle_line(line, body);
}

/* ===================== 对外 API ===================== */

void DebugLink_Init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
    s_head = s_tail = 0;
    s_line_len = 0;
    memset(&s_ctrl, 0, sizeof(s_ctrl));
    memset(&s_stat, 0, sizeof(s_stat));
    s_ctrl.mode     = DBG_AUTO;
    s_ctrl.vis_mode = 0xFF;
    s_last_rc_ms    = HAL_GetTick();

    HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);
    send_version();
}

void DebugLink_OnRxByte(void)
{
    uint16_t next = (uint16_t)((s_head + 1u) & (RXBUF_SIZE - 1u));
    if (next != s_tail) {
        s_rxbuf[s_head] = s_rx_byte;
        s_head = next;
        s_stat.rx_bytes++;
    } else {
        s_stat.rx_drop++;                 /* 缓冲满：主循环卡了太久 */
        s_link_fault = 1;
    }
    HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);   /* 漏这行就只收一个字节 */
}

void DebugLink_OnError(void)
{
    /* F1 上清 ORE：依次读 SR 再读 DR */
    volatile uint32_t tmp;
    tmp = s_uart->Instance->SR;
    tmp = s_uart->Instance->DR;
    (void)tmp;

    s_stat.uart_err++;
    s_link_fault = 1;
    s_line_len = 0;
    HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1);
}

void DebugLink_Poll(void)
{
    /* 1. 把缓冲区里攒下的字节全部取出来组装成行 */
    while (s_tail != s_head)
    {
        uint8_t b = s_rxbuf[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) & (RXBUF_SIZE - 1u));

        if (b == '\n' || b == '\r') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                on_line(s_line, s_line_len);
                s_line_len = 0;
            }
        } else if (s_line_len < LINE_MAX - 1) {
            s_line[s_line_len++] = (char)b;
        } else {
            s_line_len = 0;                 /* 行过长，整行丢弃 */
            s_stat.line_ovf++;
        }
    }

    /* 2. 遥控心跳超时：200 ms 没消息就把速度全部归零 */
    if (s_ctrl.mode == DBG_MANUAL &&
        (uint32_t)(HAL_GetTick() - s_last_rc_ms) > RC_TIMEOUT_MS)
    {
        if (s_ctrl.vx || s_ctrl.vy || s_ctrl.w_c) {
            s_ctrl.vx = s_ctrl.vy = s_ctrl.w_c = 0;
            s_ctrl.servo[1] = 0;                 /* 拨轮也停 */
            s_ctrl.servo_new |= 0x02;
            DebugLink_Log("RC TIMEOUT");
        }
    }
}

void DebugLink_SendTelem(const dbg_telem_t *t)
{
    uint8_t fault = t->fault;
    if (s_ctrl.mode == DBG_MANUAL &&
        (uint32_t)(HAL_GetTick() - s_last_rc_ms) > RC_TIMEOUT_MS) fault |= DBG_FAULT_RC_TIMEOUT;
    if (s_link_fault) fault |= DBG_FAULT_LINK;

    char b[LINE_MAX];
    int n = snprintf(b, sizeof(b),
        "T,%lu,%d,%d,%d,%d,%d,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%u",
        (unsigned long)HAL_GetTick(),
        t->v_lf, t->v_lb, t->v_rf, t->v_rb,
        t->yaw_c,
        (unsigned)t->vis_class, t->vis_bear_c, (unsigned)t->vis_dist,
        (unsigned)t->vis_conf, (unsigned)t->vision_lost,
        (unsigned)t->tof[0], (unsigned)t->tof[1], (unsigned)t->tof[2],
        (unsigned)t->intake, (unsigned)t->state, (unsigned)t->n_col, (unsigned)fault,
        t->cmd_vx, t->cmd_vy, t->cmd_w_c,
        t->srv[0], t->srv[1], t->srv[2],
        (unsigned)s_ctrl.mode);
    if (n > 0) send_line(b, (uint16_t)n);
}

const dbg_ctrl_t *DebugLink_Ctrl(void) { return &s_ctrl; }
const dbg_stat_t *DebugLink_Stat(void) { return &s_stat; }

int32_t DebugLink_Param(dbg_param_id_t id)
{
    return (id < P_COUNT) ? s_params[id].val : 0;
}
