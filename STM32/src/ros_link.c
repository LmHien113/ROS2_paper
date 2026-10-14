#include "ros_link.h"
#include "can_bus.h"
#include "canopen.h"
#include "bsp_clock.h"
#include "uart_debug.h"

/* ========================================================================= */
/*                             Trang thai module                             */
/* ========================================================================= */

static uint8_t     g_mode     = CTRL_MODE_DEFAULT;
static uint8_t     g_mode_req = CTRL_MODE_DEFAULT;
static const char *g_mode_src = "boot";

/* Lenh moi nhat tu ROS2 (da clamp, quy uoc ROS2) va gia tri gui driver. */
static int32_t  g_cmd_l, g_cmd_r;
static uint32_t g_cmd_ms;
static uint8_t  g_cmd_seen;
static int32_t  g_out_l, g_out_r;

static uint8_t  g_auto_armed;
static uint8_t  g_arm_asked;        /* da xin main Enable, dang cho */
static uint8_t  g_wd_warned;

//static uint8_t  g_fb_enabled;       /* 1 = da nhan 0x050 = 01 */
//static uint32_t g_fb_next_ms;
//static uint32_t g_st_next_ms;
static uint8_t  g_fb_pending;       /* 1 = co 0x050 chua tra loi */
static uint32_t g_st_next_ms;

/* Xoay tai cho (0x040) */
static uint8_t  g_rot_active, g_rot_started;
static uint32_t g_rot_deg;
static int32_t  g_rot_target;       /* pulse moi banh */
static int32_t  g_rot_start_l, g_rot_start_r;
static int32_t  g_rot_done_l, g_rot_done_r;
static uint32_t g_rot_start_ms, g_rot_limit_ms;

/* SELECT x N */
static uint8_t  g_sel_count;
static uint32_t g_sel_first_ms;

/* Bo dem chan doan */
static struct {
    uint32_t cmd, modecmd, rej_mode, rej_dlc, rej_rng, clamped;
    uint32_t rot, rot_rej, rej_rot, fb_req, fb_sent, fb_merged;
} g_cnt;

/* ========================================================================= */
/*                                Tien ich                                   */
/* ========================================================================= */

static int32_t rd_i32(const uint8_t *d)
{
    return (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                     ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
}

static void wr_i32(uint8_t *d, int32_t v)
{
    uint32_t u = (uint32_t)v;

    d[0] = (uint8_t)u;
    d[1] = (uint8_t)(u >> 8);
    d[2] = (uint8_t)(u >> 16);
    d[3] = (uint8_t)(u >> 24);
}

static int32_t clamp_i32(int32_t v, int32_t lim)
{
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

static int32_t abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* |v| > lim, khong tran voi INT32_MIN. */
static uint8_t exceeds(int32_t v, int32_t lim)
{
    return (uint8_t)((v > lim) || (v < -lim));
}

/* Tran = MIN(AUTO_VEL_MAX_PULSE, 0x607F nho nhat doc duoc tu driver). */
static int32_t auto_vel_limit(void)
{
    int32_t lim = AUTO_VEL_MAX_PULSE;
    uint8_t n;

    for (n = NODE_L; n <= NODE_R; n++)
        if (g_gear[n].valid && (g_gear[n].vmax != 0UL) && (g_gear[n].vmax < (uint32_t)lim))
            lim = (int32_t)g_gear[n].vmax;

    return lim;
}

static int32_t slew_towards(int32_t cur, int32_t tgt)
{
    if (AUTO_SLEW_MAX_PULSE > 0)
        return cur + clamp_i32(tgt - cur, AUTO_SLEW_MAX_PULSE);
    return tgt;
}

static const char *mode_name(uint8_t m)
{
    return (m == CTRL_MODE_AUTO) ? "AUTO" : "PS2";
}

static void rot_cancel(void)
{
    g_rot_active = 0U;
    g_rot_started = 0U;
    g_rot_start_ms = 0U;
}

/* Ve trang thai an toan: v = 0, huy ARM, huy xoay. */
static void zero_all(void)
{
    g_cmd_l = g_cmd_r = 0;
    g_out_l = g_out_r = 0;
    g_cmd_seen = 0U;
    g_auto_armed = 0U;
    g_arm_asked = 0U;
    rot_cancel();
}

///* 0x050: byte 0 = first, moi byte con lai (theo DLC) = 0. */
//static uint8_t fb_cmd_match(const uint8_t *data, uint8_t dlc, uint8_t first)
//{
//    uint8_t i;

//    if ((dlc < 1U) || (dlc > 8U) || (data[0] != first))
//        return 0U;
//    for (i = 1U; i < dlc; i++)
//        if (data[i] != 0x00U)
//            return 0U;
//    return 1U;
//}

//static void fb_handle_frame(const uint8_t *data, uint8_t dlc)
//{
//    uint8_t en;

//    if (fb_cmd_match(data, dlc, ROS_FB_START_BYTE))
//        en = 1U;
//    else if (fb_cmd_match(data, dlc, ROS_FB_STOP_BYTE))
//        en = 0U;
//    else
//    {
//        g_cnt.fb_bad++;
//        return;
//    }

//    g_cnt.fb_start++;
//    if (en != g_fb_enabled)
//    {
//        g_fb_enabled = en;
//        g_fb_next_ms = Millis();        /* gui ngay vong toi */
//        UART2_SendString(en ? "[ROS FB] 0x050 = 01 -> BAT DAU gui 0x011/0x012\r\n"
//                            : "[ROS FB] 0x050 = 00 -> DUNG gui 0x011/0x012\r\n");
//    }
//}

static void fb_handle_frame(void)
{
    g_cnt.fb_req++;
    if (g_fb_pending)
        g_cnt.fb_merged++;          /* 0x050 truoc chua kip tra loi */
    g_fb_pending = 1U;
}

/*
 * Ket thuc xoay EM: v = 0 nhung VAN giu ARM (driver ham theo ramp). Gia lap
 * mot frame 0x030 = 0 vua toi; het AUTO_CMD_DISARM_MS thi tu DISARM.
 */
static void rot_finish(const char *why)
{
    if (!g_rot_active)
        return;

    rot_cancel();
    g_cmd_l = g_cmd_r = 0;
    g_cmd_seen = 1U;
    g_cmd_ms = Millis();
    g_wd_warned = 0U;

    UART2_SendString("[ROT ] DUNG xoay: ");
    UART2_SendString(why);
    UART2_SendString(" | L=");
    UART2_SendInt32(g_rot_done_l);
    UART2_SendString(" R=");
    UART2_SendInt32(g_rot_done_r);
    UART2_SendString(" / ");
    UART2_SendInt32(g_rot_target);
    UART2_SendString(" pulse\r\n");
}

static uint8_t rot_pos_fresh(uint32_t now_ms)
{
    return (uint8_t)((g_actual_pos_ms[NODE_L] != 0U) && (g_actual_pos_ms[NODE_R] != 0U) &&
                     ((now_ms - g_actual_pos_ms[NODE_L]) <= ROTATE_POS_STALE_MS) &&
                     ((now_ms - g_actual_pos_ms[NODE_R]) <= ROTATE_POS_STALE_MS));
}

/* |now - start| an toan ca khi bo dem vi tri tran int32. */
static int32_t pos_delta_abs(int32_t now, int32_t start)
{
    return abs_i32((int32_t)((uint32_t)now - (uint32_t)start));
}

static uint8_t auto_ready(void)
{
    return (uint8_t)((g_mode == CTRL_MODE_AUTO) && (g_mode_req == CTRL_MODE_AUTO));
}

static void rot_handle_frame(const uint8_t *data, uint8_t dlc)
{
    uint32_t val;

    if (dlc != 8U)
    {
        g_cnt.rot_rej++;
        return;
    }

    val = (uint32_t)rd_i32(data);
    if (val == 0U)
        return;

    if (g_rot_active)                       /* dang xoay -> bo qua lenh moi */
    {
        g_cnt.rot_rej++;
        return;
    }

    if (!auto_ready())
    {
        g_cnt.rot_rej++;
        UART2_SendString("[ROT ] 0x040 tu choi: dang o PS2 (gui 0x020=02 truoc)\r\n");
        return;
    }

#if (ROTATE_ANGLE_FROM_CAN != 0U)
    g_rot_deg = (val > ROTATE_MAX_DEG) ? ROTATE_MAX_DEG : val;
#else
    g_rot_deg = ROTATE_ANGLE_DEG;
#endif
    g_rot_target  = (int32_t)(((int64_t)g_rot_deg * ROTATE_PULSE_PER_360) / 360);
    g_rot_done_l  = 0;
    g_rot_done_r  = 0;
    g_rot_started = 0U;                     /* chot vi tri khi driver san sang */
    g_rot_start_ms = 0U;
    g_rot_active  = 1U;
    g_cnt.rot++;

    UART2_SendString("[ROT ] nhan 0x040 -> xoay ");
    UART2_SendUInt32(g_rot_deg);
    UART2_SendString(" do = ");
    UART2_SendInt32(g_rot_target);
    UART2_SendString(" pulse\r\n");
}

/* ========================================================================= */
/*                              Khoi tao, mode                               */
/* ========================================================================= */

void RosLink_Init(void)
{
    g_mode = g_mode_req = CTRL_MODE_DEFAULT;
    g_mode_src = "boot";
    zero_all();
    g_cmd_ms = g_st_next_ms = Millis();
    g_wd_warned = 0U;
    g_fb_pending = 0U;

    UART2_SendString("[ROS FB] Nhan 0x050 -> gui 0x011/0x012 \r\n");
    UART2_SendString("[MODE] khoi dong o mode ");
    UART2_SendString(mode_name(g_mode));
    UART2_SendString("\r\n");
}

uint8_t RosLink_GetMode(void)     { return g_mode; }
uint8_t RosLink_AutoIsArmed(void) { return g_auto_armed; }

void RosLink_RequestMode(uint8_t mode, const char *src)
{
    if (((mode != CTRL_MODE_PS2) && (mode != CTRL_MODE_AUTO)) || (mode == g_mode_req))
        return;

    g_mode_req = mode;
    g_mode_src = src;

    /* Cat lenh NGAY, khong doi toi RosLink_Service(). */
    zero_all();
}

uint8_t RosLink_Service(uint32_t now_ms)
{
    if (g_mode_req == g_mode)
        return 0U;

    g_mode = g_mode_req;
    zero_all();
    g_cmd_ms = now_ms;
    g_wd_warned = 0U;
    g_sel_count = 0U;

    UART2_SendString("\r\n[MODE] === DOI SANG ");
    UART2_SendString(mode_name(g_mode));
    UART2_SendString(" (nguon: ");
    UART2_SendString(g_mode_src);
    UART2_SendString(") -> velocity=0, KHOA driver ===\r\n");
    return 1U;
}

void RosLink_SelectPressed(uint32_t now_ms)
{
    if ((g_sel_count == 0U) || ((now_ms - g_sel_first_ms) > MODE_SELECT_WINDOW_MS))
    {
        g_sel_count = 0U;
        g_sel_first_ms = now_ms;
    }
    g_sel_count++;

    UART2_SendString("[MODE] SELECT ");
    UART2_SendUInt32(g_sel_count);
    UART2_SendString("/");
    UART2_SendUInt32(MODE_SELECT_PRESS_COUNT);
    UART2_SendString("\r\n");

    if (g_sel_count >= MODE_SELECT_PRESS_COUNT)
    {
        g_sel_count = 0U;
        RosLink_RequestMode((g_mode == CTRL_MODE_PS2) ? CTRL_MODE_AUTO : CTRL_MODE_PS2,
                            "PS2 SELECT x3");
    }
}

/* ========================================================================= */
/*                          Nhan frame tu ROS2                               */
/* ========================================================================= */

uint8_t RosLink_HandleCanFrame(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    int32_t l, r, lim;

    switch (std_id)
    {
    case CANID_ROS_ROTATE_CMD:
        rot_handle_frame(data, dlc);
        return 1U;

    case CANID_ROS_FB_START:
        fb_handle_frame();
        return 1U;

    case CANID_ROS_MODE_CMD:
        if (dlc < 1U)
            return 1U;
        if ((data[0] == CTRL_MODE_PS2) || (data[0] == CTRL_MODE_AUTO))
        {
            g_cnt.modecmd++;
            RosLink_RequestMode(data[0], "CAN 0x020");
        }
        else
        {
            UART2_SendString("[MODE] 0x020 gia tri la: 0x");
            UART2_SendHexByte(data[0]);
            UART2_SendString(" -> bo qua\r\n");
        }
        return 1U;

    case CANID_ROS_VEL_CMD:
        /* Chi nhan khi dang AUTO va khong co yeu cau doi mode dang cho. */
        if (!auto_ready())          { g_cnt.rej_mode++; return 1U; }
        if (g_rot_active)           { g_cnt.rej_rot++;  return 1U; }
        if (dlc < 8U)               { g_cnt.rej_dlc++;  return 1U; }

        l = rd_i32(&data[0]);       /* banh trai */
        r = rd_i32(&data[4]);       /* banh phai */

        /* Gia tri vo ly = du lieu hong -> vut ca frame, khong clamp. */
        if (exceeds(l, AUTO_VEL_REJECT_PULSE) || exceeds(r, AUTO_VEL_REJECT_PULSE))
        {
            g_cnt.rej_rng++;
            return 1U;
        }

        lim = auto_vel_limit();
        if (exceeds(l, lim) || exceeds(r, lim))
            g_cnt.clamped++;

        g_cmd_l = clamp_i32(l, lim);
        g_cmd_r = clamp_i32(r, lim);
        g_cmd_ms = Millis();
        g_cmd_seen = 1U;
        g_cnt.cmd++;
        return 1U;

    default:
        return 0U;
    }
}

/* ========================================================================= */
/*                        Vong dieu khien mode AUTO                          */
/* ========================================================================= */

uint8_t RosLink_AutoWatchdogExpired(uint32_t now_ms)
{
    if ((g_mode != CTRL_MODE_AUTO) || g_rot_active)   /* xoay co dong ho rieng */
        return 0U;

    return (uint8_t)(!g_cmd_seen || ((now_ms - g_cmd_ms) >= AUTO_CMD_TIMEOUT_MS));
}

void RosLink_AutoDisarm(const char *why)
{
    if (g_auto_armed || g_arm_asked)
    {
        UART2_SendString("[AUTO] DISARM: ");
        UART2_SendString(why);
        UART2_SendString("\r\n");
    }
    if (g_rot_active)
        UART2_SendString("[ROT ] huy xoay do DISARM\r\n");

    zero_all();
}

void RosLink_AutoArmDone(void)
{
    g_auto_armed = 1U;
    g_arm_asked = 0U;
    g_out_l = g_out_r = 0;
    UART2_SendString("[AUTO] ARMED - san sang nhan lenh tu ROS2\r\n");
}

/* Chua ARM -> xin main Enable driver mot lan, giu v = 0 trong luc cho. */
static uint8_t request_arm(uint8_t *need_enable)
{
    if (!g_arm_asked)
    {
        g_arm_asked = 1U;
        *need_enable = 1U;
    }
    return 0U;
}

/* Tra ve 1 = dang xoay va da tinh xong van toc (co the la 0). */
static uint8_t rot_update(uint32_t now_ms, int32_t *tgt_l, int32_t *tgt_r,
                          uint8_t *need_enable)
{
    int32_t v = (ROTATE_VEL_PULSE < auto_vel_limit()) ? ROTATE_VEL_PULSE : auto_vel_limit();

    *tgt_l = *tgt_r = 0;

    if (!g_auto_armed)
    {
        if (!g_arm_asked)
            UART2_SendString("[ROT ] AUTO chua ARM -> Enable driver de xoay\r\n");
        return request_arm(need_enable);
    }

    /* Chot vi tri bat dau, chi khi 0x6064 con tuoi. */
    if (!g_rot_started)
    {
        if (!rot_pos_fresh(now_ms))
        {
            if (g_rot_start_ms == 0U)
                g_rot_start_ms = now_ms;
            if ((now_ms - g_rot_start_ms) > ROTATE_POS_STALE_MS * 2U)
            {
                rot_finish("khong doc duoc vi tri 0x6064");
            }
            return 0U;
        }

        g_rot_start_l  = g_actual_pos_l;
        g_rot_start_r  = g_actual_pos_r;
        g_rot_start_ms = now_ms;
        g_rot_limit_ms = (uint32_t)(((int64_t)g_rot_target * 2000) / v) + ROTATE_TIMEOUT_MARGIN_MS;
        g_rot_started  = 1U;
        UART2_SendString("[ROT ] BAT DAU\r\n");
    }

    if (!rot_pos_fresh(now_ms))
    {
        rot_finish("mat so lieu vi tri 0x6064");
        return 1U;
    }

    g_rot_done_l = pos_delta_abs(g_actual_pos_l, g_rot_start_l);
    g_rot_done_r = pos_delta_abs(g_actual_pos_r, g_rot_start_r);

    if ((g_rot_done_l >= g_rot_target) || (g_rot_done_r >= g_rot_target))
    {
        rot_finish("DU GOC");
        return 1U;
    }
    if ((now_ms - g_rot_start_ms) >= g_rot_limit_ms)
    {
        rot_finish("QUA THOI GIAN - banh ket?");
        return 1U;
    }

    /* Xoay trai (quy uoc ROS2): trai LUI, phai TIEN. */
    *tgt_l = AUTO_SIGN_L * (-(int32_t)(ROTATE_DIRECTION) * v);
    *tgt_r = AUTO_SIGN_R * ( (int32_t)(ROTATE_DIRECTION) * v);
    return 1U;
}

uint8_t RosLink_Update(uint32_t now_ms, int32_t *vel_l, int32_t *vel_r,
                       uint8_t *need_enable)
{
    int32_t tgt_l, tgt_r;
    uint8_t run;

    *need_enable = 0U;
    *vel_l = *vel_r = 0;

    if (g_mode != CTRL_MODE_AUTO)
    {
        g_out_l = g_out_r = 0;
        return 0U;
    }

    if (g_rot_active)
    {
        run = rot_update(now_ms, &tgt_l, &tgt_r, need_enable);
        if (!g_rot_active || !run)
        {
            /* Vua dung xoay hoac dang cho: tra 0 ngay trong vong nay. */
            g_out_l = g_out_r = 0;
            return run;
        }
    }
    else
    {
        if (!g_cmd_seen)
        {
            g_out_l = g_out_r = 0;
            return 0U;
        }

        /* Watchdog: mat lenh ngan -> dung; mat lau -> huy ARM. */
        if ((now_ms - g_cmd_ms) >= AUTO_CMD_TIMEOUT_MS)
        {
            if (!g_wd_warned)
            {
                g_wd_warned = 1U;
                UART2_SendString("[AUTO] WATCHDOG: mat 0x030 -> velocity=0\r\n");
            }
            if ((now_ms - g_cmd_ms) >= AUTO_CMD_DISARM_MS)
                RosLink_AutoDisarm("mat lenh ROS2 qua lau");

            g_out_l = g_out_r = 0;
            return 0U;
        }
        g_wd_warned = 0U;

        /* ARM chi khi frame dau tien co CA HAI banh = 0. */
        if (!g_auto_armed)
        {
            g_out_l = g_out_r = 0;
            if ((g_cmd_l != 0) || (g_cmd_r != 0))
                return 0U;
            return request_arm(need_enable);
        }

        tgt_l = AUTO_SIGN_L * g_cmd_l;
        tgt_r = AUTO_SIGN_R * g_cmd_r;
    }

    g_out_l = slew_towards(g_out_l, tgt_l);
    g_out_r = slew_towards(g_out_r, tgt_r);
    *vel_l = g_out_l;
    *vel_r = g_out_r;
    return 1U;
}

/* ========================================================================= */
/*                        Gui phan hoi ve ROS2                               */
/* ========================================================================= */

static void send_pair(uint16_t id, int32_t l, int32_t r)
{
    uint8_t d[8];

    wr_i32(&d[0], AUTO_SIGN_L * l);     /* doi ve quy uoc ROS2 */
    wr_i32(&d[4], AUTO_SIGN_R * r);
    (void)CAN_SendFrame(id, d, 8U);
}

//void RosLink_SendFeedback(uint32_t now_ms)
//{
//    if (!g_fb_enabled || ((int32_t)(now_ms - g_fb_next_ms) < 0))
//        return;

//    g_fb_next_ms = now_ms + ROS_FB_PERIOD_MS;
//    send_pair(CANID_ROS_VEL_FB, g_actual_vel_l, g_actual_vel_r);
//    send_pair(CANID_ROS_POS_FB, g_actual_pos_l, g_actual_pos_r);
//}

void RosLink_SendFeedback(void)
{
    if (!g_fb_pending)
        return;

    g_fb_pending = 0U;              /* gui loi thi bo, ROS2 se hoi lai */
    send_pair(CANID_ROS_VEL_FB, g_actual_vel_l, g_actual_vel_r);
    send_pair(CANID_ROS_POS_FB, g_actual_pos_l, g_actual_pos_r);
    g_cnt.fb_sent++;
}

void RosLink_SendStatus(uint32_t now_ms, uint8_t flags)
{
#if (ROS_SEND_STATUS != 0U)
    uint8_t d[8];
    uint32_t age_l = now_ms - g_actual_vel_ms[NODE_L];
    uint32_t age_r = now_ms - g_actual_vel_ms[NODE_R];

    if ((int32_t)(now_ms - g_st_next_ms) < 0)
        return;
    g_st_next_ms = now_ms + ROS_STATUS_PERIOD_MS;

    d[0] = g_mode;
    d[1] = flags;
    d[2] = (uint8_t)g_status[NODE_L];
    d[3] = (uint8_t)(g_status[NODE_L] >> 8);
    d[4] = (uint8_t)g_status[NODE_R];
    d[5] = (uint8_t)(g_status[NODE_R] >> 8);
    d[6] = (uint8_t)((age_l > 255UL) ? 255UL : age_l);   /* tuoi 0x606C, ms */
    d[7] = (uint8_t)((age_r > 255UL) ? 255UL : age_r);
    (void)CAN_SendFrame(CANID_ROS_STATUS, d, 8U);
#else
    (void)now_ms;
    (void)flags;
#endif
}

/* ========================================================================= */
/*                                  Log                                      */
/* ========================================================================= */

static void kv(const char *key, uint32_t value)
{
    UART2_SendString(key);
    UART2_SendUInt32(value);
}

void RosLink_PrintStats(void)
{
    UART2_SendString("[MODE] ");
    UART2_SendString(mode_name(g_mode));
    UART2_SendString(g_auto_armed ? " ARMED" : " (chua ARM)");
    kv(" cmd=", g_cnt.cmd);
    kv(" modeCmd=", g_cnt.modecmd);
    kv(" | tuChoi saiMode=", g_cnt.rej_mode);
    kv(" saiDLC=", g_cnt.rej_dlc);
    kv(" voLy=", g_cnt.rej_rng);
    kv(" biCat=", g_cnt.clamped);
    UART2_SendString("\r\n[ROT ] ");
    if (g_rot_active)
    {
        UART2_SendString(g_rot_started ? "DANG XOAY L=" : "CHO ENABLE L=");
        UART2_SendInt32(g_rot_done_l);
        UART2_SendString(" R=");
        UART2_SendInt32(g_rot_done_r);
        UART2_SendString("/");
        UART2_SendInt32(g_rot_target);
    }
    else
    {
        UART2_SendString("nghi");
    }
    kv(" cnt040=", g_cnt.rot);
    kv(" tuChoi040=", g_cnt.rot_rej);
    kv(" 030dangXoay=", g_cnt.rej_rot);
    kv("\r\n[ROS FB] cnt050=", g_cnt.fb_req);
    kv(" daGui=", g_cnt.fb_sent);
    kv(" gop=", g_cnt.fb_merged);
    UART2_SendString("\r\n");
}

void RosLink_PrintTelemetry(uint32_t now_ms)
{
    UART2_SendString("[ROS RX] cmd_L=");
    UART2_SendInt32(g_cmd_l);
    UART2_SendString(" cmd_R=");
    UART2_SendInt32(g_cmd_r);
    UART2_SendString(" age_ms=");
    if (g_cmd_seen)
        UART2_SendUInt32(now_ms - g_cmd_ms);
    else
        UART2_SendString("NA");

    UART2_SendString("\r\n[ROS TX] vel_L=");
    UART2_SendInt32(AUTO_SIGN_L * g_actual_vel_l);
    UART2_SendString(" vel_R=");
    UART2_SendInt32(AUTO_SIGN_R * g_actual_vel_r);
    UART2_SendString(" pos_L=");
    UART2_SendInt32(AUTO_SIGN_L * g_actual_pos_l);
    UART2_SendString(" pos_R=");
    UART2_SendInt32(AUTO_SIGN_R * g_actual_pos_r);
    UART2_SendString("\r\n");
}
