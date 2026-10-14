/*
 * main.c - vong lap dieu khien R2.
 *
 *   PS2  : dieu khien bang tay cam.
 *   AUTO : van toc tu ROS2 qua CAN (xem ros_link.h).
 *
 * Doi mode: CAN 0x020 hoac SELECT x3 tren tay cam; START khi dang AUTO = ve
 * PS2 ngay. Moi lan doi mode deu ep v = 0, khoa driver va phai ARM lai.
 */
#include "app_config.h"
#include "bsp_clock.h"
#include "uart_debug.h"
#include "ps2.h"
#include "can_bus.h"
#include "canopen.h"
#include "ros_link.h"

/* ======================================================================== */
/*                               Trang thai                                 */
/* ======================================================================== */

typedef struct {
    uint8_t  armed;          /* da qua thu tuc Analog + Neutral            */
    uint8_t  arm_count;      /* so frame neutral lien tiep                 */
    uint16_t pressed;        /* nut da qua debounce                        */
    uint16_t candidate;
    uint8_t  agree;
    int32_t  vel_l, vel_r;
} Ps2State;

static Ps2State ps2;
static uint8_t  drive_fault;
static uint8_t  force_tx = 1U;       /* 1 = gui RPDO ngay o vong toi */
static uint32_t ps2_bad_cnt;

/* Tha moi nut, v = 0, nhung giu ARM. */
static void ps2_idle(void)
{
    ps2.pressed = 0U;
    ps2.candidate = 0U;
    ps2.agree = 0U;
    ps2.vel_l = 0;
    ps2.vel_r = 0;
}

/* Bat buoc lam lai thu tuc ARM (Analog + Neutral). */
static void ps2_disarm(void)
{
    ps2_idle();
    ps2.armed = 0U;
    ps2.arm_count = 0U;
}

static void lock_both(void)
{
    (void)Drive_Lock(NODE_L);
    (void)Drive_Lock(NODE_R);
}

static uint8_t enable_both(void)
{
    uint8_t ok = Drive_EnableOperation(NODE_L);

    ok &= Drive_EnableOperation(NODE_R);
    return ok;
}

static void set_ramp_both(uint32_t accel, uint32_t decel)
{
    if (!Drive_SetRamp(NODE_L, accel, decel)) drive_fault = 1U;
    if (!Drive_SetRamp(NODE_R, accel, decel)) drive_fault = 1U;
}

static int32_t clamp_vel(int32_t v)
{
    return (v > AUTO_VEL_MAX_PULSE) ? AUTO_VEL_MAX_PULSE :
           ((v < -AUTO_VEL_MAX_PULSE) ? -AUTO_VEL_MAX_PULSE : v);
}

/*
 * Nut -> van toc. Driver: TIEN khi banh trai am, banh phai duong.
 *   UP tien, CROSS lui, RIGHT/SQUARE cua phai/trai,
 *   TRIANGLE/CIRCLE xoay tai cho cung/nguoc chieu kim dong ho.
 */
static void buttons_to_velocity(uint16_t pressed, int32_t *vel_l, int32_t *vel_r)
{
    const int32_t full = JOG_VEL;
    const int32_t half = JOG_VEL / 2;

    *vel_l = 0;
    *vel_r = 0;

    if      (pressed & BTN_UP)       { *vel_l = -full; *vel_r = +full; }
    else if (pressed & BTN_CROSS)    { *vel_l = +full; *vel_r = -full; }
    else if (pressed & BTN_RIGHT)    { *vel_l = -half; *vel_r = +full; }
    else if (pressed & BTN_SQUARE)   { *vel_l = -full; *vel_r = +half; }
    else if (pressed & BTN_TRIANGLE) { *vel_l = +full; *vel_r = +full; }
    else if (pressed & BTN_CIRCLE)   { *vel_l = -full; *vel_r = -full; }
}

/* Debounce: chap nhan khi doc giong nhau PS2_DEBOUNCE_COUNT lan lien tiep. */
static uint8_t debounce(uint16_t now, uint16_t *cand, uint8_t *agree)
{
    if (now == *cand)
    {
        if (*agree < 0xFFU)
            (*agree)++;
    }
    else
    {
        *cand = now;
        *agree = 1U;
    }
    return (uint8_t)(*agree >= PS2_DEBOUNCE_COUNT);
}

/* ======================================================================== */
/*                               Khoi dong                                  */
/* ======================================================================== */

static void boot(void)
{
    uint8_t init_l, init_r;

    UART2_Init();
    PS2_SPI_Init();             /* doi PCLK2 -> truoc Delay_UsInit */
    Delay_UsInit();
    SysTick_Init();
    CAN_Motor_Init();

    UART2_SendString("\r\n[BOOT] PS2 + AUTO(ROS2) + CANopen, CAN2 = ");
    UART2_SendUInt32(CAN_BITRATE_KBPS);
    UART2_SendString(" kbit/s\r\n");

    RosLink_Init();

    /* Driver co the dang giu target cu: ep ve 0 + Shutdown truoc moi thu. */
    UART2_SendString("[SAFE] Ep hai driver ve 0 + Shutdown...\r\n");
    Drive_ForceSafe(NODE_L);
    Drive_ForceSafe(NODE_R);
    CAN_Poll();

#if (DRIVE_DUMP_GEAR != 0U)
    Drive_DumpGearParams();
#endif

    UART2_SendString("[DRV ] Khoi tao driver...\r\n");
    init_l = Drive_Init(NODE_L);
    init_r = Drive_Init(NODE_R);
    lock_both();                /* Drive_Init vua Enable voi target 0 */

    if (!init_l || !init_r)
    {
        UART2_SendString("[SAFE] KHOI TAO DRIVER THAT BAI -> giu khoa, khong cho chay\r\n");
        UART2_Flush();
        for (;;)
        {
            Drive_ForceSafe(NODE_L);
            Drive_ForceSafe(NODE_R);
            CAN_Poll();
            Delay_Ms(100UL);
        }
    }

    CAN_Poll();
    Drive_LinkInit();

    /* Boot-up con sot tu truoc Drive_Init() khong phai reboot moi. */
    g_node_booted[NODE_L] = 0U;
    g_node_booted[NODE_R] = 0U;

    /* Chi bat PS2 sau khi driver da an toan. */
    PS2_EnterAnalog();

    CAN_PrintErrorStatus();
    if (g_can_tx_ok == 0U)
        UART2_SendString("[CAN ] CANH BAO: chua gui duoc frame nao "
                         "(chan RX? bitrate? 120R? STBY transceiver?)\r\n");
    if (g_sdo_write_fail != 0U)
    {
        UART2_SendString("[DRV ] CANH BAO: ");
        UART2_SendUInt32(g_sdo_write_fail);
        UART2_SendString(" lenh SDO khoi tao KHONG duoc ACK\r\n");
    }
    UART2_SendString("[DRV ] San sang\r\n\r\n");
}

/* ======================================================================== */
/*                                   main                                   */
/* ======================================================================== */

int main(void)
{
    uint8_t  ps2_rx[PS2_FRAME_MAX];
    uint8_t  ok, digital_now, digital_prev = 0U;
    uint8_t  failsafe = 0U, failsafe_prev = 0U;
    uint8_t  link_lost = 0U, link_lost_prev = 0U, nmt_bad = 0U;
    uint8_t  mode, auto_ok, need_enable, armed_now, locked;
    uint8_t  prev_locked = 0xFFU;
    uint16_t all_now, button_event_latch = 0U;
    uint16_t sel_cand = 0U;
    uint8_t  sel_agree = 0U, sel_stable = 0U;
    uint8_t  sdo_slot = 0U;
    int32_t  vel_l = 0, vel_r = 0, prev_vel_l = 0, prev_vel_r = 0;
    uint32_t now_ms, next_loop_ms;
    uint32_t last_valid_ps2_ms, last_rpdo_ms = 0U, last_debug_ms, last_telem_ms;
    uint32_t last_nmt_ms = 0U, last_recover_ms = 0U, last_sdo_ms = 0U;

    boot();

    last_valid_ps2_ms = last_debug_ms = last_telem_ms = next_loop_ms = Millis();

    for (;;)
    {
        now_ms = Millis();

        /* ---------- 1. CAN den ---------- */
        CAN_Poll();

        /* ---------- 1b. Node vua boot lai -> cau hinh lai, ARM lai ---------- */
        if (g_node_booted[NODE_L] || g_node_booted[NODE_R])
        {
            uint8_t n;

            for (n = NODE_L; n <= NODE_R; n++)
            {
                if (!g_node_booted[n])
                    continue;
                g_node_booted[n] = 0U;
                UART2_SendString((n == NODE_L) ? "[DRV ] Boot-up node L -> cau hinh lai\r\n"
                                               : "[DRV ] Boot-up node R -> cau hinh lai\r\n");
                if (!Drive_Init(n))
                    drive_fault = 1U;
            }

            CAN_Poll();
            now_ms = next_loop_ms = Millis();
            vel_l = vel_r = 0;
            force_tx = 1U;
            ps2_disarm();
            RosLink_AutoDisarm("driver vua boot lai");
        }

        /* ---------- 1c. Doi mode: DIEM DUY NHAT trong vong lap ---------- */
        if (RosLink_Service(now_ms))
        {
            vel_l = vel_r = 0;
            lock_both();
            ps2_disarm();

            if (RosLink_GetMode() == CTRL_MODE_AUTO)
                set_ramp_both(AUTO_ACCEL_VAL, AUTO_DECEL_VAL);
            else
                set_ramp_both(ACCEL_VAL, DECEL_VAL);

            now_ms = next_loop_ms = last_valid_ps2_ms = Millis();  /* SetRamp chan ~20 ms */
            force_tx = 1U;
        }
        mode = RosLink_GetMode();

        /* ---------- 2. Tay cam (doc ca o AUTO de bat SELECT / E-stop) ---------- */
        ok = PS2_ReadPoll(ps2_rx);              /* 1 = frame analog */
        digital_now = PS2_IsDigitalFrame(ps2_rx);

        all_now = (ok || digital_now) ? PS2_GetAllPressed(ps2_rx) : 0U;

        if (debounce((uint16_t)((all_now & BTN_SELECT) != 0U), &sel_cand, &sel_agree))
        {
            if (sel_cand && !sel_stable)
                RosLink_SelectPressed(now_ms);    /* canh len */
            sel_stable = (uint8_t)sel_cand;
        }

        if ((mode == CTRL_MODE_AUTO) && ((all_now & AUTO_ESTOP_BTN) != 0U))
            RosLink_RequestMode(CTRL_MODE_PS2, "E-STOP tay cam");

        if (digital_now)
        {
            /* Digital = khoa dieu khien tay cam. */
            ps2_disarm();
            if (!digital_prev)
            {
                UART2_SendString("[SAFE] PS2 DIGITAL -> velocity=0, KHOA dieu khien\r\n");
                if (mode == CTRL_MODE_PS2)      /* o AUTO khong duoc dung robot */
                {
                    lock_both();
                    force_tx = 1U;
                }
            }
        }
        else if (ok)
        {
            uint16_t raw_now = PS2_GetPressedButtons(ps2_rx);

            last_valid_ps2_ms = now_ms;

            if (ps2.armed)
            {
                if (debounce(raw_now, &ps2.candidate, &ps2.agree))
                    ps2.pressed = ps2.candidate;
                button_event_latch |= ps2.pressed;
                buttons_to_velocity(ps2.pressed, &ps2.vel_l, &ps2.vel_r);
            }
            else
            {
                ps2_idle();
                ps2.arm_count = (raw_now == 0U) ? (uint8_t)(ps2.arm_count + 1U) : 0U;

                /* ARM sau N frame neutral, CHI o mode PS2 (khong bao gio tu
                   Enable khi dang AUTO chi vi vua cam tay cam). */
                if (ps2.arm_count >= PS2_ANALOG_ARM_COUNT)
                {
                    ps2.arm_count = 0U;

                    if (mode == CTRL_MODE_PS2)
                    {
                        if (!enable_both())
                        {
                            drive_fault = 1U;
                            lock_both();
                            UART2_SendString("[SAFE] enable PS2 that bai -> KHOA\r\n");
                            continue;
                        }

                        ps2.armed = 1U;
                        force_tx = 1U;
                        now_ms = next_loop_ms = Millis();   /* Enable chan ~120 ms */
                        UART2_SendString("[SAFE] PS2 ANALOG + NEUTRAL -> ARMED\r\n");
                    }
                }
            }
        }
        else
        {
            ps2_bad_cnt++;
            ps2_idle();
        }
        digital_prev = digital_now;

        /* ---------- 3b. Failsafe tay cam, chi o PS2 ---------- */
        if (mode == CTRL_MODE_PS2)
        {
            failsafe = (uint8_t)((now_ms - last_valid_ps2_ms) >= PS2_TIMEOUT_MS);
            if (failsafe)
                ps2_disarm();
            if (failsafe != failsafe_prev)
                UART2_SendString(failsafe ? "[SAFE] KHONG CO ANALOG HOP LE -> velocity=0\r\n"
                                          : "[PS2 ] Analog hop le tro lai\r\n");
        }
        else
        {
            failsafe = 0U;
            last_valid_ps2_ms = now_ms;
        }
        failsafe_prev = failsafe;

        /* ---------- 3c. Heartbeat + NMT ---------- */
        link_lost = (uint8_t)(Drive_LinkLost(NODE_L) || Drive_LinkLost(NODE_R));

        if ((now_ms - last_nmt_ms) >= 200U)
        {
            last_nmt_ms = now_ms;
            nmt_bad  = Drive_CheckNmtState(NODE_L);
            nmt_bad |= Drive_CheckNmtState(NODE_R);
        }

        if (link_lost != link_lost_prev)
        {
            if (link_lost)
            {
                UART2_SendString("[DRV ] MAT HEARTBEAT node");
                UART2_SendString(Drive_LinkLost(NODE_L) ? " L" : "");
                UART2_SendString(Drive_LinkLost(NODE_R) ? " R" : "");
                UART2_SendString(" -> dung CA HAI\r\n");
                /* Lien lac quay lai cung KHONG duoc tu chay tiep. */
                RosLink_AutoDisarm("mat heartbeat driver");
                ps2_disarm();
            }
            else
            {
                UART2_SendString("[DRV ] Heartbeat tro lai\r\n");
                force_tx = 1U;
            }
            link_lost_prev = link_lost;
        }

        /* ---------- 3d. Nguon van toc theo mode ---------- */
        if (mode == CTRL_MODE_AUTO)
        {
            auto_ok = RosLink_Update(now_ms, &vel_l, &vel_r, &need_enable);

            if (need_enable)
            {
                /* ROS2 vua gui 0x030 = (0, 0): moi cho driver Enable. */
                UART2_SendString("[AUTO] nhan lenh 0 dau tien -> ENABLE\r\n");
                if (enable_both())
                {
                    RosLink_AutoArmDone();
                }
                else
                {
                    drive_fault = 1U;
                    RosLink_AutoDisarm("enable driver that bai");
                    lock_both();
                }
                vel_l = vel_r = 0;
                now_ms = next_loop_ms = Millis();
                force_tx = 1U;
            }

            if (!auto_ok)
                vel_l = vel_r = 0;
            armed_now = RosLink_AutoIsArmed();
        }
        else
        {
            vel_l = ps2.vel_l;
            vel_r = ps2.vel_r;
            armed_now = ps2.armed;
        }

        if (nmt_bad)
        {
            ps2_disarm();
            RosLink_AutoDisarm("NMT driver khong Operational");
        }

        /* ---------- 3e. Phuc hoi CiA 402 (chi khi dang ARMED) ---------- */
        if (!armed_now || ((mode == CTRL_MODE_PS2) && digital_now) || link_lost || nmt_bad)
        {
            drive_fault = 1U;
        }
        else if ((now_ms - last_recover_ms) >= DRIVE_RECOVER_PERIOD_MS)
        {
            last_recover_ms = now_ms;
            drive_fault  = Drive_CheckAndRecover(NODE_L);
            drive_fault |= Drive_CheckAndRecover(NODE_R);

            if (drive_fault)
            {
                ps2_disarm();
                RosLink_AutoDisarm("driver loi / chua Enable");
            }
        }

        /*
         * ---------- 4. Hoi driver: 1 SDO / node / 20 ms, 5 slot ----------
         *   slot 0,1,3 : 0x6064
         *   slot 2     : 0x606C neu TPDO van toc da thiu, nguoc lai 0x6064
         *   slot 4     : 0x6041
         */
        if ((now_ms - last_sdo_ms) >= ROS_POS_READ_PERIOD_MS)
        {
            uint8_t n;

            last_sdo_ms = now_ms;
            for (n = NODE_L; n <= NODE_R; n++)
            {
                uint16_t idx = 0x6064U;

                if (sdo_slot == 4U)
                    idx = 0x6041U;
                else if ((sdo_slot == 2U) && !Drive_ActualVelFresh(n))
                    idx = 0x606CU;
                (void)SDO_Read_Request(n, idx, 0x00U);
            }
            sdo_slot = (uint8_t)((sdo_slot + 1U) % 5U);
        }

        now_ms = Millis();   /* CheckAndRecover co the vua chan ~60 ms */

        /*
         * ---------- 5. Cong an toan cuoi cung truoc khi ra CAN ----------
         * locked = chua ARM / loi -> RPDO mang CW 0x0006 (Shutdown), van gui
         *          dinh ky de nuoi watchdog RPDO cua driver.
         * Watchdog AUTO het han -> chi v = 0, driver van Enabled.
         */
        locked = (uint8_t)(link_lost || nmt_bad || drive_fault);
        if (mode == CTRL_MODE_PS2)
        {
            locked |= (uint8_t)(!ps2.armed || digital_now || failsafe);
            if (locked)
                vel_l = vel_r = 0;
        }
        else
        {
            locked |= (uint8_t)!RosLink_AutoIsArmed();
            if (locked || RosLink_AutoWatchdogExpired(now_ms))
                vel_l = vel_r = 0;
            vel_l = clamp_vel(vel_l);       /* tran cung, doc lap ros_link */
            vel_r = clamp_vel(vel_r);
        }

        if (locked != prev_locked)
        {
            force_tx = 1U;
            prev_locked = locked;
        }

        if (force_tx || (vel_l != prev_vel_l) || (vel_r != prev_vel_r) ||
            ((now_ms - last_rpdo_ms) >= RPDO_KEEPALIVE_MS))
        {
            uint8_t tx_ok;

            if (locked)
            {
                tx_ok  = Drive_Lock(NODE_L);
                tx_ok &= Drive_Lock(NODE_R);
            }
            else
            {
                tx_ok  = Drive_SetVelocity(NODE_L, vel_l);
                tx_ok &= Drive_SetVelocity(NODE_R, vel_r);
            }

            if (tx_ok)
            {
                force_tx = 0U;
                prev_vel_l = vel_l;
                prev_vel_r = vel_r;
                last_rpdo_ms = now_ms;
            }
            else
            {
                drive_fault = 1U;
                ps2_disarm();
                RosLink_AutoDisarm("CAN RPDO transmit fail");
                force_tx = 1U;
            }
        }

        /* ---------- 5b. Phan hoi ve ROS2 ---------- */
        RosLink_SendFeedback();
        RosLink_SendStatus(now_ms,
            (uint8_t)(((RosLink_AutoIsArmed() || ps2.armed) ? ROS_FLAG_ARMED : 0U) |
                      (link_lost   ? ROS_FLAG_LINK_LOST : 0U) |
                      (nmt_bad     ? ROS_FLAG_NMT_BAD : 0U) |
                      (drive_fault ? ROS_FLAG_DRV_FAULT : 0U) |
                      (RosLink_AutoWatchdogExpired(now_ms) ? ROS_FLAG_WD_EXPIRED : 0U) |
                      (ok          ? ROS_FLAG_PS2_OK : 0U)));

        /* ---------- 6. Log ---------- */
        if ((now_ms - last_telem_ms) >= ROS_UART_TELEM_PERIOD_MS)
        {
            last_telem_ms = now_ms;
            RosLink_PrintTelemetry(now_ms);
        }

        if ((now_ms - last_debug_ms) >= UART_DEBUG_PERIOD_MS)
        {
            uint8_t n;

            last_debug_ms = now_ms;
            RosLink_PrintStats();

            UART2_SendFrame("[PS2 RX] ", ps2_rx, PS2_FRAME_MAX);
            if (digital_now)
                UART2_SendString("[PS2 ] dang chay o che do DIGITAL\r\n");
            else if (!ok)
                UART2_SendString("[SPI ERR] timeout / header la\r\n");
            UART2_SendButtonList("[BTN NOW]   ", ps2.pressed);
            UART2_SendButtonList("[BTN EVENT] ", button_event_latch);
            button_event_latch = 0U;

            UART2_SendString("[CAN] txOK=");
            UART2_SendUInt32(g_can_tx_ok);
            UART2_SendString(" txFail=");
            UART2_SendUInt32(g_can_tx_fail);
            UART2_SendString(" cancel=");
            UART2_SendUInt32(g_can_tx_cancel);
            UART2_SendString(" rx=");
            UART2_SendUInt32(g_can_rx_cnt);
            UART2_SendString(" | PS2 badFrames=");
            UART2_SendUInt32(ps2_bad_cnt);
            UART2_SendString("\r\n");
            CAN_PrintErrorStatus();

            for (n = NODE_L; n <= NODE_R; n++)
            {
                UART2_SendString((n == NODE_L) ? "[DRV] L: SW=0x" : "[DRV] R: SW=0x");
                UART2_SendHex16(g_status[n]);
                UART2_SendString(" hb=");
                UART2_SendUInt32(now_ms - g_hb_ms[n]);
                UART2_SendString("ms NMT=0x");
                UART2_SendHexByte(g_nmt_state[n]);
                UART2_SendString(Drive_ActualVelFresh(n) ? "" : " TPDO/0x606C THIU");
                UART2_SendString("\r\n");
            }
            if (drive_fault)
                UART2_SendString("[DRV] <<< LOI / CHUA ARM\r\n");

            if (UART2_GetDropped() != 0UL)
            {
                UART2_SendString("[UART] da vut ");
                UART2_SendUInt32(UART2_GetDropped());
                UART2_SendString(" byte log\r\n");
            }
            UART2_SendString("\r\n");
        }

        /* ---------- 7. Giu chu ky vong lap, hut CAN trong luc cho ---------- */
        next_loop_ms += LOOP_PERIOD_MS;
        if ((int32_t)(Millis() - next_loop_ms) >= 0)
            next_loop_ms = Millis();
        else
            while ((int32_t)(Millis() - next_loop_ms) < 0){
                CAN_Poll();
								RosLink_SendFeedback();
						}
    }
}
