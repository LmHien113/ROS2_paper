#include "canopen.h"
#include "can_bus.h"
#include "bsp_clock.h"
#include "uart_debug.h"
#include "ros_link.h"

int32_t  g_actual_vel_l = 0, g_actual_vel_r = 0;
int32_t  g_actual_pos_l = 0, g_actual_pos_r = 0;
uint32_t g_actual_vel_ms[3];
uint32_t g_actual_pos_ms[3];
uint16_t g_status[3];
uint32_t g_hb_ms[3];
uint8_t  g_nmt_state[3];
uint8_t  g_node_booted[3];
uint32_t g_sdo_write_fail = 0U;
DriveGear g_gear[3];

static uint32_t s_status_ms[3];       /* 0 = chua co statusword        */
static uint8_t  s_tpdo_valid[3];      /* TPDO1 da duoc xac nhan mapping */

#define VALID_NODE(n)            (((n) == NODE_L) || ((n) == NODE_R))
#define SDO_WRITE_TIMEOUT_MS     100UL
#define SDO_READ_TIMEOUT_MS      200UL
#define STATUS_STALE_TIMEOUT_MS  500UL

/*
 * Driver bo SDO thu hai neu gui chong vao cung node (do duoc 5%).
 * Moi SDO moi phai cach SDO doc truoc it nhat khoang nay.
 */
#define SDO_MIN_GAP_MS           5UL

static uint32_t s_sdo_req_ms[3];
static uint8_t  s_sdo_req_seen[3];

static void canopen_handle_frame(const CanRxMsg *rx);

/* ======================================================================== */
/*                               Tien ich                                   */
/* ======================================================================== */

static uint32_t rd_u32(const uint8_t *d)
{
    return (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
           ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}

static void wr_u32(uint8_t *d, uint32_t v)
{
    d[0] = (uint8_t)v;
    d[1] = (uint8_t)(v >> 8);
    d[2] = (uint8_t)(v >> 16);
    d[3] = (uint8_t)(v >> 24);
}

static void sdo_make(uint8_t *d, uint8_t cs, uint16_t index, uint8_t sub, uint32_t value)
{
    d[0] = cs;
    d[1] = (uint8_t)index;
    d[2] = (uint8_t)(index >> 8);
    d[3] = sub;
    wr_u32(&d[4], value);
}

static void sdo_print(const char *what, uint8_t node, uint16_t index, uint8_t sub)
{
    UART2_SendString("[SDO ] ");
    UART2_SendString(what);
    UART2_SendString(" node");
    UART2_SendUInt32(node);
    UART2_SendString(" idx=0x");
    UART2_SendHex16(index);
    UART2_SendString(" sub=");
    UART2_SendUInt32(sub);
}

static void sdo_print_abort(uint8_t node, const CanRxMsg *rx)
{
    sdo_print("ABORT", node,
              (uint16_t)(rx->Data[1] | ((uint16_t)rx->Data[2] << 8)), rx->Data[3]);
    UART2_SendString(" code=0x");
    UART2_SendHex16((uint16_t)(rd_u32(&rx->Data[4]) >> 16));
    UART2_SendHex16((uint16_t)rd_u32(&rx->Data[4]));
    UART2_SendString("\r\n");
}

static void receive_fifo(uint8_t fifo, CanRxMsg *rx)
{
    CAN_Receive(CAN2, fifo, rx);
    g_can_rx_cnt++;
}

static void sdo_wait_gap(uint8_t node)
{
    if (s_sdo_req_seen[node])
        while ((Millis() - s_sdo_req_ms[node]) < SDO_MIN_GAP_MS)
            CAN_Poll();
}

/*
 * Giai ma upload response expedited (0x42/0x43/0x47/0x4B/0x4F).
 * Segmented (bit 1 = 0) bi tu choi: Data[4..7] khi do la kich thuoc.
 */
static uint8_t sdo_parse_upload(const CanRxMsg *rx, uint32_t *out)
{
    uint8_t cmd = rx->Data[0];
    uint8_t nbytes = 4U;

    if (((cmd & 0xE0U) != 0x40U) || ((cmd & 0x02U) == 0U))
        return 0U;

    if ((cmd & 0x01U) != 0U)
        nbytes = (uint8_t)(4U - ((cmd >> 2) & 0x03U));

    *out = rd_u32(&rx->Data[4]);
    if (nbytes < 4U)
        *out &= (1UL << (8U * nbytes)) - 1UL;
    return 1U;
}

static void store_sdo_value(uint8_t node, uint16_t index, uint8_t sub, uint32_t value)
{
    if (sub != 0x00U)
        return;

    if (index == 0x606CU)
    {
        *((node == NODE_L) ? &g_actual_vel_l : &g_actual_vel_r) = (int32_t)value;
        g_actual_vel_ms[node] = Millis();
    }
    else if (index == 0x6064U)
    {
        *((node == NODE_L) ? &g_actual_pos_l : &g_actual_pos_r) = (int32_t)value;
        g_actual_pos_ms[node] = Millis();
    }
    else if (index == 0x6041U)
    {
        g_status[node] = (uint16_t)value;
        s_status_ms[node] = Millis();
    }
}

/* ======================================================================== */
/*                         SDO (CiA 301), co cho ket qua                    */
/* ======================================================================== */

/*
 * Gui 1 SDO va cho response cua DUNG (node, index, sub). Moi frame khac
 * den trong luc cho deu duoc xu ly binh thuong (heartbeat, lenh ROS2).
 * out = 0 -> ghi (cho 0x60), out != 0 -> doc (cho upload expedited).
 */
static uint8_t sdo_transact(uint8_t node, const uint8_t *req, uint32_t *out)
{
    CanRxMsg rx;
    uint16_t index = (uint16_t)(req[1] | ((uint16_t)req[2] << 8));
    uint32_t deadline;
    uint32_t value;

    if (!VALID_NODE(node))
        return 0U;

    sdo_wait_gap(node);
    CAN_Poll();                     /* xu ly het frame cu truoc khi gui */

    if (out != 0)
    {
        s_sdo_req_ms[node] = Millis();
        s_sdo_req_seen[node] = 1U;
    }

    if (!CAN_SendFrame((uint16_t)(0x600U + node), req, 8U))
        goto fail;

    deadline = Millis() + ((out != 0) ? SDO_READ_TIMEOUT_MS : SDO_WRITE_TIMEOUT_MS);

    while ((int32_t)(Millis() - deadline) < 0)
    {
        while (CAN_MessagePending(CAN2, CAN_FIFO1) > 0U)
        {
            receive_fifo(CAN_FIFO1, &rx);
            canopen_handle_frame(&rx);
        }

        while (CAN_MessagePending(CAN2, CAN_FIFO0) > 0U)
        {
            receive_fifo(CAN_FIFO0, &rx);

            if ((rx.StdId != (0x580UL + node)) || (rx.DLC < 4U) ||
                (rx.Data[1] != req[1]) || (rx.Data[2] != req[2]) ||
                (rx.Data[3] != req[3]))
            {
                canopen_handle_frame(&rx);
                continue;
            }

            if (rx.Data[0] == 0x80U)
            {
                if (rx.DLC >= 8U)
                    sdo_print_abort(node, &rx);
                goto fail;
            }

            if (out == 0)
            {
                if (rx.Data[0] == 0x60U)
                    return 1U;
                canopen_handle_frame(&rx);      /* response doc cu con trong FIFO */
                continue;
            }

            if ((rx.DLC < 8U) || !sdo_parse_upload(&rx, &value))
                goto fail;

            *out = value;
            store_sdo_value(node, index, req[3], value);
            return 1U;
        }
    }

    sdo_print((out != 0) ? "TIMEOUT doc" : "TIMEOUT ghi", node, index, req[3]);
    UART2_SendString("\r\n");

fail:
    if (out == 0)
        g_sdo_write_fail++;
    return 0U;
}

/* size = 1, 2, 4 byte -> command specifier 0x2F, 0x2B, 0x23. */
static uint8_t sdo_write(uint8_t node, uint16_t index, uint8_t sub,
                         uint32_t value, uint8_t size)
{
    uint8_t d[8];
    uint8_t cs = (size == 1U) ? 0x2FU : ((size == 2U) ? 0x2BU : 0x23U);

    sdo_make(d, cs, index, sub, value);
    return sdo_transact(node, d, 0);
}

/* Chi dung ngoai vong lap chinh (khoi tao, enable). */
static uint8_t sdo_read(uint8_t node, uint16_t index, uint8_t sub, uint32_t *out)
{
    uint8_t d[8];

    *out = 0UL;
    sdo_make(d, 0x40U, index, sub, 0UL);
    return sdo_transact(node, d, out);
}

uint8_t SDO_Read_Request(uint8_t node, uint16_t index, uint8_t sub)
{
    uint8_t d[8];

    if (!VALID_NODE(node))
        return 0U;

    sdo_wait_gap(node);
    s_sdo_req_ms[node] = Millis();
    s_sdo_req_seen[node] = 1U;

    sdo_make(d, 0x40U, index, sub, 0UL);
    return CAN_SendFrame((uint16_t)(0x600U + node), d, 8U);
}

/* NMT 0x000: 0x80 = Pre-operational, 0x01 = Start. */
static uint8_t nmt_send(uint8_t cmd, uint8_t node)
{
    uint8_t d[8] = {0U};

    d[0] = cmd;
    d[1] = node;
    return CAN_SendFrame(0x000U, d, 2U);
}

/* ======================================================================== */
/*                       Electronic gear (chi doc, luc boot)                */
/* ======================================================================== */

static uint32_t pulses_to_rpm(uint8_t node, uint32_t pulses_per_s)
{
    uint64_t den = (uint64_t)g_gear[node].ppr * g_gear[node].gear_den;

    if (!g_gear[node].valid || (den == 0ULL))
        return 0UL;

    return (uint32_t)((60ULL * pulses_per_s * g_gear[node].gear_num) / den);
}

static void gear_read_print(uint8_t node, const char *label)
{
    DriveGear *g = &g_gear[node];
    uint32_t rpm;
    uint8_t ok = 1U;

    ok &= sdo_read(node, 0x2A90U, 0x00U, &g->ppr);
    ok &= sdo_read(node, 0x2A93U, 0x01U, &g->gear_num);
    ok &= sdo_read(node, 0x2A93U, 0x02U, &g->gear_den);
    ok &= sdo_read(node, 0x607FU, 0x00U, &g->vmax);
    g->valid = (uint8_t)(ok && (g->ppr != 0UL) && (g->gear_num != 0UL) &&
                         (g->gear_den != 0UL));

    UART2_SendString("[GEAR] node ");
    UART2_SendString(label);

    if (!g->valid)
    {
        UART2_SendString(": DOC THAT BAI (sai Node-ID / khong co 0x2A90, 0x2A93)\r\n");
        return;
    }

    UART2_SendString(": 0x2A90=");
    UART2_SendUInt32(g->ppr);
    UART2_SendString(" gear=");
    UART2_SendUInt32(g->gear_num);
    UART2_SendString("/");
    UART2_SendUInt32(g->gear_den);
    UART2_SendString((g->gear_num == g->gear_den) ? " (1:1)" :
                     " (KHAC 1: 0x60FF va 0x606C khac thang)");

    rpm = pulses_to_rpm(node, (uint32_t)JOG_VEL);
    UART2_SendString("\r\n[GEAR]   JOG_VEL -> ");
    UART2_SendUInt32(rpm);
    UART2_SendString(" rpm");
    if (rpm > 6000UL)
        UART2_SendString("  <<< VUOT 6000, driver se chan");

    rpm = pulses_to_rpm(node, g->vmax);
    UART2_SendString(" | 0x607F=");
    UART2_SendUInt32(g->vmax);
    UART2_SendString(" -> ");
    UART2_SendUInt32(rpm);
    UART2_SendString(" rpm\r\n");
}

void Drive_DumpGearParams(void)
{
    gear_read_print(NODE_L, "L");
    gear_read_print(NODE_R, "R");

    if (!g_gear[NODE_L].valid || !g_gear[NODE_R].valid)
    {
        UART2_SendString("[GEAR] chua chot duoc he don vi -> KE BANH khi chay thu\r\n");
    }
    else if ((g_gear[NODE_L].ppr      != g_gear[NODE_R].ppr) ||
             (g_gear[NODE_L].gear_num != g_gear[NODE_R].gear_num) ||
             (g_gear[NODE_L].gear_den != g_gear[NODE_R].gear_den))
    {
        UART2_SendString("[GEAR] *** HAI NODE LECH THAM SO -> robot di cong ***\r\n");
    }
}

/* ======================================================================== */
/*                          Khoi tao / dieu khien                           */
/* ======================================================================== */

/* RPDO1 = Controlword 0x6040 (16 bit) + Target Velocity 0x60FF (32 bit), async. */
static uint8_t rpdo1_map(uint8_t node)
{
    uint32_t cobid = 0x200UL + node;
    uint8_t ok = 1U;

    ok &= sdo_write(node, 0x1400U, 0x01U, 0x80000000UL | cobid, 4U);  /* tat */
    ok &= sdo_write(node, 0x1600U, 0x00U, 0U, 1U);
    ok &= sdo_write(node, 0x1600U, 0x01U, 0x60400010UL, 4U);
    ok &= sdo_write(node, 0x1600U, 0x02U, 0x60FF0020UL, 4U);
    ok &= sdo_write(node, 0x1600U, 0x00U, 2U, 1U);
    ok &= sdo_write(node, 0x1400U, 0x02U, 0xFFU, 1U);
    ok &= sdo_write(node, 0x1400U, 0x01U, cobid, 4U);                 /* bat */
    return ok;
}

/*
 * TPDO1 = 0x606C (32 bit) + 0x6041 (16 bit), async + event timer.
 * Doc lai mapping de chac chan; that bai thi van hoi bang SDO.
 */
static uint8_t tpdo1_map(uint8_t node)
{
    uint32_t cobid = 0x180UL + node;
    uint32_t map_vel = 0UL, map_sw = 0UL, tx_type = 0UL, event_ms = 0UL;
    uint8_t ok = 1U;

    s_tpdo_valid[node] = 0U;

    ok &= sdo_write(node, 0x1800U, 0x01U, 0x80000000UL | cobid, 4U);
    ok &= sdo_write(node, 0x1A00U, 0x00U, 0U, 1U);
    ok &= sdo_write(node, 0x1A00U, 0x01U, 0x606C0020UL, 4U);
    ok &= sdo_write(node, 0x1A00U, 0x02U, 0x60410010UL, 4U);
    ok &= sdo_write(node, 0x1A00U, 0x00U, 2U, 1U);
    ok &= sdo_write(node, 0x1800U, 0x02U, 0xFFU, 1U);
    ok &= sdo_write(node, 0x1800U, 0x03U, 0U, 2U);
    ok &= sdo_write(node, 0x1800U, 0x05U, TPDO1_EVENT_MS, 2U);
    ok &= sdo_write(node, 0x1800U, 0x01U, cobid, 4U);

    ok = (uint8_t)(ok &&
                   sdo_read(node, 0x1A00U, 0x01U, &map_vel) &&
                   sdo_read(node, 0x1A00U, 0x02U, &map_sw) &&
                   sdo_read(node, 0x1800U, 0x02U, &tx_type) &&
                   sdo_read(node, 0x1800U, 0x05U, &event_ms) &&
                   (map_vel == 0x606C0020UL) && (map_sw == 0x60410010UL) &&
                   (tx_type == 0xFFUL) && (event_ms == (uint32_t)TPDO1_EVENT_MS));

    s_tpdo_valid[node] = ok;
    if (!ok)
    {
        UART2_SendString("[TPDO] node");
        UART2_SendUInt32(node);
        UART2_SendString(" remap that bai -> hoi bang SDO\r\n");
    }
    return ok;
}

uint8_t Drive_SetRamp(uint8_t node, uint32_t accel, uint32_t decel)
{
    uint8_t ok = sdo_write(node, 0x6083U, 0x00U, accel, 4U);

    ok &= sdo_write(node, 0x6084U, 0x00U, decel, 4U);
    return ok;
}

/*
 * CiA 402: target = 0 -> Fault reset -> Shutdown -> Switch on -> target = 0
 * lan hai -> Enable Operation. Chi thanh cong khi statusword = Op Enabled.
 */
uint8_t Drive_EnableOperation(uint8_t node)
{
    static const struct { uint16_t index; uint32_t value; uint8_t size; } seq[] = {
        { 0x60FFU, 0UL,     4U },
        { 0x6040U, 0x0080U, 2U },   /* Fault reset */
        { 0x6040U, 0x0006U, 2U },   /* Shutdown    */
        { 0x6040U, 0x0007U, 2U },   /* Switch on   */
        { 0x60FFU, 0UL,     4U },
        { 0x6040U, 0x000FU, 2U },   /* Enable Op   */
    };
    uint32_t status = 0UL;
    uint8_t ok = 1U;
    uint8_t i;

    if (!VALID_NODE(node))
        return 0U;

    for (i = 0U; i < (uint8_t)(sizeof(seq) / sizeof(seq[0])); i++)
    {
        ok &= sdo_write(node, seq[i].index, 0x00U, seq[i].value, seq[i].size);
        Delay_Ms(10UL);
    }

    return (uint8_t)(ok && sdo_read(node, 0x6041U, 0x00U, &status) &&
                     SW_OP_ENABLED((uint16_t)status));
}

static void reset_recovery(uint8_t node);

/* Cau hinh day du 1 driver. Chan ~270 ms. */
uint8_t Drive_Init(uint8_t node)
{
    uint8_t ok = 1U;

    if (!VALID_NODE(node))
        return 0U;

    /* Xoa so lieu cu de khong coi la hien tai. */
    g_node_booted[node] = 0U;
    g_status[node] = 0U;
    s_status_ms[node] = 0U;
    g_actual_vel_ms[node] = 0U;
    g_actual_pos_ms[node] = 0U;
    *((node == NODE_L) ? &g_actual_vel_l : &g_actual_vel_r) = 0;
    *((node == NODE_L) ? &g_actual_pos_l : &g_actual_pos_r) = 0;

    /* PDO mapping phai lam o Pre-operational. */
    ok &= nmt_send(0x80U, node);
    Delay_Ms(20UL);
    ok &= sdo_write(node, 0x6060U, 0x00U, (uint32_t)MODE_PROFILE_VELOCITY, 1U);
    Delay_Ms(20UL);
    ok &= sdo_write(node, 0x1017U, 0x00U, HEARTBEAT_PERIOD_MS, 2U);
    Delay_Ms(20UL);
    ok &= rpdo1_map(node);
    (void)tpdo1_map(node);          /* khong bat buoc: co duong lui SDO */
    ok &= Drive_SetRamp(node, ACCEL_VAL, DECEL_VAL);

    if (ok)
    {
        ok &= nmt_send(0x01U, node);
        Delay_Ms(20UL);
        ok &= Drive_EnableOperation(node);
    }

    reset_recovery(node);

    if (!ok)
    {
        UART2_SendString("[DRV ] node");
        UART2_SendUInt32(node);
        UART2_SendString(" khoi tao THAT BAI: SDO timeout/abort\r\n");
    }
    return ok;
}

/* RPDO1: byte 0..1 = Controlword, byte 2..5 = Target Velocity. */
static uint8_t send_rpdo(uint8_t node, uint16_t controlword, int32_t velocity)
{
    uint8_t d[8] = {0U};

    if (!VALID_NODE(node))
        return 0U;

    d[0] = (uint8_t)controlword;
    d[1] = (uint8_t)(controlword >> 8);
    wr_u32(&d[2], (uint32_t)velocity);
    return CAN_SendFrame((uint16_t)(0x200U + node), d, 8U);
}

/* v = 0 van giu Operation Enabled (tha nut PS2 thi dung nhung san sang). */
uint8_t Drive_SetVelocity(uint8_t node, int32_t velocity)
{
    return send_rpdo(node, 0x000FU, velocity);
}

/* Shutdown + v = 0: driver ra khoi trang thai sinh moment. */
uint8_t Drive_Lock(uint8_t node)
{
    return send_rpdo(node, 0x0006U, 0);
}

void Drive_ForceSafe(uint8_t node)
{
    (void)sdo_write(node, 0x60FFU, 0x00U, 0UL, 4U);
    (void)sdo_write(node, 0x6040U, 0x00U, 0x0006U, 2U);
    (void)Drive_Lock(node);       /* neu RPDO da duoc map tu lan truoc */
}

/* ======================================================================== */
/*                        Nhan va phan loai frame CAN                       */
/* ======================================================================== */

static void canopen_handle_frame(const CanRxMsg *rx)
{
    uint16_t fc   = (uint16_t)(rx->StdId & 0x780U);
    uint8_t  node = (uint8_t)(rx->StdId & 0x07FU);
    uint32_t value;

    if ((fc == 0x700U) && (rx->DLC == 1U) && VALID_NODE(node))
    {
        /* Heartbeat; 0x00 = vua boot lai */
        g_hb_ms[node] = Millis();
        g_nmt_state[node] = rx->Data[0];
        if (rx->Data[0] == 0x00U)
            g_node_booted[node] = 1U;
    }
    else if (fc == 0x580U)
    {
        if (!VALID_NODE(node) || (rx->DLC < 8U))
            return;
        if (rx->Data[0] == 0x80U)
            sdo_print_abort(node, rx);
        else if (sdo_parse_upload(rx, &value))
            store_sdo_value(node, (uint16_t)(rx->Data[1] | ((uint16_t)rx->Data[2] << 8)),
                            rx->Data[3], value);
    }
    else if (fc == 0x180U)
    {
        /* TPDO1 = 0x606C (4 byte) + 0x6041 (2 byte) */
        if (!VALID_NODE(node) || !s_tpdo_valid[node] || (rx->DLC < 4U))
            return;
        store_sdo_value(node, 0x606CU, 0x00U, rd_u32(rx->Data));
        if (rx->DLC >= 6U)
            store_sdo_value(node, 0x6041U, 0x00U,
                            (uint32_t)(rx->Data[4] | ((uint16_t)rx->Data[5] << 8)));
    }
    else
    {
        (void)RosLink_HandleCanFrame((uint16_t)rx->StdId, rx->Data, rx->DLC);
    }
}

void CAN_Poll(void)
{
    CanRxMsg rx;

    while (CAN_MessagePending(CAN2, CAN_FIFO0) > 0U)
    {
        receive_fifo(CAN_FIFO0, &rx);
        canopen_handle_frame(&rx);
    }
    while (CAN_MessagePending(CAN2, CAN_FIFO1) > 0U)
    {
        receive_fifo(CAN_FIFO1, &rx);
        canopen_handle_frame(&rx);
    }
}

uint8_t Drive_ActualVelFresh(uint8_t node)
{
    return (uint8_t)(VALID_NODE(node) && (g_actual_vel_ms[node] != 0U) &&
                     ((Millis() - g_actual_vel_ms[node]) < ACTUAL_VEL_STALE_MS));
}

/* ======================================================================== */
/*                          Giam sat va phuc hoi                            */
/* ======================================================================== */

static uint32_t s_recover_next_ms[3];
static uint8_t  s_recover_attempts[3];
static uint8_t  s_recover_warned[3];
static uint8_t  s_recover_gave_up[3];

static void reset_recovery(uint8_t node)
{
    s_recover_next_ms[node]  = Millis();
    s_recover_attempts[node] = 0U;
    s_recover_warned[node]   = 0U;
    s_recover_gave_up[node]  = 0U;
}

static void log_node(const char *msg, uint8_t node)
{
    UART2_SendString("[DRV ] node");
    UART2_SendUInt32(node);
    UART2_SendString(msg);
}

uint8_t Drive_CheckAndRecover(uint8_t node)
{
    uint16_t s;

    if (!VALID_NODE(node))
        return 0U;

    /* Statusword chua co hoac da cu -> khong duoc lam dieu kien cho chay. */
    if ((s_status_ms[node] == 0U) ||
        ((Millis() - s_status_ms[node]) >= STATUS_STALE_TIMEOUT_MS))
    {
        if (!s_recover_warned[node])
        {
            s_recover_warned[node] = 1U;
            log_node(" 0x6041 chua co/da cu -> KHONG cho chay\r\n", node);
        }
        return 1U;
    }

    s = g_status[node];
    if (SW_OP_ENABLED(s))
    {
        s_recover_attempts[node] = 0U;
        return 0U;
    }

    if ((int32_t)(Millis() - s_recover_next_ms[node]) < 0)
        return 1U;
    s_recover_next_ms[node] = Millis() + 500U;     /* toi da 2 lan / giay */

    if (s_recover_attempts[node] >= 5U)
    {
        if (!s_recover_gave_up[node])
        {
            s_recover_gave_up[node] = 1U;
            log_node(" BO CUOC sau 5 lan - kiem tra co khi/nguon\r\n", node);
        }
        return 1U;
    }
    s_recover_attempts[node]++;

    log_node(" SW=0x", node);
    UART2_SendHex16(s);

    if (SW_FAULT(s))
    {
        UART2_SendString(" FAULT -> reset\r\n");
        (void)sdo_write(node, 0x6040U, 0x00U, 0x0080U, 2U);
    }
    else
    {
        UART2_SendString(" chua Enabled -> enable lai\r\n");
        if (!Drive_EnableOperation(node))
            UART2_SendString("[DRV ] enable lai that bai\r\n");
    }
    return 1U;
}

void Drive_LinkInit(void)
{
    g_hb_ms[NODE_L] = Millis();
    g_hb_ms[NODE_R] = Millis();
}

uint8_t Drive_LinkLost(uint8_t node)
{
    return (uint8_t)(!VALID_NODE(node) || ((Millis() - g_hb_ms[node]) >= NODE_TIMEOUT_MS));
}

/*
 * Node tut khoi Operational -> gui NMT Start va xoa bo dem phuc hoi;
 * Drive_CheckAndRecover() se chay lai chuoi Enable o nhip sau.
 */
uint8_t Drive_CheckNmtState(uint8_t node)
{
    static uint32_t next_try_ms[3];

    if (!VALID_NODE(node))
        return 0U;
    if (g_nmt_state[node] == 0x05U)
        return 0U;
    if (Drive_LinkLost(node))
        return 1U;
    if ((int32_t)(Millis() - next_try_ms[node]) < 0)
        return 1U;
    next_try_ms[node] = Millis() + 500U;

    log_node(" NMT=0x", node);
    UART2_SendHexByte(g_nmt_state[node]);
    UART2_SendString(" -> gui NMT Start + cho enable lai\r\n");

    (void)nmt_send(0x01U, node);
    reset_recovery(node);
    return 1U;
}
