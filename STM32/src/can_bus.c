#include "can_bus.h"
#include "bsp_clock.h"
#include "uart_debug.h"

uint32_t g_can_tx_ok = 0U;
uint32_t g_can_tx_fail = 0U;
uint32_t g_can_tx_cancel = 0U;
uint32_t g_can_rx_cnt = 0U;

/*
 * 21 tq/bit (1 + 15 + 5), sample point 76%, SJW 4. Da do: 14 tq + SJW 1
 * lam mat 30-50% frame tu driver. APB1 = 42 MHz.
 */
#define CAN_TQ_PER_BIT   21UL
#define CAN_PRESCALER    (42000UL / (CAN_TQ_PER_BIT * (uint32_t)CAN_BITRATE_KBPS))

#if ((42000UL % (21UL * CAN_BITRATE_KBPS)) != 0UL)
  #error "CAN_BITRATE_KBPS khong chia het APB1 42 MHz voi 21 tq/bit"
#endif

/* Bank 17 dung mask 0x7EF de bat ca 0x020 lan 0x030 (chi khac bit 4). */
#if (((CANID_ROS_MODE_CMD) & 0x010U) != 0U) || \
    (((CANID_ROS_MODE_CMD) | 0x010U) != (CANID_ROS_VEL_CMD))
  #error "CANID_ROS_VEL_CMD phai = CANID_ROS_MODE_CMD | 0x010 (filter bank 17)"
#endif

static void filter_mask32(uint8_t bank, uint16_t id, uint16_t mask, uint16_t fifo)
{
    CAN_FilterInitTypeDef f;

    f.CAN_FilterNumber         = bank;
    f.CAN_FilterMode           = CAN_FilterMode_IdMask;
    f.CAN_FilterScale          = CAN_FilterScale_32bit;
    f.CAN_FilterIdHigh         = (uint16_t)(id << 5);
    f.CAN_FilterIdLow          = 0x0000U;
    f.CAN_FilterMaskIdHigh     = (uint16_t)(mask << 5);
    f.CAN_FilterMaskIdLow      = 0x0000U;
    f.CAN_FilterFIFOAssignment = fifo;
    f.CAN_FilterActivation     = ENABLE;
    CAN_FilterInit(&f);
}

void CAN_Motor_Init(void)
{
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef f;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

    /* CAN2: RX = PB5, TX = PB13 (AF9) */
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);
    can.CAN_ABOM = ENABLE;          /* tu thoat Bus-Off             */
    can.CAN_AWUM = ENABLE;
    can.CAN_TXFP = ENABLE;          /* phat theo thu tu nap         */
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_4tq;
    can.CAN_BS1  = CAN_BS1_15tq;
    can.CAN_BS2  = CAN_BS2_5tq;
    can.CAN_Prescaler = (uint16_t)CAN_PRESCALER;

    if (CAN_Init(CAN2, &can) != CAN_InitStatus_Success)
        UART2_SendString("[CAN ] init FAILED\r\n");

    /* Chia bank TRUOC khi cau hinh filter. */
    CAN_SlaveStartBank(14U);

    /*
     * FIFO0 = luong so lieu (SDO, TPDO), co the don cuc khi vong lap bi chan.
     * FIFO1 = nhung gi quyet dinh robot co duoc chay: heartbeat + lenh ROS2.
     */
    filter_mask32(14U, 0x580U, 0x7FCU, CAN_Filter_FIFO0);              /* SDO   */
    filter_mask32(15U, 0x700U, 0x7FCU, CAN_Filter_FIFO1);              /* HB    */
    filter_mask32(16U, 0x180U, 0x7FCU, CAN_Filter_FIFO0);              /* TPDO1 */
    filter_mask32(17U, CANID_ROS_MODE_CMD, 0x7EFU, CAN_Filter_FIFO1);  /* 020/030 */

    /* Bank 18: list 16-bit chinh xac cho 0x040 va 0x050. */
    f.CAN_FilterNumber         = 18U;
    f.CAN_FilterMode           = CAN_FilterMode_IdList;
    f.CAN_FilterScale          = CAN_FilterScale_16bit;
    f.CAN_FilterIdHigh         = (uint16_t)(CANID_ROS_ROTATE_CMD << 5);
    f.CAN_FilterIdLow          = (uint16_t)(CANID_ROS_FB_START   << 5);
    f.CAN_FilterMaskIdHigh     = (uint16_t)(CANID_ROS_ROTATE_CMD << 5);
    f.CAN_FilterMaskIdLow      = (uint16_t)(CANID_ROS_FB_START   << 5);
    f.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;
    f.CAN_FilterActivation     = ENABLE;
    CAN_FilterInit(&f);
}

/*
 * CAN_Transmit() chi la "da nap mailbox". Phai cho CAN_TxStatus_Ok; qua han
 * thi CancelTransmit, neu khong bxCAN retry vo han va ket ca 3 mailbox.
 */
uint8_t CAN_SendFrame(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    CanTxMsg msg;
    uint8_t mailbox = CAN_TxStatus_NoMailBox;
    uint8_t i;
    uint16_t t;

    msg.StdId = std_id;
    msg.ExtId = 0U;
    msg.IDE = CAN_Id_Standard;
    msg.RTR = CAN_RTR_Data;
    msg.DLC = dlc;
    for (i = 0U; i < 8U; i++)
        msg.Data[i] = (data != 0) ? data[i] : 0U;

    for (t = 0U; t < (uint16_t)(CAN_TX_MAILBOX_WAIT_US / 10UL); t++)
    {
        mailbox = CAN_Transmit(CAN2, &msg);
        if (mailbox != CAN_TxStatus_NoMailBox)
            break;
        Delay_Us(10U);
    }

    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        g_can_tx_fail++;
        return 0U;
    }

    for (t = 0U; t < (uint16_t)(CAN_TX_TIMEOUT_US / 10UL); t++)
    {
        if (CAN_TransmitStatus(CAN2, mailbox) == CAN_TxStatus_Ok)
        {
            g_can_tx_ok++;
            return 1U;
        }
        Delay_Us(10U);
    }

    CAN_CancelTransmit(CAN2, mailbox);
    g_can_tx_cancel++;
    g_can_tx_fail++;
    return 0U;
}

/* LEC: 3 = khong ai ACK, 5 = Bit dominant (sai chan RX / CANH-CANL). */
void CAN_PrintErrorStatus(void)
{
    uint32_t esr = CAN2->ESR;
    uint32_t lec = (esr >> 4) & 0x07UL;

    UART2_SendString("[CAN ERR] LEC=");
    UART2_SendUInt32(lec);
    UART2_SendString(" TEC=");
    UART2_SendUInt32((esr >> 16) & 0xFFUL);
    UART2_SendString(" REC=");
    UART2_SendUInt32((esr >> 24) & 0xFFUL);

    if ((esr & CAN_ESR_EWGF) != 0UL) UART2_SendString(" EWGF");
    if ((esr & CAN_ESR_EPVF) != 0UL) UART2_SendString(" EPVF");
    if ((esr & CAN_ESR_BOFF) != 0UL) UART2_SendString(" BUS-OFF");

    if (lec == 3UL)
        UART2_SendString("  <- ACK error: khong ai tra loi tren bus");
    else if (lec == 5UL)
        UART2_SendString("  <- Bit error: kiem tra chan RX / CANH-CANL");

    UART2_SendString("\r\n");
}
