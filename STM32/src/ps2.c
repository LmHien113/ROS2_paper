#include "ps2.h"
#include "bsp_clock.h"
#include "uart_debug.h"

#define PS2_ATT_PORT    GPIOE
#define PS2_ATT_PIN     GPIO_Pin_3

/* ======================================================================== */
/*                                   SPI1                                   */
/* ======================================================================== */

static void spi_clear_rx(void)
{
    volatile uint16_t dummy;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != RESET)
        dummy = SPI_I2S_ReceiveData(SPI1);

    /* Xoa OVR: doc DR roi SR. */
    dummy = SPI1->DR;
    dummy = SPI1->SR;
    (void)dummy;
}

static uint8_t spi_xfer(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = PS2_SPI_TIMEOUT;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        if (timeout-- == 0UL)
            return 0U;

    SPI_I2S_SendData(SPI1, tx);

    timeout = PS2_SPI_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        if (timeout-- == 0UL)
            return 0U;

    *rx = (uint8_t)SPI_I2S_ReceiveData(SPI1);
    Delay_Us(PS2_INTERBYTE_US);
    return 1U;
}

static void att_begin(void)
{
    spi_clear_rx();
    GPIO_ResetBits(PS2_ATT_PORT, PS2_ATT_PIN);
    Delay_Us(PS2_LEADIN_US);
}

static uint8_t att_end(uint8_t result)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) != RESET) { }
    GPIO_SetBits(PS2_ATT_PORT, PS2_ATT_PIN);
    Delay_Us(PS2_TAIL_US);
    return result;
}

void PS2_SPI_Init(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef spi;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);

    /* APB2 = HCLK/4 = 42 MHz -> SPI1 = 42 MHz / 256 = 164 kHz. */
    RCC_PCLK2Config(RCC_HCLK_Div4);

    /* SPI1: PA5 = SCK, PA6 = DATA, PA7 = CMD */
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    /* ATT: PE3, tich cuc muc thap */
    gpio.GPIO_Pin = PS2_ATT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(PS2_ATT_PORT, &gpio);
    GPIO_SetBits(PS2_ATT_PORT, PS2_ATT_PIN);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    SPI_I2S_DeInit(SPI1);

    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_1Edge;      /* mode 2: do tren board, mode 3 lech 1 bit */
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    spi.SPI_FirstBit = SPI_FirstBit_LSB;
    SPI_Init(SPI1, &spi);
    SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
    SPI_Cmd(SPI1, ENABLE);

    spi_clear_rx();
}

/* ======================================================================== */
/*                                PS2 frame                                 */
/* ======================================================================== */

/* Gui nguyen mot frame co do dai co dinh (dung cho lenh cau hinh). */
static uint8_t ps2_raw_frame(const uint8_t *tx, uint8_t *rx, uint8_t length)
{
    uint8_t i;

    att_begin();
    for (i = 0U; i < length; i++)
        if (!spi_xfer(tx[i], &rx[i]))
            return att_end(0U);

    return att_end(1U);
}

/*
 * Doc frame poll: doc 3 byte header roi doc tiep dung so byte theo header,
 * khong nha ATT giua chung. Frame digital (0x41) bi tu choi sau header.
 */
uint8_t PS2_ReadPoll(uint8_t *rx)
{
    uint8_t i;
    uint8_t total;

    for (i = 0U; i < PS2_FRAME_MAX; i++)
        rx[i] = 0U;

    att_begin();

    if (!spi_xfer(0x01U, &rx[0]) || !spi_xfer(0x42U, &rx[1]) ||
        !spi_xfer(0x00U, &rx[2]) || (rx[2] != 0x5AU))
        return att_end(0U);

    if (rx[1] == 0x73U)
        total = 9U;
    else if (rx[1] == 0x79U)
        total = 21U;
    else
        return att_end(0U);             /* 0x41 digital hoac header la */

    for (i = 3U; i < total; i++)
        if (!spi_xfer(0x00U, &rx[i]))
            return att_end(0U);

    return att_end(1U);
}

static uint8_t ps2_is_analog(const uint8_t *rx)
{
    return (uint8_t)((rx[2] == 0x5AU) && ((rx[1] == 0x73U) || (rx[1] == 0x79U)));
}

uint8_t PS2_IsDigitalFrame(const uint8_t *rx)
{
    return (uint8_t)((rx[2] == 0x5AU) && (rx[1] == 0x41U));
}

/* Bit nut PS2 tich cuc THAP -> doi sang 1 = dang nhan. */
uint16_t PS2_GetAllPressed(const uint8_t *rx)
{
    uint16_t raw = (uint16_t)(((uint16_t)rx[4] << 8) | rx[3]);
    return (uint16_t)(~raw & (uint16_t)~PS2_STUCK_MASK);
}

uint16_t PS2_GetPressedButtons(const uint8_t *rx)
{
    return (uint16_t)(PS2_GetAllPressed(rx) & BTN_USED_MASK);
}

void PS2_EnterAnalog(void)
{
    static const uint8_t poll[5]       = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U};
    static const uint8_t enter_cfg[5]  = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U};
    static const uint8_t set_analog[9] = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U,
                                          0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t exit_cfg[9]   = {0x01U, 0x43U, 0x00U, 0x00U,
                                          0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};
    uint8_t rx[PS2_FRAME_MAX];
    uint8_t retry;

    GPIO_SetBits(PS2_ATT_PORT, PS2_ATT_PIN);
    Delay_Ms(500UL);

    for (retry = 0U; retry < 5U; retry++)
    {
        (void)ps2_raw_frame(poll, rx, 5U);
        Delay_Ms(16UL);
    }

    for (retry = 0U; retry < 3U; retry++)
    {
        (void)ps2_raw_frame(enter_cfg,  rx, 5U);  Delay_Ms(16UL);
        (void)ps2_raw_frame(set_analog, rx, 9U);  Delay_Ms(16UL);
        (void)ps2_raw_frame(exit_cfg,   rx, 9U);  Delay_Ms(16UL);

        if (PS2_ReadPoll(rx))
            break;
        Delay_Ms(50UL);
    }

    (void)PS2_ReadPoll(rx);   /* rx[1] van duoc doc ke ca khi that bai */

    if (ps2_is_analog(rx))
        UART2_SendString("[PS2 ] ANALOG OK\r\n");
    else if (rx[1] == 0x41U)
        UART2_SendString("[PS2 ] DIGITAL - bam nut ANALOG tren tay cam\r\n");
    else
        UART2_SendString("[PS2 ] KHONG doc duoc header hop le\r\n");
}
