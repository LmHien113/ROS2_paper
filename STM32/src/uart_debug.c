#include "uart_debug.h"

#if ((UART_TX_BUF_SIZE & (UART_TX_BUF_SIZE - 1U)) != 0U)
  #error "UART_TX_BUF_SIZE phai la luy thua cua 2"
#endif

#define UART_TX_BUF_MASK  (UART_TX_BUF_SIZE - 1U)

static volatile uint8_t  g_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t g_tx_head = 0U;    /* chi main ghi */
static volatile uint16_t g_tx_tail = 0U;    /* chi ISR ghi  */
static volatile uint32_t g_tx_dropped = 0U;

/* Xoa USART2_IRQHandler trong stm32f4xx_it.c neu bao "multiple definition". */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_TXE) == RESET)
        return;

    if (g_tx_head == g_tx_tail)
    {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
    else
    {
        USART_SendData(USART2, g_tx_buf[g_tx_tail]);
        g_tx_tail = (uint16_t)((g_tx_tail + 1U) & UART_TX_BUF_MASK);
    }
}

void UART2_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    USART_StructInit(&usart);
    usart.USART_BaudRate = UART_BAUDRATE;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usart);

    NVIC_SetPriority(USART2_IRQn, 2U);   /* thap hon SysTick */
    NVIC_EnableIRQ(USART2_IRQn);
    USART_Cmd(USART2, ENABLE);
}

void UART2_SendByte(uint8_t value)
{
    uint16_t next = (uint16_t)((g_tx_head + 1U) & UART_TX_BUF_MASK);
    uint32_t primask;

    if (next == g_tx_tail)
    {
        g_tx_dropped++;
        return;
    }

    g_tx_buf[g_tx_head] = value;
    g_tx_head = next;

    /* ISR cung sua CR1 -> khoa ngat quanh doc-sua-ghi. */
    primask = __get_PRIMASK();
    __disable_irq();
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    __set_PRIMASK(primask);
}

uint32_t UART2_GetDropped(void)
{
    return g_tx_dropped;
}

void UART2_Flush(void)
{
    while (g_tx_head != g_tx_tail) { }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) { }
}

void UART2_SendString(const char *text)
{
    while ((text != 0) && (*text != '\0'))
        UART2_SendByte((uint8_t)*text++);
}

void UART2_SendHexByte(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    UART2_SendByte((uint8_t)hex[value >> 4]);
    UART2_SendByte((uint8_t)hex[value & 0x0FU]);
}

void UART2_SendHex16(uint16_t value)
{
    UART2_SendHexByte((uint8_t)(value >> 8));
    UART2_SendHexByte((uint8_t)value);
}

void UART2_SendUInt32(uint32_t value)
{
    char buf[10];
    uint8_t i = 0U;

    do
    {
        buf[i++] = (char)('0' + (value % 10UL));
        value /= 10UL;
    } while (value != 0UL);

    while (i-- != 0U)
        UART2_SendByte((uint8_t)buf[i]);
}

void UART2_SendInt32(int32_t value)
{
    uint32_t u = (uint32_t)value;

    if (value < 0)
    {
        UART2_SendByte('-');
        u = ~u + 1UL;               /* an toan voi INT32_MIN */
    }
    UART2_SendUInt32(u);
}

void UART2_SendFrame(const char *name, const uint8_t *data, uint8_t length)
{
    uint8_t i;

    UART2_SendString(name);
    for (i = 0U; i < length; i++)
    {
        if (i != 0U)
            UART2_SendByte(' ');
        UART2_SendHexByte(data[i]);
    }
    UART2_SendString("\r\n");
}

void UART2_SendButtonList(const char *prefix, uint16_t pressed)
{
    static const struct { uint16_t bit; const char *name; } btn[] = {
        { BTN_UP, "UP " },             { BTN_DOWN, "DOWN " },
        { BTN_RIGHT, "RIGHT " },       { BTN_LEFT, "LEFT " },
        { BTN_TRIANGLE, "TRIANGLE " }, { BTN_SQUARE, "SQUARE " },
        { BTN_CIRCLE, "CIRCLE " },     { BTN_CROSS, "CROSS " },
    };
    uint8_t i;

    UART2_SendString(prefix);
    for (i = 0U; i < (uint8_t)(sizeof(btn) / sizeof(btn[0])); i++)
    {
        if ((pressed & btn[i].bit) != 0U)
            UART2_SendString(btn[i].name);
    }
    UART2_SendString(((pressed & BTN_USED_MASK) == 0U) ? "NONE\r\n" : "\r\n");
}
