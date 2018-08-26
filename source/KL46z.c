#include"KL46z.h"


void init_led()
{
    /*config LCK*/
    SIM_SCGC5 |= (1U << SIM_SCGC5_SHIFT(PORTD)) | (1U << SIM_SCGC5_SHIFT(PORTE));

    /*config mux GPIO*/
    PORT_PCR(PORTD, GREEN_LED_PIN) = ((PORT_PCR(PORTD, GREEN_LED_PIN)) & (~PORT_PCR_MUX_MASK)) | (1U << PORT_PCR_MUX_SHIFT);
    PORT_PCR(PORTE, RED_LED_PIN) = ((PORT_PCR(PORTE, RED_LED_PIN)) & (~PORT_PCR_MUX_MASK)) | (1U << PORT_PCR_MUX_SHIFT);

    /*config output*/
    GPIO_PDDR(PORTD) |= (1U << GREEN_LED_PIN);
    GPIO_PDDR(PORTE) |= (1U << RED_LED_PIN);

    /*turn off led*/
    led_off(PORTD, GREEN_LED_PIN);
    led_off(PORTE, RED_LED_PIN);
}

void clock_config()
{
    /*config clock Rx Tx*/
    SIM_SCGC5 |= (1U << SIM_SCGC5_SHIFT(PORTA));
    /*setting low clock 32kHz and FLL 1464 for clock 48MHz*/
    MCG_C4 = (MCG_C4 & 0xE0u) | (1u << MCG_C4_DMX32_SHIFT) | (1u << MCG_C4_DRS_SHIFT);
    SystemCoreClockUpdate();
    /*enable clock*/
    SIM_SCGC4 |= (1u << SIM_SCGC4_UART0_SHIFT);
}

void uart0_config(u32_t uart0clk, u32_t baud_rate)
{
    u32_t calculated_baud = 0;
    u32_t baud_diff = 0;
    u32_t osr_val = 0;
    u32_t sbr_val = 0;
    u32_t temp = 0;
    u8_t i = 0;

    /*config mux GPIO*/
    PORT_PCR(PORTA,RX_PIN) = ((PORT_PCR(PORTA,RX_PIN)) & (~PORT_PCR_MUX_MASK)) | (2U << PORT_PCR_MUX_SHIFT);
    PORT_PCR(PORTA,TX_PIN) = ((PORT_PCR(PORTA,TX_PIN)) & (~PORT_PCR_MUX_MASK)) | (2U << PORT_PCR_MUX_SHIFT);

    /*setting NVIC*/
    NVIC_ICPR |= (1U << ID_INT_UART0);
    NVIC_IPR(3) |= (1U << NVIC_IPR_PRI_N0_SHIFT);
    NVIC_ISER |= (1U << ID_INT_UART0);

    /*******************************************************************/

    /*Calculate the first baud rate using the lowest OSR value possible.*/
    i = 4;
    sbr_val = (u32_t)(uart0clk / (baud_rate * (i+1)));
    calculated_baud = (uart0clk / (sbr_val * (i+1)));

    if (calculated_baud > baud_rate)
        baud_diff = calculated_baud - baud_rate;
    else
        baud_diff = baud_rate - calculated_baud;

    osr_val = i;

    /*Select the best OSR value*/
    for (i = 5; i <= 32; i++)
    {
        sbr_val = (u32_t)(uart0clk / (baud_rate * (i+1)));
        calculated_baud = (uart0clk / (sbr_val * (i+1)));

        if (calculated_baud > baud_rate)
            temp = calculated_baud - baud_rate;
        else
            temp = baud_rate - calculated_baud;

        if (temp <= baud_diff)
        {
            baud_diff = temp;
            osr_val = i;
        }
    }
    sbr_val = (u32_t)(uart0clk / (baud_rate * (osr_val+1)));

    if (baud_diff >= (baud_rate/100)*3)
    {
        while(1);
    }
    /*disable receive and transmit*/
    UART0_C2 &= (~0x0Cu);
    /*config source clock MCGFLLCLK*/
    SIM_SOPT2 = (SIM_SOPT2 & (~(3u << SIM_SOPT2_UART0SRC_SHIFT))) | (1u << SIM_SOPT2_UART0SRC_SHIFT) | (0u << SIM_SOPT2_PLLFLLSEL_SHIFT);
    /*config pin Tx Rx*/
    SIM_SOPT5 &= ~(1u << SIM_SOPT5_UART0RXSRC_SHIFT | 3u << SIM_SOPT5_UART0TXSRC_SHIFT);
    // Setup OSR value
    UART0_C4 = (UART0_C4 & ~(0x1Fu | 1 << UART0_C4_M10_SHIFT)) | (osr_val&0x1Fu);

    /* Save off the current value of the uartx_BDH except for the SBR field */
    UART0_BDH = (UART0_BDH & ~(0x1Fu)) | ((sbr_val & 0x1F00u)>>8);
    UART0_BDH &= ~(1u << UART0_BDH_SBNS_SHIFT);

    UART0_BDL = (u8_t)(sbr_val & 0xFFu);

    UART0_C1= 0x00u;
    /*clear NF flag*/
    UART0_S1 |= (1<<2u);
    /* Enable receiver and transmitter and interrupt receive*/
    UART0_C2 = 0x0Cu | 1<<UART0_C2_RIE_SHIFT;
}

void led_on(u8_t port, u8_t pin)
{
    GPIO_PCOR(port) |= (1U << pin);
}

void led_off(u8_t port, u8_t pin)
{
    GPIO_PSOR(port) |= (1U << pin);
}

void toggle_led(u8_t port, u8_t pin)
{
    GPIO_PTOR(port) |= (1U << pin);
}

void usart_send_byte(u8_t data_input)
{
    while (!(UART0_S1 & (1 << UART0_S1_TDRE_SHIFT)));
    UART0_D = data_input;
}


char usart_receive_byte()
{
    while (!(UART0_S1 & (1 << UART0_S1_RDRF_SHIFT)));
    return UART0_D;
}

void usart_send_string(unsigned char *str_data)
{
    while (*str_data)
    {
        usart_send_byte(*str_data);
        str_data++;
    }
}

void UART0_IRQHandler()
{
    u8_t ch=0;

    ch=UART0_D;

    usart_send_byte(ch);
}

