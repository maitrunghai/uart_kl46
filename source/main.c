#include "KL46z.h"


void main(void)
{
    u8_t ch=0;
    init_led();
    clock_config();
    uart0_config(SystemCoreClock, 115200);

    usart_send_string("Hello World!\r\n");

    while(1);
    /*{
        ch = usart_receive_byte();
        usart_send_byte(ch);
    }*/
}
