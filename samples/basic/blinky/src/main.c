/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>

#include <ch32v30x.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void delay(uint32_t t)
{
	do{
		__asm("nop");
	}while(--t);
}

void error(uint32_t t)
{
	uint8_t i = 0;
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	t *= 2;

	do
    {
		GPIO_WriteBit(GPIOC, GPIO_Pin_0, (i == 0) ? (i = 1) : (i = 0));
        // k_msleep(SLEEP_TIME_MS);
		delay(1000 * 1000 * 2);
    }while(t--);
}

void debug_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, 1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, 1);
}


void putchar(const char ch)
{
	USART_SendData(USART1, ch);
}

void printch(const char ch)   //输出字符
{  
    putchar(ch);  
}  
 
 
void printint(const int dec)     //输出整型数
{  
    if(dec == 0)  
    {  
        return;  
    }  
    printint(dec / 10);  
    putchar((char)(dec % 10 + '0'));  
}  
 
 
void printstr(const char *ptr)        //输出字符串
{  
    while(*ptr)  
    {  
        putchar(*ptr);  
        ptr++;  
    }  
}  


void my_printf(const char *format,...)  
{  
    va_list ap;  
    va_start(ap,format);     //将ap指向第一个实际参数的地址
    while(*format)  
    {  
        if(*format != '%')  
        {  
            putchar(*format);  
            format++;  
        }  
        else  
        {  
            format++;  
            switch(*format)  
            {  
                case 'c':  
                {  
                    char valch = va_arg(ap,int);  //记录当前实践参数所在地址
                    printch(valch);  
                    format++;  
                    break;  
                }  
                case 'd':  
                {  
                    int valint = va_arg(ap,int);  
                    printint(valint);  
                    format++;  
                    break;  
                }  
                case 's':  
                {  
                    char *valstr = va_arg(ap,char *);  
                    printstr(valstr);  
                    format++;  
                    break;  
                }  
                // case 'f':  
                // {  
                //     float valflt = va_arg(ap,double);  
                //     printfloat(valflt);  
                //     format++;  
                //     break;  
                // }  
                default:  
                {  
                    printch(*format);  
                    format++;  
                }  
            }    
        }  
    }
    va_end(ap);         
}



void main(void)
{
	int ret;

	error(2);
	delay(1000 * 1000 * 10);

	if (!device_is_ready(led.port)) {
		error(3);
		return;
	}

	debug_init();

	error(2);
	delay(1000 * 1000 * 10);

	while(1){
		// USART_SendData(USART1, 0x55);
		error(1);
		printk("hello\n");
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	error(2);




	if (ret < 0) {
		error(4);
		return;
	}

	error(2);
	delay(1000 * 1000 * 10);

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			error(5);
			return;
		}
		// k_msleep(SLEEP_TIME_MS);
		delay(1000 * 1000);
	}
}




// void main(void)
// {
// 	uint8_t i = 0;
// 	GPIO_InitTypeDef GPIO_InitStructure = {0};
	
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_Init(GPIOC, &GPIO_InitStructure);

// 	while(1)
//     {
// 		GPIO_WriteBit(GPIOC, GPIO_Pin_0, (i == 0) ? (i = 1) : (i = 0));
//         // k_msleep(SLEEP_TIME_MS);
// 		delay(1000 * 1000);
//     }
// }