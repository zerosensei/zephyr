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

    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, 1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	// USART_InitStructure.USART_BaudRate = 115200;
    // USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    // USART_InitStructure.USART_StopBits = USART_StopBits_1;
    // USART_InitStructure.USART_Parity = USART_Parity_No;
    // USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // USART_InitStructure.USART_Mode = USART_Mode_Tx;

    // USART_Init(USART1, &USART_InitStructure);
    // USART_Cmd(USART1, 1);
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


extern uint32_t pinmux_fun[2];
extern uint8_t pincnt;
extern uint32_t pinmux[2];

extern uint16_t pinnn;
extern int mmode;
extern int oospeed;
extern uint32_t *port;

extern int pincfg;
extern int ping2;
extern int function;

	printk("pincnt: %d\n", pincnt);
	printk("pinmux fun: %#x %#x\n", pinmux_fun[0], pinmux_fun[1]);
	printk("pinmux: %#lx %#lx\n", pinmux[0], pinmux[1]);

	printk("pincfg: %#lx\n", pincfg);
	printk("ping2: %#lx\n", ping2);
	printk("function: %#lx\n", function);

	printk("cfg pin: %#lx\n", pinnn);
	printk("cfg mode: %lx\n", mmode);
	printk("cfg speed: %lx\n", oospeed);
	printk("cfg port: %lx\n", port);


	while(1){
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