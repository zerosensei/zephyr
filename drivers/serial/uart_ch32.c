/*
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_usart

#include <drivers/clock_control.h>
#include <drivers/clock_control/ch32_clock_control.h>
#include <drivers/pinctrl.h>
#include <drivers/uart.h>
#include <soc.h>

struct usart_ch32_config {
	/* USART instance */
	USART_TypeDef *usart;
	/* clock subsystem driving this peripheral */
	struct ch32_pclken pclken;
	/* initial hardware flow control, 1 for RTS/CTS */
	bool hw_flow_control;
	/* initial parity, 0 for none, 1 for odd, 2 for even */
	int parity;
	/* switch to enable single wire / half duplex feature */
	bool single_wire;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct usart_ch32_data {
	uint32_t baud_rate;
	const struct device *clock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_ch32_isr(const struct device *dev)
{
	struct usart_ch32_data *const data = dev->data;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int usart_ch32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct usart_ch32_config *cfg = dev->config;

	/* Clear overrun error flag */		
	if(USART_GetFlagStatus(cfg->usart, USART_FLAG_ORE)) {
		USART_ClearFlag(cfg->usart, USART_FLAG_ORE);
	}

	if(!USART_GetFlagStatus(cfg->usart, USART_FLAG_RXNE)) {
		return -1;
	}

	*c = (unsigned char)USART_ReceiveData(cfg->usart);

	return 0;
}

static void usart_ch32_poll_out(const struct device *dev, unsigned char c)
{
	const struct usart_ch32_config *cfg = dev->config;
	int key;

	while(1) {
		if(USART_GetFlagStatus(cfg->usart, USART_FLAG_TXE)) {
			key = irq_lock();
			if(USART_GetFlagStatus(cfg->usart, USART_FLAG_TXE)) {
				break;
			}
			irq_unlock(key);
		}
	}

	USART_SendData(cfg->usart, c);
	irq_unlock(key);
}

static int usart_ch32_err_check(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;
	int err = 0;

	if(USART_GetFlagStatus(cfg->usart, USART_FLAG_ORE)){
		err |= UART_ERROR_OVERRUN;
	}

	if(USART_GetFlagStatus(cfg->usart, USART_FLAG_FE)){
		err |= UART_ERROR_FRAMING;
	}

	if(USART_GetFlagStatus(cfg->usart, USART_FLAG_PE)){
		err |= UART_ERROR_PARITY;
	}

	/* Clear error */
	(void)USART_ReceiveData(cfg->usart);

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
int usart_ch32_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len)
{
	const struct usart_ch32_config *cfg = dev->config;
	uint8_t num_tx = 0U;
	int key;

	if(!USART_GetFlagStatus(cfg->usart, USART_FLAG_TXE)) {
		return num_tx;
	}

	/* Lock interrupts to prevent nested interrupts or thread switch */
	key = irq_lock();

	while ((len - num_tx > 0) &&
	    	USART_GetFlagStatus(cfg->usart, USART_FLAG_TXE)) {
		USART_SendData(cfg->usart, tx_data[num_tx++]);
	}

	irq_unlock(key);

	return num_tx;
}

int usart_ch32_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	const struct usart_ch32_config *cfg = dev->config;
	uint8_t num_rx = 0U;

	while ((size - num_rx > 0) &&
	       USART_GetFlagStatus(cfg->usart, USART_FLAG_RXNE)) {
		rx_data[num_rx++] = USART_ReceiveData(cfg->usart);

		/* Clear overrun error flag */		
		if(USART_GetFlagStatus(cfg->usart, USART_FLAG_ORE)) {
			USART_ClearFlag(cfg->usart, USART_FLAG_ORE);
		}
	}

	return num_rx;
}

static void usart_ch32_irq_tx_enable(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;

	USART_ITConfig(cfg->usart, USART_IT_TC, ENABLE);
}

static void usart_ch32_irq_tx_disable(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;

	USART_ITConfig(cfg->usart, USART_IT_TC, DISABLE);
}

static int usart_ch32_irq_tx_ready(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;

	return USART_GetFlagStatus(cfg->usart, USART_FLAG_TXE) &&
		USART_GetITStatus(cfg->usart, USART_IT_TC);
}

static int usart_ch32_irq_tx_complete(const sturct device *dev)
{
	cosnt struct usart_ch32_config *cfg = dev->config;

	return USART_GetFlagStatus(cfg->usart, USART_FLAG_TC);
}

static void usart_ch32_irq_rx_enable(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;

	USART_ITConfig(cfg->usart, USART_IT_RXNE, ENABLE);
}

static void usart_ch32_irq_rx_disable(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;

	USART_ITConfig(cfg->usart, USART_IT_RXNE, DISABLE);
}

static int usart_ch32_irq_rx_ready(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;

	return USART_GetFlagStatus(cfg->usart, USART_FLAG_RXNE);
}

static void usart_ch32_irq_err_enable(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;

	USART_ITConfig(cfg->usart, USART_IT_ERR, ENABLE);
}

static void usart_ch32_irq_err_disable(const struct device *dev)
{
	const struct usart_ch32_config *cfg = dev->config;

	USART_ITConfig(cfg->usart, USART_IT_ERR, DISABLE);
}

static int usart_ch32_irq_is_pending(const struct device *dev)
{
	const struct usart_ch32_config+ *cfg = dev->config;

	return ((USART_GetFlagStatus(cfg->usart, USART_FLAG_RXNE) &&
		USART_GetITStatus(cfg->usart, USART_IT_RXNE)) ||
		(USART_GetFlagStatus(cfg->usart, USART_FLAG_TC) &&
		USART_GetITStatus(cfg->usart, USART_IT_TC)));
}

static void usart_ch32_irq_callback_set(const struct device *dev
					uart_irq_callback_user_data_t cb,
					void *cb_data)
{
	struct usart_ch32_data *data = dev->data;

	data->user_cb = cb;
	data->user_data = cb_data;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


static int usart_ch32_init(const struct device *dev)
{
	const struct usart_ch32_config *config = dev->config;
	struct usart_ch32_data *data = dev->data;
	int err;
	USART_InitTypeDef USART_InitStructure = {0};

	data->clock = DEVICE_DT_GET(CH32_CLOCK_CONTROL_NODE);
	/* enable clock */
	if (clock_control_on(data->clock, (clock_control_subsys_t *)&config->pclken) != 0) {
		return -EIO;
	}

	/* Configure dt provided device signals when available */
	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	switch (config->parity) {
	case UART_CFG_PARITY_NONE:
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		break;
	case UART_CFG_PARITY_ODD:
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		break;
	case UART_CFG_PARITY_EVEN:
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
		break;
	default:
		return -ENOTSUP;
	}

	USART_DeInit(config->usart);

	USART_InitStructure.USART_BaudRate = data->baud_rate;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	if(config->hw_flow_control){
    	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	}
	USART_Init(config->usart, &USART_InitStructure);
	USART_Cmd(config->usart, ENABLE);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	// while(1){
		// error(2);
	// }

	return 0;
}

static const struct uart_driver_api usart_ch32_driver_api = {
	.poll_in = usart_ch32_poll_in,
	.poll_out = usart_ch32_poll_out,
	.err_check = usart_ch32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uasrt_ch32_fifo_fill,
	.fifo_read = usart_ch32_fifo_read,
	.irq_tx_enable = usart_ch32_irq_tx_enable,
	.irq_tx_disable = usart_ch32_irq_tx_disable,
	.irq_tx_ready = usart_ch32_irq_tx_ready,
	.irq_tx_complete = usart_ch32_irq_tx_complete,
	.irq_rx_enable = usart_ch32_irq_rx_enable,
	.irq_rx_disable = usart_ch32_irq_rx_disable,
	.irq_rx_ready = usart_ch32_irq_rx_ready,
	.irq_err_enable = usart_ch32_irq_err_enable,
	.irq_err_disable = usart_ch32_irq_err_disable,
	.irq_is_pending = usart_ch32_irq_is_pending,
	.irq_callback_set = usart_ch32_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define CH32_USART_IRQ_HANDLER(n)						\
	static void usart_ch32_config_func_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    usart_ch32_isr,					\
			    DEVICE_DT_INST_GET(n),				\
			    0);							\
		irq_enable(DT_INST_IRQN(n));					\
	}
#define CH32_USART_IRQ_HANDLER_FUNC_INIT(n)					\
	.irq_config_func = usart_ch32_config_func_##n
#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define CH32_USART_IRQ_HANDLER(n)
#define CH32_USART_IRQ_HANDLER_FUNC_INIT(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define CH32_USART_INIT(n)                          \
	PINCTRL_DT_INST_DEFINE(n);						\
	CH32_USART_IRQ_HANDLER(n)						\
    static struct usart_ch32_data usart_ch32_data_##n = {           \
        .baud_rate = DT_INST_PROP(n, current_speed),            \
    };          \
    static const struct usart_ch32_config usart_ch32_config_##n = {         \
        .usart = (USART_TypeDef *)DT_INST_REG_ADDR(n),      \
        .pclken = { .bus = DT_INST_CLOCKS_CELL(n, bus),     \
                    .enr = DT_INST_CLOCKS_CELL(n, bits)     \
        },          \
        .hw_flow_control = DT_INST_PROP(n, hw_flow_control),        \
        .parity = DT_INST_ENUM_IDX_OR(n, parity, UART_CFG_PARITY_NONE),		\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.single_wire = DT_INST_PROP_OR(n, single_wire, false),		\
		CH32_USART_IRQ_HANDLER_FUNC_INIT(n)			\
    };          \
	DEVICE_DT_INST_DEFINE(n, &usart_ch32_init,			\
				NULL,			\
				&usart_ch32_data_##n,			\
				&usart_ch32_config_##n,			\
				PRE_KERNEL_1,			\
				CONFIG_SERIAL_INIT_PRIORITY,			\
				&usart_ch32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CH32_USART_INIT)