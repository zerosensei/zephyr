/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_uart


#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_ch5xx, CONFIG_UART_LOG_LEVEL);

#include <soc.h>

typedef struct __attribute__((__packed__))
{
    struct {
        volatile uint8_t MCR;
        volatile uint8_t IER;
        volatile uint8_t FCR;
        volatile uint8_t LCR;
    } CTRL;

    struct {
        volatile uint8_t IIR;
        volatile uint8_t LSR;
        volatile uint8_t MSR;
        uint8_t RESERVED;
    } STAT;

    struct {
        union 
        {
            volatile uint8_t RBR;
            volatile uint8_t THR;
        };
        uint8_t RESERVED;
        volatile uint8_t RFC;
        volatile uint8_t TFC;
    } FIFO;

    struct {
        volatile uint16_t DL;              
        volatile uint8_t DIV;        
        volatile uint8_t ADR;
        uint8_t RESERVED;
    } SETUP;
} WCH_UART_Type;


#define UART_LCR_PAR_MOD_POS        (4U)
#define UART_LCR_PAR_MOD_MSK        (3U << UART_LCR_PAR_MOD_POS)
typedef enum {
    UART_PARITY_ODD,
    UART_PARITY_EVEN,
    UART_PARITY_MARK,
    UART_PARITY_SPACE
} uart_parity_mode_t;

#define UART_LCR_STOP_BIT_POS      (2U)
#define UART_LCR_STOP_BIT_MSK       (1U << UART_LCR_STOP_BIT_POS)
typedef enum {
    UART_STOP_BITS_1,
    UART_STOP_BITS_2
} uart_stop_bits_t;

#define UART_LCR_WORD_SZ_POS        (0U)
#define UART_LCR_WORD_SZ_MSK        (3U << UART_LCR_WORD_SZ_POS)
typedef enum {
    UART_DATA_5_BITS,
    UART_DATA_6_BITS,
    UART_DATA_7_BITS,
    UART_DATA_8_BITS
} uart_word_size_t;

#define UART_FCR_FIFO_TRIG_POS              (6U)
#define UART_FCR_FIFO_TRIG_MSK              (3U << UART_FCR_FIFO_TRIG_POS)
typedef enum {
    UART_FIFO_TRIG_BITS_1,
    UART_FIFO_TRIG_BITS_2,
    UART_FIFO_TRIG_BITS_4,
    UART_FIFO_TRIG_BITS_7
} uart_fifo_trig_t;

typedef enum {
    UART_INT_RCV_RDY = BIT(0),
    UART_INT_THR_ENMPTY = BIT(1),
    UART_INT_LINE_STAT = BIT(2),
    UART_INT_MODEM_CHG = BIT(3),
} uart_interrupt_t;

typedef enum {
    UART_INT_FLAG_MODEM_CHG = 0,
    UART_INT_FLAG_NO_INTER,
    UART_INT_FLAG_THR_EMPTY,
    UART_INT_FALG_RCV_RDY = 0x04,
    UART_INT_FLAG_LINE_STAT = 0x06,
    UART_INT_FLAG_RCV_TIMEOUT = 0x0c,
    UART_INT_FALG_SLV_ADDR = 0x0e,
} uart_int_flag_t;

typedef enum {
    UART_LINE_DATA_RDY = BIT(0),
    UART_LINT_FIFO_OVERFLOW = BIT(1),
    UART_LINE_PAR_ERR = BIT(2),
    UART_LINE_FRAME_ERR = BIT(3),
    UART_LINE_BREAK_ERR = BIT(4),
    UART_LINE_TX_FIFO_EMP = BIT(5),
    UART_LINE_TX_ALL_EMP = BIT(6),
    UART_LINE_RX_FIFO_ERR = BIT(7)
} uart_line_state_t;


static inline void hal_uart_frequency_div(WCH_UART_Type *uart, uint8_t div)
{
    uart->SETUP.DIV = div & BIT_MASK(7);
}

static inline void hal_uart_break_config(WCH_UART_Type *uart, bool cfg)
{
    if(cfg) {
        uart->CTRL.LCR |= RB_LCR_BREAK_EN;
    } else {
        uart->CTRL.LCR &= ~RB_LCR_BREAK_EN;
    }
}

static inline void hal_uart_pairty_set(WCH_UART_Type *uart, uart_parity_mode_t mode)
{
    uart->CTRL.LCR |= RB_LCR_PAR_EN;
    uart->CTRL.LCR &= ~UART_LCR_PAR_MOD_MSK; 
    uart->CTRL.LCR |= ((mode << UART_LCR_PAR_MOD_POS) & UART_LCR_PAR_MOD_MSK);
}

static inline uint8_t hal_uart_pairty_get(WCH_UART_Type *uart)
{
    return (uart->CTRL.LCR & UART_LCR_PAR_MOD_MSK) >> UART_LCR_PAR_MOD_POS;
}

static inline void hal_uart_pairty_disable(WCH_UART_Type *uart)
{
    uart->CTRL.LCR &= ~RB_LCR_PAR_EN;
}

static inline uint8_t hal_uart_pairty_is_enabled(WCH_UART_Type *uart)
{
    return uart->CTRL.LCR & RB_LCR_PAR_EN;
}

static inline void hal_uart_stop_bit_set(WCH_UART_Type *uart, uart_stop_bits_t stop_bits)
{
    uart->CTRL.LCR &= ~UART_LCR_STOP_BIT_MSK;
    uart->CTRL.LCR |= ((stop_bits << UART_LCR_STOP_BIT_POS) & UART_LCR_STOP_BIT_MSK);
}

static inline uint8_t hal_uart_stop_bit_get(WCH_UART_Type *uart)
{
    return (uart->CTRL.LCR & UART_LCR_STOP_BIT_MSK) >> UART_LCR_STOP_BIT_POS;
}

static inline void hal_uart_word_size_set(WCH_UART_Type *uart, uart_word_size_t word_sz)
{
    uart->CTRL.LCR &= ~UART_LCR_WORD_SZ_MSK;
    uart->CTRL.LCR |= ((word_sz << UART_LCR_WORD_SZ_POS) & UART_LCR_WORD_SZ_MSK);
}

static inline uint8_t hal_uart_word_size_get(WCH_UART_Type *uart)
{
    return (uart->CTRL.LCR & UART_LCR_WORD_SZ_MSK) >> UART_LCR_WORD_SZ_POS;
}

static inline void hal_uart_tx_enable(WCH_UART_Type *uart)
{
    uart->CTRL.IER |= RB_IER_TXD_EN;
}

static inline void hal_uart_tx_disable(WCH_UART_Type *uart)
{
    uart->CTRL.IER &= ~RB_IER_TXD_EN;
}

static inline uint8_t hal_uart_is_tx_enabled(WCH_UART_Type *uart)
{
    return uart->CTRL.IER & RB_IER_TXD_EN;
}

static inline void hal_uart_rx_fifo_trig(WCH_UART_Type *uart, uart_fifo_trig_t trig)
{
    uart->CTRL.FCR &= ~UART_FCR_FIFO_TRIG_MSK;
    uart->CTRL.FCR |= ((trig << UART_FCR_FIFO_TRIG_POS) & UART_FCR_FIFO_TRIG_MSK);
}

static inline void hal_uart_fifo_cfg(WCH_UART_Type *uart, bool fifo_en)
{
    if(fifo_en) {
        uart->CTRL.FCR |= RB_FCR_FIFO_EN;
    } else {
        uart->CTRL.FCR &= ~RB_FCR_FIFO_EN;
    }
}

static inline void hal_uart_fifo_tx_clear(WCH_UART_Type *uart)
{
    uart->CTRL.FCR |= RB_FCR_TX_FIFO_CLR;
}

static inline void hal_uart_fifo_rx_clear(WCH_UART_Type *uart)
{
    uart->CTRL.FCR |= RB_FCR_RX_FIFO_CLR;
}

static inline void hal_uart_interrupt_enable(WCH_UART_Type *uart, uint8_t interrupt)
{
    uart->CTRL.IER |= (interrupt) & BIT_MASK(4);
    uart->CTRL.MCR |= RB_MCR_INT_OE;
}

static inline void hal_uart_interrupt_disable(WCH_UART_Type *uart, uint8_t interrupt)
{
    uart->CTRL.IER &= ~((interrupt) & BIT_MASK(4));
}

static inline bool hal_uart_interrupt_is_enabled(WCH_UART_Type *uart, uart_interrupt_t interrupt)
{
    return (uart->CTRL.IER & (interrupt & BIT_MASK(4)) ? true : false);
}

static inline uint8_t hal_uart_get_int_flag(WCH_UART_Type *uart)
{
    return uart->STAT.IIR & RB_IIR_INT_MASK;
}

static inline uint8_t hal_uart_get_line_state(WCH_UART_Type *uart)
{
    return uart->STAT.LSR;
}

static inline void hal_uart_reset(WCH_UART_Type *uart)
{
    uart->CTRL.IER |= RB_IER_RESET;
}

static inline uint8_t hal_uart0_get_modem(void)
{
    return R8_UART0_MSR;
}

static inline void hal_uart0_modem_rts_enable(void)
{
    R8_UART0_IER |= RB_IER_RTS_EN;
}

static inline void hal_uart0_modem_rts_disable(void)
{
    R8_UART0_IER &= ~RB_IER_RTS_EN;
}

static inline void hal_uart0_modem_dtr_enable(void)
{
    R8_UART0_IER |= RB_IER_DTR_EN;
}

static inline void hal_uart0_modem_dtr_disable(void)
{
    R8_UART0_IER &= ~RB_IER_DTR_EN;
}

static inline void hal_uart0_modem_tnow_enable(void)
{
    R8_UART0_IER |= RB_MCR_TNOW;
}

static inline void hal_uart0_modem_tnow_disable(void)
{
    R8_UART0_IER &= ~RB_MCR_TNOW;
}

static inline void hal_uart0_modem_autoflow_enable(void)
{
    R8_UART0_IER |= RB_MCR_AU_FLOW_EN;
}

static inline void hal_uart0_modem_autoflow_disable(void)
{
    R8_UART0_IER &= ~RB_MCR_AU_FLOW_EN;
}

static inline uint8_t hal_uart0_modem_autoflow_is_enabled(void)
{
    return R8_UART0_IER & RB_MCR_AU_FLOW_EN;
}

static inline void hal_uart0_modem_loop_test_enable(void)
{
    R8_UART0_IER |= RB_MCR_LOOP;
}

static inline void hal_uart0_modem_loop_test_disable(void)
{
    R8_UART0_IER &= ~RB_MCR_LOOP;
}

static inline void hal_uart0_modem_out_set(void)
{
    R8_UART0_IER |= RB_MCR_OUT1;
}

static inline void hal_uart0_modem_out_reset(void)
{
    R8_UART0_IER &= ~RB_MCR_OUT1;
}

static inline void hal_uart0_modem_rst_set_low(void)
{
    R8_UART0_IER |= RB_MCR_RTS;
}

static inline void hal_uart0_modem_rst_set_high(void)
{
    R8_UART0_IER &= ~RB_MCR_RTS;
}

static inline void hal_uart0_modem_dtr_set_low(void)
{
    R8_UART0_IER |= RB_MCR_DTR;
}

static inline void hal_uart0_modem_dtr_set_high(void)
{
    R8_UART0_IER &= ~RB_MCR_DTR;
}

static inline void hal_uart0_halfduplex_enable(void)
{
    R8_UART0_MCR |= RB_MCR_HALF;
}

static inline void hal_uart0_halfduplex_disable(void)
{
    R8_UART0_MCR &= ~RB_MCR_HALF;
}

static inline void hal_uart0_set_slv_addr(uint8_t addr)
{
    R8_UART0_ADR = addr;
}

static inline uint8_t hal_uart0_get_slv_addr(void)
{
    return R8_UART0_ADR;
}

static inline void hal_uart_tx(WCH_UART_Type *uart, uint8_t data)
{
    uart->FIFO.THR = data;
}

static inline uint8_t hal_uart_rx(WCH_UART_Type *uart)
{
    return uart->FIFO.RBR;
}

static inline uint8_t hal_uart_get_rx_fifo_len(WCH_UART_Type *uart)
{
    return uart->FIFO.RFC;
}

static inline uint8_t hal_uart_get_tx_fifo_len(WCH_UART_Type *uart)
{
    return uart->FIFO.TFC;
}

/* baudrate = Fsys * 2 / (div * 16 * dl)*/
static void hal_uart_baudrate_set(WCH_UART_Type *uart, uint32_t baudrate)
{
    uint32_t dl;

    dl = DIV_ROUND_CLOSEST(10 * GetSysClock() * 2 
            /(16 * baudrate * uart->SETUP.DIV), 10);
    uart->SETUP.DL = (uint16_t)dl;
}

static uint32_t hal_uart_baudrate_get(WCH_UART_Type *uart)
{
    return (GetSysClock() * 2 / (16 * uart->SETUP.DL * uart->SETUP.DIV));
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static size_t hal_uart_fifo_tx(WCH_UART_Type *uart, const uint8_t *data, size_t len)
{
    size_t txlen = 0;

    while (len - txlen > 0) {
        if(hal_uart_get_tx_fifo_len(uart) != UART_FIFO_SIZE) {
            hal_uart_tx(uart, *data++);
            txlen++;
        }
    }

    return txlen;
}

static uint8_t hal_uart_fifo_rx(WCH_UART_Type *uart, uint8_t *data)
{
    uint8_t len = 0;

    if (hal_uart_get_rx_fifo_len(uart)) {
        *data++ = hal_uart_rx(uart);
        len++;
    }

    return len;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static void hal_uart_definit(WCH_UART_Type *uart)
{
    hal_uart_reset(uart);
    hal_uart_baudrate_set(uart, 115200);
    hal_uart_pairty_disable(uart);
    hal_uart_stop_bit_set(uart, UART_STOP_BITS_1);
    hal_uart_word_size_set(uart, UART_DATA_8_BITS);
    hal_uart_fifo_cfg(uart, true);
    hal_uart_rx_fifo_trig(uart, UART_FIFO_TRIG_BITS_4);
    hal_uart_tx_enable(uart);

    hal_uart_frequency_div(uart, 1);
}

struct uart_wch_config {
    WCH_UART_Type *uart;
    struct uart_config uart_cfg;
    const struct pinctrl_dev_config *pin_cfg;
    uart_irq_config_func_t irq_config_func;
};

struct uart_wch_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

void uart_isr_handler(const struct device *dev)
{
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    const struct uart_wch_data *data = dev->data;

    if (data->callback) {
        data->callback(dev, data->cb_data);
    }
#endif
}

static inline int uart_wch_set_pairty(WCH_UART_Type *uart, uint8_t pairty)
{
    switch (pairty) {
    case UART_CFG_PARITY_NONE:
        hal_uart_pairty_disable(uart);
        break;
    case UART_CFG_PARITY_ODD:
        hal_uart_pairty_set(uart, UART_PARITY_ODD);
        break;
    case UART_CFG_PARITY_EVEN:
        hal_uart_pairty_set(uart, UART_PARITY_EVEN);
        break;
    case UART_CFG_PARITY_MARK:
        hal_uart_pairty_set(uart, UART_PARITY_MARK);
        break;
    case UART_CFG_PARITY_SPACE:
        hal_uart_pairty_set(uart, UART_PARITY_SPACE);
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static inline int uart_wch_get_pairty(WCH_UART_Type *uart, uint8_t *pairty)
{
    uint8_t hal_pairty;

    if (!hal_uart_pairty_is_enabled(uart)) {
        *pairty = UART_CFG_PARITY_NONE;

        return 0;
    }

    hal_pairty = hal_uart_pairty_get(uart);

    switch (hal_pairty) {
    case UART_PARITY_ODD:
        *pairty = UART_CFG_PARITY_ODD;
        break;
    case UART_PARITY_EVEN:
        *pairty = UART_CFG_PARITY_EVEN;
        break;
    case UART_PARITY_MARK:
        *pairty = UART_CFG_PARITY_MARK;
        break;
    case UART_PARITY_SPACE:
        *pairty = UART_CFG_PARITY_SPACE;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static inline int uart_wch_set_stop_bits(WCH_UART_Type *uart, uint8_t stop_bits)
{
    switch (stop_bits) {
    case UART_CFG_STOP_BITS_1:
        hal_uart_stop_bit_set(uart, UART_STOP_BITS_1);
        break;
    case UART_CFG_STOP_BITS_2:
        hal_uart_stop_bit_set(uart, UART_STOP_BITS_2);
        break;
    case UART_CFG_STOP_BITS_0_5:
    case UART_CFG_STOP_BITS_1_5:
        return -ENOTSUP;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static inline int uart_wch_get_stop_bits(WCH_UART_Type *uart, uint8_t *stop_bits)
{
    uint8_t hal_stop_bits = hal_uart_stop_bit_get(uart);

    switch (hal_stop_bits) {
    case UART_STOP_BITS_1:
        *stop_bits = UART_CFG_STOP_BITS_1;
        break;
    case UART_STOP_BITS_2:
        *stop_bits = UART_CFG_STOP_BITS_2;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static inline int uart_wch_set_data_bits(WCH_UART_Type *uart, uint8_t data_bits)
{
    switch (data_bits) {
    case UART_CFG_DATA_BITS_5:
        hal_uart_word_size_set(uart, UART_DATA_5_BITS);
        break;
    case UART_CFG_DATA_BITS_6:
        hal_uart_word_size_set(uart, UART_DATA_6_BITS);
        break;
    case UART_CFG_DATA_BITS_7:
        hal_uart_word_size_set(uart, UART_DATA_7_BITS);
        break;
    case UART_CFG_DATA_BITS_8:
        hal_uart_word_size_set(uart, UART_DATA_8_BITS);
        break;
    case UART_CFG_DATA_BITS_9:
        return -ENOSYS;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static inline int uart_wch_get_data_bits(WCH_UART_Type *uart, uint8_t *data_bits)
{
    uint8_t hal_data_bits = hal_uart_word_size_get(uart);

    switch (hal_data_bits) {
        case UART_DATA_5_BITS:
            *data_bits = UART_CFG_DATA_BITS_5;
            break;
        case UART_DATA_6_BITS:
            *data_bits = UART_CFG_DATA_BITS_6;
            break;
        case UART_DATA_7_BITS:
            *data_bits = UART_CFG_DATA_BITS_6;
            break;
        case UART_DATA_8_BITS:
            *data_bits = UART_CFG_DATA_BITS_6;
            break;
        default:
            return -EINVAL;
            break;
    }

    return 0;
}

static inline int uart_wch_set_flow_ctrl(WCH_UART_Type *uart, uint8_t flow_ctrl)
{
#ifdef CONFIG_HAS_HW_WCH_UART0
    if (uart != UART0) {
        return -ENOTSUP;
    }

    if (flow_ctrl) {
        hal_uart0_modem_autoflow_enable();
        hal_uart0_modem_rts_enable();
        hal_uart0_modem_rst_set_low();
    } else {
        hal_uart0_modem_autoflow_disable();
        hal_uart0_modem_rst_set_high();
        hal_uart0_modem_rts_disable();
    }

    return 0;
#else
    return -ENOTSUP;
#endif
}

static inline int uart_wch_get_flow_ctrl(WCH_UART_Type *uart, uint8_t *flow_ctrl)
{
#ifdef CONFIG_HAS_HW_WCH_UART0  //TODO: 
    if (hal_uart0_modem_autoflow_is_enabled()) {
        *flow_ctrl = true;
    } else {
        *flow_ctrl = false;
    }

    return 0;
#else
    return -ENOTSUP;
#endif 
}

static int uart_wch_poll_in(const struct device *dev, unsigned char *c)
{
    const struct uart_wch_config *cfg = dev->config;

    if (!(hal_uart_get_line_state(cfg->uart) 
            & UART_LINE_DATA_RDY)) {
        return -1;
    }

    *c = hal_uart_rx(cfg->uart);

    return 0;
}

static void uart_wch_poll_out(const struct device *dev, unsigned char c)
{
    const struct uart_wch_config *cfg = dev->config;

    while (!(hal_uart_get_line_state(cfg->uart) 
            & UART_LINE_TX_ALL_EMP)) {
        ;
    }
    hal_uart_tx(cfg->uart, c);
}

static int uart_wch_err_check(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;
    int err = 0;
    uint8_t status = hal_uart_get_line_state(cfg->uart);

    if(status & UART_LINT_FIFO_OVERFLOW) {
        err |= UART_ERROR_OVERRUN;
    }
    
    if (status & UART_LINE_PAR_ERR) {
        err |= UART_ERROR_PARITY;
    }

    if (status & UART_LINE_FRAME_ERR) {
        err |= UART_ERROR_FRAMING;
    }

    if (status & UART_LINE_BREAK_ERR) {
        err |= UART_BREAK;
    }

    return err; 
}

static int uart_wch_configure(const struct device *dev, 
            const struct uart_config *cfg)
{
    const struct uart_wch_config *config = dev->config;
    int err = 0;

    if (cfg->baudrate) {
        hal_uart_baudrate_set(config->uart, cfg->baudrate);
    } else {
        return -EINVAL;
    }

    if ((err = uart_wch_set_pairty(config->uart, cfg->parity))) {
        return err;
    }

    if ((err = uart_wch_set_data_bits(config->uart, cfg->data_bits))) {
        return err;
    }

    if ((err = uart_wch_set_stop_bits(config->uart, cfg->stop_bits))) {
        return err;
    }

    if ((err = uart_wch_set_flow_ctrl(config->uart, cfg->data_bits))) {
        return err;
    }

    return 0;
}

static int uart_wch_configure_get(const struct device *dev, 
            struct uart_config *cfg)
{
    const struct uart_wch_config *config = dev->config;
    uint8_t data_bits, pairty, stop_bits, flow_ctrl;
    int err = 0;
    
    if ((err = uart_wch_get_data_bits(config->uart, &data_bits))) {
        return err;
    }
    if ((err = uart_wch_get_pairty(config->uart, &pairty))) {
        return err;
    }

    if ((err = uart_wch_get_stop_bits(config->uart, &stop_bits))) {
        return err;
    }

    if ((err = uart_wch_get_flow_ctrl(config->uart, &flow_ctrl))) {
        return err;
    }

    cfg->baudrate = hal_uart_baudrate_get(config->uart);
    cfg->data_bits = data_bits;
    cfg->parity = pairty;
    cfg->stop_bits = stop_bits;
    cfg->flow_ctrl = flow_ctrl;

    return 0;
}


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_wch_fifo_fill(const struct device *dev,
            const uint8_t *tx_data, int len)
{
    const struct uart_wch_config *cfg = dev->config;

    return hal_uart_fifo_tx(cfg->uart, tx_data, len);
}

static int uart_wch_fifo_read(const struct device *dev,
            uint8_t *rx_data, const int size)
{
    const struct uart_wch_config *cfg = dev->config;
    uint8_t num_rx = 0;

    while ((size - num_rx > 0) && hal_uart_get_rx_fifo_len(cfg->uart)) {
        num_rx += hal_uart_fifo_rx(cfg->uart, &rx_data[num_rx]);
    }

    return num_rx;
}

static void uart_wch_irq_tx_enable(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;
    hal_uart_interrupt_enable(cfg->uart, UART_INT_THR_ENMPTY);
}

static void uart_wch_irq_tx_disable(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;

    hal_uart_interrupt_disable(cfg->uart, UART_INT_THR_ENMPTY);
}

static void uart_wch_irq_rx_enable(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;

    hal_uart_interrupt_enable(cfg->uart, UART_INT_RCV_RDY);
}

static void uart_wch_irq_rx_disable(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;

    hal_uart_interrupt_disable(cfg->uart, UART_INT_RCV_RDY);
}

static int uart_wch_irq_tx_complete(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;

    return hal_uart_get_int_flag(cfg->uart) & UART_INT_FLAG_THR_EMPTY;
}

static int uart_wch_irq_tx_ready(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;

    return (hal_uart_get_int_flag(cfg->uart) & UART_INT_FLAG_THR_EMPTY) && 
            hal_uart_interrupt_is_enabled(cfg->uart, UART_INT_THR_ENMPTY);
}

static int uart_wch_irq_rx_ready(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;
    
    return hal_uart_interrupt_is_enabled(cfg->uart, UART_INT_RCV_RDY);
}

static void uart_wch_irq_err_enable(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;

    hal_uart_interrupt_enable(cfg->uart, UART_INT_LINE_STAT);
}

static void uart_wch_irq_err_disable(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;

    hal_uart_interrupt_disable(cfg->uart, UART_INT_LINE_STAT);
}

static int uart_wch_irq_is_pending(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;
    uint8_t flag = hal_uart_get_int_flag(cfg->uart);

    if (hal_uart_interrupt_is_enabled(cfg->uart, UART_INT_RCV_RDY)) {
        if (flag == UART_INT_FALG_RCV_RDY) {
            return true;
        }

        if (flag == UART_INT_FLAG_RCV_TIMEOUT) {
            return true;
        }
    }

    if (hal_uart_interrupt_is_enabled(cfg->uart, UART_INT_THR_ENMPTY)) {
        if (flag == UART_INT_FLAG_LINE_STAT) {
            return true;
        }
    }


#ifdef CONFIG_WCH_UART_0
    if (hal_uart_interrupt_is_enabled(cfg->uart, UART_INT_MODEM_CHG)) {
        if (flag == UART_INT_FLAG_LINE_STAT) {
            return true;
        }
    }
#endif

    return false;

}

static int uart_wch_irq_update(const struct device *dev)
{
    return 1;
}

static void uart_wch_irq_callback_set(const struct device *dev,
            uart_irq_callback_user_data_t cb,
            void *cb_data)
{
    struct uart_wch_data *data = dev->data;

    data->callback = cb;
    data->cb_data = cb_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_wch_init(const struct device *dev)
{
    const struct uart_wch_config *cfg = dev->config;
    int err = 0;

	err = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

    hal_uart_definit(cfg->uart);

    if (cfg->uart_cfg.baudrate) {
        hal_uart_baudrate_set(cfg->uart, cfg->uart_cfg.baudrate);
    } else {
        return -EINVAL;
    }

    if ((err = uart_wch_set_pairty(cfg->uart, cfg->uart_cfg.parity))) {
        return err;
    }

    if ((err = uart_wch_set_data_bits(cfg->uart, cfg->uart_cfg.data_bits))) {
        return err;
    }

    if ((err = uart_wch_set_stop_bits(cfg->uart, cfg->uart_cfg.stop_bits))) {
        return err;
    }

    if (cfg->uart_cfg.flow_ctrl && 
            (err = uart_wch_set_flow_ctrl(cfg->uart, cfg->uart_cfg.flow_ctrl))) {
        return err;
    }

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    cfg->irq_config_func(dev);
#endif

    return 0;
}

static const struct uart_driver_api uart_wch_driver_api = {
	.poll_in		    = uart_wch_poll_in,
	.poll_out		    = uart_wch_poll_out,
	.err_check		    = uart_wch_err_check,
    .configure          = uart_wch_configure,
    .config_get         = uart_wch_configure_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill          = uart_wch_fifo_fill,
	.fifo_read		    = uart_wch_fifo_read,
	.irq_tx_enable	    = uart_wch_irq_tx_enable,
	.irq_tx_disable	    = uart_wch_irq_tx_disable,
	.irq_tx_ready	    = uart_wch_irq_tx_ready,
	.irq_rx_enable	    = uart_wch_irq_rx_enable,
	.irq_rx_disable	    = uart_wch_irq_rx_disable,
	.irq_tx_complete    = uart_wch_irq_tx_complete,
	.irq_rx_ready	    = uart_wch_irq_rx_ready,
	.irq_err_enable	    = uart_wch_irq_err_enable,
	.irq_err_disable    = uart_wch_irq_err_disable,
	.irq_is_pending	    = uart_wch_irq_is_pending,
    .irq_update         = uart_wch_irq_update,
	.irq_callback_set   = uart_wch_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};  

#define UART_CH5XX_INIT(index)   \
    static void uart_wch_irq_cfg_func_##index(const struct device *dev)     \
    {       \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN,                                           \
			   (IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), uart_isr_handler, \
					DEVICE_DT_INST_GET(index), 0);                                 \
			    irq_enable(DT_INST_IRQN(index));))    \
    };       \
	PINCTRL_DT_INST_DEFINE(index);		\
    static const struct uart_wch_config uart_cfg_##index = {    \
        .uart = (WCH_UART_Type *)DT_INST_REG_ADDR(index),   \
        .uart_cfg = {           \
			.baudrate = DT_INST_PROP(index, current_speed),		\
			.parity = DT_INST_ENUM_IDX_OR(index, parity, UART_CFG_PARITY_NONE),  \
			.stop_bits = DT_INST_ENUM_IDX_OR(index, stop_bits,                   \
								UART_CFG_STOP_BITS_1),            \
			.data_bits = DT_INST_ENUM_IDX_OR(index, data_bits,                   \
								UART_CFG_DATA_BITS_8),            \
			.flow_ctrl = MAX(COND_CODE_1(DT_INST_PROP(index, hw_rs485_hd_mode),  \
								(UART_CFG_FLOW_CTRL_RS485),           \
								(UART_CFG_FLOW_CTRL_NONE)),           \
						COND_CODE_1(DT_INST_PROP(index, hw_flow_control),   \
								(UART_CFG_FLOW_CTRL_RTS_CTS),         \
								(UART_CFG_FLOW_CTRL_NONE)))	\
		},         \
        .pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),        \
        .irq_config_func = uart_wch_irq_cfg_func_##index,      \
    };     \
    static struct uart_wch_data uart_data_##index;        \
    DEVICE_DT_INST_DEFINE(index, uart_wch_init, NULL,        \
            &uart_data_##index,     \
            &uart_cfg_##index,      \
			PRE_KERNEL_1,	\
			CONFIG_SERIAL_INIT_PRIORITY, \
            &uart_wch_driver_api);     

DT_INST_FOREACH_STATUS_OKAY(UART_CH5XX_INIT)
