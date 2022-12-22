#include <linux/errno.h>
#include <linux/bits.h>

#include "swd_gpio.h"

#define SWD_RESET_LEN 50
#define RETRY       600
#define START       1
#define STOP        0
#define PARK        1

#define JTAG_TO_SWD 0xE79E

#define SWD_ORUNERRCLR_OFF  4
#define SWD_WDERRCLR_OFF    3
#define SWD_STKERRCLR_OFF   2
#define SWD_DAPABORT_OFF    0
#define SWD_ORUNERRCLR_MSK  BIT(SWD_ORUNERRCLR_OFF)
#define SWD_WDERRCLR_MSK    BIT(SWD_WDERRCLR_OFF)
#define SWD_STKERRCLR_MSK   BIT(SWD_STKERRCLR_OFF)
#define SWD_DABABORT_MSK    BIT(SWD_DAPABORT_OFF)

#define SWD_CSYSPWRUPREQ_OFF    30
#define SWD_CDBGPWRUPREQ_OFF    28
#define SWD_WDATAERR_OFF        7
#define SWD_STICKYERR_OFF       5
#define SWD_STICKYORUN_OFF      1
#define SWD_CSYSPWRUPREQ_MSK    BIT(SWD_CSYSPWRUPREQ_OFF)
#define SWD_CDBGPWRUPREQ_MSK    BIT(SWD_CDBGPWRUPREQ_OFF)
#define SWD_WDATAERR_MSK        BIT(SWD_WDATAERR_OFF)
#define SWD_STICKYERR_MSK       BIT(SWD_STICKYERR_OFF)
#define SWD_STICKYORUN_MSK      BIT(SWD_STICKYORUN_OFF)


// #define ERR(fmt,...) pr_err(fmt,...)
// #define INFO(fmt,...) pr_info(fmt,...)
#define ERR(fmt,...) {}
#define INFO(fmt,...) {}

static inline void _swd_send_bit(struct swd_gpio *sg, int v)
{
    sg->SWDIO_SET(v);
    sg->SWCLK_SET(1);
    sg->_delay();
    sg->SWCLK_SET(0);
    sg->_delay();
}

static inline int _swd_read_bit(struct swd_gpio *sg)
{
    int ret;

    ret = sg->SWDIO_GET();
    sg->SWCLK_SET(1);
    sg->_delay();
    sg->SWCLK_SET(0);
    sg->_delay();

    return ret;
}

static inline void _swd_send_cmd(struct swd_gpio *sg, u8 cmd)
{
    int i;

    sg->signal_begin();
    for (i = 0 ; i < 8 ; i++) {
        _swd_send_bit(sg, cmd & 1);
        cmd = cmd >> 1;
    }
    sg->signal_end();
}

static inline void _swd_send_data(struct swd_gpio *sg, u32 data)
{
    int i;
    u8 parity = 0;

    sg->signal_begin();
    for (i = 0 ; i < 32 ; i++) {
        _swd_send_bit(sg, data & 1);
        parity = parity ^ (data & 1);
        data = data >> 1;
    }
    _swd_send_bit(sg, parity);
    sg->signal_end();
}

static inline u32 _swd_read_data(struct swd_gpio *sg)
{
    int i;
    u32 data = 0;
    u8 parity = 0;

    sg->signal_begin();
    for (i = 0 ; i < 32 ; i++) {
        data |= _swd_read_bit(sg) << i;
    }
    parity = _swd_read_bit(sg);
    sg->signal_end();

    return data;
}

static inline void _swd_trn(struct swd_gpio *sg)
{
    sg->SWCLK_SET(1);
    sg->_delay();
    sg->SWCLK_SET(0);
    sg->_delay();
}

inline void _swd_reset(struct swd_gpio *sg)
{
    int i;

    sg->signal_begin();

    sg->SWDIO_SET(1);
    for (i = 0 ; i < SWD_RESET_LEN ; i++) {
        sg->SWCLK_SET(1);
        sg->_delay();
        sg->SWCLK_SET(0);
        sg->_delay();
    }

    sg->SWDIO_SET(0);
    for (i = 0 ; i < 2 ; i++) {
        sg->SWCLK_SET(1);
        sg->_delay();
        sg->SWCLK_SET(0);
        sg->_delay();
    }

    sg->signal_end();
}

void _swd_jtag_to_swd(struct swd_gpio *sg)
{
    _swd_send_cmd(sg, JTAG_TO_SWD & 0xFF);
    _swd_send_cmd(sg, (JTAG_TO_SWD >> 8) & 0xFF);
    _swd_reset(sg);
}

static void inline __wait_handling(struct swd_gpio *sg);
static void inline __fault_err_handling(struct swd_gpio *sg);
static void _swd_fault_handling(struct swd_gpio *sg, u8 ack);

u8 _swd_send(struct swd_gpio *sg,
            u8 APnDP, u8 RnW, u8 A, u32 data, bool handle_err)
{
    u8 cmd = 0;
    u8 A_2 = 0;
    u8 A_3 = 0;
    u8 ack = 0;
    u8 parity = 0;
    int retry = RETRY;

    // 1. start
    // 7. Park
    cmd = (PARK << 7) | START;

    // 2. APnDP
    // 3. RnW
    cmd = cmd | (APnDP << 1) | (RnW << 2);

    // 4. A[2:3]
    A_2 = ((A >> 2) & 1);
    A_3 = ((A >> 3) & 1);
    cmd = cmd | (A_2 << 3) | (A_3 << 4);

    // 5. Parity
    parity = APnDP ^ RnW ^ A_2 ^ A_3;
    cmd = cmd | (parity << 5);

    // 6. Stop
    do {
        _swd_send_cmd(sg, cmd);

        _swd_trn(sg);

        sg->SWDIO_DIR_IN();
        ack = _swd_read_bit(sg);
        ack = (_swd_read_bit(sg) << 1) | ack;
        ack = (_swd_read_bit(sg) << 2) | ack;

        _swd_trn(sg);
    } while((ack == SWD_WAIT) && retry--);

    if ((ack != SWD_OK) && handle_err) {
        _swd_fault_handling(sg, ack);
        return ack;
    }

    sg->SWDIO_DIR_OUT();
    _swd_send_data(sg, data);

    return ack;
}

u8 _swd_read(struct swd_gpio *sg,
            u8 APnDP, u8 RnW, u8 A, u32 *data, bool handle_err)
{
    u8 cmd = 0;
    u8 A_2 = 0;
    u8 A_3 = 0;
    u8 ack = 0;
    u8 parity = 0;
    int retry = RETRY;

    // 1. start
    // 7. Park
    cmd = (PARK << 7) | START;

    // 2. APnDP
    // 3. RnW
    cmd = cmd | (APnDP << 1) | (RnW << 2);

    // 4. A[2:3]
    A_2 = ((A >> 2) & 1);
    A_3 = ((A >> 3) & 1);
    cmd = cmd | (A_2 << 3) | (A_3 << 4);

    // 5. Parity
    parity = APnDP ^ RnW ^ A_2 ^ A_3;
    cmd = cmd | (parity << 5);

    // 6. Stop
    do {
        _swd_send_cmd(sg, cmd);

        _swd_trn(sg);

        sg->SWDIO_DIR_IN();

        ack = _swd_read_bit(sg);
        ack = (_swd_read_bit(sg) << 1) | ack;
        ack = (_swd_read_bit(sg) << 2) | ack;

        if (ack == SWD_WAIT)
             _swd_trn(sg);
    } while((ack == SWD_WAIT) && retry--);

    if ((ack != SWD_OK) && handle_err){
        _swd_fault_handling(sg, ack);
        return ack;
    }

    *data = _swd_read_data(sg);

    _swd_trn(sg);

    sg->SWDIO_DIR_OUT();

    return ack;
}
static void inline __wait_handling(struct swd_gpio *sg)
{
    u32 abort_val = SWD_DABABORT_MSK;

    // set the DAPABORT in abort reg, abort data transmition
    _swd_send(sg, SWD_DP, SWD_READ, SWD_DP_ABORT_REG, abort_val, false);
}

static void inline __fault_err_handling(struct swd_gpio *sg)
{
    u32 ctrlstat_val = 0;
    u32 abort_val = 0;

    // read the ctrlstat reg and set the abort reg
    _swd_read(sg, SWD_DP, SWD_READ, SWD_DP_CTRLSTAT_REG, &ctrlstat_val, false);
    ERR("%s: [%s] %d ctrlstat_val:%08x\n", __FILE__, __func__, __LINE__, ctrlstat_val);

    if (ctrlstat_val & SWD_STICKYERR_MSK)
        abort_val |= SWD_STKERRCLR_MSK;

    if (ctrlstat_val & SWD_WDATAERR_MSK)
        abort_val |= SWD_WDERRCLR_MSK;

    if (ctrlstat_val & SWD_STICKYORUN_MSK)
        abort_val |= SWD_ORUNERRCLR_MSK;

    INFO("%s: [%s] %d abort_val:%08x\n", __FILE__, __func__, __LINE__, abort_val);
    _swd_send(sg, SWD_DP, SWD_WRITE, SWD_DP_ABORT_REG, abort_val, false);

    // after clear the wdataerr bit, reset the line and read the idcode
    if (ctrlstat_val & SWD_WDATAERR_MSK) {
        _swd_jtag_to_swd(sg);
        _swd_read(sg, SWD_DP, SWD_READ, SWD_DP_IDCODE_REG, &abort_val, false);
    }
}

static void _swd_fault_handling(struct swd_gpio *sg, u8 ack)
{
    switch (ack) {
    case SWD_WAIT:
        ERR("%s: [%s] %d wait\n", __FILE__, __func__, __LINE__);
        __wait_handling(sg);
    break;
    case SWD_FAULT: // for fault
    default: // for protocol error
        ERR("%s: [%s] %d fault ack:%d\n", __FILE__, __func__, __LINE__, ack);
        __fault_err_handling(sg);
    }
}

ssize_t _swd_ap_read(struct swd_gpio *sg, void* to, u32 base, const ssize_t len)
{
    int i;
    u8 ack = 0;
    u32 data = 0;
    u32 *buf = to;
    ssize_t len_to_read = len / 4;

    ack = _swd_send(sg, SWD_DP, SWD_WRITE, SWD_DP_SELECT_REG, SWD_AP_TAR_REG & 0xF0, true);
    if (ack != SWD_OK) {
        ERR("%s: [%s] %d select first AP bank fail\n", __FILE__, __func__, __LINE__);
        return -ENODEV;
    }

    ack = _swd_send(sg, SWD_AP, SWD_WRITE, SWD_AP_TAR_REG & 0xC, base, true);
    if (ack != SWD_OK) {
        ERR("%s: [%s] %d select first AP bank fail\n", __FILE__, __func__, __LINE__);
        return -ENODEV;
    }

    ack = _swd_read(sg, SWD_AP, SWD_READ, SWD_AP_DRW_REG & 0xC, &data, true);
    if (ack != SWD_OK) {
        ERR("%s: [%s] %d ack:%d\n", __FILE__, __func__, __LINE__, ack);
        return -ENODEV;
    }

    for (i = 0 ; i < len_to_read-1 ; i++) {
        ack = _swd_read(sg, SWD_AP, SWD_READ, SWD_AP_DRW_REG & 0xC, &data, true);
        if (ack != SWD_OK) {
            ERR("%s: [%s] %d ack:%d\n", __FILE__, __func__, __LINE__, ack);
            break;
        }
        buf[i] = data;
    }

    _swd_read(sg, SWD_DP, SWD_READ, SWD_DP_RDBUFF_REG, &data, true);
    buf[i] = data;

    return (++i) * 4;
}

ssize_t _swd_ap_write(struct swd_gpio *sg, void *from, u32 base, u32 len)
{
    int i;
    u8 ack = 0;
    u32 *buf = from;
    ssize_t len_to_write = len / sizeof(u32);

    ack = _swd_send(sg, SWD_DP, SWD_WRITE, SWD_DP_SELECT_REG, SWD_AP_TAR_REG & 0xF0, true);
    if (ack != SWD_OK) {
        ERR("%s: [%s] %d select first AP bank fail\n", __FILE__, __func__, __LINE__);
        return -ENODEV;
    }

    ack = _swd_send(sg, SWD_AP, SWD_WRITE, SWD_AP_TAR_REG & 0xC, base, true);
    if (ack != SWD_OK) {
        ERR("%s: [%s] %d select first AP bank fail\n", __FILE__, __func__, __LINE__);
        return -ENODEV;
    }

    for (i = 0 ; i < len_to_write ; i++) {
        ack = _swd_send(sg, SWD_AP, SWD_WRITE, SWD_AP_DRW_REG & 0xC, buf[i], true);
        if (ack != SWD_OK) {
            ERR("%s: [%s] %d ack:%d\n", __FILE__, __func__, __LINE__, ack);
            break;
        }
    }

    return (++i) * 4;
}
