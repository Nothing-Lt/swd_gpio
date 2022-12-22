#ifndef SWD_GPIO_H
#define SWD_GPIO_H

#include <linux/types.h>

struct swd_gpio {
    // functions for protecting signal
    void (*signal_begin)(void);
    void (*signal_end)(void);

    // functions for swd pins
    void (*SWCLK_SET)(int v);
    void (*SWDIO_DIR_IN)(void);
    void (*SWDIO_DIR_OUT)(void);
    void (*SWDIO_SET)(int v);
    int (*SWDIO_GET)(void);

    // function for delay
    void (*_delay)(void);
};

enum SWD_ACK {
    SWD_OK      = 1,
    SWD_WAIT    = 2,
    SWD_FAULT   = 4
};

enum SWD_RW {
    SWD_WRITE   = 0,
    SWD_READ    = 1
};

enum SWD_APNDP {
    SWD_DP  = 0,
    SWD_AP  = 1
};

enum SWD_REG {
    // DP regs
    SWD_DP_IDCODE_REG   = 0x0,
    SWD_DP_ABORT_REG    = 0x0,
    SWD_DP_CTRLSTAT_REG = 0x4,
    SWD_DP_RESEND_REG   = 0x8,
    SWD_DP_SELECT_REG   = 0x8,
    SWD_DP_RDBUFF_REG   = 0xC,
    
    // AP regs
    SWD_AP_CSW_REG  = 0x0,
    SWD_AP_TAR_REG  = 0x4,
    SWD_AP_DRW_REG  = 0xC,
    SWD_AP_IDR_REG  = 0xFC,
};

void _swd_jtag_to_swd(struct swd_gpio *);

u8 _swd_send(struct swd_gpio *,
            u8, u8, u8, u32, bool);

u8 _swd_read(struct swd_gpio *,
            u8, u8, u8, u32 *, bool);

ssize_t _swd_ap_read(struct swd_gpio *, void *, u32, const ssize_t);

ssize_t _swd_ap_write(struct swd_gpio *, void *, u32, u32);

inline void _swd_reset(struct swd_gpio *);

void _swd_jtag_to_swd(struct swd_gpio *);

#endif