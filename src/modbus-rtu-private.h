/*
 * Copyright © Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * 
 * Modified for RP2350 microcontroller support
 * Adds support for Pico SDK, DMA transfers, and flexible RS-485 control
 */

#ifndef MODBUS_RTU_PRIVATE_H
#define MODBUS_RTU_PRIVATE_H

#ifndef _MSC_VER
#include <stdint.h>
#else
#include "stdint.h"
#endif

#if defined(_WIN32)
#include <windows.h>
#elif defined(PICO_SDK)
/* RP2350/Pico SDK includes */
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#else
#include <termios.h>
#endif

/* RTU header constants */
#define _MODBUS_RTU_HEADER_LENGTH      1
#define _MODBUS_RTU_PRESET_REQ_LENGTH  6
#define _MODBUS_RTU_PRESET_RSP_LENGTH  2
#define _MODBUS_RTU_CHECKSUM_LENGTH    2

/* Maximum ADU length for RTU */
#define MODBUS_RTU_MAX_ADU_LENGTH      256

/* RS-485 control modes */
typedef enum {
    RS485_MODE_NONE = 0,        /* No RS-485 control */
    RS485_MODE_COMBINED,         /* RE and DE tied together (single pin) */
    RS485_MODE_SEPARATE          /* RE and DE on separate pins */
} rs485_mode_t;

/* Supported transceiver types */
typedef enum {
    TRANSCEIVER_SP3485,          /* SP3485: RE active LOW, DE active HIGH */
    TRANSCEIVER_MAX485,          /* MAX485: RE active LOW, DE active HIGH */
    TRANSCEIVER_SN75176,         /* SN75176: RE active LOW, DE active HIGH */
    TRANSCEIVER_ADM2582,         /* ADM2582: RE active LOW, DE active HIGH */
    TRANSCEIVER_CUSTOM           /* Custom configuration */
} transceiver_type_t;

#if defined(_WIN32)
#if !defined(ENOTSUP)
#define ENOTSUP WSAEOPNOTSUPP
#endif

/* WIN32: struct containing serial handle and a receive buffer */
#define PY_BUF_SIZE 512

struct win32_ser {
    /* File handle */
    HANDLE fd;
    /* Receive buffer */
    uint8_t buf[PY_BUF_SIZE];
    /* Received chars */
    DWORD n_bytes;
};
#endif /* _WIN32 */

#ifdef PICO_SDK
/* DMA configuration structure */
typedef struct _modbus_rtu_dma {
    /* DMA channels */
    int tx_channel;
    int rx_channel;
    
    /* DMA configurations */
    dma_channel_config tx_config;
    dma_channel_config rx_config;
    
    /* DMA buffers */
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    
    /* Circular buffer for continuous RX (optional) */
    uint8_t *rx_circular_buffer;
    uint32_t rx_circular_size;
    volatile uint32_t rx_head;
    volatile uint32_t rx_tail;
    
    /* DMA state */
    volatile bool tx_complete;
    volatile bool rx_complete;
    
    /* Synchronization */
    semaphore_t tx_sem;
    semaphore_t rx_sem;
} modbus_rtu_dma_t;

/* Multi-core configuration */
typedef struct _modbus_rtu_multicore {
    uint core_num;               /* Which core this instance runs on */
    mutex_t uart_mutex;          /* Mutex for UART access */
    volatile bool initialized;   /* Initialization flag */
} modbus_rtu_multicore_t;
#endif /* PICO_SDK */

typedef struct _modbus_rtu {
    /* Device: "/dev/ttyS0", "/dev/ttyUSB0" or "/dev/tty.USA19*" on Mac OS X
       For RP2350: "uart0", "uart1", "uart0:2", "uart1:2:3" etc. */
    char *device;
    
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    int baud;
    
    /* Data bit: 5, 6, 7, 8 */
    uint8_t data_bit;
    
    /* Stop bit: 1, 2 */
    uint8_t stop_bit;
    
    /* Parity: 'N', 'O', 'E' */
    char parity;
    
#if defined(_WIN32)
    /* Windows serial port handling */
    struct win32_ser w_ser;
    DCB old_dcb;
    
#elif defined(PICO_SDK)
    /* RP2350/Pico SDK specific */
    
    /* UART hardware instance */
    uart_inst_t *uart;
    
    /* GPIO pins for UART */
    uint tx_pin;
    uint rx_pin;
    
    /* RS-485 control */
    rs485_mode_t rs485_mode;    /* Control mode */
    uint re_pin;                 /* RE pin (active LOW) */
    uint de_pin;                 /* DE pin (active HIGH) */
    bool re_inverted;            /* Some transceivers have inverted RE */
    bool de_inverted;            /* Some transceivers have inverted DE */
    transceiver_type_t transceiver_type;  /* Transceiver model */
    
    /* Timing parameters */
    uint32_t t35_us;             /* 3.5 character time in microseconds */
    uint32_t t15_us;             /* 1.5 character time in microseconds */
    uint32_t char_time_us;       /* Single character time in microseconds */
    uint32_t dir_delay_us;       /* Additional delay for direction switching */
    
    /* Optional DMA support */
#ifdef MODBUS_USE_DMA
    modbus_rtu_dma_t *dma;       /* DMA configuration (NULL if not used) */
#endif
    
    /* Optional multi-core support */
#ifdef MODBUS_MULTICORE_SUPPORT
    modbus_rtu_multicore_t *multicore;  /* Multi-core configuration */
#endif
    
    /* Error statistics */
    uint32_t rx_errors;          /* Count of RX errors */
    uint32_t tx_errors;          /* Count of TX errors */
    uint32_t frame_errors;       /* Count of framing errors */
    uint32_t parity_errors;      /* Count of parity errors */
    uint32_t overrun_errors;     /* Count of overrun errors */
    
#else
    /* POSIX/Linux serial port handling */
    
    /* Save old termios settings */
    struct termios old_tios;
    
#if HAVE_DECL_TIOCSRS485
    int serial_mode;
#endif

#if HAVE_DECL_TIOCM_RTS
    int rts;
    int rts_delay;
    int onebyte_time;
    void (*set_rts)(modbus_t *ctx, int on);
#endif
    
#endif /* Platform specific */
    
    /* To handle many slaves on the same link */
    int confirmation_to_ignore;
    
} modbus_rtu_t;

/* RTU backend functions - must be defined in modbus-rtu.c */
int _modbus_rtu_build_request_basis(modbus_t *ctx, int function,
                                    int addr, int nb,
                                    uint8_t *req);
int _modbus_rtu_build_response_basis(sft_t *sft, uint8_t *rsp);
int _modbus_rtu_prepare_response_tid(const uint8_t *req, int *req_length);
int _modbus_rtu_send_msg_pre(uint8_t *req, int req_length);
ssize_t _modbus_rtu_send(modbus_t *ctx, const uint8_t *req, int req_length);
int _modbus_rtu_receive(modbus_t *ctx, uint8_t *req);
ssize_t _modbus_rtu_recv(modbus_t *ctx, uint8_t *rsp, int rsp_length);
int _modbus_rtu_check_integrity(modbus_t *ctx, uint8_t *msg,
                                const int msg_length);
int _modbus_rtu_pre_check_confirmation(modbus_t *ctx, const uint8_t *req,
                                       const uint8_t *rsp, int rsp_length);
int _modbus_rtu_connect(modbus_t *ctx);
void _modbus_rtu_close(modbus_t *ctx);
int _modbus_rtu_flush(modbus_t *ctx);
int _modbus_rtu_select(modbus_t *ctx, fd_set *rset, struct timeval *tv, int length_to_read);
void _modbus_rtu_free(modbus_t *ctx);

#ifdef PICO_SDK
/* RP2350 specific helper functions */

/* RS-485 direction control */
void rs485_set_mode_tx(modbus_rtu_t *ctx_rtu);
void rs485_set_mode_rx(modbus_rtu_t *ctx_rtu);
int rs485_init_pins(modbus_rtu_t *ctx_rtu);

/* Configuration parsing */
int parse_device_config(modbus_rtu_t *ctx_rtu, const char *device_str);

/* Timing calculations */
void calculate_rtu_timing(modbus_rtu_t *ctx_rtu);

/* DMA functions (if enabled) */
#ifdef MODBUS_USE_DMA
int _modbus_rtu_init_dma(modbus_t *ctx);
void _modbus_rtu_deinit_dma(modbus_rtu_t *ctx_rtu);
ssize_t _modbus_rtu_send_dma(modbus_t *ctx, const uint8_t *req, int req_length);
ssize_t _modbus_rtu_recv_dma(modbus_t *ctx, uint8_t *rsp, int rsp_length);
void _modbus_rtu_start_circular_dma(modbus_rtu_t *ctx_rtu);
uint32_t _modbus_rtu_circular_available(modbus_rtu_t *ctx_rtu);
uint8_t _modbus_rtu_circular_read_byte(modbus_rtu_t *ctx_rtu);
#endif

/* Multi-core functions (if enabled) */
#ifdef MODBUS_MULTICORE_SUPPORT
modbus_t* modbus_new_rtu_on_core(const char *device, int baud, 
                                 char parity, int data_bit, 
                                 int stop_bit, uint core_num);
int modbus_send_request_async(modbus_t *ctx, const uint8_t *req, int req_length);
void modbus_core1_entry(void);
#endif

/* Extended configuration functions */
void modbus_rtu_set_direction_delay(modbus_t *ctx, uint32_t delay_us);
void modbus_rtu_set_transceiver_mode(modbus_t *ctx, bool re_inverted, bool de_inverted);
void modbus_rtu_set_transceiver_type(modbus_t *ctx, transceiver_type_t type);
rs485_mode_t modbus_rtu_get_rs485_mode(modbus_t *ctx);
void modbus_rtu_get_error_stats(modbus_t *ctx, uint32_t *rx_errors, 
                                uint32_t *tx_errors, uint32_t *frame_errors,
                                uint32_t *parity_errors, uint32_t *overrun_errors);
void modbus_rtu_clear_error_stats(modbus_t *ctx);

/* Debug functions */
void modbus_rtu_debug_pins(modbus_t *ctx);
void modbus_rtu_debug_timing(modbus_t *ctx);
#ifdef MODBUS_USE_DMA
void modbus_rtu_debug_dma_status(modbus_t *ctx);
#endif

#endif /* PICO_SDK */

#endif /* MODBUS_RTU_PRIVATE_H */