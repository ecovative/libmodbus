/*
 * Copyright © Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * 
 * Modified for RP2350 microcontroller support
 * Adds support for Pico SDK, DMA transfers, and flexible RS-485 control
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <assert.h>

#include "modbus-private.h"
#include "modbus-rtu.h"
#include "modbus-rtu-private.h"

#ifndef _MSC_VER
#include <sys/ioctl.h>
#endif

#ifdef PICO_SDK
/* RP2350 specific implementations */

/* Parse device configuration string */
int parse_device_config(modbus_rtu_t *ctx_rtu, const char *device_str) {
    char device_copy[64];
    strncpy(device_copy, device_str, sizeof(device_copy) - 1);
    device_copy[sizeof(device_copy) - 1] = '\0';
    
    /* Parse format: "uart[0-1][:re_pin[:de_pin]]" */
    char *token;
    char *saveptr;
    int param_count = 0;
    int params[3] = {-1, -1, -1};  /* uart_num, re_pin, de_pin */
    
    /* First token is the UART device */
    token = strtok_r(device_copy, ":", &saveptr);
    if (token == NULL) {
        return -1;
    }
    
    /* Extract UART number and set default pins */
    if (strncmp(token, "uart", 4) == 0) {
        int uart_num = token[4] - '0';
        if (uart_num == 0) {
            ctx_rtu->uart = uart0;
            ctx_rtu->tx_pin = 0;  /* Default GPIO0 for UART0 TX */
            ctx_rtu->rx_pin = 1;  /* Default GPIO1 for UART0 RX */
        } else if (uart_num == 1) {
            ctx_rtu->uart = uart1;
            ctx_rtu->tx_pin = 4;  /* Default GPIO4 for UART1 TX */
            ctx_rtu->rx_pin = 5;  /* Default GPIO5 for UART1 RX */
        } else {
            return -1;  /* Invalid UART number */
        }
    } else {
        return -1;  /* Invalid format */
    }
    
    /* Parse optional RE pin */
    token = strtok_r(NULL, ":", &saveptr);
    if (token != NULL) {
        params[1] = atoi(token);
        param_count++;
        
        /* Parse optional DE pin */
        token = strtok_r(NULL, ":", &saveptr);
        if (token != NULL) {
            params[2] = atoi(token);
            param_count++;
        }
    }
    
    /* Configure RS-485 mode based on parameters */
    if (param_count == 0) {
        /* No RS-485 control pins specified */
        ctx_rtu->rs485_mode = RS485_MODE_NONE;
        if (ctx_rtu->debug) {
            fprintf(stderr, "UART configured without RS-485 control\n");
        }
    } else if (param_count == 1) {
        /* Single pin for combined RE/DE control */
        ctx_rtu->rs485_mode = RS485_MODE_COMBINED;
        ctx_rtu->re_pin = params[1];
        ctx_rtu->de_pin = params[1];
        if (ctx_rtu->debug) {
            fprintf(stderr, "RS-485: Combined RE/DE control on GPIO%d\n", params[1]);
        }
    } else if (param_count == 2) {
        /* Two pins specified */
        ctx_rtu->re_pin = params[1];
        ctx_rtu->de_pin = params[2];
        
        if (ctx_rtu->re_pin == ctx_rtu->de_pin) {
            /* Same pin specified twice - combined mode */
            ctx_rtu->rs485_mode = RS485_MODE_COMBINED;
            if (ctx_rtu->debug) {
                fprintf(stderr, "RS-485: Combined RE/DE control on GPIO%d\n", ctx_rtu->re_pin);
            }
        } else {
            /* Different pins - separate mode */
            ctx_rtu->rs485_mode = RS485_MODE_SEPARATE;
            if (ctx_rtu->debug) {
                fprintf(stderr, "RS-485: Separate control - RE on GPIO%d, DE on GPIO%d\n", 
                       ctx_rtu->re_pin, ctx_rtu->de_pin);
            }
        }
    }
    
    return 0;
}

/* Calculate Modbus RTU timing parameters */
void calculate_rtu_timing(modbus_rtu_t *ctx_rtu) {
    /* Calculate character transmission time in microseconds */
    /* Character = 1 start + data_bits + parity + stop_bits */
    uint32_t bits_per_char = 1 + ctx_rtu->data_bit + 
                            (ctx_rtu->parity != 'N' ? 1 : 0) + 
                            ctx_rtu->stop_bit;
    
    ctx_rtu->char_time_us = (bits_per_char * 1000000) / ctx_rtu->baud;
    
    /* Modbus RTU inter-frame delay (3.5 character times) */
    /* Minimum 1750us as per Modbus specification */
    ctx_rtu->t35_us = ctx_rtu->char_time_us * 35 / 10;
    if (ctx_rtu->t35_us < 1750) {
        ctx_rtu->t35_us = 1750;
    }
    
    /* Inter-character timeout (1.5 character times) */
    /* Minimum 750us as per Modbus specification */
    ctx_rtu->t15_us = ctx_rtu->char_time_us * 15 / 10;
    if (ctx_rtu->t15_us < 750) {
        ctx_rtu->t15_us = 750;
    }
}

/* Set transceiver to transmit mode */
void rs485_set_mode_tx(modbus_rtu_t *ctx_rtu) {
    switch (ctx_rtu->rs485_mode) {
        case RS485_MODE_NONE:
            /* No RS-485 control needed */
            break;
            
        case RS485_MODE_COMBINED:
            /* Single pin controls both RE and DE */
            /* For SP3485: HIGH = transmit (RE=HIGH, DE=HIGH) */
            gpio_put(ctx_rtu->re_pin, !ctx_rtu->re_inverted);
            break;
            
        case RS485_MODE_SEPARATE:
            /* Separate control pins */
            /* RE = HIGH (disabled), DE = HIGH (enabled) */
            gpio_put(ctx_rtu->re_pin, !ctx_rtu->re_inverted);  /* Disable receiver */
            gpio_put(ctx_rtu->de_pin, !ctx_rtu->de_inverted);  /* Enable driver */
            break;
    }
    
    if (ctx_rtu->rs485_mode != RS485_MODE_NONE) {
        /* SP3485 requires max 120ns to enable driver */
        if (ctx_rtu->dir_delay_us > 0) {
            busy_wait_us(ctx_rtu->dir_delay_us);
        } else {
            busy_wait_at_least_cycles(16);  /* ~120ns at 133MHz */
        }
    }
}

/* Set transceiver to receive mode */
void rs485_set_mode_rx(modbus_rtu_t *ctx_rtu) {
    if (ctx_rtu->rs485_mode == RS485_MODE_NONE) {
        return;
    }
    
    /* Ensure transmit is complete */
    uart_tx_wait_blocking(ctx_rtu->uart);
    
    /* Additional wait for last stop bit(s) */
    busy_wait_us(2);
    
    switch (ctx_rtu->rs485_mode) {
        case RS485_MODE_COMBINED:
            /* Single pin controls both RE and DE */
            /* For SP3485: LOW = receive (RE=LOW, DE=LOW) */
            gpio_put(ctx_rtu->re_pin, ctx_rtu->re_inverted);
            break;
            
        case RS485_MODE_SEPARATE:
            /* Separate control pins */
            /* RE = LOW (enabled), DE = LOW (disabled) */
            gpio_put(ctx_rtu->de_pin, ctx_rtu->de_inverted);   /* Disable driver first */
            gpio_put(ctx_rtu->re_pin, ctx_rtu->re_inverted);   /* Enable receiver */
            break;
            
        default:
            break;
    }
    
    if (ctx_rtu->rs485_mode != RS485_MODE_NONE) {
        /* SP3485 requires max 60ns to enable receiver */
        if (ctx_rtu->dir_delay_us > 0) {
            busy_wait_us(ctx_rtu->dir_delay_us);
        } else {
            busy_wait_at_least_cycles(8);  /* ~60ns at 133MHz */
        }
    }
}

/* Initialize RS-485 control pins */
int rs485_init_pins(modbus_rtu_t *ctx_rtu) {
    switch (ctx_rtu->rs485_mode) {
        case RS485_MODE_NONE:
            /* No initialization needed */
            return 0;
            
        case RS485_MODE_COMBINED:
            /* Initialize single control pin */
            gpio_init(ctx_rtu->re_pin);
            gpio_set_dir(ctx_rtu->re_pin, GPIO_OUT);
            /* Start in receive mode (LOW for SP3485) */
            gpio_put(ctx_rtu->re_pin, ctx_rtu->re_inverted);
            if (ctx_rtu->debug) {
                fprintf(stderr, "Initialized combined RE/DE control on GPIO%d\n", ctx_rtu->re_pin);
            }
            break;
            
        case RS485_MODE_SEPARATE:
            /* Initialize RE pin */
            gpio_init(ctx_rtu->re_pin);
            gpio_set_dir(ctx_rtu->re_pin, GPIO_OUT);
            gpio_put(ctx_rtu->re_pin, ctx_rtu->re_inverted);  /* Enable receiver */
            
            /* Initialize DE pin */
            gpio_init(ctx_rtu->de_pin);
            gpio_set_dir(ctx_rtu->de_pin, GPIO_OUT);
            gpio_put(ctx_rtu->de_pin, ctx_rtu->de_inverted);  /* Disable driver */
            
            if (ctx_rtu->debug) {
                fprintf(stderr, "Initialized separate RE (GPIO%d) and DE (GPIO%d) control\n", 
                       ctx_rtu->re_pin, ctx_rtu->de_pin);
            }
            break;
    }
    
    return 0;
}

#ifdef MODBUS_USE_DMA
/* DMA interrupt handlers */
static void dma_tx_handler(void) {
    /* Get context from DMA channel */
    /* Note: This is a simplified version - you'd need to store context globally or per-channel */
    /* Clear the interrupt and signal completion */
}

static void dma_rx_handler(void) {
    /* Get context from DMA channel */
    /* Clear the interrupt and signal completion */
}

/* Initialize DMA for UART transfers */
int _modbus_rtu_init_dma(modbus_t *ctx) {
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    
    /* Allocate DMA structure */
    ctx_rtu->dma = (modbus_rtu_dma_t *)calloc(1, sizeof(modbus_rtu_dma_t));
    if (ctx_rtu->dma == NULL) {
        errno = ENOMEM;
        return -1;
    }
    
    /* Claim DMA channels */
    ctx_rtu->dma->tx_channel = dma_claim_unused_channel(true);
    ctx_rtu->dma->rx_channel = dma_claim_unused_channel(true);
    
    /* Allocate DMA buffers */
    ctx_rtu->dma->tx_buffer = (uint8_t *)malloc(MODBUS_RTU_MAX_ADU_LENGTH);
    ctx_rtu->dma->rx_buffer = (uint8_t *)malloc(MODBUS_RTU_MAX_ADU_LENGTH);
    ctx_rtu->dma->rx_circular_buffer = (uint8_t *)malloc(1024);
    ctx_rtu->dma->rx_circular_size = 1024;
    
    if (!ctx_rtu->dma->tx_buffer || !ctx_rtu->dma->rx_buffer || 
        !ctx_rtu->dma->rx_circular_buffer) {
        _modbus_rtu_deinit_dma(ctx_rtu);
        errno = ENOMEM;
        return -1;
    }
    
    /* Configure TX DMA */
    ctx_rtu->dma->tx_config = dma_channel_get_default_config(ctx_rtu->dma->tx_channel);
    channel_config_set_transfer_data_size(&ctx_rtu->dma->tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&ctx_rtu->dma->tx_config, true);
    channel_config_set_write_increment(&ctx_rtu->dma->tx_config, false);
    channel_config_set_dreq(&ctx_rtu->dma->tx_config, 
        uart_get_dreq(ctx_rtu->uart, true));  /* TX DREQ */
    
    /* Configure RX DMA */
    ctx_rtu->dma->rx_config = dma_channel_get_default_config(ctx_rtu->dma->rx_channel);
    channel_config_set_transfer_data_size(&ctx_rtu->dma->rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&ctx_rtu->dma->rx_config, false);
    channel_config_set_write_increment(&ctx_rtu->dma->rx_config, true);
    channel_config_set_dreq(&ctx_rtu->dma->rx_config, 
        uart_get_dreq(ctx_rtu->uart, false));  /* RX DREQ */
    
    /* Initialize synchronization */
    sem_init(&ctx_rtu->dma->tx_sem, 0, 1);
    sem_init(&ctx_rtu->dma->rx_sem, 0, 1);
    
    ctx_rtu->dma->rx_head = 0;
    ctx_rtu->dma->rx_tail = 0;
    
    return 0;
}

/* Deinitialize DMA */
void _modbus_rtu_deinit_dma(modbus_rtu_t *ctx_rtu) {
    if (ctx_rtu->dma) {
        /* Free DMA channels */
        if (ctx_rtu->dma->tx_channel >= 0) {
            dma_channel_unclaim(ctx_rtu->dma->tx_channel);
        }
        if (ctx_rtu->dma->rx_channel >= 0) {
            dma_channel_unclaim(ctx_rtu->dma->rx_channel);
        }
        
        /* Free buffers */
        free(ctx_rtu->dma->tx_buffer);
        free(ctx_rtu->dma->rx_buffer);
        free(ctx_rtu->dma->rx_circular_buffer);
        
        /* Free DMA structure */
        free(ctx_rtu->dma);
        ctx_rtu->dma = NULL;
    }
}

/* Send data using DMA */
ssize_t _modbus_rtu_send_dma(modbus_t *ctx, const uint8_t *req, int req_length) {
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    
    if (!ctx_rtu->dma) {
        /* DMA not initialized, fall back to blocking */
        return _modbus_rtu_send(ctx, req, req_length);
    }
    
    /* Copy data to DMA buffer */
    memcpy(ctx_rtu->dma->tx_buffer, req, req_length);
    
    /* Reset completion flag */
    ctx_rtu->dma->tx_complete = false;
    
    /* Configure and start DMA transfer */
    dma_channel_configure(
        ctx_rtu->dma->tx_channel,
        &ctx_rtu->dma->tx_config,
        &uart_get_hw(ctx_rtu->uart)->dr,  /* Write to UART data register */
        ctx_rtu->dma->tx_buffer,           /* Read from buffer */
        req_length,                        /* Transfer count */
        true                               /* Start immediately */
    );
    
    /* Wait for completion */
    dma_channel_wait_for_finish_blocking(ctx_rtu->dma->tx_channel);
    
    return req_length;
}
#endif /* MODBUS_USE_DMA */

/* Extended configuration functions */
void modbus_rtu_set_direction_delay(modbus_t *ctx, uint32_t delay_us) {
    if (ctx && ctx->backend_data) {
        modbus_rtu_t *ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
        ctx_rtu->dir_delay_us = delay_us;
    }
}

void modbus_rtu_set_transceiver_mode(modbus_t *ctx, bool re_inverted, bool de_inverted) {
    if (ctx && ctx->backend_data) {
        modbus_rtu_t *ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
        ctx_rtu->re_inverted = re_inverted;
        ctx_rtu->de_inverted = de_inverted;
    }
}

void modbus_rtu_set_transceiver_type(modbus_t *ctx, transceiver_type_t type) {
    if (ctx && ctx->backend_data) {
        modbus_rtu_t *ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
        ctx_rtu->transceiver_type = type;
        
        switch (type) {
            case TRANSCEIVER_SP3485:
            case TRANSCEIVER_MAX485:
            case TRANSCEIVER_SN75176:
            case TRANSCEIVER_ADM2582:
                /* These all use standard logic */
                ctx_rtu->re_inverted = false;  /* RE active LOW */
                ctx_rtu->de_inverted = false;  /* DE active HIGH */
                break;
                
            case TRANSCEIVER_CUSTOM:
                /* User must set manually */
                break;
        }
    }
}

rs485_mode_t modbus_rtu_get_rs485_mode(modbus_t *ctx) {
    if (ctx && ctx->backend_data) {
        modbus_rtu_t *ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
        return ctx_rtu->rs485_mode;
    }
    return RS485_MODE_NONE;
}

/* Debug functions */
void modbus_rtu_debug_pins(modbus_t *ctx) {
    if (ctx && ctx->backend_data) {
        modbus_rtu_t *ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
        
        fprintf(stderr, "RS-485 Configuration:\n");
        fprintf(stderr, "  Mode: ");
        switch (ctx_rtu->rs485_mode) {
            case RS485_MODE_NONE:
                fprintf(stderr, "No RS-485 control\n");
                break;
            case RS485_MODE_COMBINED:
                fprintf(stderr, "Combined RE/DE on GPIO%d\n", ctx_rtu->re_pin);
                fprintf(stderr, "  Current state: %s\n", 
                       gpio_get(ctx_rtu->re_pin) ? "TRANSMIT" : "RECEIVE");
                break;
            case RS485_MODE_SEPARATE:
                fprintf(stderr, "Separate control\n");
                fprintf(stderr, "  RE on GPIO%d (active %s): %d\n", 
                       ctx_rtu->re_pin, 
                       ctx_rtu->re_inverted ? "HIGH" : "LOW",
                       gpio_get(ctx_rtu->re_pin));
                fprintf(stderr, "  DE on GPIO%d (active %s): %d\n", 
                       ctx_rtu->de_pin,
                       ctx_rtu->de_inverted ? "LOW" : "HIGH",
                       gpio_get(ctx_rtu->de_pin));
                break;
        }
        fprintf(stderr, "  Direction delay: %u us\n", ctx_rtu->dir_delay_us);
    }
}

void modbus_rtu_debug_timing(modbus_t *ctx) {
    if (ctx && ctx->backend_data) {
        modbus_rtu_t *ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
        fprintf(stderr, "Modbus RTU Timing:\n");
        fprintf(stderr, "  Baud rate: %d\n", ctx_rtu->baud);
        fprintf(stderr, "  Character time: %u us\n", ctx_rtu->char_time_us);
        fprintf(stderr, "  T1.5 (inter-char): %u us\n", ctx_rtu->t15_us);
        fprintf(stderr, "  T3.5 (inter-frame): %u us\n", ctx_rtu->t35_us);
    }
}

#endif /* PICO_SDK */

/* CRC calculation for Modbus RTU */
static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

/* Build RTU request header */
int _modbus_rtu_build_request_basis(modbus_t *ctx, int function,
                                    int addr, int nb,
                                    uint8_t *req)
{
    assert(ctx->slave != -1);
    req[0] = ctx->slave;
    req[1] = function;
    req[2] = addr >> 8;
    req[3] = addr & 0x00ff;
    req[4] = nb >> 8;
    req[5] = nb & 0x00ff;

    return _MODBUS_RTU_PRESET_REQ_LENGTH;
}

/* Build RTU response header */
int _modbus_rtu_build_response_basis(sft_t *sft, uint8_t *rsp)
{
    rsp[0] = sft->slave;
    rsp[1] = sft->function;

    return _MODBUS_RTU_PRESET_RSP_LENGTH;
}

/* Prepare response tid (not used in RTU) */
int _modbus_rtu_prepare_response_tid(const uint8_t *req, int *req_length)
{
    (*req_length) -= _MODBUS_RTU_CHECKSUM_LENGTH;
    /* No TID in RTU */
    return 0;
}

/* Add CRC to message before sending */
int _modbus_rtu_send_msg_pre(uint8_t *req, int req_length)
{
    uint16_t crc = crc16(req, req_length);
    req[req_length++] = crc >> 8;
    req[req_length++] = crc & 0x00FF;

    return req_length;
}

#ifdef PICO_SDK
/* RP2350 specific send implementation */
ssize_t _modbus_rtu_send(modbus_t *ctx, const uint8_t *req, int req_length)
{
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    
    /* Clear any pending RX data */
    while (uart_is_readable(ctx_rtu->uart)) {
        uart_getc(ctx_rtu->uart);
    }
    
    /* Switch to transmit mode if RS-485 is enabled */
    rs485_set_mode_tx(ctx_rtu);
    
#ifdef MODBUS_USE_DMA
    if (ctx_rtu->dma) {
        /* Use DMA transfer */
        ssize_t result = _modbus_rtu_send_dma(ctx, req, req_length);
        /* Switch back to receive mode */
        rs485_set_mode_rx(ctx_rtu);
        return result;
    }
#endif
    
    /* Blocking transfer */
    uart_write_blocking(ctx_rtu->uart, req, req_length);
    
    /* Switch back to receive mode after transmission complete */
    rs485_set_mode_rx(ctx_rtu);
    
    return req_length;
}

/* RP2350 specific receive implementation */
ssize_t _modbus_rtu_recv(modbus_t *ctx, uint8_t *rsp, int rsp_length)
{
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    int msg_length = 0;
    
    /* Ensure we're in receive mode */
    rs485_set_mode_rx(ctx_rtu);
    
    /* Calculate timeouts */
    absolute_time_t frame_timeout = make_timeout_time_us(ctx_rtu->t35_us);
    absolute_time_t response_timeout = make_timeout_time_ms(
        ctx->response_timeout.tv_sec * 1000 + 
        ctx->response_timeout.tv_usec / 1000
    );
    
    /* Receive loop with inter-character timeout */
    while (msg_length < rsp_length) {
        if (uart_is_readable_within_us(ctx_rtu->uart, ctx_rtu->t15_us)) {
            uint8_t byte = uart_getc(ctx_rtu->uart);
            rsp[msg_length++] = byte;
            
            /* Reset frame timeout on each received byte */
            frame_timeout = make_timeout_time_us(ctx_rtu->t35_us);
        } else {
            /* Check for frame timeout (3.5 character times of silence) */
            if (time_reached(frame_timeout)) {
                if (msg_length > 0) {
                    /* Frame complete */
                    break;
                }
            }
            
            /* Check for overall response timeout */
            if (time_reached(response_timeout)) {
                if (msg_length == 0) {
                    errno = ETIMEDOUT;
                    /* Increment error counter */
                    ctx_rtu->rx_errors++;
                    return -1;
                }
                break;
            }
        }
    }
    
    return msg_length;
}

/* Connect to Modbus RTU */
int _modbus_rtu_connect(modbus_t *ctx)
{
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    
    /* Parse device configuration string */
    if (parse_device_config(ctx_rtu, ctx_rtu->device) < 0) {
        if (ctx->debug) {
            fprintf(stderr, "Invalid device configuration: %s\n", ctx_rtu->device);
        }
        errno = EINVAL;
        return -1;
    }
    
    /* Initialize UART */
    uart_init(ctx_rtu->uart, ctx_rtu->baud);
    
    /* Configure UART pins */
    gpio_set_function(ctx_rtu->tx_pin, GPIO_FUNC_UART);
    gpio_set_function(ctx_rtu->rx_pin, GPIO_FUNC_UART);
    
    /* Configure UART format */
    uart_parity_t parity_mode;
    switch (ctx_rtu->parity) {
        case 'N': parity_mode = UART_PARITY_NONE; break;
        case 'E': parity_mode = UART_PARITY_EVEN; break;
        case 'O': parity_mode = UART_PARITY_ODD; break;
        default:
            errno = EINVAL;
            return -1;
    }
    
    uart_set_format(ctx_rtu->uart, ctx_rtu->data_bit, ctx_rtu->stop_bit, parity_mode);
    uart_set_hw_flow(ctx_rtu->uart, false, false);
    uart_set_fifo_enabled(ctx_rtu->uart, true);
    
    /* Set FIFO trigger levels for optimal performance */
    hw_write_masked(&uart_get_hw(ctx_rtu->uart)->ifls,
                    0 << UART_UARTIFLS_RXIFLSEL_LSB |  /* RX FIFO 1/8 full */
                    0 << UART_UARTIFLS_TXIFLSEL_LSB,   /* TX FIFO 1/8 full */
                    UART_UARTIFLS_RXIFLSEL_BITS | UART_UARTIFLS_TXIFLSEL_BITS);
    
    /* Initialize RS-485 control pins */
    if (rs485_init_pins(ctx_rtu) < 0) {
        errno = EIO;
        return -1;
    }
    
    /* Calculate Modbus timing parameters */
    calculate_rtu_timing(ctx_rtu);
    
    /* Set default direction switching delay */
    ctx_rtu->dir_delay_us = 1;  /* 1 microsecond default */
    
    /* Default to SP3485 transceiver */
    ctx_rtu->transceiver_type = TRANSCEIVER_SP3485;
    ctx_rtu->re_inverted = false;
    ctx_rtu->de_inverted = false;
    
    /* Clear error statistics */
    ctx_rtu->rx_errors = 0;
    ctx_rtu->tx_errors = 0;
    ctx_rtu->frame_errors = 0;
    ctx_rtu->parity_errors = 0;
    ctx_rtu->overrun_errors = 0;
    
#ifdef MODBUS_USE_DMA
    /* Initialize DMA if enabled */
    _modbus_rtu_init_dma(ctx);
#endif
    
    if (ctx->debug) {
        fprintf(stderr, "Modbus RTU connected: %s at %d baud, %d-%c-%d\n", 
               ctx_rtu->device, ctx_rtu->baud, ctx_rtu->data_bit, 
               ctx_rtu->parity, ctx_rtu->stop_bit);
        modbus_rtu_debug_timing(ctx);
    }
    
    /* Mark as connected */
    ctx->s = 1;  /* Use 1 as a dummy file descriptor for Pico SDK */
    
    return 0;
}

/* Close Modbus RTU connection */
void _modbus_rtu_close(modbus_t *ctx)
{
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    
    /* Set RS-485 to receive mode before closing */
    rs485_set_mode_rx(ctx_rtu);
    
#ifdef MODBUS_USE_DMA
    /* Deinitialize DMA if used */
    if (ctx_rtu->dma) {
        _modbus_rtu_deinit_dma(ctx_rtu);
    }
#endif
    
    /* Deinitialize UART */
    uart_deinit(ctx_rtu->uart);
    
    /* Mark as disconnected */
    ctx->s = -1;
}

/* Flush UART buffers */
int _modbus_rtu_flush(modbus_t *ctx)
{
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    
    /* Clear UART RX buffer */
    while (uart_is_readable(ctx_rtu->uart)) {
        uart_getc(ctx_rtu->uart);
    }
    
    /* Wait for TX to complete */
    uart_tx_wait_blocking(ctx_rtu->uart);
    
    return 0;
}

/* Select implementation for Pico SDK (simplified) */
int _modbus_rtu_select(modbus_t *ctx, fd_set *rset, struct timeval *tv, int length_to_read)
{
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    
    /* Convert timeval to microseconds */
    uint64_t timeout_us = 0;
    if (tv) {
        timeout_us = tv->tv_sec * 1000000 + tv->tv_usec;
    }
    
    /* Check if data is available */
    if (uart_is_readable_within_us(ctx_rtu->uart, timeout_us)) {
        return 1;  /* Data available */
    }
    
    return 0;  /* Timeout */
}

#else /* !PICO_SDK - Original POSIX/Windows implementation */

/* Original libmodbus implementations for non-Pico platforms */
/* These would include the original termios-based serial port handling */
/* Not included here for brevity, but would be copied from original libmodbus */

#endif /* PICO_SDK */

/* Common functions for all platforms */

/* Check CRC of received message */
int _modbus_rtu_check_integrity(modbus_t *ctx, uint8_t *msg,
                                const int msg_length)
{
    uint16_t crc_calculated;
    uint16_t crc_received;
    int slave = msg[0];

    /* Filter on the Modbus unit identifier (slave) in RTU mode to avoid useless
     * CRC computing. */
    if (slave != ctx->slave && slave != MODBUS_BROADCAST_ADDRESS) {
        if (ctx->debug) {
            printf("Request for slave %d ignored (not %d)\n", slave, ctx->slave);
        }
        /* Following call to check_confirmation handles this error */
        return 0;
    }

    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of msg */
    if (crc_calculated == crc_received) {
        return msg_length;
    } else {
        if (ctx->debug) {
            fprintf(stderr, "ERROR CRC received 0x%0X != CRC calculated 0x%0X\n",
                    crc_received, crc_calculated);
        }
#ifdef PICO_SDK
        if (ctx->backend_data) {
            modbus_rtu_t *ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
            ctx_rtu->frame_errors++;
        }
#endif
        if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
            _modbus_rtu_flush(ctx);
        }
        errno = EMBBADCRC;
        return -1;
    }
}

/* The check_confirmation function shall return 0 if the indication is not
 * for us or an error (invalid function or slave) */
int _modbus_rtu_pre_check_confirmation(modbus_t *ctx, const uint8_t *req,
                                       const uint8_t *rsp, int rsp_length)
{
    /* Check responding slave is the slave we requested (except for broadcast
     * request) */
    if (req[0] != rsp[0] && req[0] != MODBUS_BROADCAST_ADDRESS) {
        if (ctx->debug) {
            fprintf(stderr,
                    "The responding slave %d isn't the requested slave %d\n",
                    rsp[0], req[0]);
        }
        errno = EMBBADSLAVE;
        return -1;
    } else {
        return 0;
    }
}

/* Free RTU context */
void _modbus_rtu_free(modbus_t *ctx)
{
    if (ctx->backend_data) {
        free(((modbus_rtu_t *)ctx->backend_data)->device);
        free(ctx->backend_data);
    }
    free(ctx);
}

/* Modbus RTU backend structure */
const modbus_backend_t _modbus_rtu_backend = {
    _MODBUS_BACKEND_TYPE_RTU,
    _MODBUS_RTU_HEADER_LENGTH,
    _MODBUS_RTU_CHECKSUM_LENGTH,
    MODBUS_RTU_MAX_ADU_LENGTH,
    _modbus_set_slave,
    _modbus_rtu_build_request_basis,
    _modbus_rtu_build_response_basis,
    _modbus_rtu_prepare_response_tid,
    _modbus_rtu_send_msg_pre,
    _modbus_rtu_send,
    _modbus_rtu_receive,
    _modbus_rtu_recv,
    _modbus_rtu_check_integrity,
    _modbus_rtu_pre_check_confirmation,
    _modbus_rtu_connect,
    _modbus_rtu_close,
    _modbus_rtu_flush,
    _modbus_rtu_select,
    _modbus_rtu_free
};

/* Create new RTU context */
modbus_t* modbus_new_rtu(const char *device,
                         int baud, char parity, int data_bit,
                         int stop_bit)
{
    modbus_t *ctx;
    modbus_rtu_t *ctx_rtu;

    /* Check device argument */
    if (device == NULL || *device == 0) {
        fprintf(stderr, "The device string is empty\n");
        errno = EINVAL;
        return NULL;
    }

    /* Check baud argument */
    if (baud == 0) {
        fprintf(stderr, "The baud rate value must not be zero\n");
        errno = EINVAL;
        return NULL;
    }

    ctx = (modbus_t *)malloc(sizeof(modbus_t));
    if (ctx == NULL) {
        return NULL;
    }
    _modbus_init_common(ctx);
    ctx->backend = &_modbus_rtu_backend;
    ctx->backend_data = (modbus_rtu_t *)malloc(sizeof(modbus_rtu_t));
    if (ctx->backend_data == NULL) {
        modbus_free(ctx);
        errno = ENOMEM;
        return NULL;
    }
    ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
    memset(ctx_rtu, 0, sizeof(modbus_rtu_t));

    /* Device name and settings */
    ctx_rtu->device = (char *)malloc(strlen(device) + 1);
    if (ctx_rtu->device == NULL) {
        modbus_free(ctx);
        errno = ENOMEM;
        return NULL;
    }
    strcpy(ctx_rtu->device, device);

    ctx_rtu->baud = baud;
    if (parity == 'N' || parity == 'E' || parity == 'O') {
        ctx_rtu->parity = parity;
    } else {
        modbus_free(ctx);
        errno = EINVAL;
        return NULL;
    }
    ctx_rtu->data_bit = data_bit;
    ctx_rtu->stop_bit = stop_bit;

#ifdef PICO_SDK
    /* Initialize Pico-specific fields */
    ctx_rtu->uart = NULL;
    ctx_rtu->rs485_mode = RS485_MODE_NONE;
    ctx_rtu->transceiver_type = TRANSCEIVER_SP3485;
    ctx_rtu->re_inverted = false;
    ctx_rtu->de_inverted = false;
    ctx_rtu->dir_delay_us = 1;
#ifdef MODBUS_USE_DMA
    ctx_rtu->dma = NULL;
#endif
#ifdef MODBUS_MULTICORE_SUPPORT
    ctx_rtu->multicore = NULL;
#endif
#endif

    ctx_rtu->confirmation_to_ignore = FALSE;

    return ctx;
}

/* RTU CRC tables */
/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};