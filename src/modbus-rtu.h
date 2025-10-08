/*
 * Copyright © Stéphane Raimbault <stephane.raimbault@gmail.com>
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * RP2350 Port Modifications
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "modbus.h"

MODBUS_BEGIN_DECLS

/* RTU mode definitions */
#define MODBUS_RTU_RS232 0
#define MODBUS_RTU_RS485 1

/* RTS mode definitions */
#define MODBUS_RTU_RTS_NONE 0
#define MODBUS_RTU_RTS_UP 1
#define MODBUS_RTU_RTS_DOWN 2

/* RP2350-specific transceiver types */
#ifdef PICO_SDK
#define MODBUS_RTU_TRANSCEIVER_AUTO 0
#define MODBUS_RTU_TRANSCEIVER_SP3485 1
#define MODBUS_RTU_TRANSCEIVER_MAX485 2
#define MODBUS_RTU_TRANSCEIVER_MAX3485 3
#define MODBUS_RTU_TRANSCEIVER_CUSTOM 99

/* Default GPIO pins (can be overridden) */
#define MODBUS_RTU_DEFAULT_UART0_TX 0
#define MODBUS_RTU_DEFAULT_UART0_RX 1
#define MODBUS_RTU_DEFAULT_UART1_TX 4
#define MODBUS_RTU_DEFAULT_UART1_RX 5
#define MODBUS_RTU_DEFAULT_DE_PIN 2
#define MODBUS_RTU_INVALID_PIN 0xFF

/* Timing constants for common transceivers (microseconds) */
#define MODBUS_RTU_SP3485_DELAY_US 10
#define MODBUS_RTU_MAX485_DELAY_US 5
#define MODBUS_RTU_MAX3485_DELAY_US 5
#define MODBUS_RTU_DEFAULT_DELAY_US 20

/* UART instance identification */
#define MODBUS_RTU_UART0 0
#define MODBUS_RTU_UART1 1
#endif /* PICO_SDK */

/* Standard libmodbus RTU API */

/**
 * @brief Create a new RTU context
 * 
 * @param device Device string. On RP2350: "uart0" or "uart1"
 *               On POSIX: "/dev/ttyS0", "/dev/ttyUSB0", etc.
 *               On Windows: "COM1", "COM2", etc.
 * @param baud Baud rate (e.g., 9600, 19200, 38400, 57600, 115200)
 * @param parity Parity: 'N' (none), 'E' (even), 'O' (odd)
 * @param data_bit Number of data bits (7 or 8)
 * @param stop_bit Number of stop bits (1 or 2)
 * @return modbus_t* Pointer to modbus context or NULL on error
 * 
 * @note RP2350: Device string must be "uart0" or "uart1"
 *       Default pins: UART0 (TX=0, RX=1), UART1 (TX=4, RX=5)
 */
MODBUS_API modbus_t *modbus_new_rtu(const char *device,
                                    int baud,
                                    char parity,
                                    int data_bit,
                                    int stop_bit);

/**
 * @brief Set serial mode (RS232 or RS485)
 * 
 * @param ctx Modbus context
 * @param mode MODBUS_RTU_RS232 or MODBUS_RTU_RS485
 * @return int 0 on success, -1 on error
 */
MODBUS_API int modbus_rtu_set_serial_mode(modbus_t *ctx, int mode);

/**
 * @brief Get current serial mode
 * 
 * @param ctx Modbus context
 * @return int Current mode or -1 on error
 */
MODBUS_API int modbus_rtu_get_serial_mode(modbus_t *ctx);

/**
 * @brief Set RTS mode for RS485 communication
 * 
 * @param ctx Modbus context
 * @param mode MODBUS_RTU_RTS_NONE, MODBUS_RTU_RTS_UP, or MODBUS_RTU_RTS_DOWN
 * @return int 0 on success, -1 on error
 * 
 * @note RP2350: Use modbus_rtu_set_direction_pin() for GPIO-based control
 */
MODBUS_API int modbus_rtu_set_rts(modbus_t *ctx, int mode);

/**
 * @brief Get current RTS mode
 * 
 * @param ctx Modbus context
 * @return int Current RTS mode or -1 on error
 */
MODBUS_API int modbus_rtu_get_rts(modbus_t *ctx);

/**
 * @brief Set custom RTS function
 * 
 * @param ctx Modbus context
 * @param set_rts Custom function to control RTS
 * @return int 0 on success, -1 on error
 */
MODBUS_API int modbus_rtu_set_custom_rts(modbus_t *ctx, void (*set_rts)(modbus_t *ctx, int on));

/**
 * @brief Get RTS delay in microseconds
 * 
 * @param ctx Modbus context
 * @return int RTS delay in microseconds or -1 on error
 */
MODBUS_API int modbus_rtu_get_rts_delay(modbus_t *ctx);

/**
 * @brief Set RTS delay in microseconds
 * 
 * @param ctx Modbus context
 * @param us Delay in microseconds
 * @return int 0 on success, -1 on error
 */
MODBUS_API int modbus_rtu_set_rts_delay(modbus_t *ctx, int us);

/* RP2350-specific extended API */
#ifdef PICO_SDK

/**
 * @brief Set GPIO pin for RS-485 direction control (DE/RE)
 * 
 * @param ctx Modbus context
 * @param gpio_pin GPIO pin number (0-29)
 * @return int 0 on success, -1 on error
 * 
 * @note This pin controls the driver enable (DE) and receiver enable (RE)
 *       For most RS-485 transceivers, DE and RE are tied together
 *       Pin goes HIGH for transmit, LOW for receive
 */
MODBUS_API int modbus_rtu_set_direction_pin(modbus_t *ctx, uint gpio_pin);

/**
 * @brief Get current direction control GPIO pin
 * 
 * @param ctx Modbus context
 * @return int GPIO pin number or -1 if not set
 */
MODBUS_API int modbus_rtu_get_direction_pin(modbus_t *ctx);

/**
 * @brief Set direction control delay (DE enable to TX start)
 * 
 * @param ctx Modbus context
 * @param delay_us Delay in microseconds
 * @return int 0 on success, -1 on error
 * 
 * @note Typical values:
 *       - SP3485: 10-30 µs
 *       - MAX485: 5-15 µs
 *       - Safe default: 20 µs
 */
MODBUS_API int modbus_rtu_set_direction_delay(modbus_t *ctx, uint32_t delay_us);

/**
 * @brief Get current direction control delay
 * 
 * @param ctx Modbus context
 * @return uint32_t Delay in microseconds
 */
MODBUS_API uint32_t modbus_rtu_get_direction_delay(modbus_t *ctx);

/**
 * @brief Set transceiver type for automatic timing configuration
 * 
 * @param ctx Modbus context
 * @param type Transceiver type (MODBUS_RTU_TRANSCEIVER_*)
 * @return int 0 on success, -1 on error
 * 
 * @note Automatically sets optimal timing parameters for the transceiver
 */
MODBUS_API int modbus_rtu_set_transceiver_type(modbus_t *ctx, int type);

/**
 * @brief Get current transceiver type
 * 
 * @param ctx Modbus context
 * @return int Transceiver type or -1 on error
 */
MODBUS_API int modbus_rtu_get_transceiver_type(modbus_t *ctx);

/**
 * @brief Configure UART TX and RX pins
 * 
 * @param ctx Modbus context
 * @param tx_pin GPIO pin for TX (must be valid for selected UART)
 * @param rx_pin GPIO pin for RX (must be valid for selected UART)
 * @return int 0 on success, -1 on error
 * 
 * @note Valid pin mappings:
 *       UART0: TX pins (0, 12, 16, 28), RX pins (1, 13, 17, 29)
 *       UART1: TX pins (4, 8, 20, 24), RX pins (5, 9, 21, 25)
 */
MODBUS_API int modbus_rtu_set_uart_pins(modbus_t *ctx, uint tx_pin, uint rx_pin);

/**
 * @brief Get current UART TX and RX pins
 * 
 * @param ctx Modbus context
 * @param tx_pin Pointer to store TX pin number
 * @param rx_pin Pointer to store RX pin number
 * @return int 0 on success, -1 on error
 */
MODBUS_API int modbus_rtu_get_uart_pins(modbus_t *ctx, uint *tx_pin, uint *rx_pin);

/**
 * @brief Enable or disable hardware flow control (CTS/RTS)
 * 
 * @param ctx Modbus context
 * @param enable 1 to enable, 0 to disable
 * @return int 0 on success, -1 on error
 * 
 * @note Not typically used for RS-485 Modbus
 */
MODBUS_API int modbus_rtu_set_flow_control(modbus_t *ctx, int enable);

/**
 * @brief Get hardware flow control status
 * 
 * @param ctx Modbus context
 * @return int 1 if enabled, 0 if disabled, -1 on error
 */
MODBUS_API int modbus_rtu_get_flow_control(modbus_t *ctx);

/**
 * @brief Set UART FIFO thresholds
 * 
 * @param ctx Modbus context
 * @param tx_threshold TX FIFO interrupt threshold (0-7)
 * @param rx_threshold RX FIFO interrupt threshold (0-7)
 * @return int 0 on success, -1 on error
 * 
 * @note Default: TX=4, RX=4 (trigger at half-full)
 */
MODBUS_API int modbus_rtu_set_fifo_thresholds(modbus_t *ctx, uint tx_threshold, uint rx_threshold);

/**
 * @brief Get UART FIFO thresholds
 * 
 * @param ctx Modbus context
 * @param tx_threshold Pointer to store TX threshold
 * @param rx_threshold Pointer to store RX threshold
 * @return int 0 on success, -1 on error
 */
MODBUS_API int modbus_rtu_get_fifo_thresholds(modbus_t *ctx, uint *tx_threshold, uint *rx_threshold);

/* DMA Support (conditional compilation) */
#ifdef MODBUS_USE_DMA

/**
 * @brief Enable or disable DMA for UART transfers
 * 
 * @param ctx Modbus context
 * @param enable 1 to enable, 0 to disable
 * @return int 0 on success, -1 on error
 * 
 * @note Improves performance for high-speed or high-traffic applications
 */
MODBUS_API int modbus_rtu_enable_dma(modbus_t *ctx, int enable);

/**
 * @brief Check if DMA is enabled
 * 
 * @param ctx Modbus context
 * @return int 1 if enabled, 0 if disabled, -1 on error
 */
MODBUS_API int modbus_rtu_is_dma_enabled(modbus_t *ctx);

/**
 * @brief Set DMA channel priorities
 * 
 * @param ctx Modbus context
 * @param tx_priority TX DMA channel priority (0-3, higher = more priority)
 * @param rx_priority RX DMA channel priority (0-3)
 * @return int 0 on success, -1 on error
 */
MODBUS_API int modbus_rtu_set_dma_priority(modbus_t *ctx, uint tx_priority, uint rx_priority);

#endif /* MODBUS_USE_DMA */

/* Multi-core Support (conditional compilation) */
#ifdef MODBUS_MULTICORE_SUPPORT

/**
 * @brief Create RTU context pinned to specific core
 * 
 * @param device Device string ("uart0" or "uart1")
 * @param baud Baud rate
 * @param parity Parity ('N', 'E', 'O')
 * @param data_bit Data bits (7 or 8)
 * @param stop_bit Stop bits (1 or 2)
 * @param core_num Core number (0 or 1)
 * @return modbus_t* Modbus context or NULL on error
 * 
 * @note Context operations will run on specified core
 */
MODBUS_API modbus_t *modbus_new_rtu_on_core(const char *device,
                                            int baud,
                                            char parity,
                                            int data_bit,
                                            int stop_bit,
                                            uint core_num);

/**
 * @brief Get core affinity for context
 * 
 * @param ctx Modbus context
 * @return int Core number (0 or 1) or -1 if not set
 */
MODBUS_API int modbus_rtu_get_core(modbus_t *ctx);

/**
 * @brief Acquire lock for multi-core safe operations
 * 
 * @param ctx Modbus context
 * @param timeout_ms Timeout in milliseconds (0 = no wait, -1 = infinite)
 * @return int 0 on success, -1 on timeout
 * 
 * @note Must be paired with modbus_rtu_lock_release()
 */
MODBUS_API int modbus_rtu_lock_acquire(modbus_t *ctx, uint32_t timeout_ms);

/**
 * @brief Release lock acquired with modbus_rtu_lock_acquire()
 * 
 * @param ctx Modbus context
 */
MODBUS_API void modbus_rtu_lock_release(modbus_t *ctx);

#endif /* MODBUS_MULTICORE_SUPPORT */

/* Debug and Diagnostics */

/**
 * @brief Print debug information about GPIO pin configuration
 * 
 * @param ctx Modbus context
 * 
 * @note Outputs to USB serial (if enabled) or UART debug port
 */
MODBUS_API void modbus_rtu_debug_pins(modbus_t *ctx);

/**
 * @brief Print debug information about timing configuration
 * 
 * @param ctx Modbus context
 */
MODBUS_API void modbus_rtu_debug_timing(modbus_t *ctx);

/**
 * @brief Get communication statistics
 * 
 * @param ctx Modbus context
 * @param tx_count Pointer to store transmit count
 * @param rx_count Pointer to store receive count
 * @param error_count Pointer to store error count
 */
MODBUS_API void modbus_rtu_get_stats(modbus_t *ctx,
                                     uint32_t *tx_count,
                                     uint32_t *rx_count,
                                     uint32_t *error_count);

/**
 * @brief Reset communication statistics
 * 
 * @param ctx Modbus context
 */
MODBUS_API void modbus_rtu_reset_stats(modbus_t *ctx);

/**
 * @brief Print detailed error information
 * 
 * @param ctx Modbus context
 */
MODBUS_API void modbus_rtu_debug_errors(modbus_t *ctx);

/**
 * @brief Dump UART hardware registers for debugging
 * 
 * @param ctx Modbus context
 */
MODBUS_API void modbus_rtu_dump_uart_regs(modbus_t *ctx);

/* Low-level UART Access (advanced users) */

/**
 * @brief Get pointer to UART hardware instance
 * 
 * @param ctx Modbus context
 * @return uart_inst_t* UART instance pointer or NULL on error
 * 
 * @note Provides direct access to Pico SDK UART functions
 *       Use with caution - may interfere with modbus operations
 */
MODBUS_API uart_inst_t *modbus_rtu_get_uart_inst(modbus_t *ctx);

/**
 * @brief Get UART instance number (0 or 1)
 * 
 * @param ctx Modbus context
 * @return int UART number or -1 on error
 */
MODBUS_API int modbus_rtu_get_uart_num(modbus_t *ctx);

/**
 * @brief Check if UART is busy transmitting
 * 
 * @param ctx Modbus context
 * @return int 1 if busy, 0 if idle, -1 on error
 */
MODBUS_API int modbus_rtu_is_uart_busy(modbus_t *ctx);

/**
 * @brief Wait for UART TX to complete
 * 
 * @param ctx Modbus context
 * @param timeout_us Timeout in microseconds (0 = no wait, -1 = infinite)
 * @return int 0 on success, -1 on timeout
 */
MODBUS_API int modbus_rtu_wait_tx_complete(modbus_t *ctx, uint32_t timeout_us);

/* Utility Functions */

/**
 * @brief Calculate character transmission time in microseconds
 * 
 * @param baud Baud rate
 * @param data_bits Number of data bits (7 or 8)
 * @param stop_bits Number of stop bits (1 or 2)
 * @param parity Parity ('N', 'E', or 'O')
 * @return uint32_t Character time in microseconds
 * 
 * @note Used for calculating proper inter-frame delays
 */
MODBUS_API uint32_t modbus_rtu_calc_char_time_us(int baud, int data_bits, int stop_bits, char parity);

/**
 * @brief Calculate frame delay time (3.5 character times)
 * 
 * @param baud Baud rate
 * @param data_bits Number of data bits
 * @param stop_bits Number of stop bits
 * @param parity Parity
 * @return uint32_t Frame delay in microseconds
 */
MODBUS_API uint32_t modbus_rtu_calc_frame_delay_us(int baud, int data_bits, int stop_bits, char parity);

/**
 * @brief Validate UART pin assignment
 * 
 * @param uart_num UART number (0 or 1)
 * @param tx_pin TX pin number
 * @param rx_pin RX pin number
 * @return int 1 if valid, 0 if invalid
 */
MODBUS_API int modbus_rtu_validate_pins(int uart_num, uint tx_pin, uint rx_pin);

/* IRQ Handler Management (advanced users) */

/**
 * @brief Set custom RX interrupt handler
 * 
 * @param ctx Modbus context
 * @param handler Custom IRQ handler function
 * @return int 0 on success, -1 on error
 * 
 * @warning Advanced feature - improper use can break modbus functionality
 */
MODBUS_API int modbus_rtu_set_rx_irq_handler(modbus_t *ctx, void (*handler)(void));

/**
 * @brief Enable or disable RX interrupt
 * 
 * @param ctx Modbus context
 * @param enable 1 to enable, 0 to disable
 * @return int 0 on success, -1 on error
 */
MODBUS_API int modbus_rtu_enable_rx_irq(modbus_t *ctx, int enable);

/**
 * @brief Get current RX interrupt status
 * 
 * @param ctx Modbus context
 * @return int 1 if enabled, 0 if disabled, -1 on error
 */
MODBUS_API int modbus_rtu_get_rx_irq_status(modbus_t *ctx);

/**
 * @brief Force manual direction pin control (bypass automatic control)
 * 
 * @param ctx Modbus context
 * @param state 1 for TX mode (DE high), 0 for RX mode (DE low)
 * @return int 0 on success, -1 on error
 * 
 * @warning Only use for testing/debugging. Normal operations use automatic control.
 */
MODBUS_API int modbus_rtu_force_direction(modbus_t *ctx, int state);

/**
 * @brief Perform hardware loopback test
 * 
 * @param ctx Modbus context
 * @param test_data Pointer to test data to send
 * @param length Length of test data
 * @return int Number of bytes successfully looped back, -1 on error
 * 
 * @note Requires RX and TX pins to be physically connected
 */
MODBUS_API int modbus_rtu_loopback_test(modbus_t *ctx, const uint8_t *test_data, int length);

#endif /* PICO_SDK */

MODBUS_END_DECLS

#endif /* MODBUS_RTU_H */