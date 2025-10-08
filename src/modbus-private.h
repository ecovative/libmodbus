/*
 * Copyright © Stéphane Raimbault <stephane.raimbault@gmail.com>
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * RP2350 Port Modifications
 */

#ifndef MODBUS_PRIVATE_H
#define MODBUS_PRIVATE_H

#ifndef _MSC_VER
#include <stdint.h>
#else
#include "stdint.h"
#endif

#include <sys/types.h>
#include "modbus.h"

MODBUS_BEGIN_DECLS

/* Platform-specific includes and definitions */
#ifdef PICO_SDK
/* RP2350 Platform */
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

#ifdef MODBUS_USE_DMA
#include "hardware/dma.h"
#endif

#ifdef MODBUS_MULTICORE_SUPPORT
#include "pico/multicore.h"
#include "pico/sync.h"
#endif

/* RP2350 doesn't use POSIX sockets for RTU */
#define MODBUS_NO_POSIX_SOCKETS
#define MODBUS_NO_SELECT

/* File descriptor substitute for RP2350 */
typedef int modbus_socket_t;
#define MODBUS_INVALID_SOCKET (-1)

/* Custom fd_set for RP2350 (minimal implementation) */
typedef struct {
    uint32_t ready_flags;
} fd_set;

#define FD_ZERO(set) ((set)->ready_flags = 0)
#define FD_SET(fd, set) ((set)->ready_flags |= (1U << (fd)))
#define FD_CLR(fd, set) ((set)->ready_flags &= ~(1U << (fd)))
#define FD_ISSET(fd, set) (((set)->ready_flags & (1U << (fd))) != 0)

/* Time structure */
#ifndef HAVE_SYS_TIME_H
struct timeval {
    long tv_sec;
    long tv_usec;
};
#endif

#else
/* Standard POSIX/Windows Platform */
#include <sys/time.h>
#include <sys/select.h>

#if defined(_WIN32)
#include <winsock2.h>
typedef SOCKET modbus_socket_t;
#define MODBUS_INVALID_SOCKET INVALID_SOCKET
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
typedef int modbus_socket_t;
#define MODBUS_INVALID_SOCKET (-1)
#endif

#endif /* PICO_SDK */

/* Common definitions */
#define MSG_LENGTH_UNDEFINED -1

/* Modbus function codes */
#define MODBUS_FC_READ_COILS 0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS 0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FC_READ_INPUT_REGISTERS 0x04
#define MODBUS_FC_WRITE_SINGLE_COIL 0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER 0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS 0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS 0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS 0x10
#define MODBUS_FC_REPORT_SLAVE_ID 0x11
#define MODBUS_FC_MASK_WRITE_REGISTER 0x16
#define MODBUS_FC_WRITE_AND_READ_REGISTERS 0x17

#define MODBUS_BROADCAST_ADDRESS 0

/* Protocol exceptions */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION 0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE 0x03
#define MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE 0x04
#define MODBUS_EXCEPTION_ACKNOWLEDGE 0x05
#define MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY 0x06
#define MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE 0x07
#define MODBUS_EXCEPTION_MEMORY_PARITY 0x08
#define MODBUS_EXCEPTION_NOT_DEFINED 0x09
#define MODBUS_EXCEPTION_GATEWAY_PATH 0x0A
#define MODBUS_EXCEPTION_GATEWAY_TARGET 0x0B

#define EMBXILFUN -MODBUS_EXCEPTION_ILLEGAL_FUNCTION
#define EMBXILADD -MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS
#define EMBXILVAL -MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE
#define EMBXSFAIL -MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE
#define EMBXACK -MODBUS_EXCEPTION_ACKNOWLEDGE
#define EMBXSBUSY -MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY
#define EMBXNACK -MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE
#define EMBXMEMPAR -MODBUS_EXCEPTION_MEMORY_PARITY
#define EMBXGPATH -MODBUS_EXCEPTION_GATEWAY_PATH
#define EMBXGTAR -MODBUS_EXCEPTION_GATEWAY_TARGET

/* Native libmodbus error codes */
#define EMBBADCRC -MODBUS_ENOBASE - 1
#define EMBBADDATA -MODBUS_ENOBASE - 2
#define EMBBADEXC -MODBUS_ENOBASE - 3
#define EMBUNKEXC -MODBUS_ENOBASE - 4
#define EMBMDATA -MODBUS_ENOBASE - 5
#define EMBBADSLAVE -MODBUS_ENOBASE - 6

extern const unsigned int libmodbus_version_major;
extern const unsigned int libmodbus_version_minor;
extern const unsigned int libmodbus_version_micro;

#define MODBUS_ENOBASE 112345678

/* Internal use */
#define MSG_CONFIRMATION 1
#define MSG_INDICATION 2

/* Exported version */
#define LIBMODBUS_VERSION_MAJOR 3
#define LIBMODBUS_VERSION_MINOR 1
#define LIBMODBUS_VERSION_MICRO 10

/* Max between RTU and TCP max adu length (so TCP) */
#define MAX_MESSAGE_LENGTH 260

/* 3 steps are used to parse the query */
typedef enum { _STEP_FUNCTION, _STEP_META, _STEP_DATA } _step_t;

typedef enum { _MSG_INDICATION, _MSG_CONFIRMATION } _msg_type_t;

void _modbus_init_common(modbus_t *ctx);
void _error_print(modbus_t *ctx, const char *context);
int _modbus_receive_msg(modbus_t *ctx, uint8_t *msg, msg_type_t msg_type);

#ifndef HAVE_STRLCPY
size_t strlcpy(char *dest, const char *src, size_t dest_size);
#endif

/* Backend structure for modbus operations */
typedef struct _modbus_backend {
    unsigned int backend_type;
    unsigned int header_length;
    unsigned int checksum_length;
    unsigned int max_adu_length;
    int (*set_slave)(modbus_t *ctx, int slave);
    int (*build_request_basis)(modbus_t *ctx, int function, int addr, int nb, uint8_t *req);
    int (*build_response_basis)(sft_t *sft, uint8_t *rsp);
    int (*prepare_response_tid)(const uint8_t *req, int *req_length);
    int (*send_msg_pre)(uint8_t *req, int req_length);
    ssize_t (*send)(modbus_t *ctx, const uint8_t *req, int req_length);
    int (*receive)(modbus_t *ctx, uint8_t *req);
    ssize_t (*recv)(modbus_t *ctx, uint8_t *rsp, int rsp_length);
    int (*check_integrity)(modbus_t *ctx, uint8_t *msg, const int msg_length);
    int (*pre_check_confirmation)(modbus_t *ctx, const uint8_t *req, const uint8_t *rsp, int rsp_length);
    int (*connect)(modbus_t *ctx);
    void (*close)(modbus_t *ctx);
    int (*flush)(modbus_t *ctx);
    int (*select)(modbus_t *ctx, fd_set *rset, struct timeval *tv, int length_to_read);
    void (*free)(modbus_t *ctx);
} modbus_backend_t;

/* Main modbus context structure */
struct _modbus {
    /* Slave address */
    int slave;
    /* Socket or file descriptor */
    modbus_socket_t s;
    int debug;
    int error_recovery;
    struct timeval response_timeout;
    struct timeval byte_timeout;
    struct timeval indication_timeout;
    const modbus_backend_t *backend;
    void *backend_data;
};

void _sleep_response_timeout(modbus_t *ctx);

/* RP2350-specific helper functions */
#ifdef PICO_SDK
int _modbus_rtu_select(modbus_t *ctx, fd_set *rset, struct timeval *tv, int length_to_read);
int modbus_get_current_time(struct timeval *tv);
void modbus_sleep_us(uint32_t us);
uint64_t modbus_time_diff_us(struct timeval *start, struct timeval *end);
#endif

MODBUS_END_DECLS

#endif /* MODBUS_PRIVATE_H */