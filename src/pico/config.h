/* src/pico/config.h - RP2350 Platform Configuration */
#ifndef MODBUS_PICO_CONFIG_H
#define MODBUS_PICO_CONFIG_H

/* Include this file instead of the autotools-generated config.h */

/* Standard C headers available */
#define HAVE_STDINT_H 1
#define HAVE_INTTYPES_H 1
#define HAVE_STRING_H 1
#define HAVE_STDLIB_H 1
#define HAVE_ERRNO_H 1
#define HAVE_LIMITS_H 1
#define HAVE_TIME_H 1
#define HAVE_MALLOC 1
#define HAVE_MEMSET 1
#define STDC_HEADERS 1

/* POSIX headers NOT available on bare metal */
#undef HAVE_UNISTD_H
#undef HAVE_SYS_SOCKET_H
#undef HAVE_NETINET_IN_H
#undef HAVE_ARPA_INET_H
#undef HAVE_SYS_IOCTL_H
#undef HAVE_TERMIOS_H
#undef HAVE_FCNTL_H
#undef HAVE_SYS_TIME_H
#undef HAVE_SYS_SELECT_H
#undef HAVE_DECL_TIOCSRS485
#undef HAVE_DECL_TIOCM_RTS

/* Endianness - RP2350 is little-endian */
#undef WORDS_BIGENDIAN

/* Platform marker */
#define PICO_SDK 1

/* Enable all source */
#ifndef _ALL_SOURCE
# define _ALL_SOURCE 1
#endif

#endif /* MODBUS_PICO_CONFIG_H */