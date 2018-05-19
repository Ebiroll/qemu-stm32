
#if defined(TARGET_MIPS) || defined(TARGET_HPPA) || defined(TARGET_ALPHA)
#include "sockbits.h"
#else

#if defined(TARGET_SPARC)
    /** sock_type - Socket types
     *
     * Please notice that for binary compat reasons SPARC has to
     * override the enum sock_type in include/linux/net.h, so
     * we define ARCH_HAS_SOCKET_TYPES here.
     *
     * @SOCK_DGRAM - datagram (conn.less) socket
     * @SOCK_STREAM - stream (connection) socket
     * @SOCK_RAW - raw socket
     * @SOCK_RDM - reliably-delivered message
     * @SOCK_SEQPACKET - sequential packet socket
     * @SOCK_DCCP - Datagram Congestion Control Protocol socket
     * @SOCK_PACKET - linux specific way of getting packets at the dev level.
     *                For writing rarp and other similar things on the user
     *                level.
     * @SOCK_CLOEXEC - sets the close-on-exec (FD_CLOEXEC) flag.
     * @SOCK_NONBLOCK - sets the O_NONBLOCK file status flag.
     */

    #define ARCH_HAS_SOCKET_TYPES          1

    enum sock_type {
           TARGET_SOCK_STREAM      = 1,
           TARGET_SOCK_DGRAM       = 2,
           TARGET_SOCK_RAW         = 3,
           TARGET_SOCK_RDM         = 4,
           TARGET_SOCK_SEQPACKET   = 5,
           TARGET_SOCK_DCCP        = 6,
           TARGET_SOCK_PACKET      = 10,
           TARGET_SOCK_CLOEXEC     = 020000000,
           TARGET_SOCK_NONBLOCK    = 040000,
    };

    #define TARGET_SOCK_MAX (TARGET_SOCK_PACKET + 1)
    #define TARGET_SOCK_TYPE_MASK    0xf  /* Covers up to TARGET_SOCK_MAX-1. */

    #define TARGET_SO_PASSSEC        31
#else
    #define TARGET_SO_PASSSEC        34
#endif

    /* For setsockopt(2) */
    #define TARGET_SOL_SOCKET      1

    #define TARGET_SO_DEBUG        1
    #define TARGET_SO_REUSEADDR    2
    #define TARGET_SO_TYPE         3
    #define TARGET_SO_ERROR        4
    #define TARGET_SO_DONTROUTE    5
    #define TARGET_SO_BROADCAST    6
    #define TARGET_SO_SNDBUF       7
    #define TARGET_SO_RCVBUF       8
    #define TARGET_SO_SNDBUFFORCE  32
    #define TARGET_SO_RCVBUFFORCE  33
    #define TARGET_SO_KEEPALIVE    9
    #define TARGET_SO_OOBINLINE    10
    #define TARGET_SO_NO_CHECK     11
    #define TARGET_SO_PRIORITY     12
    #define TARGET_SO_LINGER       13
    #define TARGET_SO_BSDCOMPAT    14
    /* To add :#define TARGET_SO_REUSEPORT 15 */
#if defined(TARGET_PPC)
    #define TARGET_SO_RCVLOWAT     16
    #define TARGET_SO_SNDLOWAT     17
    #define TARGET_SO_RCVTIMEO     18
    #define TARGET_SO_SNDTIMEO     19
    #define TARGET_SO_PASSCRED     20
    #define TARGET_SO_PEERCRED     21
#else
    #define TARGET_SO_PASSCRED     16
    #define TARGET_SO_PEERCRED     17
    #define TARGET_SO_RCVLOWAT     18
    #define TARGET_SO_SNDLOWAT     19
    #define TARGET_SO_RCVTIMEO     20
    #define TARGET_SO_SNDTIMEO     21
#endif

    /* Security levels - as per NRL IPv6 - don't actually do anything */
    #define TARGET_SO_SECURITY_AUTHENTICATION              22
    #define TARGET_SO_SECURITY_ENCRYPTION_TRANSPORT        23
    #define TARGET_SO_SECURITY_ENCRYPTION_NETWORK          24

    #define TARGET_SO_BINDTODEVICE 25

    /* Socket filtering */
    #define TARGET_SO_ATTACH_FILTER        26
    #define TARGET_SO_DETACH_FILTER        27

    #define TARGET_SO_PEERNAME             28
    #define TARGET_SO_TIMESTAMP            29
    #define TARGET_SCM_TIMESTAMP           TARGET_SO_TIMESTAMP

    #define TARGET_SO_ACCEPTCONN           30

    #define TARGET_SO_PEERSEC              31

#endif

#ifndef ARCH_HAS_SOCKET_TYPES
    /** sock_type - Socket types - default values
     *
     *
     * @SOCK_STREAM - stream (connection) socket
     * @SOCK_DGRAM - datagram (conn.less) socket
     * @SOCK_RAW - raw socket
     * @SOCK_RDM - reliably-delivered message
     * @SOCK_SEQPACKET - sequential packet socket
     * @SOCK_DCCP - Datagram Congestion Control Protocol socket
     * @SOCK_PACKET - linux specific way of getting packets at the dev level.
     *                For writing rarp and other similar things on the user
     *                level.
     * @SOCK_CLOEXEC - sets the close-on-exec (FD_CLOEXEC) flag.
     * @SOCK_NONBLOCK - sets the O_NONBLOCK file status flag.
     */
    enum sock_type {
           TARGET_SOCK_STREAM      = 1,
           TARGET_SOCK_DGRAM       = 2,
           TARGET_SOCK_RAW         = 3,
           TARGET_SOCK_RDM         = 4,
           TARGET_SOCK_SEQPACKET   = 5,
           TARGET_SOCK_DCCP        = 6,
           TARGET_SOCK_PACKET      = 10,
           TARGET_SOCK_CLOEXEC     = 02000000,
           TARGET_SOCK_NONBLOCK    = 04000,
    };

    #define TARGET_SOCK_MAX (TARGET_SOCK_PACKET + 1)
    #define TARGET_SOCK_TYPE_MASK    0xf  /* Covers up to TARGET_SOCK_MAX-1. */

#endif
