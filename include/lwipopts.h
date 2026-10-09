#ifndef LWIPOPTS_H
#define LWIPOPTS_H

/*
 * Buddy 1 - lwIP configuration for Pico W.
 *
 * Used with:
 * pico_cyw43_arch_lwip_threadsafe_background
 */

/* This CYW43 architecture uses the lwIP RAW API. */
#define NO_SYS                          1

/* Disable sequential/netconn and socket APIs. */
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0

/* Memory configuration. */
#define MEM_LIBC_MALLOC                 0
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        4000
#define MEMP_NUM_TCP_SEG                32
#define MEMP_NUM_ARP_QUEUE              10
#define PBUF_POOL_SIZE                  24
#define MEMP_NUM_SYS_TIMEOUT            16

/* Network protocols. */
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_IPV4                       1
#define LWIP_DHCP                       1
#define LWIP_DNS                        1
#define LWIP_TCP                        1
#define LWIP_UDP                        1

/* TCP configuration. */
#define TCP_MSS                         1460
#define TCP_WND                         (8 * TCP_MSS)
#define TCP_SND_BUF                     (8 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * TCP_SND_BUF + (TCP_MSS - 1)) / TCP_MSS)
#define LWIP_TCP_KEEPALIVE              1

/* Network interface configuration. */
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_TX_SINGLE_PBUF       1

/* DHCP configuration. */
#define DHCP_DOES_ARP_CHECK             0
#define LWIP_DHCP_DOES_ACD_CHECK        0

/* Statistics disabled for release/normal use. */
#define MEM_STATS                       0
#define SYS_STATS                       0
#define MEMP_STATS                      0
#define LINK_STATS                      0
#define LWIP_STATS                      0

/* Checksum algorithm used by Pico examples. */
#define LWIP_CHKSUM_ALGORITHM           3

#endif /* LWIPOPTS_H */