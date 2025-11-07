#ifndef NIC_H
#define NIC_H

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/etherdevice.h>
#include <linux/types.h>

/* TX Registers */
#define E1000_REG_TDBAL    0x3800
#define E1000_REG_TDBAH    0x3804
#define E1000_REG_TDLEN    0x3808
#define E1000_REG_TDH      0x3810
#define E1000_REG_TDT      0x3818
#define E1000_REG_TCTL     0x0400
#define E1000_REG_TIPG     0x0410

/* RX Registers */
#define E1000_REG_RDBAL    0x2800
#define E1000_REG_RDBAH    0x2804
#define E1000_REG_RDLEN    0x2808
#define E1000_REG_RDH      0x2810
#define E1000_REG_RDT      0x2818
#define E1000_REG_RCTL     0x0100

/* Interrupt Registers */
#define E1000_REG_ICR      0x00C0
#define E1000_REG_IMS      0x00D0
#define E1000_REG_IMC      0x00D8

/* Interrupt Bits */
#define E1000_ICR_TXDW     (1 << 0)
#define E1000_ICR_TXQE     (1 << 1)
#define E1000_ICR_LSC      (1 << 2)
#define E1000_ICR_RXSEQ    (1 << 3)
#define E1000_ICR_RXDMT0   (1 << 4)
#define E1000_ICR_RXO      (1 << 6)
#define E1000_ICR_RXT0     (1 << 7)

/* RCTL Bits */
#define E1000_RCTL_EN      (1 << 1)
#define E1000_RCTL_SBP     (1 << 2)
#define E1000_RCTL_UPE     (1 << 3)
#define E1000_RCTL_MPE     (1 << 4)
#define E1000_RCTL_LPE     (1 << 5)
#define E1000_RCTL_BAM     (1 << 15)
#define E1000_RCTL_SECRC   (1 << 26)

/* Descriptor Status */
#define E1000_RXD_STAT_DD  (1 << 0)
#define E1000_RXD_STAT_EOP (1 << 1)
#define E1000_TXD_STAT_DD  (1 << 0)

/* MAC Address */
#define E1000_REG_RAL      0x5400
#define E1000_REG_RAH      0x5404

/* Status */
#define E1000_REG_STATUS   0x0008
#define E1000_STATUS_LU    (1 << 1)

/* Descriptor Structures */
struct tx_desc {
    u64 buffer_addr;
    u16 length;
    u8 cso;
    u8 cmd;
    u8 status;
    u8 css;
    u16 special;
};

struct rx_desc {
    u64 buffer_addr;
    u16 length;
    u16 checksum;
    u8 status;
    u8 errors;
    u16 special;
};

#define NUM_TX_DESC 16
#define NUM_RX_DESC 16
#define RX_BUF_SIZE 1536

struct m_adapter {
    struct pci_dev *pci_dev;
    struct net_device *netdev;
    void __iomem *hw_addr;

    struct tx_desc *tx_ring;
    dma_addr_t tx_ring_dma;
    struct sk_buff *tx_skbs[NUM_TX_DESC];
    u16 tx_next;
    u16 tx_dirty;

    struct rx_desc *rx_ring;
    dma_addr_t rx_ring_dma;
    struct sk_buff *rx_skbs[NUM_RX_DESC];
    u16 rx_next;

    int irq;
};

#endif /* NIC_H */