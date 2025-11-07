#ifndef MAIN_H
#define MAIN_H

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/delay.h>  // For mdelay()
#include <linux/interrupt.h>
#include "nic.h"

#define DRV_NAME      "mustafa"
#define NETDEV_NAME   "mustafa%d"
#define VENDOR_ID     0x8086
#define DEVICE_ID     0x100e   // Intel 82540EM






// net dev 
static int  m_open(struct net_device *dev);
static int  m_stop(struct net_device *dev);
static netdev_tx_t m_tx(struct sk_buff *skb, struct net_device *dev);

// pcie
static int  m_probe(struct pci_dev *pci_dev, const struct pci_device_id *ent);
static void m_remove(struct pci_dev *pci_dev);

// additional
static void m_tx_setup(struct m_adapter *adapter);
static void m_rx_setup(struct m_adapter *adapter);
static void m_setup(struct net_device *dev);
static void m_clean_tx_ring(struct m_adapter *adapter);
static void m_clean_rx_ring(struct m_adapter *adapter);
static irqreturn_t m_interrupt(int irq, void *dev_id);
static void m_setup_mac(struct m_adapter *adapter);

// linux
static int  __init m_init(void);
static void __exit m_exit(void);



// to know your vendor/device id run:                   lspci -nnk | grep -iA3 ethernet
static const struct pci_device_id id_table[] = {
    { PCI_DEVICE(VENDOR_ID, DEVICE_ID) }, // Intel 82540EM
    { 0, }  // required sentinel
};



static struct pci_driver m_driver = {
	.name     = "mustafa",
	.id_table = id_table,
	.probe    = m_probe,
	.remove   = m_remove,
};


static const struct net_device_ops netdev_ops = {
	.ndo_open		= m_open,
	.ndo_stop		= m_stop,
	.ndo_start_xmit		= m_tx,
};











#endif // MAIN_H
