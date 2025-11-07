#include "main.h"
#include "nic.h"

/*
To test:
make add_min
make ip
*/

// ||||||||||||||||||||||||||||||||||||||||||||||||| _____ NET DEV _____ |||||||||||||||||||||||||||||||||||||||||||||||||


// Open event function __________________________________________________________________________________________________________________________
static int m_open(struct net_device *dev) // runs on ip link set mustafa up
{
    printk("D");

    struct m_adapter *adapter = netdev_priv(dev);

    // Check and set link status
    netif_carrier_on(dev);  // Temporary - you should read actual link status

    // Setup MAC address first
    m_setup_mac(adapter);

    // tx setup
    m_tx_setup(adapter);
    m_rx_setup(adapter);

    // Enable interrupts - tell hardware which interrupts we want
    iowrite32(E1000_ICR_TXDW | E1000_ICR_RXT0 | E1000_ICR_LSC, adapter->hw_addr + E1000_REG_IMS);

    netif_start_queue(dev); // to start queing tx packets in kernal 

    pr_info(DRV_NAME ": device opened\n");
    return 0;
}




// Stop event function __________________________________________________________________________________________________________________________
static int m_stop(struct net_device *dev) // runs on ip link set mustafa down
{

    struct m_adapter *adapter = netdev_priv(dev);
    
    
    iowrite32(0xFFFFFFFF, adapter->hw_addr + E1000_REG_IMC);// Disable interrupts
    netif_stop_queue(dev); // to stop queing tx packets in kernal 
    pr_info(DRV_NAME ": device stopped\n");
    return 0;
}




// TX event fnction __________________________________________________________________________________________________________________________
static netdev_tx_t m_tx(struct sk_buff *skb, struct net_device *dev)
{
    struct m_adapter *adapter = netdev_priv(dev);
    struct tx_desc *txd;
    dma_addr_t dma;
    u16 index;

    pr_info("=== TX DEBUG 1: Entered m_tx, skb->len=%d ===\n", skb->len);

    // Get next available slot
    index = adapter->tx_next;               pr_info("=== TX DEBUG 2: index=%d ===\n", index);
    txd = &adapter->tx_ring[index];         pr_info("=== TX DEBUG 3: got txd ===\n");

    // CRITICAL FIX: Check if skb is valid for DMA
    if (skb->len == 0) {pr_err("=== TX ERROR: skb has zero length ===\n");dev_kfree_skb(skb);return NETDEV_TX_OK;}

    // Map packet for DMA - THIS COULD BE CRASHING
    dma = dma_map_single(&adapter->pci_dev->dev, skb->data, skb->len, DMA_TO_DEVICE);
    if (dma_mapping_error(&adapter->pci_dev->dev, dma)) {pr_err("=== TX ERROR: DMA mapping failed ===\n");dev_kfree_skb(skb);return NETDEV_TX_OK;}
    
    // Fill descriptor
    txd->buffer_addr = dma;
    txd->length = (u16)skb->len;
    txd->cmd = 0x01 | 0x02 | 0x08;  // EOP + IFCS + RS
    txd->status = 0;

    adapter->tx_skbs[index] = skb;                                          pr_info("=== TX DEBUG 6: skb stored ===\n");// Store skb
    adapter->tx_next = (index + 1) % 16;                                    pr_info("=== TX DEBUG 7: tx_next=%d ===\n", adapter->tx_next);// Move to next slot
    iowrite32(adapter->tx_next, adapter->hw_addr + E1000_REG_TDT);          pr_info("=== TX DEBUG 8: TDT set to %d ===\n", adapter->tx_next);// Tell hardware about new packet

    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;
    
    pr_info("=== TX DEBUG 9: SUCCESS - returning ===\n");
    return NETDEV_TX_OK;
}












// ||||||||||||||||||||||||||||||||||||||||||||||||| _____ PCIE _____ |||||||||||||||||||||||||||||||||||||||||||||||||



// PROBE __________________________________________________________________________________________________________________________
static int m_probe(struct pci_dev *pci_dev, const struct pci_device_id *ent)
{
    
    int status;
    void __iomem *ptr_bar0;
    struct net_device *netdev;
    struct m_adapter *adapter;

    // 1. PCI init________________________________________________________
    status = pci_resource_len(pci_dev, 0);                              printk("Device size is %d (Bar0)", status);

    status = pcim_enable_device(pci_dev);                               if(status<0) {printk("enable err "); return status;}
    pci_set_master(pci_dev);
    status = pcim_iomap_regions(pci_dev, BIT(0), KBUILD_MODNAME);       if(status<0) {printk("io_m_r err "); return status;}
    ptr_bar0 = pcim_iomap_table(pci_dev)[0];                            if(ptr_bar0==NULL) {printk("null err "); return -1;}



    // 2 Net_dev init: Allocate & register net_device______________________
    netdev = alloc_netdev(sizeof(struct m_adapter),     
                        "mustafa%d", NET_NAME_UNKNOWN, m_setup);        if (!netdev) {printk("alloc_netdev error\n");return -ENOMEM;}
    status = register_netdev(netdev);                                   if (status) {printk("register_netdev error\n");free_netdev(netdev);}
    pci_set_drvdata(pci_dev, netdev);       



    // 3. Adapter init_____________________________________________________
    adapter = netdev_priv(netdev);
    adapter->pci_dev = pci_dev;
    adapter->netdev = netdev;
    adapter->hw_addr = ptr_bar0;



    // 3.2 tx ring init____________________________________________________
    adapter->tx_next = 0;
    adapter->tx_dirty = 0;
    adapter->tx_ring = dma_alloc_coherent(&pci_dev->dev,        // Allocate TX descriptor ring (16 descriptors)
                        sizeof(struct tx_desc) * 16,
                        &adapter->tx_ring_dma,GFP_KERNEL);              if (!adapter->tx_ring) { printk("ERROR: Failed to allocate TX ring\n");free_netdev(netdev);return -ENOMEM;}
    memset(adapter->tx_ring, 0, sizeof(struct tx_desc) * 16);   // Zero out the TX ring



    // 3,3 rx ring init____________________________________________________
    adapter->rx_next = 0;
    adapter->rx_ring = dma_alloc_coherent(&pci_dev->dev,        // Allocate RX descriptor ring (16 descriptors)
                        sizeof(struct rx_desc) * 16,
                        &adapter->rx_ring_dma,GFP_KERNEL);              if (!adapter->rx_ring) {printk("ERROR: Failed RX ring\n");dma_free_coherent(&pci_dev->dev, sizeof(struct tx_desc) * 16, adapter->tx_ring, adapter->tx_ring_dma);free_netdev(netdev);    return -ENOMEM;}
    memset(adapter->rx_ring, 0, sizeof(struct rx_desc) * 16);
    for (int i = 0; i < 16; i++) {adapter->rx_skbs[i] = NULL;}  // Initialize RX buffers to NULL
    


    // 3.4 irq setup_______________________________________________________
    status = pci_alloc_irq_vectors(pci_dev, 1, 1, PCI_IRQ_LEGACY | PCI_IRQ_MSI);            if (status < 0) {goto irq_fail_1;}
    adapter->irq = pci_irq_vector(pci_dev, 0);
        // status = devm_request_irq(&pci_dev->dev, pci_dev->irq, m_interrupt, 0, KBUILD_MODNAME, adapter); 
    status = request_irq(adapter->irq, m_interrupt, IRQF_SHARED, netdev->name, netdev);     if (status) {goto irq_fail_2;}


    printk("mustafa driver probed successfully\n");
    return 0;


    irq_fail_1:
        printk("Failed to allocate IRQ vectors\n");
    irq_fail_2:
        printk("Failed to request IRQ %d\n", adapter->irq);
        pci_free_irq_vectors(pci_dev);

    unregister_netdev(netdev);
    dma_free_coherent(&pci_dev->dev, sizeof(struct tx_desc) * 16,
                        adapter->tx_ring, adapter->tx_ring_dma);
    dma_free_coherent(&pci_dev->dev, sizeof(struct rx_desc) * 16,
                        adapter->rx_ring, adapter->rx_ring_dma);
    free_netdev(netdev);
    return status;

}




// PROBE/Setup FUNCs 
static void m_setup(struct net_device *dev) // runs when alloc_netdevice() is ran
{
    ether_setup(dev);          // sets up Ethernet-like device
    dev->netdev_ops = &netdev_ops;
    eth_hw_addr_random(dev);   // assign random MAC

    // >>> DISABLE OFFLOAD FEATURES <<<
    dev->features &= ~(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM | NETIF_F_TSO | NETIF_F_GSO);
    dev->hw_features = 0;  // no hardware offload capabilities
    dev->hw_features &= ~(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM | NETIF_F_TSO | NETIF_F_GSO);
}



// REMOVE FUNC __________________________________________________________________________________________________________________________
static void m_remove(struct pci_dev *pci_dev) {
    struct net_device *netdev = pci_get_drvdata(pci_dev);
    struct m_adapter *adapter = netdev_priv(netdev);
    
    if (netdev) {

        // Free IRQ
        free_irq(adapter->irq, netdev);
        pci_free_irq_vectors(pci_dev);

        // Bring down the interface
        if (netif_running(netdev)) {
            netif_stop_queue(netdev);
            netif_carrier_off(netdev);
        }
        // ============ ADD THIS: FREE TX/RX RING MEMORY ============
        if (adapter->tx_ring) {
            dma_free_coherent(&pci_dev->dev,
                            sizeof(struct tx_desc) * 16,
                            adapter->tx_ring, 
                            adapter->tx_ring_dma);
            printk("TX ring freed\n");
        }

            // Free RX ring
        if (adapter->rx_ring) {
            int i;
            for (i = 0; i < 16; i++) {
                if (adapter->rx_skbs[i]) {
                    dma_unmap_single(&pci_dev->dev,
                                   adapter->rx_ring[i].buffer_addr,
                                   1536, DMA_FROM_DEVICE);
                    dev_kfree_skb(adapter->rx_skbs[i]);
                }
            }
            dma_free_coherent(&pci_dev->dev,
                            sizeof(struct rx_desc) * 16,
                            adapter->rx_ring, adapter->rx_ring_dma);
        }
        // ============ END OF ADDITION ============
        unregister_netdev(netdev);
        free_netdev(netdev);
    }
    printk("mustafa driver removed\n");
}





// |||||||||||||||||||||||||||||||||||||||||||||||| ____ Helper ______ ||||||||||||||||||||||||||||||||||||||||||||||||||||



static void m_tx_setup(struct m_adapter *adapter)
{
    printk("C");
    void __iomem *hw = adapter->hw_addr;                                    pr_info("=== SETUP DEBUG 1: Starting TX setup ===\n");
    
    // 1. Tell hardware where our TX ring is
    iowrite32((u32)adapter->tx_ring_dma, hw + E1000_REG_TDBAL);             pr_info("=== SETUP DEBUG 2: Wrote TDBAL = 0x%x ===\n", (u32)adapter->tx_ring_dma);
    // iowrite32(0, hw + E1000_REG_TDBAH);                                
    iowrite32((u32)(adapter->tx_ring_dma >> 32), hw + E1000_REG_TDBAH);     pr_info("=== SETUP DEBUG 3: Wrote TDBAH = 0 ===\n");

    // 2. Tell hardware ring size
    iowrite32(16 * 16, hw + E1000_REG_TDLEN);                               pr_info("=== SETUP DEBUG 4: Wrote TDLEN = %d ===\n", 16 * 16);
    
    // 3. Reset head and tail pointers
    iowrite32(0, hw + E1000_REG_TDH);                                       pr_info("=== SETUP DEBUG 5: Wrote TDH = 0 ===\n");
    iowrite32(0, hw + E1000_REG_TDT);                                       pr_info("=== SETUP DEBUG 6: Wrote TDT = 0 ===\n");
  
    // 4. Turn on TX with proper configuration  //pg 325
    iowrite32(0x010400FA, hw + E1000_REG_TCTL);                             pr_info("=== SETUP DEBUG 7: Wrote TCTL = 0x010400FA ===\n");
   
    // 5. Set Inter Packet Gap  // TIPG register //pg 327
    iowrite32(0x0060200A, hw + E1000_REG_TIPG);                             pr_info("=== SETUP DEBUG 8: TX setup complete ===\n");
}



static void m_rx_setup(struct m_adapter *adapter)
{
    void __iomem *hw = adapter->hw_addr;
    u32 rctl;
    int i;
    
    pr_info("=== SETTING UP RX RING ===\n");

    // Program RX ring address and size
    writel((u32)adapter->rx_ring_dma, hw + E1000_REG_RDBAL);
    writel(0, hw + E1000_REG_RDBAH);
    writel(16 * 16, hw + E1000_REG_RDLEN);
    
    // Initialize head and tail pointers
    writel(0, hw + E1000_REG_RDH);
    writel(15, hw + E1000_REG_RDT);  // Start with tail at last descriptor
    
    // Allocate and map RX buffers
    for (i = 0; i < 16; i++) {
        struct sk_buff *skb;
        dma_addr_t dma;
        
        skb = netdev_alloc_skb(adapter->netdev, 1536);
        if (!skb) {
            pr_err("Failed to allocate RX skb %d\n", i);
            continue;
        }
        
        dma = dma_map_single(&adapter->pci_dev->dev, skb->data, 
                           1536, DMA_FROM_DEVICE);
        if (dma_mapping_error(&adapter->pci_dev->dev, dma)) {
            dev_kfree_skb(skb);
            pr_err("Failed to map RX skb %d\n", i);
            continue;
        }
        
        adapter->rx_skbs[i] = skb;
        adapter->rx_ring[i].buffer_addr = dma;
        adapter->rx_ring[i].length = 1536;      // ← ADD THIS
        adapter->rx_ring[i].status = 0;
        adapter->rx_ring[i].errors = 0;
        adapter->rx_ring[i].special = 0;
        
        pr_info("RX buffer %d allocated at DMA 0x%llx\n", i, dma);
    }
    
    // Enable RX with simpler configuration
    rctl = E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_SECRC;
    // Remove E1000_RCTL_LPE unless you need jumbo frames
    writel(rctl, hw + E1000_REG_RCTL);
    
    pr_info("RX enabled - RCTL: 0x%08x\n", rctl);
}



static irqreturn_t m_interrupt(int irq, void *dev_id)
{
    struct net_device *netdev = dev_id;
    struct m_adapter *adapter = netdev_priv(netdev);
    void __iomem *hw = adapter->hw_addr;
    u32 icr;

    icr = ioread32(hw + E1000_REG_ICR);
    
    if (!icr) {
        return IRQ_NONE;
    }

    pr_info("INTERRUPT: ICR=0x%08x\n", icr);

    // Handle TX completion
    if (icr & E1000_ICR_TXDW) {
        m_clean_tx_ring(adapter);
    }

    // Handle RX packet
    if (icr & (E1000_ICR_RXT0 | E1000_ICR_RXO)) {
        m_clean_rx_ring(adapter);
    }

    // Handle link status change
    if (icr & E1000_ICR_LSC) {
        pr_info("Link status changed\n");
    }
    printk("LESSS GOOOOOOOOOO");
    return IRQ_HANDLED;
}



static void m_clean_tx_ring(struct m_adapter *adapter)
{
    struct net_device *dev = adapter->netdev;
    u16 tx_index;
    struct tx_desc *txd;

    while (adapter->tx_dirty != adapter->tx_next) {
        tx_index = adapter->tx_dirty & 0xF;  // Wrap around 16
        txd = &adapter->tx_ring[tx_index];

        // Check if descriptor has been processed by hardware
        if (!(txd->status & 0xFF)) {  // No status bits set = not done
            break;
        }

        pr_info("TX Complete: desc %d, status=0x%02x\n", tx_index, txd->status);

        // Free the skb and unmap DMA
        if (adapter->tx_skbs[tx_index]) {
            dma_unmap_single(&adapter->pci_dev->dev,
                           txd->buffer_addr,
                           txd->length,
                           DMA_TO_DEVICE);
            dev_kfree_skb_any(adapter->tx_skbs[tx_index]);
            adapter->tx_skbs[tx_index] = NULL;
        }

        // Clear the descriptor
        memset(txd, 0, sizeof(struct tx_desc));
        
        adapter->tx_dirty++;
    }

    // Wake queue if we were stopped and now have space
    if (netif_queue_stopped(dev) && 
        (adapter->tx_next - adapter->tx_dirty < 8)) {
        netif_wake_queue(dev);
        pr_info("Waking TX queue\n");
    }
}



static void m_clean_rx_ring(struct m_adapter *adapter)
{
    struct net_device *dev = adapter->netdev;
    u16 rx_index;
    struct rx_desc *rxd;
    int processed = 0;

    rx_index = adapter->rx_next;
    
    while (processed < 16) {  // Process at most 16 packets per interrupt
        rxd = &adapter->rx_ring[rx_index];

        // Check if descriptor has been filled by hardware
        if (!(rxd->status & E1000_RXD_STAT_DD)) {
            break;
        }

        pr_info("RX Packet: desc %d, len=%d, status=0x%02x\n", 
               rx_index, rxd->length, rxd->status);

        if (adapter->rx_skbs[rx_index] && rxd->length > 0) {
            struct sk_buff *skb = adapter->rx_skbs[rx_index];
            u16 length = rxd->length;

            // Unmap DMA
            dma_unmap_single(&adapter->pci_dev->dev,
                           rxd->buffer_addr,
                           1536, DMA_FROM_DEVICE);

            // Set skb properties
            skb_put(skb, length);
            skb->protocol = eth_type_trans(skb, dev);


            // DEBUG: Print packet type
            if (skb->protocol == htons(ETH_P_IP)) {
                pr_info("RX: IP packet received\n");
            } else if (skb->protocol == htons(ETH_P_ARP)) {
                pr_info("RX: ARP packet received\n");
            } else {
                pr_info("RX: Unknown protocol: 0x%04x\n", ntohs(skb->protocol));
            }
            
            // Pass to network stack
            netif_rx(skb);

            // Update statistics
            dev->stats.rx_packets++;
            dev->stats.rx_bytes += length;

            pr_info("RX: Packet to kernel, len=%d\n", length);

            // Allocate new buffer for this descriptor
            skb = netdev_alloc_skb(dev, 1536);
            if (skb) {
                dma_addr_t dma = dma_map_single(&adapter->pci_dev->dev,
                                              skb->data, 1536, DMA_FROM_DEVICE);
                if (!dma_mapping_error(&adapter->pci_dev->dev, dma)) {
                    adapter->rx_skbs[rx_index] = skb;
                    rxd->buffer_addr = dma;
                } else {
                    dev_kfree_skb(skb);
                    adapter->rx_skbs[rx_index] = NULL;
                }
            } else {
                adapter->rx_skbs[rx_index] = NULL;
            }
        }

        // Clear descriptor status
        rxd->status = 0;
        
        // Move to next descriptor
        rx_index = (rx_index + 1) % 16;
        processed++;
    }

    // Update our next pointer
    adapter->rx_next = rx_index;

    // Update hardware RDT (Receive Descriptor Tail)
    writel((rx_index == 0) ? 15 : (rx_index - 1), 
           adapter->hw_addr + E1000_REG_RDT);

    pr_info("RX: Processed %d packets, next=%d\n", processed, adapter->rx_next);
}



// Add this function:
static void m_setup_mac(struct m_adapter *adapter)
{
    void __iomem *hw = adapter->hw_addr;
    u8 *mac_addr = adapter->netdev->dev_addr;
    u32 ral, rah;
    
    // Set Receive Address Low (RAL)
    ral = (mac_addr[0] << 0) | (mac_addr[1] << 8) | 
          (mac_addr[2] << 16) | (mac_addr[3] << 24);
    writel(ral, hw + E1000_REG_RAL);
    
    // Set Receive Address High (RAH) with Address Valid bit
    rah = (mac_addr[4] << 0) | (mac_addr[5] << 8) | (1 << 31); // AV bit
    writel(rah, hw + E1000_REG_RAH);
    
    pr_info("MAC Address set: %pM\n", mac_addr);
}



// ||||||||||||||||||||||||||||||||||||||||||||||||| _____ LINUX _____ |||||||||||||||||||||||||||||||||||||||||||||||||



//INIT ___________________________________________________________________________________________________
static int __init m_init(void)
{
    printk("1");
    pr_info("%s\n", "---------- MUSTAFA ---------- minimal ethernet driver");
    return pci_register_driver(&m_driver);
}



//EXIT ___________________________________________________________________________________________________
static void __exit m_exit(void)
{
    pci_unregister_driver(&m_driver);
    pr_info(DRV_NAME "MUSTAFA unloaded\n");
}







// ___________________________________________________________________________________________________
module_init(m_init);
module_exit(m_exit);

MODULE_DEVICE_TABLE(pci, id_table);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mustafa");
MODULE_DESCRIPTION("Minimal driver");