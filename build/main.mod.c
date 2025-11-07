#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x2a19f8a8, "pcim_iomap_table" },
	{ 0x24ad6361, "alloc_netdev_mqs" },
	{ 0x5231d2f, "register_netdev" },
	{ 0xac5bfa41, "dma_alloc_attrs" },
	{ 0x85826c26, "pci_alloc_irq_vectors" },
	{ 0xa32bb6d, "pci_irq_vector" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x99fb7e2d, "pci_free_irq_vectors" },
	{ 0xbdb56c60, "unregister_netdev" },
	{ 0x9f3bd1c3, "dma_free_attrs" },
	{ 0xbed2fe94, "free_netdev" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x69e8d0a5, "pci_unregister_driver" },
	{ 0x47f26ecb, "ether_setup" },
	{ 0x41ed3709, "get_random_bytes" },
	{ 0x32a68c03, "dev_addr_mod" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xc31db0ce, "is_vmalloc_addr" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0x97651e6c, "vmemmap_base" },
	{ 0x6fba437, "dma_map_page_attrs" },
	{ 0x4a453f53, "iowrite32" },
	{ 0xa2a65ba1, "consume_skb" },
	{ 0xa4bffd29, "dev_driver_string" },
	{ 0x56470118, "__warn_printk" },
	{ 0x4c9d28b0, "phys_base" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0xc1514a3b, "free_irq" },
	{ 0xbaa526e8, "dma_unmap_page_attrs" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x7fd383d9, "netif_carrier_off" },
	{ 0x9868babd, "netif_carrier_on" },
	{ 0x684fb036, "__netdev_alloc_skb" },
	{ 0xa78af5f3, "ioread32" },
	{ 0x40ed958e, "dev_kfree_skb_any_reason" },
	{ 0x12abf265, "netif_tx_wake_queue" },
	{ 0x5aab3954, "skb_put" },
	{ 0x1bf79915, "eth_type_trans" },
	{ 0x230caca3, "netif_rx" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x122c3a7e, "_printk" },
	{ 0xe067206b, "__pci_register_driver" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x3de60d26, "pcim_enable_device" },
	{ 0x7afd69b6, "pci_set_master" },
	{ 0x34c06377, "pcim_iomap_regions" },
	{ 0xe2fd41e5, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v00008086d0000100Esv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "5DDE0A31C3593A6C8DE7739");
