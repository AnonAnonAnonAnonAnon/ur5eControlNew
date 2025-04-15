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
	{ 0x8e17b3ae, "idr_destroy" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x69acdf38, "memcpy" },
	{ 0xc6b3a418, "usb_autopm_get_interface_async" },
	{ 0xf5cc6953, "usb_anchor_urb" },
	{ 0x4d8945bb, "usb_autopm_get_interface" },
	{ 0x8dc64d3f, "usb_control_msg" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x24980310, "kmalloc_caches" },
	{ 0x1d199deb, "kmalloc_trace" },
	{ 0x37a0cba, "kfree" },
	{ 0x84e3473f, "usb_ifnum_to_if" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xb8f11603, "idr_alloc" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xf9c632c9, "tty_port_init" },
	{ 0x40895c98, "usb_alloc_coherent" },
	{ 0x5ea56f08, "usb_alloc_urb" },
	{ 0x7665a95b, "idr_remove" },
	{ 0x7f9547ea, "usb_free_coherent" },
	{ 0x6d169046, "_dev_info" },
	{ 0x3858461e, "usb_driver_claim_interface" },
	{ 0x1ed63277, "usb_get_intf" },
	{ 0xcb184993, "tty_port_register_device" },
	{ 0x8ba48fbe, "usb_free_urb" },
	{ 0x1fe9f97b, "__tty_insert_flip_string_flags" },
	{ 0xd7cbd47b, "tty_flip_buffer_push" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x20978fb9, "idr_find" },
	{ 0x78af330e, "tty_standard_install" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0xc6cbbc89, "capable" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0xaad8c7d6, "default_wake_function" },
	{ 0x2b6e2cf, "pcpu_hot" },
	{ 0x4afb2238, "add_wait_queue" },
	{ 0x1000e51, "schedule" },
	{ 0x37110088, "remove_wait_queue" },
	{ 0xcd9c13a3, "tty_termios_hw_change" },
	{ 0xbd394d8, "tty_termios_baud_rate" },
	{ 0x68f17305, "usb_put_intf" },
	{ 0x7ab8a251, "tty_port_tty_get" },
	{ 0x5b76151f, "tty_vhangup" },
	{ 0x38486ac5, "tty_kref_put" },
	{ 0x43ec76c3, "tty_unregister_device" },
	{ 0x54b620ce, "usb_driver_release_interface" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x8498f7ee, "usb_submit_urb" },
	{ 0x5bd0f510, "_dev_err" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x2565de6f, "usb_autopm_put_interface_async" },
	{ 0xfa8a5ed5, "usb_kill_urb" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x4b750f53, "_raw_spin_unlock_irq" },
	{ 0xa34ad844, "tty_port_put" },
	{ 0x9a6c08a9, "__tty_alloc_driver" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0xa408799d, "tty_register_driver" },
	{ 0xc512a579, "usb_register_driver" },
	{ 0x122c3a7e, "_printk" },
	{ 0x5c300249, "tty_unregister_driver" },
	{ 0xbdb06dc0, "tty_driver_kref_put" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0xe2964344, "__wake_up" },
	{ 0xa14c5f46, "__dynamic_dev_dbg" },
	{ 0x8929e2df, "tty_port_tty_hangup" },
	{ 0x6528633a, "usb_autopm_get_interface_no_resume" },
	{ 0x538f76e5, "usb_autopm_put_interface" },
	{ 0x21476bdd, "usb_get_from_anchor" },
	{ 0x7992e209, "tty_port_tty_wakeup" },
	{ 0x7c64053f, "tty_port_hangup" },
	{ 0xd411089a, "tty_port_close" },
	{ 0xf6334b63, "tty_port_open" },
	{ 0x82d424fa, "usb_deregister" },
	{ 0x6ad2b3e, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v1A86p7523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p7522d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p5523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86pE523d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v4348p5523d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "8B035E6078723C06DFB968D");
