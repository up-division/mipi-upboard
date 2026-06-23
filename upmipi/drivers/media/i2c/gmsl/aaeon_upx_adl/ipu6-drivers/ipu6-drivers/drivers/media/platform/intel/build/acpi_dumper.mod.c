#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

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



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xd5ed4c7f, "acpi_has_method" },
	{ 0xb12ae692, "is_acpi_device_node" },
	{ 0x0e1afaf1, "gpiod_put" },
	{ 0x7105bf82, "i2c_get_adapter" },
	{ 0xd710adbf, "__kmalloc_noprof" },
	{ 0x40a621c5, "snprintf" },
	{ 0x587292b0, "pci_dev_put" },
	{ 0xbb6ca247, "acpi_bus_for_each_dev" },
	{ 0x112ddde8, "gpiod_get_index" },
	{ 0xa53f4e29, "memcpy" },
	{ 0xcb8b6ec6, "kfree" },
	{ 0xbe011736, "__dynamic_dev_dbg" },
	{ 0xd272d446, "__fentry__" },
	{ 0x5a844b26, "__x86_indirect_thunk_rax" },
	{ 0xe8213e80, "_printk" },
	{ 0xbd03ed67, "__ref_stack_chk_guard" },
	{ 0xd272d446, "__stack_chk_fail" },
	{ 0x7d74049b, "put_device" },
	{ 0x9479a1e8, "strnlen" },
	{ 0x9b1de7cb, "_dev_info" },
	{ 0x90a48d82, "__ubsan_handle_out_of_bounds" },
	{ 0xd70733be, "sized_strscpy" },
	{ 0x55e5dc35, "acpi_get_handle" },
	{ 0x97d7321b, "acpi_format_exception" },
	{ 0xce40870d, "acpi_evaluate_dsm" },
	{ 0x9b1de7cb, "_dev_err" },
	{ 0x1c88d2b4, "bus_find_device" },
	{ 0xbd03ed67, "random_kmalloc_seed" },
	{ 0x2435d559, "strncmp" },
	{ 0xaf84a203, "acpi_get_pci_dev" },
	{ 0xb989d98f, "acpi_evaluate_reference" },
	{ 0xe54e0a6b, "__fortify_panic" },
	{ 0xd272d446, "__x86_return_thunk" },
	{ 0x888b8f57, "strcmp" },
	{ 0xce4af33b, "kstrdup" },
	{ 0xa68efaaa, "desc_to_gpio" },
	{ 0x3a6a95ce, "acpi_fetch_acpi_dev" },
	{ 0x9fbaa470, "acpi_get_object_info" },
	{ 0x651bbffe, "acpi_evaluate_object" },
	{ 0x5c8e95e7, "platform_bus_type" },
	{ 0x23f25c0a, "__dynamic_pr_debug" },
	{ 0xc064623f, "__kmalloc_cache_noprof" },
	{ 0x59318c2d, "acpi_dev_get_next_match_dev" },
	{ 0xee6ab913, "set_primary_fwnode" },
	{ 0xe4de56b4, "__ubsan_handle_load_invalid_value" },
	{ 0x43a349ca, "strlen" },
	{ 0xfaabfe5e, "kmalloc_caches" },
	{ 0x81e1dc15, "acpi_walk_resources" },
	{ 0xbebe66ff, "module_layout" },
};

static const u32 ____version_ext_crcs[]
__used __section("__version_ext_crcs") = {
	0xd5ed4c7f,
	0xb12ae692,
	0x0e1afaf1,
	0x7105bf82,
	0xd710adbf,
	0x40a621c5,
	0x587292b0,
	0xbb6ca247,
	0x112ddde8,
	0xa53f4e29,
	0xcb8b6ec6,
	0xbe011736,
	0xd272d446,
	0x5a844b26,
	0xe8213e80,
	0xbd03ed67,
	0xd272d446,
	0x7d74049b,
	0x9479a1e8,
	0x9b1de7cb,
	0x90a48d82,
	0xd70733be,
	0x55e5dc35,
	0x97d7321b,
	0xce40870d,
	0x9b1de7cb,
	0x1c88d2b4,
	0xbd03ed67,
	0x2435d559,
	0xaf84a203,
	0xb989d98f,
	0xe54e0a6b,
	0xd272d446,
	0x888b8f57,
	0xce4af33b,
	0xa68efaaa,
	0x3a6a95ce,
	0x9fbaa470,
	0x651bbffe,
	0x5c8e95e7,
	0x23f25c0a,
	0xc064623f,
	0x59318c2d,
	0xee6ab913,
	0xe4de56b4,
	0x43a349ca,
	0xfaabfe5e,
	0x81e1dc15,
	0xbebe66ff,
};
static const char ____version_ext_names[]
__used __section("__version_ext_names") =
	"acpi_has_method\0"
	"is_acpi_device_node\0"
	"gpiod_put\0"
	"i2c_get_adapter\0"
	"__kmalloc_noprof\0"
	"snprintf\0"
	"pci_dev_put\0"
	"acpi_bus_for_each_dev\0"
	"gpiod_get_index\0"
	"memcpy\0"
	"kfree\0"
	"__dynamic_dev_dbg\0"
	"__fentry__\0"
	"__x86_indirect_thunk_rax\0"
	"_printk\0"
	"__ref_stack_chk_guard\0"
	"__stack_chk_fail\0"
	"put_device\0"
	"strnlen\0"
	"_dev_info\0"
	"__ubsan_handle_out_of_bounds\0"
	"sized_strscpy\0"
	"acpi_get_handle\0"
	"acpi_format_exception\0"
	"acpi_evaluate_dsm\0"
	"_dev_err\0"
	"bus_find_device\0"
	"random_kmalloc_seed\0"
	"strncmp\0"
	"acpi_get_pci_dev\0"
	"acpi_evaluate_reference\0"
	"__fortify_panic\0"
	"__x86_return_thunk\0"
	"strcmp\0"
	"kstrdup\0"
	"desc_to_gpio\0"
	"acpi_fetch_acpi_dev\0"
	"acpi_get_object_info\0"
	"acpi_evaluate_object\0"
	"platform_bus_type\0"
	"__dynamic_pr_debug\0"
	"__kmalloc_cache_noprof\0"
	"acpi_dev_get_next_match_dev\0"
	"set_primary_fwnode\0"
	"__ubsan_handle_load_invalid_value\0"
	"strlen\0"
	"kmalloc_caches\0"
	"acpi_walk_resources\0"
	"module_layout\0"
;

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "8CA1398E8A1880C005A109D");
