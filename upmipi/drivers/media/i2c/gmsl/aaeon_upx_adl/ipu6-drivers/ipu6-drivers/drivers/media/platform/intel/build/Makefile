# 自創的驅動模組名稱
obj-m := acpi_dumper.o

# 指定這個模組是由哪幾個 C 檔編譯組合而成的
acpi_dumper-objs := ipu-acpi.o ipu-acpi-common.o ipu-acpi-pdata.o

# 告訴編譯器在當前目錄尋找標頭檔
ccflags-y += -I$(PWD)

# 核心原始碼路徑
KDIR := /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean