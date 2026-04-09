# ISX031 sensor driver
obj-m += isx031_driver.o
isx031_driver-objs := isx031.o

# 共用函式庫
obj-m += max-gmsl-lib.o
max-gmsl-lib-objs := max_des.o max_ser.o max_serdes.o

# Max96724 驅動
obj-m += max96724_driver.o       # 模組名稱叫 max96724_driver.ko
max96724_driver-objs := max96724.o

# Max96717 驅動
obj-m += max96717_driver.o       # 模組名稱叫 max96717_driver.ko
max96717_driver-objs := max96717.o

ccflags-y += -I$(src)

