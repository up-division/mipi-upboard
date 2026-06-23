#!/bin/bash

# *************************************************************************
#                       SZ Sensing TECH CO.,LTD
#                        www.sensing-world.com
#                            0755-28990915
# *************************************************************************


# MAX96712 LinkA~D Initialization to pair with GMSL2 Serializers
#
# max96712 I2C address: 0x29(7Bit)Designed according to customer hardware
# max96717F I2C address: 0x40(7Bit)consistent


#Command description:
#i2ctransfer -y -f 1 w3@0x48 0x04 0x0B 0x00

# “1” represents the I2C bus number.It is subject to the actual situation 
#    of the customer's own platform.
# “0x48” represents the I2C address of the deserializer.
# “0x04 0x0B” represents the register address of the deserializer, which is 0x40B.
# “0x00” represents the value corresponding to the deserializer register address.


# Pipe 0 BPP Software Override Register 0x040b
i2ctransfer -y -f  1  w3@0x29 0x04 0x0b 0x00


# GMSL Link/PHY Enable and Mode Select Registe 0x0006
# Bit 7: PHY D, 0 = GMSL1, 1 = GMSL2.
# Bit 6: PHY C, 0 = GMSL1, 1 = GMSL2
# Bit 5: PHY B, 0 = GMSL1, 1 = GMSL2
# Bit 4: PHY A, 0 = GMSL1, 1 = GMSL2
# Bit 3: PHY D, 0 = Link Disable, 1 = Link Enable
# Bit 2: PHY C, 0 = Link Disable, 1 = Link Enable
# Bit 1: PHY B, 0 = Link Disable, 1 = Link Enable
# Bit 0: PHY A, 0 = Link Disable, 1 = Link Enable
i2ctransfer -y -f  1  w3@0x29 0x00 0x06 0xff


# GMSL Link/PHY Rate Select Register: 0x0010 0x0011
# Bits [7:6]: Tx Rate on Link B
# Bits [5:4]: Rx Rate on Link B
# Bits [3:2]: Tx Rate on Link A
# Bits [1:0]: Rx Rate on Link A
# Tx Rate (Transmitter rate; i.e., reverse channel):
# 00 = 187.5Mbps
# Rx Rate (Receiver rate; i.e., forward channel):
# 01 = 3Gbps
# 10 = 6Gbps
# Link A-D 3Gbps
i2ctransfer -y -f  1  w3@0x29 0x00 0x10 0x11 
i2ctransfer -y -f  1  w3@0x29 0x00 0x11 0x11

# GMSL Link Reset Register: 0x0018
# Bits [7:4]: Link reset register for each link D/C/B/A
# 0 = Release link reset
# 1 = Activate link reset
# Bits [3:0]: One-shot link reset for each link D/C/B/A
# 0 = No action
# 1 = Reset data path (self-clear)
i2ctransfer -y -f  1  w3@0x29 0x00 0x18 0x0F

sleep 0.3                          


01010000
11111010
# Video Pipe Select Register: 0x00f0 0x00f1
# Bits [7:6]: Link Selection for Pipe 1
# Bits [5:4]: Stream ID Selection for Pipe 1
# Bits [3:2]: Link Selection for Pipe 0
# Bits [1:0]: Stream ID Selection for Pipe 0
# GMSL Link Selection:
# 00 = Link A
# 01 = Link B
# 10 = Link C
# 11 = Link D
# Serializer Stream ID Selection:
# 00 = Stream 0, default serializer Pipe X
# 01 = Stream 1, default serializer Pipe Y
# 10 = Stream 2, default serializer Pipe Z
# 11 = Stream 3, default serializer Pipe U
# default value 0x62: link A -> pipe 0 stream id = 2, link B -> pipe 1 stream id = 2
# default value 0xea: link C -> pipe 2 stream id = 2, link D -> pipe 3 stream id = 2
i2ctransfer -y -f  1  w3@0x29 0x00 0xf0 0x62
i2ctransfer -y -f  1  w3@0x29 0x00 0xf1 0xea

# Video Pipe Enable Register: 0x00f4
# Bit 4: Stream Select All
# 0 = Disable; 1 = Enable
# Bit 3: Enable video pipe 3
# Bit 2: Enable video pipe 2
# Bit 1: Enable video pipe 1
# Bit 0: Enable video pipe 0
# 0 = Disable; 1 = Enable
i2ctransfer -y -f  1  w3@0x29 0x00 0xf4 0x0f

# Efficiency updates for pipe 0-3 in MAX96724 that carry image data. LIM_HEART = 1
# 0x4E,0x0106,0x0A
# 0x4E,0x0118,0x0A
# 0x4E,0x012A,0x0A
# 0x4E,0x013C,0x0A
i2ctransfer -y -f  1  w3@0x29 0x01 0x06 0x0a
i2ctransfer -y -f  1  w3@0x29 0x01 0x18 0x0a
i2ctransfer -y -f  1  w3@0x29 0x01 0x2a 0x0a
i2ctransfer -y -f  1  w3@0x29 0x01 0x3c 0x0a		 


###################################3 YUV422 8bit, video pipe 0, map FS/FE   ###################################
# Mapping Enable Low Byte Register: 0x090b
# 0000_0000 = No mapping enabled.
# XXXX_XXX1 = Map SRC_0 to DES_0.
# …
# 1XXX_XXXX = Map SRC_7 to DES_7
# Each bit enables 1 of max 16 mapping and distribution entries
# for current video stream. 
# Main point: Turn on 3 maps (map0, map1, map2)
i2ctransfer -y -f  1  w3@0x29 0x09 0x0b 0x07

# MAP Destination Controller Register: 0x092d
# Bits [7:6]: Destination Controller for Map 3
# Bits [5:4]: Destination Controller for Map 2
# Bits [3:2]: Destination Controller for Map 1
# Bits [1:0]: Destination Controller for Map 0
# 00 = Map to Controller 0
# 01 = Map to Controller 1
# 10 = Map to Controller 2
# 11 = Map to Controller 3
i2ctransfer -y -f  1  w3@0x29 0x09 0x2d 0x15


# MAP_SRC_x Register: 0x090d 0x090f 0x0911
# Bits [7:6]: VC = 0
# Bits [5:0]: DT
# MAP_DES_x Register: 0x090e 0x0910 0x0912
# Bits [7:6]: VC = 0
# Bits [5:0]: DT
# Main points: 1. Source and Destination Data type is yuv422(0x1e)
# Main points: 2. FS: Frame Start = 0x00, FE: Frame End = 0x01
# Main points: 3. VC: Virturl Channel = 0
i2ctransfer -y -f  1  w3@0x29 0x09 0x0d 0x1e
i2ctransfer -y -f  1  w3@0x29 0x09 0x0e 0x1e
i2ctransfer -y -f  1  w3@0x29 0x09 0x0f 0x00
i2ctransfer -y -f  1  w3@0x29 0x09 0x10 0x00
i2ctransfer -y -f  1  w3@0x29 0x09 0x11 0x01
i2ctransfer -y -f  1  w3@0x29 0x09 0x12 0x01  


################################### YUV422 8bit, video pipe 1, map FS/FE ###################################
i2ctransfer -y -f  1  w3@0x29 0x09 0x4b 0x07
i2ctransfer -y -f  1  w3@0x29 0x09 0x6d 0x15
i2ctransfer -y -f  1  w3@0x29 0x09 0x4d 0x1e
i2ctransfer -y -f  1  w3@0x29 0x09 0x4e 0x5e
i2ctransfer -y -f  1  w3@0x29 0x09 0x4f 0x00
i2ctransfer -y -f  1  w3@0x29 0x09 0x50 0x40
i2ctransfer -y -f  1  w3@0x29 0x09 0x51 0x01
i2ctransfer -y -f  1  w3@0x29 0x09 0x52 0x41


################################### YUV422 8bit, video pipe 2, map FS/FE ###################################
i2ctransfer -y -f  1  w3@0x29 0x09 0x8b 0x07
i2ctransfer -y -f  1  w3@0x29 0x09 0xad 0x15
i2ctransfer -y -f  1  w3@0x29 0x09 0x8d 0x1e
i2ctransfer -y -f  1  w3@0x29 0x09 0x8e 0x9e
i2ctransfer -y -f  1  w3@0x29 0x09 0x8f 0x00
i2ctransfer -y -f  1  w3@0x29 0x09 0x90 0x80
i2ctransfer -y -f  1  w3@0x29 0x09 0x91 0x01
i2ctransfer -y -f  1  w3@0x29 0x09 0x92 0x81


################################### YUV422 8bit, video pipe 3, map FS/FE ###################################
i2ctransfer -y -f  1  w3@0x29 0x09 0xcb 0x07
i2ctransfer -y -f  1  w3@0x29 0x09 0xed 0x15
i2ctransfer -y -f  1  w3@0x29 0x09 0xcd 0x1e
i2ctransfer -y -f  1  w3@0x29 0x09 0xce 0xde
i2ctransfer -y -f  1  w3@0x29 0x09 0xcf 0x00
i2ctransfer -y -f  1  w3@0x29 0x09 0xd0 0xc0
i2ctransfer -y -f  1  w3@0x29 0x09 0xd1 0x01
i2ctransfer -y -f  1  w3@0x29 0x09 0xd2 0xc1

sleep 0.3          	

################################### MIPI PHY Settin ###################################
# MIPI PHY Mode Select Register: 0x08a0
# Bit 7: Set to force all MIPI clocks running.
# Bit 6: Set to force PHY 3 MIPI clock running.
# Bit 5: Set to force PHY 0 MIPI clock running.
# Bit 4: MIPI PHY 1x4b + 2x2 Mode.
# Bit 3: MIPI PHY 1x4a + 2x2 Mode.
# Bit 2: MIPI PHY 2x4 Mode.
# Bit 1: MIPI PHY 1x4 Mode.
# Bit 0: MIPI PHY 4x2 Mode.
# Main point: 0x04 -> 2x4 Mode.
i2ctransfer -y -f  1  w3@0x29 0x08 0xa0 0x04 

# MIPI PHY 0 and 1 Lane Mapping Register: 0x08a3
# Bits [7:6]: Set PHY 1 Data Lane 1
# Bits [5:4]: Set PHY 1 Data Lane 0
# Bits [3:2]: Set PHY 0 Data Lane 1
# Bits [1:0]: Set PHY 0 Data Lane 0
# 00 = Map D0.
# 01 = Map D1.
# 10 = Map D2.
# 11 = Map D3.
# Main point: 0xe4: PHY1 L1 = Map D3, PHY1 L0 = Map D2, PHY0 L1 = Map D1, PHY0 L0 = Map D0 
i2ctransfer -y -f  1  w3@0x29 0x08 0xa3 0xe4

# MIPI PHY 3 and 2 Lane Mapping Register: 0x08a4
# Bits [7:6]: Set PHY 3 Data Lane 1
# Bits [5:4]: Set PHY 3 Data Lane 0
# Bits [3:2]: Set PHY 2 Data Lane 1
# Bits [1:0]: Set PHY 2 Data Lane 0
# 00 = Map D0.
# 01 = Map D1.
# 10 = Map D2.
# 11 = Map D3.
# Main point: 0xe4: PHY3 L1 = Map D3, PHY3 L0 = Map D2, PHY2 L1 = Map D1, PHY2 L0 = Map D0 
i2ctransfer -y -f  1  w3@0x29 0x08 0xa4 0xe4

# MIPI PHY0-3 Lane Count / C-PHY enable Register: 0x090a 0x094a 0x098a 0x09ca
# Bits [7:6]: Set lane count.
# 00 = 1 data lane
# 01 = 2 data lanes
# 10 = 3 data lanes
# 11 = 4 data lanes
# Bit 5: Enable C-PHY; D-PHY mode by default.
# 0 = C-PHY disabled, 1 = C-PHY enabled.
# Main point: 0xc0 c-phy disable
i2ctransfer -y -f  1  w3@0x29 0x09 0x0a 0xc0
i2ctransfer -y -f  1  w3@0x29 0x09 0x4a 0xc0
i2ctransfer -y -f  1  w3@0x29 0x09 0x8a 0xc0
i2ctransfer -y -f  1  w3@0x29 0x09 0xca 0xc0


# Turn on MIPI PH   
# MIPI PHY Enable Register: 0x08a2
# Bit 7: Enable MIPI PHY 3
# Bit 6: Enable MIPI PHY 2
# Bit 5: Enable MIPI PHY 1
# Bit 4: Enable MIPI PHY 0
# 0 = PHY in standby
# 1 = PHY enabled.             
# Main point: PHY0-3 Enable  
i2ctransfer -y -f  1  w3@0x29 0x08 0xa2 0xf0

# MIPI PHY0-3 DPLL Freq Register: 0x0415 0x0418 0x041b 0x041e
# Set DPLL frequency on multiple of 100MHz.
# D-PHY
# Clock freq is half; Data rate is equivalent bps/lane.
# 00010 = 200MHz DPLL, 200Mbps/lane data rate.
# …
# 11001 = 2500MHz DPLL, 2.5Gbps/lane data rate.
# C-PHY
# 2.28bits/symbol.
# 00010 = 200MHz DPLL, 456Mbps/lane data rate.
# …
# 11001 = 2500MHz DPLL, 5.7Gbps/lane data rate.
# Main point: 0x2f: 1500MHz DPLL, 1.5Gbps/lane data rate
i2ctransfer -y -f  1  w3@0x29 0x04 0x15 0x2f
i2ctransfer -y -f  1  w3@0x29 0x04 0x18 0x2f
i2ctransfer -y -f  1  w3@0x29 0x04 0x1b 0x2f
i2ctransfer -y -f  1  w3@0x29 0x04 0x1e 0x2f



################################### linkA ################################### 	
# GMSL Link/PHY Enable and Mode Select Register:
# Bit 7: PHY D, 0 = GMSL1, 1 = GMSL2.
# Bit 6: PHY C, 0 = GMSL1, 1 = GMSL2
# Bit 5: PHY B, 0 = GMSL1, 1 = GMSL2
# Bit 4: PHY A, 0 = GMSL1, 1 = GMSL2
# Bit 3: PHY D, 0 = Link Disable, 1 = Link Enable
# Bit 2: PHY C, 0 = Link Disable, 1 = Link Enable
# Bit 1: PHY B, 0 = Link Disable, 1 = Link Enable
# Bit 0: PHY A, 0 = Link Disable, 1 = Link Enable			
# Main point; 0xf1: PHYA-D is GMSL2, PHY link enable   
i2ctransfer -y -f  1  w3@0x29 0x00 0x06 0xf1

sleep 0.3          			       

# Serilizer Setting i2c addr = 0x40
# GPIO0 A: gpioa of gpio0, this is reset to isx031
# Bit 7: RES_CFG
# 0 = 40KΩ, 1 = 1MΩ
# Bit 4: GPIO_OUT
# 0 = Drive output to 0, 1 = Drive output to 1
# Bit 3: GPIO_IN, GPIO input level 0 or 1
# Bit 2: GPIO_RX_EN
# 0 = Disable receiving from the link.
# 1 = Enable receiving from the link
# Bit 1: GPIO_TX_EN
# 0 = Disable transmitting to the link.
# 1 = Enable transmitting to the link
# Bit 0: GPIO_OUT_DIS
# 0 = Output driver enabled
# 1 = Output driver disabled
# Main point: 0x10: RES_CFG = 40KΩ, GPIO_OUT=Drive output to 1, GPIO_IN=input level 0
# Main Point: 0x10: GPIO_RX_EN=Disable receiving from the link, GPIO_TX_EN=Disable transmitting to the link, GPIO_OUT_DIS=Output driver enabled
# Main Point: 0x10: isx031 reset set to high
i2ctransfer -y -f  1  w3@0x40 0x02 0xbe 0x10

# bit6 0 = Datatype filtering disabled
# bit6 1 = Datatype filtering enabled
# The value of bits 5:0 in this register should equal the data
# type ID of the data type you wish to allow onto the video
# pipe (e.g., RAW12 = 0x2C)
# Main point: 0x5e: 01011110 -> Bit6=1, DataType=0x1e, Datatype filtering enabled and DataType is yuv422
i2ctransfer -y -f  1  w3@0x40 0x03 0x18 0x5e
								   
#camera trigger  MFP7  low to  high         
# GPIO0 A: the gpioa of MFP7(gpio7), fsin to isx031
# Bit 7: RES_CFG
# 0 = 40KΩ, 1 = 1MΩ
# Bit 4: GPIO_OUT
# 0 = Drive output to 0, 1 = Drive output to 1
# Bit 3: GPIO_IN, GPIO input level 0 or 1
# Bit 2: GPIO_RX_EN
# 0 = Disable receiving from the link.
# 1 = Enable receiving from the link
# Bit 1: GPIO_TX_EN
# 0 = Disable transmitting to the link.
# 1 = Enable transmitting to the link
# Bit 0: GPIO_OUT_DIS
# 0 = Output driver enabled
# 1 = Output driver disabled   
# Main point: isx031 fsin set to low
i2ctransfer -y -f  1  w3@0x40 0x02 0xd3 0x00 
sleep 0.3 
# Main point: isx031 fsin set to high
i2ctransfer -y -f  1  w3@0x40 0x02 0xd3 0x10
								   
#camera trigger  MFP8   low to  high      
i2ctransfer -y -f  1  w3@0x40 0x02 0xd6 0x00 
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xd6 0x10

# Change I2C address for this Link A serializer
i2ctransfer -y -f  1  w3@0x40 0x00 0x00 0x82
								   
# linkB************************************ 
i2ctransfer -y -f  1  w3@0x29 0x00 0x06 0xf2
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xbe 0x10
i2ctransfer -y -f  1  w3@0x40 0x03 0x18 0x5e
#camera trigger  MFP7              
i2ctransfer -y -f  1  w3@0x40 0x02 0xd3 0x00 
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xd3 0x10
								   
#camera trigger  MFP8              
i2ctransfer -y -f  1  w3@0x40 0x02 0xd6 0x00 
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xd6 0x10
								   
i2ctransfer -y -f  1  w3@0x40 0x00 0x00 0x84
								   
# linkC*************************** ********* 
i2ctransfer -y -f  1  w3@0x29 0x00 0x06 0xf4
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xbe 0x10
i2ctransfer -y -f  1  w3@0x40 0x03 0x18 0x5e
#camera trigger  MFP7              
i2ctransfer -y -f  1  w3@0x40 0x02 0xd3 0x00 
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xd3 0x10
								   
#camera trigger  MFP8              
i2ctransfer -y -f  1  w3@0x40 0x02 0xd6 0x00 
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xd6 0x10
								   
i2ctransfer -y -f  1  w3@0x40 0x00 0x00 0x86

# linkD************************************ 
i2ctransfer -y -f  1  w3@0x29 0x00 0x06 0xf8
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xbe 0x10	 
i2ctransfer -y -f  1  w3@0x40 0x03 0x18 0x5e
#camera trigger  MFP7              
i2ctransfer -y -f  1  w3@0x40 0x02 0xd3 0x00 
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xd3 0x10
								   
#camera trigger  MFP8              
i2ctransfer -y -f  1  w3@0x40 0x02 0xd6 0x00 
sleep 0.3                          
i2ctransfer -y -f  1  w3@0x40 0x02 0xd6 0x10
								   
i2ctransfer -y -f  1  w3@0x40 0x00 0x00 0x88
#                                 


sleep 0.3   

# GMSL Link/PHY Enable and Mode Select Register:
# Bit 7: PHY D, 0 = GMSL1, 1 = GMSL2.
# Bit 6: PHY C, 0 = GMSL1, 1 = GMSL2
# Bit 5: PHY B, 0 = GMSL1, 1 = GMSL2
# Bit 4: PHY A, 0 = GMSL1, 1 = GMSL2
# Bit 3: PHY D, 0 = Link Disable, 1 = Link Enable
# Bit 2: PHY C, 0 = Link Disable, 1 = Link Enable
# Bit 1: PHY B, 0 = Link Disable, 1 = Link Enable
# Bit 0: PHY A, 0 = Link Disable, 1 = Link Enable
# Turn on all link
i2ctransfer -y -f  1  w3@0x29 0x00 0x06 0xff

# GMSL Link Reset Register: *
# Bits [7:4]: Link reset register for each link D/C/B/A
# 0 = Release link reset
# 1 = Activate link reset
# Bits [3:0]: One-shot link reset for each link D/C/B/A
# 0 = No action
# 1 = Reset data path (self-clear)
# Main point: 0x0f: One-shot link reset for each link D/C/B/A, Reset data path
i2ctransfer -y -f  1  w3@0x29 0x00 0x18 0x0f			

# PHY copy Register: 0x08a9 0x08aa
# Bit 7: 0 = Disable, 1 = Enable PHY0/1 copy
# Bits [6:5]: PHY copy destination.
# Bits [4:3]: PHY copy source.
#i2ctransfer -y -f  1  w3@0x29 0x08 0xa9 0xc8
#i2ctransfer -y -f  1  w3@0x29 0x08 0xaa 0xc8

# Pipe 0 BPP Software Override Register:
# Software override video data BPP.
i2ctransfer -y -f  1  w3@0x29 0x04 0x0b 0x02

# MIPI PHY Mode Select Register:
# Bit 7: Set to force all MIPI clocks running.
# Bit 6: Set to force PHY 3 MIPI clock running.
# Bit 5: Set to force PHY 0 MIPI clock running.
# Bit 4: MIPI PHY 1x4b + 2x2 Mode.
# Bit 3: MIPI PHY 1x4a + 2x2 Mode.
# Bit 2: MIPI PHY 2x4 Mode.
# Bit 1: MIPI PHY 1x4 Mode.
# Bit 0: MIPI PHY 4x2 Mode.
# Main point: 0x84: 10000100 -> Set to force all MIPI clocks running, MIPI PHY 2x4 Mode.
i2ctransfer -y -f  1  w3@0x29 0x08 0xa0 0x84