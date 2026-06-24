#!/bin/bash

# *************************************************************************
#                       SZ Sensing TECH CO.,LTD
#                        www.sensing-world.com
#                            0755-28990915
# *************************************************************************

# MAX9296 LinkA Initialization to pair with GMSL2 Serializers
#
# Use max9296's linkA input and portA output
# max9296 I2C address: 0x48(7Bit)Designed according to customer hardware
# max96717F I2C address: 0x40(7Bit)consistent


#Command description:
#i2ctransfer -y -f 1 w3@0x48 0x04 0x0B 0x00

# “1” represents the I2C bus number.It is subject to the actual situation 
#    of the customer's own platform.
# “0x48” represents the I2C address of the deserializer.
# “0x04 0x0B” represents the register address of the deserializer, which is 0x40B.
# “0x00” represents the value corresponding to the deserializer register address.


#disable MIPI output
i2ctransfer -y -f 1 w3@0x48 0x03 0x13 0x00
#3G
i2ctransfer -y -f 1 w3@0x48 0x00 0x01 0x01
sleep 0.3

#camera reset
i2ctransfer -y -f 1 w3@0x40 0x02 0xBE 0x10
#
i2ctransfer -y -f 1 w3@0x40 0x00 0x5B 0x01
# datatype  YUV422
i2ctransfer -y -f 1 w3@0x40 0x03 0x18 0x5E

#camera trigger  MFP7   low to  high
i2ctransfer -y -f 1 w3@0x40 0x02 0xD3 0x00
sleep 0.5
i2ctransfer -y -f 1 w3@0x40 0x02 0xD3 0x10

#camera trigger  MFP8    low to  high
i2ctransfer -y -f 1 w3@0x40 0x02 0xD6 0x00
sleep 0.5
i2ctransfer -y -f 1 w3@0x40 0x02 0xD6 0x10

# MIPI 1200Mbps
i2ctransfer -y -f 1 w3@0x48 0x03 0x20 0x2C
#enable MIPI output
i2ctransfer -y -f 1 w3@0x48 0x03 0x13 0x02