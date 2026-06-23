# <b> Manual I²C Register Access </b>

The Linux `i2ctransfer` utility can be used to manually read and write registers in a GMSL Serializer or Deserializer.

Before executing the command, the following information is required:

* **I²C bus number**, for example `5`
* **I²C slave address**, for example `0x27` for the Deserializer or `0x43` for the Serializer
* **Register address**, for example `0x0903` or `0x02BE`
* **Register value**, which is the 8-bit value to be written

---

### <b> 1. Write a Value to a Register </b>

To write an 8-bit value to a device using a 16-bit register address, use the following command format:

```bash
sudo i2ctransfer -y -f <bus> w3@<device_addr> <reg_addr_high> <reg_addr_low> <value>
```

#### <b> Parameter Description </b>

| Parameter         | Description                                                                      |
| ----------------- | -------------------------------------------------------------------------------- |
| `sudo`            | Executes the command with administrator privileges                               |
| `i2ctransfer`     | Linux utility for performing I²C transactions                                    |
| `-y`              | Disables the interactive confirmation prompt                                     |
| `-f`              | Forces access even if the device address is currently claimed by a kernel driver |
| `<bus>`           | I²C bus number                                                                   |
| `w3`              | Writes three bytes to the device                                                 |
| `@<device_addr>`  | I²C slave address of the target device                                           |
| `<reg_addr_high>` | Upper 8 bits of the 16-bit register address                                      |
| `<reg_addr_low>`  | Lower 8 bits of the 16-bit register address                                      |
| `<value>`         | 8-bit value to be written                                                        |

The three transmitted bytes are arranged as follows:

```text
Byte 1: Register Address[15:8]
Byte 2: Register Address[7:0]
Byte 3: Register Value[7:0]
```

---

### <b> 2. Read a Value from a Register </b>

To read an 8-bit value from a device using a 16-bit register address, use the following command format:

```bash
sudo i2ctransfer -y -f <bus> w2@<device_addr> <reg_addr_high> <reg_addr_low> r1
```

#### <b> Parameter Description </b>

| Parameter         | Description                                      |
| ----------------- | ------------------------------------------------ |
| `w2`              | Writes two bytes containing the register address |
| `@<device_addr>`  | I²C slave address of the target device           |
| `<reg_addr_high>` | Upper 8 bits of the 16-bit register address      |
| `<reg_addr_low>`  | Lower 8 bits of the 16-bit register address      |
| `r1`              | Reads one byte from the device                   |

This command performs a combined I²C transaction:

```text
Write: Register Address High + Register Address Low
Read : Register Value
```

---

## <b> Deserializer Register Access Example </b>

Assume the Deserializer uses the following configuration:

```text
I²C Bus          : 5
I²C Slave Address: 0x27
Register Address : 0x0903
```

The 16-bit register address `0x0903` must be divided into two bytes:

```text
Upper 8 bits: 0x09
Lower 8 bits: 0x03
```

### <b> Set DES Register `0x0903` </b>

The following command writes `0x00` to register `0x0903` of the Deserializer:

```bash
sudo i2ctransfer -y -f 5 w3@0x27 0x09 0x03 0x00
```

The command represents:

```text
I²C Bus         = 5
DES I²C Address = 0x27
Register Address= 0x0903
Write Value     = 0x00
```

The transmitted data bytes are:

```text
0x09 0x03 0x00
```

### <b> Get DES Register `0x0903` </b>

The following command reads the current value of register `0x0903`:

```bash
sudo i2ctransfer -y -f 5 w2@0x27 0x09 0x03 r1
```

For example, if the register value is `0x00`, the output will be:

```text
0x00
```

The register can also be written and read back immediately for verification:

```bash
sudo i2ctransfer -y -f 5 w3@0x27 0x09 0x03 0x00
sudo i2ctransfer -y -f 5 w2@0x27 0x09 0x03 r1
```

---

## <b> Serializer Register Access Example </b>

Assume the Serializer uses the following configuration:

```text
I²C Bus          : 5
I²C Slave Address: 0x43
Register Address : 0x02BE
```

The 16-bit register address `0x02BE` must be divided into two bytes:

```text
Upper 8 bits: 0x02
Lower 8 bits: 0xBE
```

### <b> Set SER Register `0x02BE` </b>

The following command writes `0x10` to register `0x02BE` of the Serializer:

```bash
sudo i2ctransfer -y -f 5 w3@0x43 0x02 0xbe 0x10
```

The command represents:

```text
I²C Bus         = 5
SER I²C Address = 0x43
Register Address= 0x02BE
Write Value     = 0x10
```

The transmitted data bytes are:

```text
0x02 0xBE 0x10
```

### <b> Get SER Register `0x02BE` </b>

The following command reads the current value of register `0x02BE`:

```bash
sudo i2ctransfer -y -f 5 w2@0x43 0x02 0xbe r1
```

For example, if the register value is `0x10`, the output will be:

```text
0x10
```

The register can also be written and read back immediately for verification:

```bash
sudo i2ctransfer -y -f 5 w3@0x43 0x02 0xbe 0x10
sudo i2ctransfer -y -f 5 w2@0x43 0x02 0xbe r1
```

---

## <b> Register Address Conversion </b>

For a 16-bit register address, the upper and lower bytes can be calculated as follows:

```text
Register Address High = (Register Address >> 8) & 0xFF
Register Address Low  = Register Address & 0xFF
```

For register `0x0903`:

```text
High Byte = 0x09
Low Byte  = 0x03
```

For register `0x02BE`:

```text
High Byte = 0x02
Low Byte  = 0xBE
```

---

## <b> Command Summary </b>

### <b> Write Register </b>

```bash
sudo i2ctransfer -y -f <bus> \
    w3@<device_addr> \
    <reg_addr_high> <reg_addr_low> <value>
```

### <b> Read Register </b>

```bash
sudo i2ctransfer -y -f <bus> \
    w2@<device_addr> \
    <reg_addr_high> <reg_addr_low> \
    r1
```

### <b> Deserializer Example </b>

```bash
# Set DES register 0x0903 to 0x00
sudo i2ctransfer -y -f 5 w3@0x27 0x09 0x03 0x00

# Get DES register 0x0903
sudo i2ctransfer -y -f 5 w2@0x27 0x09 0x03 r1
```

### <b> Serializer Example </b>

```bash
# Set SER register 0x02BE to 0x10
sudo i2ctransfer -y -f 5 w3@0x43 0x02 0xbe 0x10

# Get SER register 0x02BE
sudo i2ctransfer -y -f 5 w2@0x43 0x02 0xbe r1
```

> **Note:**
> `0x27` and `0x43` are I²C slave addresses, while `0x0903` and `0x02BE` are internal register addresses. These two address types must not be confused.
>
> The `-f` option forces access to the I²C device even when the address is claimed by a kernel driver. Manually modifying registers while the driver is active may cause conflicts or unexpected device behavior.
