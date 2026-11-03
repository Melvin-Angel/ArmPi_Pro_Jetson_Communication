"""Candidate Hiwonder count register; does not configure or command motors."""
from wheel_math import decode_counts


def read_counts(bus_number=1, address=0x34, register=0x3c):
    from smbus2 import SMBus, i2c_msg
    with SMBus(bus_number) as bus:
        # Select register, then read; never write count or motor values.
        select = i2c_msg.write(address, [register])
        result = i2c_msg.read(address, 16)
        bus.i2c_rdwr(select, result)
        return decode_counts(list(result))
