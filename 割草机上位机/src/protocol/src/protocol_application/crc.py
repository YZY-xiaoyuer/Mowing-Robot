#!/usr/bin/env python3
# license removed for brevity
import rospy
import binascii
try:
    from crcmod.crcmod import *
    import crcmod.predefined
except ImportError:
    # Make this backward compatible
    from crcmod import *
    import predefined



class CRCGenerator(object):
    def __init__(self):
        self.module = 'crc-8-maxim'

    def create(self, input):
        crc8 = crcmod.predefined.Crc(self.module)
        hexData = input
        #print(hexData)
        hexData = binascii.unhexlify(hexData)
        crc8.update(hexData)
        result = crc8.crcValue
        #print(result)
        return result


if __name__ == "__main__":
    crc = CRCGenerator()