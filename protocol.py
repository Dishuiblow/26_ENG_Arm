import struct

CMD_STOP          = 0x00
CMD_HOME          = 0x01
CMD_GRAVITY_COMP  = 0x02
CMD_TEACH_MODE    = 0x03
CMD_TRAJ_DATA     = 0x04

HEADER = b'\xAA\x55'
TAIL   = b'\x0D\x0A'

class Protocol:
    @staticmethod
    def pack_cmd(cmd_id, param_bytes=b''):
        data_len = len(param_bytes)
        checksum = (cmd_id + data_len)
        for b in param_bytes: checksum += b
        checksum &= 0xFF
        return HEADER + struct.pack('BB', cmd_id, data_len) + param_bytes + struct.pack('B', checksum) + TAIL

    @staticmethod
    def unpack_data(buffer):
        MIN_LEN = 31
        while len(buffer) >= MIN_LEN:
            idx = buffer.find(HEADER)
            if idx == -1: return b'', None
            
            buffer = buffer[idx:]
            if len(buffer) < MIN_LEN: return buffer, None
            
            data_len = buffer[3]
            total_len = 4 + data_len + 1 + 2
            if len(buffer) < total_len: return buffer, None
            
            packet = buffer[:total_len]
            if packet[-2:] != TAIL: 
                buffer = buffer[2:]; continue
            
            try:
                payload = packet[4 : 4+data_len]
                angles = struct.unpack('<6f', payload) # 小端解包
                buffer = buffer[total_len:]
                return buffer, list(angles)
            except:
                buffer = buffer[2:]; continue
        return buffer, None