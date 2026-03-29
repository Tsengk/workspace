import struct

SPI_BUFFER_SIZE = 720
CAN2_OFFSET = 360

def swap_endian(val):
    return ((val >> 24) & 0xFF) | ((val >> 8) & 0xFF00) | \
           ((val << 8) & 0xFF0000) | ((val << 24) & 0xFF000000)

buffer = [0] * SPI_BUFFER_SIZE

# 注入 0x60A (帧起始)
idx = CAN2_OFFSET
buffer[idx]   = swap_endian(0x60A)
buffer[idx+1] = swap_endian(0x01000000)
buffer[idx+2] = swap_endian(0x00000000)
idx += 3

# 注入 0x60B (目标数据: ID=1, X=50m, Y=2m, Vx=-10m/s)
buffer[idx]   = swap_endian(0x60B)
buffer[idx+1] = swap_endian(0x0155F409) 
buffer[idx+2] = swap_endian(0x76200000) 
idx += 3

# 再次注入 0x60A (触发发布上一帧)
buffer[idx]   = swap_endian(0x60A)
buffer[idx+1] = swap_endian(0x01000000)
buffer[idx+2] = swap_endian(0x00000000)

# 生成二进制文件
with open("mock_radar_spi.bin", "wb") as f:
    for val in buffer:
        f.write(struct.pack('<I', val))

print("模拟 SPI 数据已生成: mock_radar_spi.bin")