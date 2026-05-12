import struct

SPI_BUFFER_SIZE = 720
CAN3_OFFSET = 0
CAN2_OFFSET = 360

def swap_endian(val):
    return ((val >> 24) & 0xFF) | ((val >> 8) & 0xFF00) | \
           ((val << 8) & 0xFF0000) | ((val << 24) & 0xFF000000)

def inject_can_message(buffer, offset, can_id, data_high, data_low):
    buffer[offset]   = swap_endian(can_id)
    buffer[offset+1] = swap_endian(data_high)
    buffer[offset+2] = swap_endian(data_low)

buffer = [0] * SPI_BUFFER_SIZE

# ============ CAN3 (前雷达) ============
# 前雷达目标: ID=2, X=10m, Y=-3m, Vx=15m/s, Vy=0m/s
idx = CAN3_OFFSET
inject_can_message(buffer, idx, 0x60A, 0x01000000, 0x00000000)
idx += 3
# 计算: raw_dl=(10+500)/0.2=2550=0x9F6 → data[1]=0x4F, data[2]高5位=0x16
#       raw_dlat=(-3+204.6)/0.2=1008=0x3F0 → data[2]低3位=0x03, data[3]=0xF0
#       data[2]=0xB3, data_high=0x024FB3F0
#       raw_vl=(15+128)/0.25=572=0x23C → data[4]=0x8F, data[5]高2位=0
#       raw_vlat=(0+64)/0.25=256=0x100 → data[5]低6位=0x20, data[6]=0
#       data[5]=0x20, data_low=0x8F200000
inject_can_message(buffer, idx, 0x60B, 0x024FB3F0, 0x8F200000)
idx += 3
inject_can_message(buffer, idx, 0x60A, 0x01000000, 0x00000000)
idx += 3

# ============ CAN2 (后雷达) ============
# 后雷达目标: ID=1, X=5m, Y=1m, Vx=-5m/s, Vy=0m/s
# 计算: raw_dl=(5+500)/0.2=2525 → data[1]=0x4E, data[2]高5位=0x1D
#       raw_dlat=(1+204.6)/0.2=1028 → data[2]低3位=0x04, data[3]=0x04
#       data[2]=0xEC, data_high=0x014EEC04
#       raw_vl=(-5+128)/0.25=492 → data[4]=0x7B, data[5]高2位=0
#       raw_vlat=(0+64)/0.25=256 → data[5]低6位=0x20, data[6]=0
#       data[5]=0x20, data_low=0x7B200000
idx = CAN2_OFFSET
inject_can_message(buffer, idx, 0x60A, 0x01000000, 0x00000000)
idx += 3
inject_can_message(buffer, idx, 0x60B, 0x014EEC04, 0x7B200000)
idx += 3
inject_can_message(buffer, idx, 0x60A, 0x01000000, 0x00000000)

# 生成二进制文件
with open("mock_radar_spi.bin", "wb") as f:
    for val in buffer:
        f.write(struct.pack('<I', val))

print("模拟 SPI 数据已生成: mock_radar_spi.bin")
print("  前雷达(CAN3): ID=2, X=10m, Y=-3m, Vx=15m/s")
print("  后雷达(CAN2): ID=1, X=5m, Y=1m, Vx=-5m/s")