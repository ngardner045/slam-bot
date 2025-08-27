import time, serial, struct

PORT = "/dev/ttyACM0"   # USB CDC device on the Pi
BAUD = 38400            # Value here is ignored by USB CDC, but harmless
ADDR = 0x80             # Packet-serial address (Mode 7). 0x80–0x87 are valid

CRC_POLY = 0x1021

def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc

def write_packet(ser, cmd, payload=b"", expect_ack=True, timeout=0.2):
    pkt = bytes([ADDR, cmd]) + payload
    c = crc16(pkt)
    pkt += bytes([(c >> 8) & 0xFF, c & 0xFF])
    # Send whole packet in one write to avoid 10ms inter-byte timeout
    ser.write(pkt); ser.flush()  # (Manual: >10ms gap discards packet)
    if not expect_ack:
        return True
    ser.timeout = timeout
    ack = ser.read(1)
    return (len(ack) == 1 and ack[0] == 0xFF)

def read_exact(ser, n, timeout=0.5):
    ser.timeout = timeout
    out = b""
    while len(out) < n:
        chunk = ser.read(n - len(out))
        if not chunk:
            break
        out += chunk
    return out

def get_version(ser) -> str | None:
    # Command 21: returns ASCII bytes ending with LF(10) then NUL(0), then 2 CRC bytes
    if not write_packet(ser, 21, b"", expect_ack=False):
        return None
    s = bytearray()
    # Read up to 64 bytes until NUL
    for _ in range(64):
        b = read_exact(ser, 1)
        if not b:
            return None
        if b == b"\x00":
            break
        s += b
    # Consume CRC16 (2 bytes)
    _ = read_exact(ser, 2)
    try:
        return s.decode("ascii", errors="ignore").strip()
    except:
        return None

def read_main_batt(ser) -> float | None:
    # Command 24: 2 data bytes (0.1V units) + CRC16
    if not write_packet(ser, 24, b"", expect_ack=False):
        return None
    resp = read_exact(ser, 4)  # 2 data + 2 CRC
    if len(resp) != 4:
        return None
    val = (resp[0] << 8) | resp[1]
    return val * 0.1

def read_status(ser) -> int | None:
    # Command 90: returns 4-byte Status + CRC16 (manual lists many 32-bit bitmasks)
    if not write_packet(ser, 90, b"", expect_ack=False):
        return None
    resp = read_exact(ser, 6)
    if len(resp) != 6:
        return None
    status = struct.unpack(">I", resp[:4])[0]
    # Optionally validate CRC here using manual's rule (calc over ADDR+CMD+data)
    return status

def duty_m1m2(ser, m1, m2):
    # Command 34: signed 16-bit each, range -32768..32767
    m1 = max(-32768, min(32767, int(m1)))
    m2 = max(-32768, min(32767, int(m2)))
    payload = struct.pack(">hh", m1, m2)
    ok = write_packet(ser, 34, payload, expect_ack=True)
    if not ok:
        print("WARN: did not get 0xFF ACK for duty command")

def decode_status_bits(status: int) -> list[str]:
    labels = {
        0x000001: "E-Stop",
        0x000008: "Main Voltage High ERROR",
        0x000020: "Logic Voltage Low ERROR",
        0x000040: "M1 Driver Fault",
        0x000080: "M2 Driver Fault",
        0x010000: "M1 Over Current WARNING",
        0x020000: "M2 Over Current WARNING",
        0x040000: "Main Voltage High WARNING",
        0x080000: "Main Voltage Low WARNING",
        0x400000: "S4 triggered",
        0x800000: "S5 triggered",
    }
    return [name for bit, name in labels.items() if status & bit]

if __name__ == "__main__":
    ser = serial.Serial(PORT, baudrate=BAUD, timeout=0.2)
    ser.setDTR(False); ser.setRTS(False)
    ser.reset_input_buffer(); ser.reset_output_buffer()

    ver = get_version(ser)
    vbat = read_main_batt(ser)
    print(f"Version: {ver}")
    print(f"Main battery: {vbat} V")

    status = read_status(ser)
    print(f"Status: 0x{status:08X}" if status is not None else "Status: <none>")
    if status is not None:
        flags = decode_status_bits(status)
        if flags:
            print("Status flags:", ", ".join(flags))
        if status & 0x000001:
            print("E-Stop is active; clear it in Motion Studio or via S4/S5 and retry.")
            ser.close(); exit(1)

    print("Forward...")
    duty_m1m2(ser, 9000, 9000)
    time.sleep(1.5)

    print("Reverse...")
    duty_m1m2(ser, -9000, -9000)
    time.sleep(1.5)

    print("Stop.")
    duty_m1m2(ser, 0, 0)
    ser.close()
