import time, serial, struct

PORT="/dev/ttyACM0"; BAUD=38400; ADDR=0x80
CRC_POLY=0x1021
def crc16(d):
    c=0
    for b in d:
        c^=(b<<8)
        for _ in range(8):
            c=((c<<1)^CRC_POLY)&0xFFFF if (c&0x8000) else ((c<<1)&0xFFFF)
    return c
def tx(ser, cmd, payload=b"", ack=False, to=0.2):
    pkt=bytes([ADDR,cmd])+payload
    c=crc16(pkt); pkt+=bytes([(c>>8)&0xFF,c&0xFF])
    ser.write(pkt); ser.flush()
    if not ack: return True
    ser.timeout=to
    a=ser.read(1)
    return (len(a)==1 and a[0]==0xFF)
def rx_exact(ser,n,to=0.4):
    ser.timeout=to; out=b""
    while len(out)<n:
        ch=ser.read(n-len(out))
        if not ch: break
        out+=ch
    return out

def read_enc1(ser):
    tx(ser,16)
    r=rx_exact(ser,7)  # 4 data + 1 status + 2 CRC
    if len(r)!=7: return None
    val=struct.unpack(">i", r[:4])[0]
    return val

def read_enc2(ser):
    tx(ser,17)
    r=rx_exact(ser,7)
    if len(r)!=7: return None
    val=struct.unpack(">i", r[:4])[0]
    return val


def read_speed12(ser):
    tx(ser,108)  # Read average speeds M1/M2 (2x 4 bytes) + CRC
    r=rx_exact(ser,8+2)
    if len(r)<10: return None
    m1=struct.unpack(">i", r[:4])[0]; m2=struct.unpack(">i", r[4:8])[0]
    return m1,m2
def duty_m1m2(ser, m1, m2):
    m1=max(-32768,min(32767,int(m1))); m2=max(-32768,min(32767,int(m2)))
    tx(ser,34,struct.pack(">hh",m1,m2),ack=True)

if __name__=="__main__":
    ser=serial.Serial(PORT, baudrate=BAUD, timeout=0.2)
    ser.setDTR(False); ser.setRTS(False)
    duty_m1m2(ser, 9000, 9000)
    t0=time.time()
    for _ in range(10):
        c1=read_enc1(ser); c2=read_enc2(ser); sp=read_speed12(ser)
        print(f"enc1={c1}  enc2={c2}  speed={sp}")
        time.sleep(0.2)
    duty_m1m2(ser, 0, 0)
    ser.close()
