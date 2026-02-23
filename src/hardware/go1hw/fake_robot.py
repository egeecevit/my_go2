import socket
import struct
import time
# sudo ip addr add 192.168.123.10/24 dev lo
# sudo ip addr del 192.168.123.10/24 dev lo

LOCAL_IP   = "192.168.123.10"
LOCAL_PORT = 8007       # SDK sends to this
REPLY_PORT = 8090       # SDK listens on this

LOW_CMD_LENGTH   = 614
LOW_STATE_LENGTH = 807  # exact value from library

PosStopF = 2.146e9
VelStopF = 16000.0

# Wire format (empirically: MotorCmd reserve is 6 bytes, not 12 as in the header)
_HEADER = struct.Struct('<BBBBIIIH')   # 22 bytes: head[2]+levelFlag+frameRes+SN[2]+ver[2]+bw
_MOTOR  = struct.Struct('<Bfffff6s')  # 27 bytes: mode+q+dq+tau+Kp+Kd+reserve

MOTOR_NAMES = [
    'FR_hip','FR_thigh','FR_calf',
    'FL_hip','FL_thigh','FL_calf',
    'RR_hip','RR_thigh','RR_calf',
    'RL_hip','RL_thigh','RL_calf',
]

def decode_low_cmd(data):
    h = _HEADER.unpack_from(data, 0)
    print(f"  head=[0x{h[0]:02x} 0x{h[1]:02x}]  level=0x{h[2]:02x}  SN=({h[4]},{h[5]})")
    for i in range(20):
        mode, q, dq, tau, kp, kd, _ = _MOTOR.unpack_from(data, 22 + i * 27)
        if mode == 0 and q == 0.0 and dq == 0.0:
            continue
        name   = MOTOR_NAMES[i] if i < len(MOTOR_NAMES) else f"motor{i}"
        q_str  = " PosStop" if q  > 1e8 else f"{q:8.4f}"
        dq_str = " VelStop" if dq > 1e4 else f"{dq:8.4f}"
        print(f"  [{i:2d}]{name:>10}  mode={mode:#04x}  q={q_str}  dq={dq_str}  "
              f"tau={tau:7.4f}  Kp={kp:.2f}  Kd={kd:.2f}")
    crc, = struct.unpack_from('<I', data, 610)
    print(f"  crc=0x{crc:08x}")

def print_qd(data):
    """Compact view: only joint positions and velocities, one line per packet."""
    motors = [_MOTOR.unpack_from(data, 22 + i * 27) for i in range(12)]
    q   = ["  ---  " if m[1] > 1e8 else f"{m[1]:6.3f}" for m in motors]
    dq  = ["  ---  " if m[2] > 1e4 else f"{m[2]:6.3f}" for m in motors]
    print("q  " + " ".join(q))
    print("dq " + " ".join(dq))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LOCAL_IP, LOCAL_PORT))
print(f"Fake Go1 listening on {LOCAL_IP}:{LOCAL_PORT}")

_last_t = None
while True:
    data, addr = sock.recvfrom(4096)
    now = time.monotonic()
    dt  = (now - _last_t) * 1000 if _last_t is not None else float('nan')
    _last_t = now

    if len(data) == LOW_CMD_LENGTH:
        print(f"--- dt={dt:6.2f} ms")
        # print_qd(data)
        decode_low_cmd(data)

    # Send back a zeroed LowState (CRC will be wrong, SDK counts it
    # as RecvCRCError but won't crash — good enough to verify flow)
    reply = bytes(LOW_STATE_LENGTH)
    sock.sendto(reply, (addr[0], REPLY_PORT))