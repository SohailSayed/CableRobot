import serial, time

ser = serial.Serial('COM4', 115200, timeout=1)
time.sleep(0.1)

# 1 bottom right, 2 top left, 3 bottom left, 4 top right
# don't do 4 yet until short is fixed

def move_motors(commands):
    # commands: [motor_id, dir, steps_per_sec, pulses]
    packet = bytes([len(commands)])
    for m in commands:
        speed_enc = m[2] // 10
        packet += bytes([
            m[0],               # motor_id
            m[1],               # direction
            speed_enc >> 8,     # speed high
            speed_enc & 0xFF,   # speed low
            m[3] >> 8,          # steps high
            m[3] & 0xFF,        # steps low
        ])
    print(packet)
    return packet

cmd = move_motors([
    [1,0,300, 2000],   # motor 1, CW,  800 steps/sec, 2000 pulses
    [3,1,2000,4800],   # motor 3, CCW, 2000 steps/sec, 1600 pulses
    [2,0,2000,4800]
])

print(list(cmd))
ser.write(cmd)
while True:
    line = ser.readline()
    if line:
        print(line)

ser.close()


