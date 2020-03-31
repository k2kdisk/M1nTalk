import sensor,image,time
import KPU as kpu
from fpioa_manager import fm
from machine import UART

fm.register (board_info.PIN10, fm.fpioa.UART1_RX)
fm.register (board_info.PIN17, fm.fpioa.UART1_TX)

uart = UART (UART.UART1, 9600, 8, None, 1, timeout = 1000, read_buf_len = 256)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.run(1)

clock = time.clock()
classes = [
    ('aeroplane', ''),
    ('bicycle', ''),
    ('bird', 'tori'),
    ('boat', ''),
    ('bottle', ''),
    ('bus', ''),
    ('car', ''),
    ('cat', 'neko'),
    ('chair', ''),
    ('cow', ''),
    ('diningtable', ''),
    ('dog', 'inu'),
    ('horse', ''),
    ('motorbike', ''),
    ('person', 'hito'),
    ('pottedplant', ''),
    ('sheep', ''),
    ('sofa', ''),
    ('train', ''),
    ('tvmonitor', ''),
    ]
task = kpu.load(0x500000)
anchor = (1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52)
nw = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)

avg_len = 5
count = [[0] * avg_len for i in range(len(classes))]
idx = 0

while(True):
    clock.tick()

    img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)

    infostr = str(clock.fps()) + ' : '

    for i in range(len(classes)):
        count[i][idx] = 0

    if code:
        for i in code:
            infostr = infostr + classes[i.classid()][0] + ', '
            count[i.classid()][idx] = 1

    for i in range(len(classes)):
        if sum(count[i]) > avg_len/2:
            if classes[i][1]:
                uart.write(classes[i][1] + 'gairuyo\r')

    print(infostr)
    #print(count)

    idx = (idx + 1) % avg_len
model = kpu.deinit(task)

uart.deinit ()
del uart
