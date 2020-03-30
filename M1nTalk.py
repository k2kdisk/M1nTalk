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
while(True):
    clock.tick()

    img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)

    infostr = str(clock.fps()) + ' : '

    if code:
        for i in code:
            curClass = classes[i.classid()]
            infostr = infostr + curClass[0] + ', '
            if curClass[1]:
                uart.write(curClass[1] + 'ga,iruyo\r')
    print(infostr)
model = kpu.deinit(task)

uart.deinit ()
del uart
