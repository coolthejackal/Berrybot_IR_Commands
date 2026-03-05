##########Libraries##########
from machine import Pin, PWM, Timer, ADC
import time, utime
from berrybot import TB6612, NEC_ABC, NEC_16, WS2812, IR_RX, BLE, LEDMatrix, HCSR04
##########Pin Defination##########
TX_PIN = 0
RX_PIN = 1
MOTOR_A1_PIN = 25
MOTOR_A2_PIN = 24
MOTOR_B1_PIN = 22
MOTOR_B2_PIN = 23
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21
IR_PIN = 20

Max_Speed = 65000
Mid_Speed = 50000
Low_Speed = 47000

rowPins= [7, 11, 12, 13, 17]
colPins = [18, 19, 16, 2, 3]

left =      [0x04,0x0E,0x15,0x04,0x04]
right =     [0x04,0x04,0x15,0x0E,0x04]
forward =   [0x04,0x02,0x1F,0x02,0x04]
backward =  [0x04,0x08,0x1F,0x08,0x04]
ir_img =    [0x1F,0x11,0x1F,0x11,0x1F]
#############Variables####################
ir_data = 0
data_rcvd = False

berryMode = 0
STOP =  0
FWD =   2
BWD =   1
RIGHT = 4
LEFT =  3

matrix = LEDMatrix(rowPins, colPins)

matrix.set_led_matrix()
myTimer = Timer(-1)
myTimer.init(period=5, mode=Timer.PERIODIC, callback=matrix.tick)


##########Function Declaration##########
def ir_callback(data, addr, ctrl):
    global berryMode
    global ir_data
    global ir_addr, data_rcvd
    if data > 0:
        ir_data = data
        ir_addr = addr
        print('Data {:02x} Addr {:04x}'.format(data, addr))
        data_rcvd = True
        
ir = NEC_16(Pin(IR_PIN, Pin.IN), ir_callback)
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)

while True:
    matrix.draw_screen(ir_img)
    motor.move(STOP,0)
    if data_rcvd == True:
        data_rcvd = False
        if ir_data == IR_RX.number_up:
            matrix.draw_screen(forward)
            motor.move(FWD, Mid_Speed)
            time.sleep_ms(300)
            motor.move(STOP,0)
        elif ir_data == IR_RX.number_down:
            matrix.draw_screen(backward)
            motor.move(BWD, Mid_Speed)
            time.sleep_ms(300)
            motor.move(STOP,0)
        elif ir_data == IR_RX.number_left:
            matrix.draw_screen(left)
            motor.move(LEFT, Mid_Speed)
            time.sleep_ms(130)
            motor.move(STOP,0)
        elif ir_data == IR_RX.number_right:
            matrix.draw_screen(right)
            motor.move(RIGHT, Mid_Speed)
            time.sleep_ms(130)
            motor.move(STOP,0)
