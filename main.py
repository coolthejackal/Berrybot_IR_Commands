##########Libraries##########
import machine
from machine import I2C, Pin, PWM, UART, Timer, ADC, WDT
import time, utime
from time import sleep
import random
from berrybot import TB6612, NEC_ABC, NEC_16, WS2812, IR_RX, BLE, LEDMatrix, HCSR04
##########Pin Defination##########
TX_PIN = 0
RX_PIN = 1
BUZZER_PIN = 14
NEOPIXEL_PIN = 6
MOTOR_A1_PIN = 24
MOTOR_A2_PIN = 25
MOTOR_B1_PIN = 23
MOTOR_B2_PIN = 22
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21
LDR_L_PIN = 29
LDR_R_PIN = 28
LEFT_TRACKER = 26
RIGHT_TRACKER = 27
IR_PIN = 20
ECHO_PIN = 9
TRIG_PIN = 8
rowPins= [7, 11, 12, 13, 17]
colPins = [18, 19, 16, 2, 3]
rgb_value = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0], [255, 0, 255], [0, 255, 255],[255, 127, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
#############Variables####################
STOP =  0
FWD =   1
BWD =   2
RIGHT = 3
LEFT =  4

Max_Speed = 65000
Mid_Speed = 50000
Low_Speed = 47000

TRACKER_THRESHOLD = 50000
LDR_THRESHOLD = 250
LDR_TOLERANCE = 5000

ir_data = 0
data_rcvd = False
ir_is_repeat = False
ledRow = 0
ledArrayBuffer = bytearray(5)
user_led_matrix = bytearray(5)
ble_buf = bytes()
berryMode = 0
lastButtonState = 0
led_matrix_status = 0
rgb_status = 1
bluetooth_mode = 0
directionStt = STOP
oldDirection = STOP
leftSensorValue = 0
rightSensorValue = 0
counter=0
right_counter=0
left_counter=0
distance = 0
left_turning = False
right_turning = False
sumo_mode_active = False
obstacle_mode_active = False
flow_mode_active = False
guard_mode_active = False
obstacle_turn_direction = RIGHT
obstacle_turn_end_ms = 0
obstacle_state = STOP
obstacle_last_toggle_ms = 0
OBSTACLE_TOGGLE_COOLDOWN_MS = 550
RAIDER_REPEAT_OFF_MIN_MS = 900
raider_mode_started_ms = 0
RAIDER_MODE_NAME = "Raider Mode"
BUTTON_DEBOUNCE_MS = 250
button_last_irq_ms = 0
BUTTON_MIN_PRESS_MS = 30
BUTTON_LONG_PRESS_MS = 1200
BUTTON_BOOTSEL_PRESS_MS = 2200
button_press_start_ms = 0
pending_soft_reset = False
pending_bootloader = False

# Raider mode mini AI state
RAIDER_CLEAR_DISTANCE_CM = 24
RAIDER_NEAR_DISTANCE_CM = 18
RAIDER_BLOCKED_DISTANCE_CM = 11
RAIDER_EXPLORE_CHANCE = 0.22
RAIDER_STUCK_TRIGGER = 3
RAIDER_SAME_DIR_LIMIT = 18
RAIDER_TURN_CHECK_MS = 6
raider_score_left = 3
raider_score_right = 3
raider_score_back = 2
raider_last_distance = None
raider_stuck_count = 0
raider_same_dir_count = 0
raider_last_move = STOP
raider_last_action = STOP
raider_last_action_distance = None
raider_preset = 2  # 1:cautious 2:balanced 3:aggressive
ok_hold_active = False
ok_hold_until_ms = 0
OK_HOLD_WINDOW_MS = 1400
MODE_TOGGLE_DEBOUNCE_MS = 320
last_mode_toggle_code = -1
last_mode_toggle_ms = 0

# Flow / Guard mode state
flow_turn_direction = RIGHT
flow_action_end_ms = 0
guard_action_end_ms = 0
guard_last_alert_ms = 0
GUARD_ALERT_COOLDOWN_MS = 1200

# Follow mode mini AI state
FOLLOW_BASE_TARGET_CM = 22
FOLLOW_SPEED = 36000
follow_target_cm = FOLLOW_BASE_TARGET_CM
follow_last_distance = None
follow_stable_count = 0

main_error_count = 0
MAX_MAIN_ERRORS = 5
wdt = None
#############Led Matrix###################
smile =     [0x02,0x19,0x01,0x19,0x02]
yes =       [0x00,0x01,0x02,0x14,0x08]
no =        [0x11,0x0A,0x04,0x0A,0x11]
left =      [0x04,0x0E,0x15,0x04,0x04]
right =     [0x04,0x04,0x15,0x0E,0x04]
forward =   [0x04,0x02,0x1F,0x02,0x04]
backward =  [0x04,0x08,0x1F,0x08,0x04]
empty =     [0x00,0x00,0x00,0x00,0x00]
full =      [0x1F,0x1F,0x1F,0x1F,0x1F]
heart =     [0x0A,0x1F,0x1F,0x0E,0x04]
sad =       [0x01,0x0A,0x02,0x0A,0x01]
tracker =   [0x07,0x04,0x1F,0x04,0x07]
bluetooth = [0x1F,0x00,0x0E,0x00,0x04]
ir_img =    [0x1F,0x11,0x1F,0x11,0x1F]
sunny =     [0x15,0x0E,0x1F,0x0E,0x15]
sonic =     [0x04,0x0E,0x1B,0x0E,0x04]
triangle =  [0x1F,0x11,0x11,0x0A,0x04]
shield =    [0x04,0x0E,0x1F,0x1B,0x0E]
follow =    [0x0E,0x11,0x0A,0x11,0x0E]
idle =      [0x00,0x0A,0x04,0x0A,0x00]
##########Pin Initialization##########
uart = UART(0, 115200, parity=None, stop = 1, bits = 8, tx=Pin(0), rx=Pin(1),timeout=10)
ble = BLE(uart)
ble.configure()
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
sensor = HCSR04(trigger_pin=8, echo_pin=9, echo_timeout_us=30000)
button = Pin(10, Pin.IN, Pin.PULL_DOWN)
rgb = WS2812(7,NEOPIXEL_PIN,0.2)
# Make RGB effects softer and slower.
rgb.brightness = 0.12
rgb.pixel_interval = 190
matrix = LEDMatrix(rowPins, colPins)
ldr_left = ADC(LDR_L_PIN)
ldr_right = ADC(LDR_R_PIN)
leftSensor = ADC(Pin(LEFT_TRACKER))
rightSensor = ADC(Pin(RIGHT_TRACKER))
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.freq(1000) # Buzzer frequency set to 1 kHz
buzzer.duty_u16(0)
##########Function Declaration##########
def ir_callback(data, addr, ctrl):
    global ir_data
    global ir_addr, data_rcvd, ir_is_repeat
    # REPEAT geldiğinde son geçerli komutu tekrar işle.
    if data == IR_RX.REPEAT and ir_data > 0:
        ir_is_repeat = True
        data_rcvd = True
    elif data > 0:
        ir_is_repeat = False
        ir_data = data
        ir_addr = addr
        print('Data {:02x} Addr {:04x}'.format(data, addr))
        data_rcvd = True
    
def berry_horn(soundType):
    if soundType == 1:
        buzzer.duty_u16(32768)
        time.sleep(0.3)
        buzzer.duty_u16(0)
    elif soundType == 2:
        buzzer.duty_u16(22768)
        time.sleep(0.3)
        buzzer.duty_u16(0)
    elif soundType == 3:
        buzzer.duty_u16(12768)
        time.sleep(0.3)
        buzzer.duty_u16(0)
    elif soundType == 10:
        for i in range (0, 10):
            buzzer.duty_u16(32768)
            time.sleep(0.3)
            buzzer.duty_u16(0)
            time.sleep(0.1)
    elif soundType == 20:
        for i in range (0, 10):
            buzzer.duty_u16(22768)
            time.sleep(0.3)
            buzzer.duty_u16(0)
            time.sleep(0.1)
    elif soundType == 30:
        for i in range (0, 10):
            buzzer.duty_u16(12768)
            time.sleep(0.3)
            buzzer.duty_u16(0)
            time.sleep(0.1)
    elif soundType == 100:  # İleri melody
        buzzer.freq(1200)
        buzzer.duty_u16(32768)
        time.sleep_ms(200)
        buzzer.duty_u16(0)
    elif soundType == 200:  # Sola dön melody
        buzzer.freq(800)
        buzzer.duty_u16(32768)
        time.sleep_ms(150)
        buzzer.duty_u16(0)
        time.sleep_ms(50)
        buzzer.duty_u16(32768)
        time.sleep_ms(150)
        buzzer.duty_u16(0)
    elif soundType == 300:  # Sağa dön melody
        buzzer.freq(1000)
        buzzer.duty_u16(32768)
        time.sleep_ms(100)
        buzzer.duty_u16(0)
        time.sleep_ms(50)
        buzzer.duty_u16(32768)
        time.sleep_ms(100)
        buzzer.duty_u16(0)
        time.sleep_ms(50)
        buzzer.duty_u16(32768)
        time.sleep_ms(100)
        buzzer.duty_u16(0)
    elif soundType == 400:  # Geri gel melody
        buzzer.freq(600)
        for i in range(3):
            buzzer.duty_u16(32768)
            time.sleep_ms(100)
            buzzer.duty_u16(0)
            time.sleep_ms(100)
    elif soundType == 500:  # Kaybettim melody
        buzzer.freq(400)
        buzzer.duty_u16(32768)
        time.sleep_ms(200)
        buzzer.duty_u16(0)
        time.sleep_ms(100)
        buzzer.duty_u16(32768)
        time.sleep_ms(200)
        buzzer.duty_u16(0)
        time.sleep_ms(100)
        buzzer.duty_u16(32768)
        time.sleep_ms(400)
        buzzer.duty_u16(0)

def sumo_attack():
    global ir_data, data_rcvd, ir_is_repeat, sumo_mode_active
    matrix.draw_screen(triangle)
    print("[SUMO] attack loop started")

    COLLISION_DISTANCE_CM = 8
    APPROACH_DELTA_CM = 6

    def filtered_distance(samples=5, wait_ms=12):
        values = []
        for _ in range(samples):
            feed_watchdog()
            try:
                d = sensor.distance_cm()
                if d > 0 and d < 300:
                    values.append(d)
            except:
                pass
            time.sleep_ms(wait_ms)

        if len(values) == 0:
            return None

        values.sort()
        return values[len(values) // 2]

    # Sumo modunda sadece kırmızı renk
    bright_red = (255, 0, 0)      # Parlak kırmızı (ileri)
    dimmed_red = (64, 0, 0)       # Dimli kırmızı (geri)
    normal_red = (128, 0, 0)      # Normal kırmızı (durmuş)

    def process_sumo_command():
        global data_rcvd, ir_data, ir_is_repeat, sumo_mode_active
        if not data_rcvd:
            return False

        received_code = ir_data
        received_is_repeat = ir_is_repeat
        data_rcvd = False
        ir_is_repeat = False

        if received_code == IR_RX.number_8:
            if (not received_is_repeat) and allow_mode_toggle(IR_RX.number_8):
                sumo_mode_active = False
                motor.move(STOP, 0)
                show_idle_mode_icon()
                print("[SUMO] manual off by key 8")
                print("Sumo Mode OFF")
                return True
            return False

        # Sumo açıkken diğer tuşları görmezden gel.
        print("[SUMO] ignored key in sumo mode: {:02x}".format(received_code))
        return False

    while sumo_mode_active:
        feed_watchdog()
        if process_sumo_command():
            break

        try:
            # Aşama 1: İleri hareket (4-5cm) - Parlak kırmızı
            motor.move(FWD, Max_Speed)
            for i in range(10):  # ~400ms ileri
                feed_watchdog()
                if process_sumo_command():
                    return
                # Parlak kırmızı göster
                for j in range(6):
                    rgb.pixels_set(j, bright_red)
                rgb.pixels_show()
                time.sleep_ms(40)

            # Aşama 2: Duruverin önüne kontrol - Normal kırmızı
            motor.move(STOP, 0)
            for j in range(6):
                rgb.pixels_set(j, normal_red)
            rgb.pixels_show()
            time.sleep_ms(100)

            old_distance = filtered_distance()
            print("[SUMO] old_distance={}".format(old_distance))

            # Aşama 3: Biraz geri git (hazırlanma) - Dimli kırmızı
            motor.move(BWD, Max_Speed)
            for k in range(10):  # 400ms geri (2x)
                feed_watchdog()
                if process_sumo_command():
                    return
                for j in range(6):
                    rgb.pixels_set(j, dimmed_red)
                rgb.pixels_show()
                time.sleep_ms(40)

            # Aşama 4: Yeniden ileri gel (toslamaya hazırlan) - Parlak kırmızı
            motor.move(FWD, Max_Speed)
            min_forward_distance = 999
            for k in range(10):  # 400ms ileri (2x)
                feed_watchdog()
                if process_sumo_command():
                    return
                d_step = filtered_distance(samples=1, wait_ms=0)
                if d_step is not None and d_step < min_forward_distance:
                    min_forward_distance = d_step
                for j in range(6):
                    rgb.pixels_set(j, bright_red)
                rgb.pixels_show()
                time.sleep_ms(40)

            new_distance = filtered_distance()
            print("[SUMO] new_distance={} min_forward_distance={}".format(new_distance, min_forward_distance))

            if min_forward_distance == 999:
                min_forward_distance = new_distance if new_distance is not None else 999

            # Çarpışma kontrolü
            collision_detected = False
            if min_forward_distance <= COLLISION_DISTANCE_CM:
                collision_detected = True
            elif old_distance is not None and new_distance is not None:
                if (old_distance - new_distance) >= APPROACH_DELTA_CM and new_distance <= 15:
                    collision_detected = True

            print("[SUMO] collision_detected={}".format(collision_detected))

            if not collision_detected:
                # Çarpışma yoksa ses
                motor.move(STOP, 0)
                berry_horn(100)
                # Normal kırmızı göster
                for j in range(6):
                    rgb.pixels_set(j, normal_red)
                rgb.pixels_show()
                time.sleep_ms(300)
            else:
                # Çarpışma oldu!
                print("[SUMO] collision branch entered (lost)")
                matrix.draw_screen(no)
                berry_horn(500)  # Kaybettim sesi

                # Şaşkınlıktan önce çok hafif geri kaçış.
                if process_sumo_command():
                    return
                motor.move(BWD, Mid_Speed)
                for j in range(6):
                    rgb.pixels_set(j, dimmed_red)
                rgb.pixels_show()
                time.sleep_ms(140)

                # Şaşkın hareket - Hızlı yanıp sönen kırmızı
                for _ in range(4):
                    feed_watchdog()
                    if process_sumo_command():
                        return

                    motor.move(RIGHT, Max_Speed)
                    for j in range(6):
                        rgb.pixels_set(j, bright_red)
                    rgb.pixels_show()
                    time.sleep_ms(300)

                    if process_sumo_command():
                        return

                    motor.move(LEFT, Max_Speed)
                    for j in range(6):
                        rgb.pixels_set(j, dimmed_red)
                    rgb.pixels_show()
                    time.sleep_ms(300)

                motor.move(STOP, 0)
                # Toparlanma sırasında normal kırmızı
                matrix.draw_screen(no)
                for j in range(6):
                    rgb.pixels_set(j, normal_red)
                rgb.pixels_show()

                # Kaybedince oyunu durdur ve bekleme moduna dön.
                print("[SUMO] lost -> waiting 1s then restart")
                for _ in range(10):
                    feed_watchdog()
                    if process_sumo_command():
                        return
                    time.sleep_ms(100)

                if not sumo_mode_active:
                    break

                matrix.draw_screen(triangle)
                continue

        except:
            print("[SUMO] exception in loop")
            motor.move(STOP, 0)
            break

        time.sleep_ms(50)

def raider_mode_step():
    global obstacle_mode_active, obstacle_turn_direction, obstacle_turn_end_ms, obstacle_state, data_rcvd
    global raider_score_left, raider_score_right, raider_score_back
    global raider_last_distance, raider_stuck_count, raider_same_dir_count, raider_last_move
    global raider_last_action, raider_last_action_distance

    if not obstacle_mode_active:
        return

    feed_watchdog()

    now = time.ticks_ms()

    # Mevcut manevra bitmeden yeni karar verme.
    if time.ticks_diff(obstacle_turn_end_ms, now) > 0:
        return

    def filtered_distance(samples=3, wait_ms=8):
        values = []
        for _ in range(samples):
            try:
                d = sensor.distance_cm()
                if d > 0 and d < 300:
                    values.append(d)
            except:
                pass
            if wait_ms > 0:
                time.sleep_ms(wait_ms)

        if len(values) == 0:
            return None

        values.sort()
        return values[len(values) // 2]

    def clamp_score(x):
        if x < 1:
            return 1
        if x > 8:
            return 8
        return x

    def pick_weighted_action(prefer_escape=False):
        # Basit RL yaklaşımı: başarı puanı yüksek yön daha sık seçilir,
        # ancak yerel minimuma takılmamak için keşif (random) korunur.
        actions = [LEFT, RIGHT, BWD]
        if prefer_escape and raider_last_move in actions:
            actions = [a for a in actions if a != raider_last_move]
            if len(actions) == 0:
                actions = [LEFT, RIGHT, BWD]

        if random.random() < RAIDER_EXPLORE_CHANCE:
            return random.choice(actions)

        weighted = []
        for a in actions:
            if a == LEFT:
                w = raider_score_left
            elif a == RIGHT:
                w = raider_score_right
            else:
                w = raider_score_back
            for _ in range(w):
                weighted.append(a)

        return random.choice(weighted)

    try:
        distance = filtered_distance()
    except:
        motor.move(STOP, 0)
        obstacle_mode_active = False
        return

    if distance is None:
        motor.move(STOP, 0)
        matrix.draw_screen(sonic)
        obstacle_state = STOP
        obstacle_turn_end_ms = time.ticks_add(now, 120)
        return

    # Son manevranın sonucuna göre puan güncelle (mini AI öğrenme).
    if raider_last_action in (LEFT, RIGHT, BWD) and raider_last_action_distance is not None:
        delta = distance - raider_last_action_distance
        if delta >= 2:
            if raider_last_action == LEFT:
                raider_score_left = clamp_score(raider_score_left + 1)
            elif raider_last_action == RIGHT:
                raider_score_right = clamp_score(raider_score_right + 1)
            else:
                raider_score_back = clamp_score(raider_score_back + 1)
        elif delta <= -1 or distance <= RAIDER_NEAR_DISTANCE_CM:
            if raider_last_action == LEFT:
                raider_score_left = clamp_score(raider_score_left - 1)
            elif raider_last_action == RIGHT:
                raider_score_right = clamp_score(raider_score_right - 1)
            else:
                raider_score_back = clamp_score(raider_score_back - 1)
        raider_last_action = STOP
        raider_last_action_distance = None

    # Sıkışma / havada asılı kalma benzeri durum algısı.
    if raider_last_distance is not None:
        if abs(distance - raider_last_distance) < 0.8 and obstacle_state in (FWD, LEFT, RIGHT, BWD):
            raider_stuck_count += 1
        else:
            raider_stuck_count = max(0, raider_stuck_count - 1)
    raider_last_distance = distance

    if distance > RAIDER_CLEAR_DISTANCE_CM and raider_stuck_count < RAIDER_STUCK_TRIGGER:
        motor.move(FWD, Max_Speed)
        matrix.draw_screen(forward)
        obstacle_state = FWD

        if raider_last_move == FWD:
            raider_same_dir_count += 1
        else:
            raider_same_dir_count = 0
        raider_last_move = FWD

        # Çok uzun süre aynı yönde ilerlediyse mutlaka yön değiştir.
        if raider_same_dir_count >= RAIDER_SAME_DIR_LIMIT:
            if pause_and_check_control():
                return
            forced = random.choice([LEFT, RIGHT])
            if forced == LEFT:
                motor.move(LEFT, Max_Speed)
                matrix.draw_screen(left)
                berry_horn(200)
                obstacle_state = LEFT
            else:
                motor.move(RIGHT, Max_Speed)
                matrix.draw_screen(right)
                berry_horn(300)
                obstacle_state = RIGHT
            obstacle_turn_end_ms = time.ticks_add(now, 320)
            raider_same_dir_count = 0
    else:
        # Engel yakın veya sıkıştı: mini AI ile kaçış manevrası seç.
        prefer_escape = distance <= RAIDER_BLOCKED_DISTANCE_CM or raider_stuck_count >= RAIDER_STUCK_TRIGGER
        maneuver = pick_weighted_action(prefer_escape=prefer_escape)

        raider_same_dir_count = 0

        if pause_and_check_control():
            return

        if maneuver == RIGHT:
            motor.move(RIGHT, Max_Speed)
            matrix.draw_screen(right)
            berry_horn(300)
            obstacle_turn_direction = LEFT
            obstacle_state = RIGHT
            obstacle_turn_end_ms = time.ticks_add(now, 430)
            raider_last_move = RIGHT
            raider_last_action = RIGHT
            raider_last_action_distance = distance
        elif maneuver == LEFT:
            motor.move(LEFT, Max_Speed)
            matrix.draw_screen(left)
            berry_horn(200)
            obstacle_turn_direction = RIGHT
            obstacle_state = LEFT
            obstacle_turn_end_ms = time.ticks_add(now, 430)
            raider_last_move = LEFT
            raider_last_action = LEFT
            raider_last_action_distance = distance
        else:
            motor.move(BWD, Mid_Speed)
            matrix.draw_screen(backward)
            berry_horn(400)
            obstacle_state = BWD
            obstacle_turn_end_ms = time.ticks_add(now, 260)
            raider_last_move = BWD
            raider_last_action = BWD
            raider_last_action_distance = distance

def follow_mode_step():
    global flow_mode_active, flow_action_end_ms, data_rcvd
    global follow_target_cm, follow_last_distance, follow_stable_count

    if not flow_mode_active:
        return

    feed_watchdog()
    now = time.ticks_ms()
    if time.ticks_diff(flow_action_end_ms, now) > 0:
        return

    def filtered_distance(samples=4, wait_ms=8):
        values = []
        for _ in range(samples):
            try:
                d = sensor.distance_cm()
                if d > 0 and d < 300:
                    values.append(d)
            except:
                pass
            if wait_ms > 0:
                time.sleep_ms(wait_ms)

        if len(values) == 0:
            return None
        values.sort()
        return values[len(values) // 2]

    try:
        distance = filtered_distance()
    except:
        motor.move(STOP, 0)
        flow_mode_active = False
        return

    if distance is None or distance <= 0:
        motor.move(STOP, 0)
        matrix.draw_screen(follow)
        flow_action_end_ms = time.ticks_add(now, 160)
        return

    # Mini AI: mesafe sabitse hedefi yavaşça adapte et.
    if follow_last_distance is not None and abs(distance - follow_last_distance) < 1.0:
        follow_stable_count += 1
    else:
        follow_stable_count = max(0, follow_stable_count - 1)
    follow_last_distance = distance

    if follow_stable_count >= 4 and 14 <= distance <= 34:
        follow_target_cm = int((follow_target_cm * 4 + distance) / 5)

    too_close = follow_target_cm - 5
    hold_low = follow_target_cm - 2
    hold_high = follow_target_cm + 4
    far = follow_target_cm + 7

    # Follow mode: sadece ileri/geri/dur.
    if distance <= too_close:
        motor.move(BWD, FOLLOW_SPEED)
        matrix.draw_screen(backward)
        flow_action_end_ms = time.ticks_add(now, 260)
    elif distance >= far:
        motor.move(FWD, FOLLOW_SPEED)
        matrix.draw_screen(forward)
        flow_action_end_ms = time.ticks_add(now, 250)
    elif hold_low <= distance <= hold_high or follow_stable_count >= 5:
        motor.move(STOP, 0)
        matrix.draw_screen(follow)
        flow_action_end_ms = time.ticks_add(now, 180)
    else:
        motor.move(STOP, 0)
        matrix.draw_screen(follow)
        flow_action_end_ms = time.ticks_add(now, 170)

def guard_mode_step():
    global guard_mode_active, guard_action_end_ms, guard_last_alert_ms, data_rcvd

    if not guard_mode_active:
        return

    feed_watchdog()
    now = time.ticks_ms()
    if time.ticks_diff(guard_action_end_ms, now) > 0:
        return

    try:
        distance = sensor.distance_cm()
    except:
        motor.move(STOP, 0)
        guard_mode_active = False
        return

    # Nöbet modu: normalde sabit bekle.
    motor.move(STOP, 0)

    if distance is None or distance <= 0:
        matrix.draw_screen(shield)
        guard_action_end_ms = time.ticks_add(now, 140)
        return

    if distance <= 15:
        matrix.draw_screen(no)
        if time.ticks_diff(now, guard_last_alert_ms) > GUARD_ALERT_COOLDOWN_MS:
            berry_horn(3)
            guard_last_alert_ms = now

        # Yakın tehditte kısa kaçış dene, sonra tekrar nöbete dön.
        motor.move(BWD, Mid_Speed)
        time.sleep_ms(160)
        if random.choice([LEFT, RIGHT]) == LEFT:
            motor.move(LEFT, Mid_Speed)
            matrix.draw_screen(left)
        else:
            motor.move(RIGHT, Mid_Speed)
            matrix.draw_screen(right)
        time.sleep_ms(180)
        motor.move(STOP, 0)
        guard_action_end_ms = time.ticks_add(now, 280)
    else:
        matrix.draw_screen(shield)
        guard_action_end_ms = time.ticks_add(now, 120)
    
def buttonInterruptHandler(event):    # Interrupt event, that will work when button is pressed
    global berryMode, lastButtonState, button_last_irq_ms
    global button_press_start_ms, pending_soft_reset, pending_bootloader

    now = time.ticks_ms()
    button_state = button.value()

    if button_state == 1:
        button_press_start_ms = now
        return

    if button_press_start_ms == 0:
        return

    press_ms = time.ticks_diff(now, button_press_start_ms)
    button_press_start_ms = 0

    if press_ms < BUTTON_MIN_PRESS_MS:
        return

    # Çok uzun basış: BOOTSEL/bootloader moduna geç.
    if press_ms >= BUTTON_BOOTSEL_PRESS_MS:
        pending_bootloader = True
        return

    if press_ms >= BUTTON_LONG_PRESS_MS:
        pending_soft_reset = True
        return

    if time.ticks_diff(now, button_last_irq_ms) < BUTTON_DEBOUNCE_MS:
        return

    button_last_irq_ms = now
    lastButtonState = 1
    berryMode = 0 if berryMode >= 1 else berryMode + 1
        
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def constrain(x, min_val, max_val):
    return max(min(x, max_val), min_val)

def drawScreen(buffer):
    led_array_buffer = [0] * 5  # Initialize the LED array buffer with 5 elements.
    for i in range(5):
        led_array_buffer[i] = buffer[i]
    return led_array_buffer  # Return the updated LED array buffer.

def feed_watchdog():
    global wdt
    if wdt is not None:
        try:
            wdt.feed()
        except:
            pass

def clear_rgb_leds():
    try:
        rgb.pixels_fill((0, 0, 0))
        rgb.pixels_show()
    except:
        pass

def show_idle_mode_icon():
    try:
        matrix.draw_screen(idle)
    except:
        pass

def safe_idle_state(show_ir_icon=True):
    global left_turning, right_turning, sumo_mode_active, obstacle_mode_active
    global flow_mode_active, guard_mode_active
    global obstacle_turn_end_ms, bluetooth_mode, flow_action_end_ms, guard_action_end_ms
    global follow_target_cm, follow_last_distance, follow_stable_count

    left_turning = False
    right_turning = False
    sumo_mode_active = False
    obstacle_mode_active = False
    flow_mode_active = False
    guard_mode_active = False
    obstacle_turn_end_ms = 0
    flow_action_end_ms = 0
    guard_action_end_ms = 0
    follow_target_cm = FOLLOW_BASE_TARGET_CM
    follow_last_distance = None
    follow_stable_count = 0
    bluetooth_mode = 0

    try:
        motor.move(STOP, 0)
    except:
        pass

    clear_rgb_leds()

    if show_ir_icon:
        show_idle_mode_icon()
    else:
        try:
            matrix.draw_screen(empty)
        except:
            pass

def boot_led_handshake():
    # Boot doğrulama: önce tüm LED'leri kapat, sonra kısa yeşil heartbeat göster.
    clear_rgb_leds()
    time.sleep_ms(200)

    try:
        for _ in range(2):
            rgb.pixels_fill((0, 80, 0))
            rgb.pixels_show()
            time.sleep_ms(90)
            clear_rgb_leds()
            time.sleep_ms(70)
    except:
        pass

def is_zero_button(code):
    # Bazı kumandalarda 0 tuşu aynı değeri farklı formatlarda gönderebilir.
    return code == IR_RX.number_0 or code == 0x19 or code == 25

def allow_mode_toggle(code):
    global last_mode_toggle_code, last_mode_toggle_ms
    now = time.ticks_ms()
    if code == last_mode_toggle_code and time.ticks_diff(now, last_mode_toggle_ms) < MODE_TOGGLE_DEBOUNCE_MS:
        return False

    last_mode_toggle_code = code
    last_mode_toggle_ms = now
    return True

def turn_with_beep(direction, duration_ms=760, pulse_ms=120, speed=Mid_Speed):
    end_ms = time.ticks_add(time.ticks_ms(), duration_ms)
    buzzer_on = True
    next_pulse_ms = time.ticks_ms()

    if direction == LEFT:
        buzzer.freq(800)
    else:
        buzzer.freq(1000)

    while time.ticks_diff(end_ms, time.ticks_ms()) > 0:
        motor.move(direction, speed)
        now = time.ticks_ms()
        if time.ticks_diff(now, next_pulse_ms) >= 0:
            if buzzer_on:
                buzzer.duty_u16(22000)
            else:
                buzzer.duty_u16(0)
            buzzer_on = not buzzer_on
            next_pulse_ms = time.ticks_add(now, pulse_ms)
        time.sleep_ms(20)

    buzzer.duty_u16(0)
    motor.move(STOP, 0)

def apply_raider_preset(preset_id):
    global raider_preset
    global RAIDER_CLEAR_DISTANCE_CM, RAIDER_NEAR_DISTANCE_CM, RAIDER_BLOCKED_DISTANCE_CM
    global RAIDER_EXPLORE_CHANCE, RAIDER_STUCK_TRIGGER, RAIDER_SAME_DIR_LIMIT

    if preset_id == 1:
        # Cautious: daha erken kaçınır, daha çok yön değiştirir.
        raider_preset = 1
        RAIDER_CLEAR_DISTANCE_CM = 28
        RAIDER_NEAR_DISTANCE_CM = 20
        RAIDER_BLOCKED_DISTANCE_CM = 12
        RAIDER_EXPLORE_CHANCE = 0.30
        RAIDER_STUCK_TRIGGER = 2
        RAIDER_SAME_DIR_LIMIT = 12
        print("Raider preset: Cautious")
    elif preset_id == 2:
        # Balanced: genel kullanım.
        raider_preset = 2
        RAIDER_CLEAR_DISTANCE_CM = 24
        RAIDER_NEAR_DISTANCE_CM = 18
        RAIDER_BLOCKED_DISTANCE_CM = 11
        RAIDER_EXPLORE_CHANCE = 0.22
        RAIDER_STUCK_TRIGGER = 3
        RAIDER_SAME_DIR_LIMIT = 18
        print("Raider preset: Balanced")
    elif preset_id == 3:
        # Aggressive: daha geç kaçınır, daha hızlı ileri gider.
        raider_preset = 3
        RAIDER_CLEAR_DISTANCE_CM = 21
        RAIDER_NEAR_DISTANCE_CM = 16
        RAIDER_BLOCKED_DISTANCE_CM = 9
        RAIDER_EXPLORE_CHANCE = 0.14
        RAIDER_STUCK_TRIGGER = 4
        RAIDER_SAME_DIR_LIMIT = 24
        print("Raider preset: Aggressive")

def handle_raider_preset_combo(received_code):
    global ok_hold_active, ok_hold_until_ms

    now = time.ticks_ms()
    if ok_hold_active and time.ticks_diff(ok_hold_until_ms, now) <= 0:
        ok_hold_active = False

    if received_code == IR_RX.number_ok:
        ok_hold_active = True
        ok_hold_until_ms = time.ticks_add(now, OK_HOLD_WINDOW_MS)
        return False

    if ok_hold_active and received_code in (IR_RX.number_1, IR_RX.number_2, IR_RX.number_3):
        if received_code == IR_RX.number_1:
            apply_raider_preset(1)
            matrix.draw_screen(left)
            berry_horn(1)
        elif received_code == IR_RX.number_2:
            apply_raider_preset(2)
            matrix.draw_screen(forward)
            berry_horn(2)
        else:
            apply_raider_preset(3)
            matrix.draw_screen(right)
            berry_horn(3)

        ok_hold_active = False
        return True

    return False

def toggle_raider_mode():
    global obstacle_mode_active, obstacle_turn_direction, obstacle_turn_end_ms
    global obstacle_state, obstacle_last_toggle_ms
    global raider_mode_started_ms
    global raider_last_distance, raider_stuck_count, raider_same_dir_count, raider_last_move
    global raider_last_action, raider_last_action_distance

    if not obstacle_mode_active:
        now = time.ticks_ms()
        if time.ticks_diff(now, obstacle_last_toggle_ms) < OBSTACLE_TOGGLE_COOLDOWN_MS:
            return
        obstacle_last_toggle_ms = now
        obstacle_mode_active = True
        raider_mode_started_ms = now
        obstacle_turn_direction = RIGHT
        obstacle_turn_end_ms = 0
        obstacle_state = STOP
        raider_last_distance = None
        raider_stuck_count = 0
        raider_same_dir_count = 0
        raider_last_move = STOP
        raider_last_action = STOP
        raider_last_action_distance = None
        matrix.draw_screen(sonic)
        berry_horn(1)
        print(RAIDER_MODE_NAME + " ON")
    else:
        obstacle_last_toggle_ms = 0
        obstacle_mode_active = False
        raider_mode_started_ms = 0
        obstacle_state = STOP
        obstacle_turn_end_ms = 0
        motor.move(STOP, 0)
        show_idle_mode_icon()
        print(RAIDER_MODE_NAME + " OFF")

def toggle_follow_mode():
    global flow_mode_active, guard_mode_active, obstacle_mode_active, sumo_mode_active
    global flow_action_end_ms, follow_target_cm, follow_last_distance, follow_stable_count

    if not flow_mode_active:
        guard_mode_active = False
        obstacle_mode_active = False
        sumo_mode_active = False
        flow_mode_active = True
        flow_action_end_ms = 0
        follow_target_cm = FOLLOW_BASE_TARGET_CM
        follow_last_distance = None
        follow_stable_count = 0
        matrix.draw_screen(follow)
        berry_horn(2)
        print("Follow Mode ON")
    else:
        flow_mode_active = False
        motor.move(STOP, 0)
        show_idle_mode_icon()
        print("Follow Mode OFF")

def toggle_guard_mode():
    global flow_mode_active, guard_mode_active, obstacle_mode_active, sumo_mode_active
    global guard_action_end_ms, guard_last_alert_ms

    if not guard_mode_active:
        flow_mode_active = False
        obstacle_mode_active = False
        sumo_mode_active = False
        guard_mode_active = True
        guard_action_end_ms = 0
        guard_last_alert_ms = 0
        matrix.draw_screen(shield)
        berry_horn(1)
        print("Guard Mode ON")
    else:
        guard_mode_active = False
        motor.move(STOP, 0)
        show_idle_mode_icon()
        print("Guard Mode OFF")

def toggle_sumo_mode():
    global sumo_mode_active, obstacle_mode_active, flow_mode_active, guard_mode_active

    if not sumo_mode_active:
        obstacle_mode_active = False
        flow_mode_active = False
        guard_mode_active = False
        sumo_mode_active = True
        matrix.draw_screen(triangle)
        berry_horn(1)
        print("Sumo Mode ON")
        sumo_attack()
    else:
        sumo_mode_active = False
        motor.move(STOP, 0)
        show_idle_mode_icon()
        print("Sumo Mode OFF")

def cancel_active_operations(show_icon=True):
    global data_rcvd
    safe_idle_state(show_ir_icon=show_icon)
    data_rcvd = False

def pause_and_check_control(wait_ms=RAIDER_TURN_CHECK_MS):
    # Manevra öncesi çok kısa bekleyip yeni komut geldi mi kontrol et.
    global data_rcvd
    motor.move(STOP, 0)
    time.sleep_ms(wait_ms)

    if not data_rcvd:
        return False

    # Komutu burada tüketmeyelim; remote() merkezi olarak işleyecek.
    return True

##########Modes##########
def remote():
    global data_rcvd, ir_data, ir_is_repeat, left_turning, right_turning, sumo_mode_active, obstacle_mode_active
    global flow_mode_active, guard_mode_active, flow_turn_direction, flow_action_end_ms
    global guard_action_end_ms, guard_last_alert_ms
    global obstacle_turn_direction, obstacle_turn_end_ms, obstacle_state, obstacle_last_toggle_ms
    global raider_mode_started_ms
    global ok_hold_active, ok_hold_until_ms
    if data_rcvd == True:
        received_code = ir_data
        received_is_repeat = ir_is_repeat
        data_rcvd = False
        ir_is_repeat = False

        # Mod toggle tuşlarını her zaman önce işle.
        if is_zero_button(received_code):
            if received_is_repeat:
                # Bazı kumandalarda ikinci 0 basışı sadece REPEAT dönebilir.
                # Raider açıksa ve mod yeterince uzun süredir aktifse, REPEAT ile kapatmaya izin ver.
                if obstacle_mode_active and time.ticks_diff(time.ticks_ms(), raider_mode_started_ms) >= RAIDER_REPEAT_OFF_MIN_MS:
                    toggle_raider_mode()
                return
            if allow_mode_toggle(IR_RX.number_0):
                toggle_raider_mode()
            return

        if received_code == IR_RX.number_7:
            if received_is_repeat:
                return
            if allow_mode_toggle(IR_RX.number_7):
                toggle_follow_mode()
            return

        if received_code == IR_RX.number_9:
            if received_is_repeat:
                return
            if allow_mode_toggle(IR_RX.number_9):
                toggle_guard_mode()
            return

        if received_code == IR_RX.number_8:
            if received_is_repeat:
                return
            if allow_mode_toggle(IR_RX.number_8):
                toggle_sumo_mode()
            return

        # Aktif moddayken diğer tuşlar tüm otomatik modları güvenli kapatır.
        if obstacle_mode_active or flow_mode_active or guard_mode_active or sumo_mode_active:
            cancel_active_operations(show_icon=True)
            return

        # OK + (1/2/3) ile Raider preset değiştir.
        if handle_raider_preset_combo(received_code):
            return

        if received_code == IR_RX.number_up:
            matrix.draw_screen(forward)
            motor.move(FWD, Max_Speed)
            time.sleep_ms(500)
            motor.move(STOP,0)
        elif received_code == IR_RX.number_down:
            matrix.draw_screen(backward)
            motor.move(BWD, Max_Speed)
            time.sleep_ms(500)
            motor.move(STOP,0)
        elif received_code == IR_RX.number_left:
            matrix.draw_screen(left)
            motor.move(LEFT, Max_Speed)
            time.sleep_ms(130)
            motor.move(STOP,0)
        elif received_code == IR_RX.number_right:
            matrix.draw_screen(right)
            motor.move(RIGHT, Max_Speed)
            time.sleep_ms(130)
            motor.move(STOP,0)
        elif received_code == IR_RX.number_ok:
            matrix.draw_screen(smile)
            berry_horn(1)
        elif received_code == IR_RX.number_star:
            matrix.draw_screen(yes)
            berry_horn(2)
        elif received_code == IR_RX.number_sharp:
            matrix.draw_screen(no)
            berry_horn(3)
        elif received_code == IR_RX.number_1:
            matrix.draw_screen(left)
            left_turning = False
            right_turning = False
            turn_with_beep(LEFT, duration_ms=760, pulse_ms=120, speed=Mid_Speed)
        elif received_code == IR_RX.number_3:
            matrix.draw_screen(right)
            right_turning = False
            left_turning = False
            turn_with_beep(RIGHT, duration_ms=760, pulse_ms=120, speed=Mid_Speed)
        elif received_code == IR_RX.number_2:
            matrix.draw_screen(forward)
            timeout_counter = 0
            max_timeout = 200  # 10 saniye maksimum
            while True:
                feed_watchdog()
                try:
                    distance = sensor.distance_cm()
                    if distance > 15:
                        motor.move(FWD, Max_Speed)
                        print(distance)
                        time.sleep_ms(100)
                        timeout_counter += 1
                    else:
                        motor.move(STOP, 0)
                        time.sleep_ms(200)
                        print("Bitti")
                        break
                except:
                    motor.move(STOP, 0)
                    break

                # Güvenlik timeout
                if timeout_counter > max_timeout:
                    motor.move(STOP, 0)
                    break
        elif received_code == IR_RX.number_5:
            matrix.draw_screen(backward)
            reverse_stop_distance_cm = 15
            reverse_max_steps = 30  # 30 x 50ms ~= 1.5s
            reverse_min_steps = 6
            buzzer.freq(600)

            for step in range(reverse_max_steps):
                feed_watchdog()
                motor.move(BWD, Max_Speed)
                if step % 2 == 0:
                    buzzer.duty_u16(22000)
                else:
                    buzzer.duty_u16(0)
                time.sleep_ms(50)

                try:
                    distance = sensor.distance_cm()
                except:
                    break

                # El/engel sensöre yaklaştığında anında dur.
                if step >= reverse_min_steps and distance <= reverse_stop_distance_cm:
                    break

            buzzer.duty_u16(0)
            motor.move(STOP,0)
        elif received_code == IR_RX.number_4:
            matrix.draw_screen(heart)
            berry_horn(2)
            time.sleep_ms(1000)
        elif received_code == IR_RX.number_6:
            matrix.draw_screen(sad)
            berry_horn(3)
            time.sleep_ms(1000)
        elif received_code == IR_RX.number_8:
            pass
            
def lineTracker():
    global leftSensorValue, rightSensorValue, directionStt, oldDirection
    leftSensorValue = leftSensor.read_u16()
    rightSensorValue = rightSensor.read_u16()
    time.sleep(0.02)
    
    if leftSensorValue >= TRACKER_THRESHOLD and rightSensorValue >= TRACKER_THRESHOLD:
        directionStt = FWD
    elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue > TRACKER_THRESHOLD:
        directionStt = RIGHT
    elif leftSensorValue > TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
        directionStt = LEFT
    elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD and directionStt != STOP:
        directionStt = BWD
        
    if directionStt != oldDirection:
        oldDirection = directionStt
        if directionStt == FWD:
            motor.move(FWD, Mid_Speed)
        elif directionStt == RIGHT:
            motor.move(RIGHT, Mid_Speed)
        elif directionStt == LEFT:
            motor.move(LEFT, Mid_Speed)
        elif directionStt == BWD:
            motor.move(BWD, Low_Speed)
            
def lightTracker():
    LDR_L = ldr_left.read_u16()
    LDR_R = ldr_right.read_u16()
    sleep(0.02)
    distance = sensor.distance_cm()
    sleep(0.02)
    
    if LDR_L >= LDR_THRESHOLD and LDR_R >= LDR_THRESHOLD:
        if distance < 10:
            motor.move(STOP,0)
            sleep(0.02)
            motor.move(LEFT, Mid_Speed)
            sleep(0.5)
            motor.move(STOP, 0)
        elif (LDR_R - LDR_L) >= LDR_TOLERANCE:
            motor.move(RIGHT, Mid_Speed)
        elif (LDR_L - LDR_R) >= LDR_TOLERANCE:
            motor.move(LEFT, Mid_Speed)   
        elif ( LDR_L >= 10000 and LDR_R >= 10000):
            motor.move(FWD, Max_Speed)
        else:
            motor.move(STOP, 0)
    else:
        motor.move(STOP, 0)
        
def sumo():
    global distance, leftSensorValue, rightSensorValue, counter
    
    distance = sensor.distance_cm()
    leftSensorValue = leftSensor.read_u16()
    rightSensorValue = rightSensor.read_u16()
    sleep(0.02)
    if distance <= 15:
        if leftSensorValue >= TRACKER_THRESHOLD or rightSensorValue >= TRACKER_THRESHOLD:
            motor.move(BWD, Mid_Speed)
            sleep(0.5)
        elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
            motor.move(FWD, Mid_Speed)
            sleep(0.5)
        else:
            motor.move(STOP, 0)
    else:
        if leftSensorValue >= TRACKER_THRESHOLD or rightSensorValue >= TRACKER_THRESHOLD:
            motor.move(BWD, Mid_Speed)
            sleep(0.1)
        elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
            counter = counter + 1
            if counter == 3:
                motor.move(FWD, Mid_Speed)
                sleep(0.1)
                counter = 0
            else: 
                motor.move(LEFT, Mid_Speed)
                sleep(0.1)
                motor.move(STOP, 0)
        else:
            motor.move(STOP, 0)
            
def sonic_mode():
    global counter, right_counter, left_counter
    
    if (sensor.distance_cm()) > (12):
        motor.move(FWD, Max_Speed)
    else:
        motor.move(STOP, 0)
        time.sleep((0.5))
        motor.move(BWD, Max_Speed)
        time.sleep((0.1))
        motor.move(STOP, 0)
        time.sleep((0.2))
        motor.move(LEFT, Mid_Speed)
        if left_counter==0:
            time.sleep((0.5))
            left_counter=left_counter+1
        else:
            time.sleep((1))
            left_counter=0
        motor.move(STOP, 0)
        time.sleep((0.5))
#########################################
ir = NEC_16(Pin(IR_PIN, Pin.IN), ir_callback)
button.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=buttonInterruptHandler)
matrix.set_led_matrix()
myTimer = Timer(-1)
myTimer.init(period=5, mode=Timer.PERIODIC, callback=matrix.tick)
wdt = WDT(timeout=8000)

# Güvenli başlangıç: açılışta tüm çıkışları temizle.
safe_idle_state(show_ir_icon=False)
time.sleep_ms(80)

start_time = time.ticks_ms()  # Başlangıç zamanını al
time.sleep(0.5)  # 0.5 saniye bekle
elapsed_time = time.ticks_diff(time.ticks_ms(), start_time)

# Cihaz başlangıcında daha yavaş bir geçişle 3 kere kırmızı yanıp sönsün
rgb.startup_animation(duration=320)

# Kırmızı başlangıç animasyonundan sonra tek heartbeat göster.
boot_led_handshake()
show_idle_mode_icon()


while True:
    try:
        feed_watchdog()

        if pending_bootloader:
            pending_bootloader = False
            print("Bootloader mode requested from button.")
            safe_idle_state(show_ir_icon=False)
            time.sleep_ms(80)
            machine.bootloader()

        if pending_soft_reset:
            pending_soft_reset = False
            print("Soft reset requested from button.")
            safe_idle_state(show_ir_icon=False)
            time.sleep_ms(80)
            machine.reset()

        if rgb_status == 1:
            rgb.color_function(rgb_value)

        if berryMode == 1:  #Bluetooth Mode
            if(led_matrix_status == 0):
                matrix.draw_screen(bluetooth)
            elif (led_matrix_status == 1):
                matrix.draw_screen(user_led_matrix)
                
            if(rgb_status == 1):
                rgb.color_function(rgb_value)
                
            ble_buf = ble.read()
            if ble_buf != b'':
                for i in range(len(ble_buf)):
                    print(ble_buf[i])
                if ((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 99)): #Exit modes
                    bluetooth_mode = 0
                    motor.move(STOP,0)
                elif((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 1) and (ble_buf[3] == 0)): # Neo turn off
                    bluetooth_mode = 0
                    rgb_status = 0
                    clear_rgb_leds()
                elif ((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 90) and (ble_buf[3] == 0)): # Neo turn on
                    bluetooth_mode = 0
                    rgb_status = 1
                elif ((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 2) and (ble_buf[3] == 0)): # Horn
                    bluetooth_mode = 0
                    berry_horn(1)
                elif (((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 4) and (ble_buf[3] == 0)) or (bluetooth_mode == 1)): # Sonic
                    bluetooth_mode = 1
                    sonic_mode()
                elif (((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 8) and (ble_buf[3] == 0)) or (bluetooth_mode == 2)): # Line Tracker
                    bluetooth_mode = 2
                    lineTracker()
                elif (((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 16) and (ble_buf[3] == 0)) or (bluetooth_mode == 3)): # Light Tracker
                    bluetooth_mode = 3
                    lightTracker()
                elif (((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 32) and (ble_buf[3] == 0)) or (bluetooth_mode == 4)): # Sumo
                    bluetooth_mode = 4
                    sumo()
                elif ((ble_buf[0] == 82) and (ble_buf[1] == 3) and (ble_buf[2] != 255) and (ble_buf[3] != 255)): # Move
                    bluetooth_mode = 0
                    led_matrix_status = 2
                    
                    if((ble_buf[2] == 0) and (ble_buf[3] == 0)):
                        led_matrix_status = 0
                        motor.move(STOP,0)
                        matrix.draw_screen(bluetooth)
                    else:
                        joyX = ble_buf[2]
                        joyY = ble_buf[3]

                        mappedX = map_value(joyX, 0, 255, -255, 255)
                        mappedY = map_value(joyY, 0, 255, -255, 255)
                        leftMotorSpeed = mappedY + mappedX
                        rightMotorSpeed = mappedY - mappedX

                        leftMotorSpeed = constrain(leftMotorSpeed, -255, 255)
                        rightMotorSpeed = constrain(rightMotorSpeed, -255, 255)
                        
                        motor.setMotorSpeed(leftMotorSpeed, rightMotorSpeed)
                    
                elif ((ble_buf[0] == 82) and (ble_buf[1] == 7)): # RGB Menu
                    bluetooth_mode = 0
                    
                    rgb_value[ble_buf[2]-1][0] = ble_buf[3]
                    rgb_value[ble_buf[2]-1][1] = ble_buf[4]
                    rgb_value[ble_buf[2]-1][2] = ble_buf[5]
                elif ((ble_buf[0] == 82) and (ble_buf[1] == 6) and (ble_buf[2] != 255) and (ble_buf[3] != 255) and (ble_buf[4] != 255) and (ble_buf[5] != 255) and (ble_buf[6] != 255)): #Led Matrix
                    bluetooth_mode = 0
                    
                    user_led_matrix[0] = ble_buf[2]
                    user_led_matrix[1] = ble_buf[3]
                    user_led_matrix[2] = ble_buf[4]
                    user_led_matrix[3] = ble_buf[5]
                    user_led_matrix[4] = ble_buf[6]

                    led_matrix_status = 1
                    drawScreen(user_led_matrix)
                    
                ble_buf = bytes()

        if berryMode == 0:  #IR Mode
            rgb.color_function(rgb_value)

            # Komutları önce işle, sonra aktif mod adımını çalıştır.
            remote()

            if obstacle_mode_active:
                raider_mode_step()
            elif flow_mode_active:
                follow_mode_step()
            elif guard_mode_active:
                guard_mode_step()

            if left_turning or right_turning or sumo_mode_active or obstacle_mode_active or flow_mode_active or guard_mode_active:
                # Hareket devam ediyor, LED'leri yanıp söymeye devam et
                pass
            else:
                show_idle_mode_icon()

        main_error_count = 0
    except Exception as err:
        main_error_count += 1
        print("Main loop error:", err)
        safe_idle_state(show_ir_icon=True)
        time.sleep_ms(120)

        if main_error_count >= MAX_MAIN_ERRORS:
            print("Too many errors. Resetting device...")
            safe_idle_state(show_ir_icon=False)
            time.sleep_ms(150)
            machine.reset()
