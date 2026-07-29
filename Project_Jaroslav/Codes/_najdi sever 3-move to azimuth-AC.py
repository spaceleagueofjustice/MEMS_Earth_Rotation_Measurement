# ---------------------------------------------------------------
# NAJDI SEVER - PRESNA VERZE (A/C DIFERENCIAL)
#
# Na rozdil od rychle "NAJDI_SEVER" (jedno mereni na pozici) tahle
# verze na kazdem azimutu dela POROVNANI A/C (180 stupnu), stejne
# jako klasicka mericí metodika T13-T18. Diky tomu:
#   - bias gyra se v kazdem cyklu primo odecte (A - C resp. C - A),
#     misto aby se spolehal jen na to, ze se prumeruje pryc pres
#     cely kruh
#   - efektivni signal se zdvojnasobi (diff = 2 x signal(theta))
#   - vysledny odhad severu by mel byt VYRAZNE presnejsi nez rychla
#     jednorazova verze, za cenu delsiho behu (kazdy cyklus dela
#     dva presuny o 180 stupnu navic)
#
# POZOR - ZNAMENKOVA KONVENCE (empiricky overeno testem SVJZ na surovych
# datech, ale POZOR - pro DIFERENCIAL diffX plati OPACNE nez pro surove
# cteni, viz odvozeni nize):
#   diffX = xC - xA   (POZOR, opacne poradi nez u Y!)
#   diffY = yA - yC
#   Protoze xC = bias + S(theta+180) = bias - S(theta) a xA = bias + S(theta),
#   vychazi diffX = xC - xA = -2*S(theta) - tedy s OPACNYM znamenkem nez
#   samotne S(theta). Pokud surove S(theta) ma maximum na jihu (jak plati
#   pro jednorazovou verzi najdi_sever.py), pak diffX ma naopak MAXIMUM
#   na SEVERU. Korekce sever = faze_maxima + 180 stupnu (pouzita drive)
#   byla tedy pro diferencial CHYBNA - spravne je sever = faze_maxima
#   PRIMO, bez pricitani 180 stupnu (opraveno v teto verzi).
#
# Postaveno na stejne, jiz overene mechanice jako T16/T17/T18-4
# skripty (motor, anti-backlash, kabelova ochrana, warm-up gate).
#
# POSTUP:
#   1) Prohleda se cely kruh (FIND_NORTH_STEP_DEG, default 10 deg =
#      36 pozic), kazda pozice navstivena REPEATS_PER_AZIMUTH krat
#      (default 3x = 108 cyklu celkem), v NAHODNEM poradi (Fisher-
#      Yates shuffle - decorreluje cas od azimutu, viz duvod v
#      puvodnim najdi_sever.py).
#   2) Na kazde navsteve: presun na azimut (nejkratsi cestou, anti-
#      backlash, kabelova ochrana), zmereni A, presun o +180 stupnu,
#      zmereni C. Zadny navrat zpet na A pred dalsim cyklem - dalsi
#      cyklus si najde cestu z aktualni pozice sam.
#   3) Na konci: prumer pres opakovani u kazdeho azimutu -> diffX,
#      diffY -> harmonicky fit (uzavreny tvar, presny diky
#      rovnomernemu pokryti 360 stupnu) -> faze maxima diffX ->
#      odhad SEVERU (bez korekce, viz vyse - diferencial ma opacne
#      znamenko nez surove cteni).
#   4) Rameno se otoci presne na odhadovany sever, pripadne jeste
#      dodatecne o FINAL_OFFSET_DEG (napr. 90 = vychod - znamenko
#      over pri prvnim pouziti).
#   5) Motor zustava zapnuty (drzi pozici, kabel by ji jinak pootocil).
# ---------------------------------------------------------------

import time
import math
import struct
import random
from machine import Pin, SPI, I2C

# =============================================================================
# CONFIGURATION
# =============================================================================

TESTID = "T19-2_NAJDI_SEVER_AC"

FIND_NORTH_STEP_DEG  = 5 # 10
AZIMUTHS             = [round(i * FIND_NORTH_STEP_DEG, 2)
                         for i in range(int(round(360 / FIND_NORTH_STEP_DEG)))]

# Kazdy azimut navstiven vicekrat, v NAHODNEM poradi (decorreluje cas
# a azimut). Diky A/C diferencialu (mnohem lepsi SNR na cyklus) by 3
# mely stacit i pro presny odhad, ale klidne zvys, pokud chces jeste
# vic potlacit sum (za cenu delsiho behu).
REPEATS_PER_AZIMUTH = 5 # 3

# Dodatecne pootoceni po nalezeni severu (0 = zustane na severu).
# Napr. 90.0 = vychod, -90.0 = zapad, 180.0 = jih - znamenko over
# pri prvnim pouziti (kompasem/znamym bodem).
FINAL_OFFSET_DEG = 0.0

SAMPLES_PER_POSITION = 300 # 200
SAMPLE_DELAY         = 0.01

FIXED_SETTLE_S = 4.0    # plne usazeni po kazdem presunu, pred merenim
SETTLE_PRE_S   = 0.5    # kratke usazeni po hrubem presunu, pred finalnim priblizenim
FIXED_TRIM     = 50

CABLE_SAFETY_LIMIT_DEG = 400.0   # tvrdy limit (plny sweep + A/C presuny potrebuje vic nez jen sweep sam)
UNWIND_WAIT_S          = 1.0

# --- Anti-backlash: jednosmerne najizdeni na KAZDY cil ---
APPROACH_OVERSHOOT_DEG = 15.0
APPROACH_SIGN          = 1

# --- Motor settings (A4988) - beze zmeny ---
MOTOR_STEP_DELAY_US   = 400
MOTOR_START_DELAY_US  = 1400
RAMP_STEPS            = 60
BRAKE_TIME_MS         = 150

STEPS_PER_REV   = 3200
MOTOR_DIRECTION = -1

_step_remainder = 0.0

STEP_PIN = Pin(2, Pin.OUT)
DIR_PIN  = Pin(3, Pin.OUT)
EN_PIN   = Pin(4, Pin.OUT)

# --- SPI1: ADXRS290 ---
SPI_ID     = 1
SPI_SCK    = 10
SPI_MOSI   = 11
SPI_MISO   = 12
SPI_CS_PIN = 13

# --- I2C0: MPU6050 (jen diagnosticky, kontrola naklonu) ---
I2C_ID   = 0
I2C_SDA  = 16
I2C_SCL  = 17
MPU_ADDR = 0x68

# --- Warm-up gate ---
WARMUP_ENABLED                   = True
WARMUP_CHECK_INTERVAL_S          = 5.0
WARMUP_WINDOW_S                  = 60.0
WARMUP_SLOPE_THRESHOLD_C_PER_MIN = 0.5
WARMUP_TIMEOUT_S                 = 900.0

# --- Output files ---
CSV_FILE     = f"{TESTID}_data.csv"
LOG_FILE     = f"{TESTID}_log.txt"
SUMMARY_FILE = f"{TESTID}_summary.csv"

# =============================================================================
# LOGGING
# =============================================================================

def log(msg):
    print(msg)
    with open(LOG_FILE, "a") as f:
        f.write(msg + "\n")

def trimmed_mean(samples, trim):
    if trim > 0 and len(samples) > 2 * trim:
        s = sorted(samples)[trim:-trim]
        return sum(s) / len(s)
    return sum(samples) / len(samples)

# =============================================================================
# MOTOR - A4988 (beze zmeny)
# =============================================================================

def motor_enable(enable=True):
    EN_PIN.value(0 if enable else 1)

def motor_step(direction=1, delay_us=MOTOR_STEP_DELAY_US):
    DIR_PIN.value(1 if direction > 0 else 0)
    STEP_PIN.value(1)
    time.sleep_us(delay_us)
    STEP_PIN.value(0)
    time.sleep_us(delay_us)

def motor_rotate_degrees(deg):
    global _step_remainder
    if deg == 0:
        return
    exact_steps = abs(deg) / 360 * STEPS_PER_REV + _step_remainder
    steps = int(exact_steps)
    _step_remainder = exact_steps - steps
    if steps == 0:
        return
    direction = MOTOR_DIRECTION * (1 if deg > 0 else -1)
    motor_enable(True)
    ramp = min(RAMP_STEPS, steps // 2)
    for i in range(steps):
        if ramp > 0 and i < ramp:
            frac = i / ramp
            delay = int(MOTOR_START_DELAY_US - frac * (MOTOR_START_DELAY_US - MOTOR_STEP_DELAY_US))
        elif ramp > 0 and i >= steps - ramp:
            frac = (steps - 1 - i) / ramp
            delay = int(MOTOR_START_DELAY_US - frac * (MOTOR_START_DELAY_US - MOTOR_STEP_DELAY_US))
        else:
            delay = MOTOR_STEP_DELAY_US
        motor_step(direction, delay)
    time.sleep_ms(BRAKE_TIME_MS)

# =============================================================================
# ANTI-BACKLASH POHYB + KABELOVA OCHRANA (stejne jako T17-4/najdi_sever)
# =============================================================================

_current_pos = 0.0

def move_to(target_pos, settle_after=None):
    global _current_pos
    if settle_after is None:
        settle_after = FIXED_SETTLE_S
    pre_target = target_pos - APPROACH_SIGN * APPROACH_OVERSHOOT_DEG
    delta1 = pre_target - _current_pos
    if delta1 != 0:
        motor_rotate_degrees(delta1)
        _current_pos += delta1
        time.sleep(SETTLE_PRE_S)
    delta2 = target_pos - _current_pos
    motor_rotate_degrees(delta2)
    _current_pos += delta2
    time.sleep(settle_after)

def move_relative(delta, settle_after=None):
    move_to(_current_pos + delta, settle_after=settle_after)

def move_to_azimuth(target_azimuth):
    global _current_pos
    cur_mod = _current_pos % 360
    diff = (target_azimuth - cur_mod + 180) % 360 - 180
    target_abs = _current_pos + diff
    pos_before = _current_pos

    if abs(target_abs) > CABLE_SAFETY_LIMIT_DEG:
        unwind = -_current_pos
        log(f"  [KABEL] Kumulativni natoceni {_current_pos:.1f} deg by presahlo limit "
            f"{CABLE_SAFETY_LIMIT_DEG:.0f} deg. Odvijim zpet ({unwind:+.1f} deg)...")
        motor_rotate_degrees(unwind)
        _current_pos = 0.0
        time.sleep(UNWIND_WAIT_S)
        cur_mod = _current_pos % 360
        diff = (target_azimuth - cur_mod + 180) % 360 - 180
        target_abs = _current_pos + diff
        pos_before = _current_pos

    smer_label = "CW" if diff >= 0 else "CCW"
    log(f"  Presun na azimut {target_azimuth:.2f} deg (delta={diff:+.1f} deg, smer={smer_label}, "
        f"pozice pred presunem={pos_before:+.1f} deg)")
    move_to(target_abs, settle_after=FIXED_SETTLE_S)
    return diff, smer_label

# =============================================================================
# BUZZER
# =============================================================================

def beep(freq=2700, duration_ms=200, count=3, pin=22):
    import machine
    buzzer = machine.PWM(machine.Pin(pin))
    for _ in range(count):
        buzzer.freq(freq)
        buzzer.duty_u16(32768)
        time.sleep_ms(duration_ms)
        buzzer.duty_u16(0)
        time.sleep_ms(150)
    buzzer.deinit()

# =============================================================================
# ADXRS290 - SPI (obe osy + teplota)
# =============================================================================

class ADXRS290:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs  = Pin(cs, Pin.OUT, value=1)
        self._init()

    def _write(self, addr, val):
        tx = bytearray([addr & 0x7F, val])
        self.cs.value(0)
        time.sleep_ms(1)
        self.spi.write(tx)
        time.sleep_ms(1)
        self.cs.value(1)

    def _read(self, addr):
        tx = bytearray([0x80 | addr, 0x00])
        rx = bytearray(2)
        self.cs.value(0)
        time.sleep_ms(1)
        self.spi.write_readinto(tx, rx)
        time.sleep_ms(1)
        self.cs.value(1)
        return rx[1]

    def _init(self):
        time.sleep_ms(100)
        adi  = self._read(0x00)
        mems = self._read(0x01)
        dev  = self._read(0x02)
        log(f"  ADXRS290: ADI={hex(adi)} MEMS={hex(mems)} DEV={hex(dev)}")
        if adi != 0xAD or mems != 0x1D or dev != 0x92:
            raise RuntimeError("ADXRS290 not found or wrong ID!")
        self._write(0x10, 0x03)
        time.sleep_ms(100)
        self._write(0x11, 0x00)
        time.sleep_ms(100)
        log("  ADXRS290 measurement mode ON, TSM (teplota) ON, FILTER=0x00 (raw, bez filtru)")

    def _read_burst(self):
        tx = bytearray([0x80 | 0x08, 0, 0, 0, 0, 0, 0])
        rx = bytearray(7)
        self.cs.value(0)
        time.sleep_ms(1)
        self.spi.write_readinto(tx, rx)
        time.sleep_ms(1)
        self.cs.value(1)
        raw_x = struct.unpack('<h', bytes([rx[1], rx[2]]))[0]
        raw_y = struct.unpack('<h', bytes([rx[3], rx[4]]))[0]
        temp0 = rx[5]
        temp1 = rx[6]
        temp_raw = temp0 | ((temp1 & 0x0F) << 8)
        if temp_raw & 0x800:
            temp_raw -= 0x1000
        return raw_x, raw_y, temp_raw

    def read_gyro_temp_raw(self):
        raw_x, raw_y, temp_raw = self._read_burst()
        gx = raw_x / 200.0
        gy = raw_y / 200.0
        temp_c = temp_raw / 10.0
        return gx, gy, temp_c

# =============================================================================
# MPU6050 - I2C (jen diagnosticky, kontrola naklonu)
# =============================================================================

class MPU6050:
    PWR_MGMT_1    = 0x6B
    ACCEL_XOUT_H  = 0x3B
    ACCEL_SENS_2G = 16384.0

    def __init__(self, i2c, addr=MPU_ADDR):
        self.i2c = i2c
        self.addr = addr
        self._init()

    def _init(self):
        self.i2c.writeto_mem(self.addr, 0x6B, bytes([0x80]))
        time.sleep_ms(100)
        self.i2c.writeto_mem(self.addr, 0x68, bytes([0x07]))
        time.sleep_ms(100)
        self.i2c.writeto_mem(self.addr, self.PWR_MGMT_1, bytes([0x00]))
        time.sleep_ms(100)
        who = self.i2c.readfrom_mem(self.addr, 0x75, 1)[0]
        log(f"  MPU6050: WHO_AM_I={hex(who)}")

    def read_accel_raw(self):
        data = self.i2c.readfrom_mem(self.addr, self.ACCEL_XOUT_H, 6)
        raw_x = struct.unpack('>h', data[0:2])[0]
        raw_y = struct.unpack('>h', data[2:4])[0]
        ax = raw_x / self.ACCEL_SENS_2G
        ay = raw_y / self.ACCEL_SENS_2G
        return ax, ay

# =============================================================================
# WARM-UP GATE (beze zmeny)
# =============================================================================

def wait_for_gyro_warmup(gyro, timeout_s=WARMUP_TIMEOUT_S):
    if not WARMUP_ENABLED:
        return
    log("\n" + "="*65)
    log("WARM-UP: cekam na ustaleni teploty gyra pred startem mereni...")
    log(f"  Prah={WARMUP_SLOPE_THRESHOLD_C_PER_MIN:.2f} C/min, okno={WARMUP_WINDOW_S:.0f}s, "
        f"kontrola kazdych {WARMUP_CHECK_INTERVAL_S:.0f}s, timeout={timeout_s:.0f}s")
    log("="*65)
    history = []
    t_start = time.ticks_ms()
    while True:
        elapsed_s = time.ticks_diff(time.ticks_ms(), t_start) / 1000.0
        try:
            _, _, temp_c = gyro.read_gyro_temp_raw()
        except OSError as e:
            log(f"  [WARN] Chyba pri cteni teploty behem warm-up ({e}), zkousim znovu.")
            time.sleep(WARMUP_CHECK_INTERVAL_S)
            continue
        history.append((elapsed_s, temp_c))
        while history and (elapsed_s - history[0][0]) > WARMUP_WINDOW_S:
            history.pop(0)
        if len(history) >= 2:
            dt = history[-1][0] - history[0][0]
            dtemp = history[-1][1] - history[0][1]
            slope_per_min = (dtemp / dt) * 60.0 if dt > 0 else 0.0
            log(f"  t={elapsed_s:5.0f}s  teplota={temp_c:+.2f} C  "
                f"trend(okno {dt:.0f}s)={slope_per_min:+.3f} C/min")
            if dt >= WARMUP_WINDOW_S * 0.5 and abs(slope_per_min) <= WARMUP_SLOPE_THRESHOLD_C_PER_MIN:
                log(f"\nWARM-UP OK: teplota ustalena (trend {slope_per_min:+.3f} C/min) "
                    f"po {elapsed_s:.0f}s. Startuji mereni.")
                return
        else:
            log(f"  t={elapsed_s:5.0f}s  teplota={temp_c:+.2f} C  (sbiram data...)")
        if elapsed_s >= timeout_s:
            log(f"\nWARM-UP TIMEOUT po {elapsed_s:.0f}s - STARTUJI mereni i tak.")
            return
        time.sleep(WARMUP_CHECK_INTERVAL_S)

# =============================================================================
# MEASUREMENT
# =============================================================================

def read_gyro_retry(gyro, retries=5, delay_ms=50):
    last_exc = None
    for attempt in range(retries):
        try:
            return gyro.read_gyro_temp_raw()
        except OSError as e:
            last_exc = e
            log(f"    [WARN] SPI chyba pri cteni gyra ({e}), pokus {attempt+1}/{retries}")
            time.sleep_ms(delay_ms)
    raise last_exc

def read_accel_retry(mpu, retries=5, delay_ms=50):
    last_exc = None
    for attempt in range(retries):
        try:
            return mpu.read_accel_raw()
        except OSError as e:
            last_exc = e
            log(f"    [WARN] I2C chyba pri cteni MPU6050 ({e}), pokus {attempt+1}/{retries}")
            time.sleep_ms(delay_ms)
    raise last_exc

def measure(gyro, mpu, label, trim=FIXED_TRIM):
    xs, ys, ts, axs, ays = [], [], [], [], []
    for _ in range(SAMPLES_PER_POSITION):
        gx, gy, temp_c = read_gyro_retry(gyro)
        ax, ay = read_accel_retry(mpu)
        xs.append(gx); ys.append(gy); ts.append(temp_c)
        axs.append(ax); ays.append(ay)
        time.sleep(SAMPLE_DELAY)
    gx_m = trimmed_mean(xs, trim)
    gy_m = trimmed_mean(ys, trim)
    t_m  = trimmed_mean(ts, trim)
    ax_m = trimmed_mean(axs, trim)
    ay_m = trimmed_mean(ays, trim)
    log(f"  Pozice {label}: gX={gx_m:+.8f} gY={gy_m:+.8f}  T={t_m:+.2f}C  "
        f"aX={ax_m:+.6f} aY={ay_m:+.6f}  (trim={trim})")
    return gx_m, gy_m, t_m, ax_m, ay_m

# =============================================================================
# CSV
# =============================================================================

def csv_header_once():
    try:
        with open(CSV_FILE, "r"):
            pass
    except OSError:
        with open(CSV_FILE, "w") as f:
            f.write("cyklus;azimut;xA;yA;tA;axA;ayA;xC;yC;tC;axC;ayC\n")

# =============================================================================
# HARMONICKY FIT (uzavreny tvar, presny diky rovnomernemu 360 deg pokryti)
# =============================================================================

def fit_north_phase(azimuths_deg, values):
    n = len(values)
    sum_a = 0.0
    sum_b = 0.0
    sum_c = 0.0
    for az, v in zip(azimuths_deg, values):
        rad = math.radians(az)
        sum_a += v * math.cos(rad)
        sum_b += v * math.sin(rad)
        sum_c += v
    a = (2.0 / n) * sum_a
    b = (2.0 / n) * sum_b
    mean = sum_c / n
    amplitude = math.sqrt(a * a + b * b)
    phase_deg = math.degrees(math.atan2(b, a)) % 360.0
    return phase_deg, amplitude, mean

# =============================================================================
# MAIN
# =============================================================================

def main():
    with open(LOG_FILE, "w") as f:
        f.write(f"=== NAJDI SEVER (A/C DIFERENCIAL) LOG - TESTID={TESTID} ===\n")

    time.sleep(1)
    beep(count=2)

    log("="*65)
    log("NAJDI SEVER - PRESNA VERZE (A/C diferencial pres cely kruh)")
    log(f"FIND_NORTH_STEP_DEG = {FIND_NORTH_STEP_DEG}, REPEATS_PER_AZIMUTH = {REPEATS_PER_AZIMUTH}")
    log(f"FINAL_OFFSET_DEG = {FINAL_OFFSET_DEG:+.1f} deg")
    log("NEMA17 + A4988 + ADXRS290 (+teplota) + MPU6050 (kontrola naklonu)")
    log("="*65)
    log("\nPOZOR: aktualni pozice ramene (azimut 0 deg, libovolny smer) se")
    log("bere jako lokalni nula. Sestavu pred spustenim aspon zhruba srovnej")
    log("vodovahou (naklon zkresluje odhad severu).")

    log("\nInitializing SPI1...")
    spi1 = SPI(SPI_ID, baudrate=1_000_000, polarity=1, phase=1,
               sck=Pin(SPI_SCK), mosi=Pin(SPI_MOSI), miso=Pin(SPI_MISO))
    gyro = ADXRS290(spi1, SPI_CS_PIN)

    log("\nInitializing I2C0 (MPU6050)...")
    i2c0 = I2C(I2C_ID, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400_000)
    mpu = MPU6050(i2c0)

    wait_for_gyro_warmup(gyro, timeout_s=WARMUP_TIMEOUT_S)
    csv_header_once()

    # --- Sestaveni a promichani seznamu navstev (Fisher-Yates) ---
    visit_list = []
    for az in AZIMUTHS:
        for _ in range(REPEATS_PER_AZIMUTH):
            visit_list.append(az)

    for i in range(len(visit_list) - 1, 0, -1):
        j = random.randint(0, i)
        visit_list[i], visit_list[j] = visit_list[j], visit_list[i]

    total_visits = len(visit_list)
    log("\n" + "="*65)
    log(f"NAHODNE PROMICHANY A/C SWEEP: {len(AZIMUTHS)} azimutu x "
        f"{REPEATS_PER_AZIMUTH} opakovani = {total_visits} cyklu "
        f"(kazdy cyklus = A + presun 180 + C)")
    log("="*65)

    # Akumulace po azimutech: soucty xA,yA,xC,yC + pocet
    stats = {az: [0.0, 0.0, 0.0, 0.0, 0] for az in AZIMUTHS}

    for i, az in enumerate(visit_list, 1):
        log(f"\n--- Cyklus {i}/{total_visits}: azimut {az:.1f} deg (nahodne poradi) ---")

        success = False
        for attempt in range(1, 4):
            try:
                move_to_azimuth(az)
                A_X, A_Y, A_T, A_AX, A_AY = measure(gyro, mpu, "A")

                log("\nPresouvam A->C (+180 deg, anti-backlash)...")
                move_relative(180)
                C_X, C_Y, C_T, C_AX, C_AY = measure(gyro, mpu, "C")

                with open(CSV_FILE, "a") as f:
                    f.write(f"{i};{az:g};{A_X:.8f};{A_Y:.8f};{A_T:.2f};{A_AX:.6f};{A_AY:.6f};"
                            f"{C_X:.8f};{C_Y:.8f};{C_T:.2f};{C_AX:.6f};{C_AY:.6f}\n")

                s = stats[az]
                s[0] += A_X; s[1] += A_Y
                s[2] += C_X; s[3] += C_Y
                s[4] += 1

                success = True
                break
            except OSError as e:
                log(f"  [CHYBA] Cyklus {i} (azimut {az} deg) selhal na pokusu {attempt}/3: {e}")
                time.sleep(1)

        if not success:
            log(f"  [CHYBA] Cyklus {i} (azimut {az} deg) TRVALE selhal po 3 pokusech - PRESKAKUJI.")

        if i % 20 == 0:
            beep(count=1, freq=2000, duration_ms=100)
            log(f"\n[POSTUP] Dokonceno {i}/{total_visits} cyklu.")

    # --- Prumer pres opakovani u kazdeho azimutu + diferencialy ---
    # POZOR na znamenkovou konvenci (SVJZ test): diffX = xC - xA, diffY = yA - yC
    sweep_az, sweep_diffx, sweep_diffy = [], [], []
    with open(SUMMARY_FILE, "w") as f:
        f.write("azimut;n;xA_prumer;xC_prumer;diffx;yA_prumer;yC_prumer;diffy\n")
        for az in AZIMUTHS:
            sxA, syA, sxC, syC, n = stats[az]
            if n == 0:
                log(f"  [VAROVANI] azimut {az:.1f} deg nemel ani jeden uspesny cyklus - vynechavam.")
                continue
            xA_m = sxA / n; yA_m = syA / n
            xC_m = sxC / n; yC_m = syC / n
            diffx = xC_m - xA_m
            diffy = yA_m - yC_m
            f.write(f"{az:g};{n};{xA_m:.8f};{xC_m:.8f};{diffx:.8f};{yA_m:.8f};{yC_m:.8f};{diffy:.8f}\n")
            sweep_az.append(az)
            sweep_diffx.append(diffx)
            sweep_diffy.append(diffy)
    log(f"\nSouhrnny CSV zapsan: {SUMMARY_FILE}")

    # --- Harmonicky fit na diffX (hlavni odhad severu), diffY jen diagnosticky ---
    phase_x, amp_x, mean_x = fit_north_phase(sweep_az, sweep_diffx)
    phase_y, amp_y, mean_y = fit_north_phase(sweep_az, sweep_diffy)

    log("\n" + "="*65)
    log("VYSLEDEK FITU (na diferencialech diffX = xC-xA, diffY = yA-yC):")
    log(f"  X kanal: faze maxima diffX = {phase_x:.1f} deg, "
        f"amplituda = {amp_x:.5f} deg/s (DC = {mean_x:.5f} deg/s)")
    log(f"  Y kanal (diagnosticky): faze = {phase_y:.1f} deg, "
        f"amplituda = {amp_y:.5f} deg/s (DC = {mean_y:.5f} deg/s)")
    log(f"  Rozdil faze X vs Y (ocekavano cca 85-90 deg): {(phase_x - phase_y) % 360:.1f} deg")

    # POZOR: na rozdil od jednorazove (nediferencovane) verze, kde maximum
    # surovych dat odpovida jihu, ma diferencial diffX = xC - xA OPACNE
    # znamenko nez surove S(theta) (diffX = -2*S(theta) - viz odvozeni
    # v komentari k opravene chybe). Fáze maxima diffX proto odpovida
    # PRIMO severu, BEZ pricitani 180 stupnu.
    north_estimate = phase_x % 360.0
    log(f"  Faze maxima diffX ({phase_x:.1f} deg) odpovida PRIMO severu "
        f"(diferencial ma opacne znamenko nez surove cteni) -> odhad SEVERU = {north_estimate:.1f} deg")
    log("="*65)

    final_target = (north_estimate + FINAL_OFFSET_DEG) % 360.0
    if FINAL_OFFSET_DEG != 0.0:
        log(f"\nFINAL_OFFSET_DEG = {FINAL_OFFSET_DEG:+.1f} deg -> cilova pozice = "
            f"sever + offset = {final_target:.1f} deg")
    else:
        log(f"\nFINAL_OFFSET_DEG = 0 -> cilova pozice = samotny sever = {final_target:.1f} deg")

    log(f"\nOtacim rameno na cilovou pozici ({final_target:.1f} deg)...")
    move_to_azimuth(round(final_target, 2))

    log("\n" + "="*65)
    log("HOTOVO - rameno by melo mirit na cilovou pozici.")
    if FINAL_OFFSET_DEG != 0.0:
        log(f"POZOR: over fyzicky (kompasem), ze FINAL_OFFSET_DEG={FINAL_OFFSET_DEG:+.1f} deg "
            f"opravdu odpovida smeru, ktery cekas.")
    log("="*65)
    # Motor se ZAMERNE nevypina (motor_enable(False) NENI volano) - kabel
    # by rameno po vypnuti pootocil.
    beep(count=4)

main()