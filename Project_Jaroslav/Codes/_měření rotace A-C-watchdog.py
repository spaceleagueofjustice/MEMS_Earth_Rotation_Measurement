# ---------------------------------------------------------------
# EARTH ROTATION MEASUREMENT
# FIXNI SMER - JEN POLOHA A (SEVER) <-> C (JIH, 180 stupnu) - DOKOLA
# OBE OSY GYRA (X i Y) + TEPLOTA (TEMP0/TEMP1) + WARM-UP GATE
#
# Nazev konkretniho mereni se nastavuje parametrem TESTID nize -
# pouziva se jako predpona nazvu vystupnich souboru a zapisuje se
# i do logu.
#
# NEMA17 + A4988 + ADXRS290 (SPI1, X i Y) + MPU6050 (I2C0) - kontrola naklonu
#
# FILOZOFIE TETO VERZE:
#   Rameno se poprve RUCNE nastavi co nejlepe na sever - to je pozice A
#   (azimut 0). Od ted uz s nim rucne NEHYBAT. Kazdy cyklus je proste:
#     A (zmerit) -> otocit o +180 stupnu -> C (zmerit) -> otocit o -180
#     stupnu -> zpet A -> dalsi cyklus.
#   Cisty pohyb za cyklus je 0 stupnu (+180 pak -180), takze narozdil od
#   nahodne-azimutove verze (T17-4) NENI potreba zadna ochrana kabelaze
#   ani odvijeni - kabel se nikdy nenavine vic, nez kolik je jedno
#   otoceni tam a zpet.
#
#   Anti-backlash najizdeni na KAZDOU z obou pozic (A i C) je zachovane
#   stejne jako drive - presun se dela vzdy s pevnym prekmitem
#   (APPROACH_OVERSHOOT_DEG) v pevnem smeru (APPROACH_SIGN), takze obe
#   pozice maji stejne (symetricke) mechanicke chovani.
#
#   Merene veliciny na kazde pozici: gX, gY (obe osy gyra ADXRS290),
#   teplota gyra, aX, aY (MPU6050 - jen kontrolni naklon, zadne
#   polohovani/homing na jeho zaklade).
#
# Postup: 1) rucne nastavit rameno na sever (jednou, na zacatku - pozice A).
#         2) spustit skript.
#         3) skript pocka na ustaleni teploty gyra (warm-up gate).
#         4) skript automaticky odjizdi TOTAL_CYCLES cyklu A<->C.
#         5) motor se vypne az uplne na konci.
# ---------------------------------------------------------------

import time
import struct
from machine import Pin, SPI, I2C

# =============================================================================
# CONFIGURATION
# =============================================================================

# Nazev tohoto konkretniho mereni - pouzije se jako predpona vsech
# vystupnich souboru (CSV, log, summary) a zapise se i do logu.
TESTID = "T19-1"

# Celkovy pocet mericich cyklu (kazdy = 1x zmereni A + 1x zmereni C).
# Orientacne: 1 cyklus ~ 13-15 s (viz FIXED_SETTLE_S, SAMPLES_PER_POSITION
# nize), takze napr. 1000 cyklu ~ 3.5-4 hodiny. Uprav podle toho, jak
# dlouho chces mereni nechat bezet.
TOTAL_CYCLES         = 1000
SAMPLES_PER_POSITION = 200
SAMPLE_DELAY         = 0.01

FIXED_SETTLE_S = 4.0    # plne usazeni po KAZDEM presunu (pred merenim)
SETTLE_PRE_S   = 0.5    # kratke usazeni po hrubem presunu, pred finalnim priblizenim
FIXED_TRIM     = 50

# --- Anti-backlash: jednosmerne najizdeni na KAZDOU pozici (nemenit) ---
APPROACH_OVERSHOOT_DEG = 15.0
APPROACH_SIGN          = 1      # FIXNI - nemenit behem behu

# --- Motor settings (A4988) ---
MOTOR_STEP_DELAY_US   = 400
MOTOR_START_DELAY_US  = 1400
RAMP_STEPS            = 60
BRAKE_TIME_MS         = 150

STEPS_PER_REV   = 3200
MOTOR_DIRECTION = -1

_step_remainder = 0.0
_current_pos    = 0.0   # absolutni (nezabalovana) pozice, 0 = sever (A)

STEP_PIN = Pin(2, Pin.OUT)
DIR_PIN  = Pin(3, Pin.OUT)
EN_PIN   = Pin(4, Pin.OUT)

# --- SPI1: ADXRS290 ---
SPI_ID     = 1
SPI_SCK    = 10
SPI_MOSI   = 11
SPI_MISO   = 12
SPI_CS_PIN = 13

# --- I2C0: MPU6050 (jen kontrola naklonu, zadne polohovani) ---
I2C_ID   = 0
I2C_SDA  = 16
I2C_SCL  = 17
MPU_ADDR = 0x68

# --- Warm-up gate: ceka na ustaleni teploty gyra pred startem mereni ---
WARMUP_ENABLED                   = True
WARMUP_CHECK_INTERVAL_S          = 5.0     # jak casto se ptat na teplotu
WARMUP_WINDOW_S                  = 60.0    # okno, ze ktereho se pocita trend (C/min)
WARMUP_SLOPE_THRESHOLD_C_PER_MIN = 0.5     # pod timhle trendem se teplota povazuje za ustalenou
WARMUP_TIMEOUT_S                 = 900.0   # max. cekani (15 min), pak se stejne spusti mereni

# --- Output files ---
CSV_FILE     = f"{TESTID}_mereni_data.csv"
LOG_FILE     = f"{TESTID}_mereni_log.txt"
SUMMARY_FILE = f"{TESTID}_mereni_summary.csv"

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
# MOTOR - A4988
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
# ANTI-BACKLASH POHYB (absolutni, nezabalovana pozice)
# Zachovano beze zmeny z randomizovane verze - jen se pouziva jen mezi
# dvema pevnymi cili (0 = sever = A, 180 = jih = C), zadna ochrana
# kabelaze neni potreba (cisty pohyb za cyklus = 0 stupnu).
# =============================================================================

def move_to(target_pos, settle_after=None):
    """Presun na absolutni cil s anti-backlash presahem pred finalnim
    najetim (vzdy ve smeru APPROACH_SIGN)."""
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
    def __init__(self, spi, cs, spi_id=None, sck=None, mosi=None, miso=None):
        # spi_id/sck/mosi/miso se ulozi jen kvuli pripadne reinicializaci
        # (watchdog na zaseklou SPI komunikaci) - motoru se to netyka.
        self.spi = spi
        self.cs  = Pin(cs, Pin.OUT, value=1)
        self._spi_id = spi_id
        self._sck    = sck
        self._mosi   = mosi
        self._miso   = miso
        self._init()

    def reinit(self):
        """Znovu vytvori SPI periferii a znovu nastavi registry cipu.
        POUZE softwarova operace na SPI sbernici - NEDOTYKA SE motoru
        ani jeho napajeni. Pouziva se watchdogem pri detekci zaseknute
        (bit-presne opakujici se) komunikace."""
        try:
            self.spi.deinit()
        except Exception as e:
            log(f"    [WARN] spi.deinit() pri reinit selhalo (pokracuji dal): {e}")
        if self._spi_id is not None:
            self.spi = SPI(self._spi_id, baudrate=1_000_000, polarity=1, phase=1,
                            sck=Pin(self._sck), mosi=Pin(self._mosi), miso=Pin(self._miso))
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
        # POWER_CTL (0x10): bit1=Measurement, bit0=TSM (teplotni senzor).
        self._write(0x10, 0x03)
        time.sleep_ms(100)
        self._write(0x11, 0x00)
        time.sleep_ms(100)
        log("  ADXRS290 measurement mode ON, TSM (teplota) ON, FILTER=0x00 (raw, bez filtru)")

    def _read_burst(self):
        # Registry 0x08-0x0D jsou v cipu za sebou (DATAX0,DATAX1,DATAY0,
        # DATAY1,TEMP0,TEMP1), takze jde precist vsechno jednim burstem -
        # X/Y/teplota jsou pak ze stejneho okamziku.
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
        temp_c = temp_raw / 10.0   # 10 LSB/1 C, 0 code = 0 C
        return gx, gy, temp_c

# =============================================================================
# MPU6050 - I2C (jen kontrola naklonu, zadne polohovani/homing)
# =============================================================================

class MPU6050:
    PWR_MGMT_1   = 0x6B
    ACCEL_XOUT_H = 0x3B
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
# WARM-UP GATE - ceka na ustaleni teploty gyra pred startem mereni
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
            log(f"\nWARM-UP TIMEOUT po {elapsed_s:.0f}s - teplota se nestihla ustalit na "
                f"pozadovany prah, ale STARTUJI mereni i tak (timeout ma prednost pred "
                f"nekonecnym cekanim).")
            return

        time.sleep(WARMUP_CHECK_INTERVAL_S)

# =============================================================================
# WATCHDOG - detekce zaseknute (bit-presne opakujici se) SPI komunikace
# Motivace: 24.7. se ADXRS290 uprostred dlouheho behu "zaseklo" - SPI
# transakce nehazely OSError (retry logika je tedy nezachytila), ale
# gX/gY/teplota zustaly bit-presne konstantni po zbytek behu. Tenhle
# watchdog takove ticho selhani detekuje a zkusi SPI/cip reinicializovat
# - VYHRADNE softwarove, motor (EN/DIR/STEP piny) se timhle vubec
# nedotkne, takze rameno se nemuze pootocit.
# =============================================================================

STUCK_CHECK_COUNT = 5      # kolik poslednich cteni musi byt identickych, aby to bylo bran jako zaseknuti
STUCK_EPS         = 1e-9   # tolerance pro "identicke" (plovouci desetinna cisla)

_gyro_history = []

def _is_stuck(history):
    if len(history) < STUCK_CHECK_COUNT:
        return False
    window = history[-STUCK_CHECK_COUNT:]
    ref = window[0]
    for h in window[1:]:
        if (abs(h[0] - ref[0]) > STUCK_EPS or
                abs(h[1] - ref[1]) > STUCK_EPS or
                abs(h[2] - ref[2]) > STUCK_EPS):
            return False
    return True

# =============================================================================
# MEASUREMENT
# =============================================================================

def read_gyro_retry(gyro, retries=5, delay_ms=50):
    global _gyro_history
    last_exc = None
    for attempt in range(retries):
        try:
            gx, gy, temp_c = gyro.read_gyro_temp_raw()
        except OSError as e:
            last_exc = e
            log(f"    [WARN] SPI chyba pri cteni gyra ({e}), pokus {attempt+1}/{retries}")
            time.sleep_ms(delay_ms)
            continue

        _gyro_history.append((gx, gy, temp_c))
        if len(_gyro_history) > STUCK_CHECK_COUNT:
            _gyro_history.pop(0)

        if _is_stuck(_gyro_history):
            log(f"    [VAROVANI] Poslednich {STUCK_CHECK_COUNT} cteni gyra je bit-presne "
                f"identickych (gX={gx:+.8f}, gY={gy:+.8f}, T={temp_c:+.2f}C) - "
                f"podezreni na zaseknutou SPI komunikaci (NE platna data). "
                f"Zkousim reinicializaci SPI/cipu (motor NEDOTCEN)...")
            try:
                gyro.reinit()
                log("    Reinicializace SPI/cipu OK.")
            except Exception as e2:
                log(f"    [CHYBA] Reinicializace SPI/cipu selhala: {e2}")
            _gyro_history = []
            last_exc = OSError("ADXRS290 zaseknute cteni (stuck data)")
            time.sleep_ms(delay_ms)
            continue

        return gx, gy, temp_c

    raise last_exc if last_exc is not None else OSError("ADXRS290 cteni trvale selhalo")

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
    xs, ys, ts = [], [], []
    axs, ays = [], []
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
        with open(CSV_FILE, "r") as f:
            pass
    except OSError:
        with open(CSV_FILE, "w") as f:
            f.write("cyklus;xA;yA;tA;axA;ayA;xC;yC;tC;axC;ayC\n")

# =============================================================================
# JEDEN MERICI CYKLUS - A (sever) -> C (jih, +180) -> zpet A (-180)
# =============================================================================

def run_single_cycle(gyro, mpu, cycle_num, total_cycles, stats):
    log("\n" + "-"*65)
    log(f"CYKLUS {cycle_num}/{total_cycles}")
    log("-"*65)

    success = False
    for cycle_attempt in range(1, 4):
        try:
            A_X, A_Y, A_T, A_AX, A_AY = measure(gyro, mpu, "A (sever)")

            log("\nPresouvam A->C (+180 deg, anti-backlash)...")
            move_relative(180)

            C_X, C_Y, C_T, C_AX, C_AY = measure(gyro, mpu, "C (jih)")

            log("\nPresouvam C->A (navrat, -180 deg, anti-backlash)...")
            move_relative(-180)

            with open(CSV_FILE, "a") as f:
                f.write(f"{cycle_num};"
                        f"{A_X:.8f};{A_Y:.8f};{A_T:.2f};{A_AX:.6f};{A_AY:.6f};"
                        f"{C_X:.8f};{C_Y:.8f};{C_T:.2f};{C_AX:.6f};{C_AY:.6f}\n")

            stats['n']   += 1
            stats['xA']  += A_X;  stats['xC']  += C_X
            stats['yA']  += A_Y;  stats['yC']  += C_Y
            stats['tA']  += A_T;  stats['tC']  += C_T
            stats['axA'] += A_AX; stats['axC'] += C_AX
            stats['ayA'] += A_AY; stats['ayC'] += C_AY

            log(f"\n  Cyklus {cycle_num} zapsan do CSV.")
            success = True
            break
        except OSError as e:
            log(f"  [CHYBA] Cyklus {cycle_num} selhal na pokusu {cycle_attempt}/3: {e}")
            time.sleep(1)

    if not success:
        log(f"  [CHYBA] Cyklus {cycle_num} TRVALE selhal po 3 pokusech - "
            f"PRESKAKUJI a pokracuji dalsim cyklem.")

def write_summary(stats):
    n = stats['n']
    with open(SUMMARY_FILE, "w") as f:
        f.write("n;xA_prumer;xC_prumer;diffx_prumer;"
                 "yA_prumer;yC_prumer;diffy_prumer;"
                 "tA_prumer;tC_prumer;difft_prumer;"
                 "axA_prumer;axC_prumer;diffax_prumer;"
                 "ayA_prumer;ayC_prumer;diffay_prumer\n")
        if n == 0:
            log("\n[WARN] Zadny uspesny cyklus - summary bude prazdny.")
            return
        xA_m  = stats['xA']  / n;  xC_m  = stats['xC']  / n
        yA_m  = stats['yA']  / n;  yC_m  = stats['yC']  / n
        tA_m  = stats['tA']  / n;  tC_m  = stats['tC']  / n
        axA_m = stats['axA'] / n;  axC_m = stats['axC'] / n
        ayA_m = stats['ayA'] / n;  ayC_m = stats['ayC'] / n
        f.write(f"{n};{xA_m:.8f};{xC_m:.8f};{xA_m - xC_m:.8f};"
                f"{yA_m:.8f};{yC_m:.8f};{yA_m - yC_m:.8f};"
                f"{tA_m:.2f};{tC_m:.2f};{tA_m - tC_m:.2f};"
                f"{axA_m:.8f};{axC_m:.8f};{axA_m - axC_m:.8f};"
                f"{ayA_m:.8f};{ayC_m:.8f};{ayA_m - ayC_m:.8f}\n")
    log(f"\nSouhrnny CSV zapsan: {SUMMARY_FILE}")

# =============================================================================
# MAIN
# =============================================================================

def main():
    with open(LOG_FILE, "w") as f:
        f.write(f"=== EARTH ROTATION LOG - TESTID={TESTID} (fixni smer A/C) ===\n")

    time.sleep(1)
    beep(count=2)

    log("="*65)
    log(f"EARTH ROTATION MEASUREMENT - TESTID = {TESTID}")
    log("FIXNI SMER: pozice A (sever) <-> C (jih, 180 deg), stale dokola")
    log("NEMA17 + A4988 + ADXRS290 (X,Y + teplota) + MPU6050 (kontrola naklonu)")
    log("="*65)
    log("\nPOZOR: pozice A (azimut 0 deg = sever) musi byt pred spustenim")
    log("nastavena RUCNE, sestava vyrovnana a od ted uz s ni NEHYBAT.")
    log("Vsechny dalsi presuny dela vyhradne motor (+180/-180 stridave).")
    log("Cisty pohyb za cyklus je 0 deg - zadna ochrana kabelaze neni potreba.")

    log("\nInitializing SPI1...")
    spi1 = SPI(SPI_ID, baudrate=1_000_000, polarity=1, phase=1,
               sck=Pin(SPI_SCK), mosi=Pin(SPI_MOSI), miso=Pin(SPI_MISO))
    gyro = ADXRS290(spi1, SPI_CS_PIN, spi_id=SPI_ID, sck=SPI_SCK, mosi=SPI_MOSI, miso=SPI_MISO)

    log("\nInitializing I2C0 (MPU6050)...")
    i2c0 = I2C(I2C_ID, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400_000)
    mpu = MPU6050(i2c0)

    wait_for_gyro_warmup(gyro, timeout_s=WARMUP_TIMEOUT_S)
    _gyro_history.clear()  # cista historie pred ostrym merenim (warm-up cteni se nepocitaji)

    csv_header_once()

    stats = {
        'n': 0, 'xA': 0.0, 'xC': 0.0, 'yA': 0.0, 'yC': 0.0,
        'tA': 0.0, 'tC': 0.0, 'axA': 0.0, 'axC': 0.0, 'ayA': 0.0, 'ayC': 0.0
    }

    log("\n" + "="*65)
    log(f"CELKEM {TOTAL_CYCLES} cyklu A<->C (fixni smer, sever/jih)")
    log(f"APPROACH_OVERSHOOT_DEG = {APPROACH_OVERSHOOT_DEG}, APPROACH_SIGN = {APPROACH_SIGN:+d}")
    log(f"FIXNI settle = {FIXED_SETTLE_S:.1f}s, FIXNI trim = {FIXED_TRIM}")
    log("="*65)

    for cycle_num in range(1, TOTAL_CYCLES + 1):
        run_single_cycle(gyro, mpu, cycle_num, TOTAL_CYCLES, stats)
        if cycle_num % 20 == 0:
            beep(count=1, freq=2000, duration_ms=100)
            log(f"\n[POSTUP] Dokonceno {cycle_num}/{TOTAL_CYCLES} cyklu.")

    write_summary(stats)

    log("\n" + "="*65)
    log(f"MERENI DOKONCENO - TESTID = {TESTID}")
    log("="*65)
    motor_enable(False)
    log("Motor vypnut")
    beep(count=4)

main()
