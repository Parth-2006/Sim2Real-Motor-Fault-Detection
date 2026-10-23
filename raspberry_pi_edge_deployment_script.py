import time
import math
import numpy as np
from scipy.signal import butter, filtfilt, welch, detrend
from smbus2 import SMBus
from gpiozero import Button

# =========================================================================
# 1. HARDWARE INITIALIZATION (ADXL345 I2C + TCRT5000 GPIO)
# =========================================================================
I2C_BUS = 1
ADXL345_ADDR = 0x53 # Locked when SDO is connected to GND and CS is connected to 3.3V

# ADXL345 Registers
REG_BW_RATE     = 0x2C
REG_POWER_CTL   = 0x2D
REG_DATA_FORMAT = 0x31
REG_DATAX0      = 0x32

bus = SMBus(I2C_BUS)

def init_adxl345():
    try:
        # Set +/- 16g range, full resolution (4mg/LSB)
        bus.write_byte_data(ADXL345_ADDR, REG_DATA_FORMAT, 0x0B)
        # Set Output Data Rate to 800 Hz (0x0D)
        bus.write_byte_data(ADXL345_ADDR, REG_BW_RATE, 0x0D)
        # Enable Measurement Mode
        bus.write_byte_data(ADXL345_ADDR, REG_POWER_CTL, 0x08)
        print("[OK] ADXL345 Accelerometer initialized at address 0x53")
    except Exception as e:
        print(f"[ERROR] Could not find ADXL345 at 0x53! Check CS->3.3V and SDA/SCL wiring. Details: {e}")
        exit(1)

def read_adxl345_amag():
    # Read 6 bytes of X, Y, Z data in a single I2C burst
    data = bus.read_i2c_block_data(ADXL345_ADDR, REG_DATAX0, 6)
    
    # Convert 2's complement 16-bit integers
    x = int.from_bytes(data[0:2], byteorder='little', signed=True)
    y = int.from_bytes(data[2:4], byteorder='little', signed=True)
    z = int.from_bytes(data[4:6], byteorder='little', signed=True)
    
    # Scale to m/s^2 (0.004 g/LSB * 9.80665 m/s^2)
    scale = 0.004 * 9.80665
    ax, ay, az = x * scale, y * scale, z * scale
    
    return math.sqrt(ax*ax + ay*ay + az*az)

# Initialize TCRT5000 Optical Sensor on GPIO 17 (Physical Pin 11)
pulse_count = 0
def on_shaft_pulse():
    global pulse_count
    pulse_count += 1

tcrt_sensor = Button(17, pull_up=True)
tcrt_sensor.when_pressed = on_shaft_pulse

# =========================================================================
# 2. DSP PIPELINE SETUP (MATCHING MATLAB SIMULATION)
# =========================================================================
FS = 500  # 500 Hz stable Python polling rate over I2C
b, a = butter(4, [5, (FS/2) - 1], btype='bandpass', fs=FS)

init_adxl345()
print("Starting Real-Time 1-Second Window Acquisition... Press Ctrl+C to stop.\n")

# =========================================================================
# 3. CONTINUOUS REAL-TIME ACQUISITION & FEATURE EXTRACTION LOOP
# =========================================================================
try:
    while True:
        pulse_count = 0
        samples = np.zeros(FS)
        start_time = time.perf_counter()
        
        for i in range(FS):
            samples[i] = read_adxl345_amag()
            # Precise timing loop for 1/FS seconds
            target_time = start_time + ((i + 1) / FS)
            while time.perf_counter() < target_time:
                pass
        
        # Calculate actual RPM from TCRT5000 pulses in this 1-second window
        rpm = pulse_count * 60.0
        
        # --- MATLAB-Equivalent Preprocessing ---
        sig = detrend(samples)
        sig = filtfilt(b, a, sig)
        
        # --- Extract Key Time-Domain Features ---
        t_rms = np.sqrt(np.mean(sig**2))
        t_peak = np.max(np.abs(sig))
        t_p2p = np.max(sig) - np.min(sig)
        t_crest = t_peak / (t_rms + 1e-6)
        t_kurt = np.mean((sig / (t_rms + 1e-6))**4)
        
        # --- Extract Frequency-Domain Features (Welch PSD) ---
        f, pxx = welch(sig, fs=FS, window='hann', nperseg=min(256, FS))
        f_dom = f[np.argmax(pxx)]
        
        # --- Diagnostics (Hardware Rule-Based Fallback for Edge) ---
        if t_rms < 0.30:
            status = "Healthy"
        elif t_kurt > 4.5 or t_crest > 3.5:
            status = "Bearing_Fault"
        elif rpm < 500 and t_rms >= 0.30:
            status = "Winding_Fault"
        else:
            status = "Rotor_Imbalance / Misalignment"
        
        # Console Telemetry
        print(f"RPM: {rpm:5.0f} | RMS: {t_rms:6.3f} m/s^2 | Crest: {t_crest:5.2f} | "
              f"Kurt: {t_kurt:5.2f} | DomFreq: {f_dom:5.1f} Hz --> [{status}]")

except KeyboardInterrupt:
    print("\nMonitoring stopped cleanly.")