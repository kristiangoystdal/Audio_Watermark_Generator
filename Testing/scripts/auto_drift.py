import serial
import serial.tools.list_ports
from datetime import datetime
import time
import os


# Auto-detect port
def find_stm32_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(
            x in (p.description or p.hwid or p.device or "").lower()
            for x in ["stm32", "usbmodem", "0483:5740"]
        ):
            return p.device
    return None


print("[INFO] Waiting for STM32 to be plugged in...")
port = None
while not port:
    port = find_stm32_port()
    if not port:
        time.sleep(0.1)

print(f"[INFO] Found STM32 on {port}")

measurements = []
ser = None

# Create measurements folder
measurements_folder = "test_files/rtc_measurements/drift/module_yellow"
os.makedirs(measurements_folder, exist_ok=True)

# Create log file with timestamp
log_filename = os.path.join(
    measurements_folder, f"drift_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
)

try:
    with open(log_filename, "w") as log_file:
        log_file.write(f"RTC Drift Measurement Log\n")
        log_file.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        log_file.write(f"Port: {port}\n")
        log_file.write("=" * 60 + "\n\n")
        log_file.flush()

        print(f"[INFO] Connected to {port}")
        print("[INFO] Listening for triggers... (Ctrl+C to stop)")
        print(f"[INFO] Logging to {log_filename}")

        while True:
            try:
                # Reconnect if serial is closed
                if ser is None or not ser.is_open:
                    try:
                        ser = serial.Serial(port, 115200, timeout=0.001)
                        print(f"[INFO] Reconnected to {port}")
                        log_file.write(
                            f"[RECONNECT] {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                        )
                        log_file.flush()
                    except serial.SerialException as e:
                        print(f"[WARN] Reconnect failed: {e}, retrying in 1s...")
                        time.sleep(1)
                        continue

                line = ser.readline().decode("utf-8", errors="ignore").strip()

                if line:
                    print(f"[DEBUG] Raw line: {repr(line)}")

                # Remove null bytes and extra whitespace
                line = line.replace("\0", "").strip()

                if line:
                    print(f"[DEBUG] Cleaned line: {repr(line)}")

                if line and "TIMING CHECK:" in line:
                    print(f"[DEBUG] Found TIMING CHECK in line")
                    real_time_now = datetime.now()

                    # Extract module time (handle null bytes in split)
                    parts = line.split("TIMING CHECK:")
                    print(f"[DEBUG] Split parts: {parts}")

                    if len(parts) > 1:
                        module_time_str = parts[1].replace("\0", "").strip()
                        print(f"[DEBUG] Extracted time: {module_time_str}")

                        measurements.append(
                            {
                                "real_time": real_time_now,
                                "module_time_str": module_time_str,
                            }
                        )

                        output = f"[TRIGGER] Real: {real_time_now.strftime('%H:%M:%S')}, Module: {module_time_str}"
                        print(output)
                        log_file.write(output + "\n")
                        log_file.flush()

            except serial.SerialException as e:
                print(f"[WARN] Serial error: {e}, will reconnect...")
                if ser:
                    ser.close()
                ser = None
                time.sleep(1)
                continue
            except Exception as e:
                print(f"[ERROR] {e}")
                break

except KeyboardInterrupt:
    print("\n[INFO] Stopped listening")
finally:
    if ser:
        ser.close()

# Analyze drift
if len(measurements) >= 2:
    print("\n--- Drift Analysis ---")

    analysis = "\n--- Drift Analysis ---\n"
    for i, m in enumerate(measurements):
        line = f"Measurement {i+1}: Real={m['real_time'].strftime('%H:%M:%S')}, Module={m['module_time_str']}"
        print(line)
        analysis += line + "\n"

    first = measurements[0]
    last = measurements[-1]

    real_elapsed = (last["real_time"] - first["real_time"]).total_seconds()

    def time_to_seconds(time_str):
        h, m, s = map(int, time_str.split(":"))
        return h * 3600 + m * 60 + s

    module_elapsed = time_to_seconds(last["module_time_str"]) - time_to_seconds(
        first["module_time_str"]
    )

    drift = module_elapsed - real_elapsed
    drift_ppm = (drift / real_elapsed) * 1_000_000 if real_elapsed != 0 else 0

    results = f"""
Real elapsed:   {real_elapsed:.1f} seconds
Module elapsed: {module_elapsed} seconds
Drift:          {drift:.1f} seconds
Drift rate:     {drift_ppm:.2f} ppm
Per day:        {(drift_ppm / 1_000_000) * 86400:.2f} seconds/day
"""

    print(results)
    analysis += results

    with open(log_filename, "a") as log_file:
        log_file.write(analysis)

    print(f"\n[INFO] Results saved to {log_filename}")

elif len(measurements) == 1:
    print(f"\n[INFO] Only 1 measurement. Need at least 2 for drift analysis.")
else:
    print(f"\n[INFO] No measurements collected.")
