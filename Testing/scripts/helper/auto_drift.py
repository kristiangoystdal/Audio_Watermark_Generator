import serial
import serial.tools.list_ports
from datetime import datetime
import time
import os

LOG_DURATION_MINUTES = 0.1
MODULE_COLOR = "red"


def find_stm32_port():
    """Auto-detect STM32 port."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(
            x in (p.description or p.hwid or p.device or "").lower()
            for x in ["stm32", "usbmodem", "0483:5740"]
        ):
            return p.device
    return None


def time_to_hms(timestamp):
    """Convert float seconds to HH:MM:SS.mmm format."""
    dt = datetime.fromtimestamp(timestamp)
    millis = int((timestamp % 1) * 1000)
    return f"{dt.strftime('%H:%M:%S')}.{millis:03d}"


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
measurements_folder = f"test_files/rtc_measurements/drift_2/module_{MODULE_COLOR}"
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
        log_file.write("=" * 60 + "\n")
        log_file.write(
            "Timer: time.monotonic() (immune to NTP, anchored to wall clock at start)\n"
        )
        log_file.write("=" * 60 + "\n\n")
        log_file.flush()

        print(f"[INFO] Connected to {port}")
        print(
            f"[INFO] Listening for triggers — {LOG_DURATION_MINUTES} min timer starts on first check... (Ctrl+C to stop early)"
        )
        print(f"[INFO] Logging to {log_filename}")
        print(f"[INFO] Using time.monotonic() — immune to NTP jumps\n")

        end_time = None
        last_minute_print = None
        mono_start = None
        epoch_anchor = None

        while end_time is None or time.monotonic() < end_time:
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
                line = line.replace("\0", "").strip()

                if end_time is not None:
                    elapsed = time.monotonic() - (end_time - LOG_DURATION_MINUTES * 60)
                    pct = min(int(elapsed / (LOG_DURATION_MINUTES * 60) * 100), 100)
                    mins_done = int(elapsed / 60)
                    if last_minute_print != mins_done:
                        last_minute_print = mins_done
                        bar = "#" * (pct // 5) + "-" * (20 - pct // 5)
                        print(
                            f"[INFO] [{bar}] {pct}% ({mins_done}/{LOG_DURATION_MINUTES} min)"
                        )

                if line and "TIMING CHECK:" in line:
                    payload = (
                        line.split("TIMING CHECK:", 1)[1].replace("\0", "").strip()
                    )

                    # Parse: "HH:MM:SS TICK=xxxxx"
                    if " TICK=" in payload:
                        rtc_str, tick_str = payload.split(" TICK=", 1)
                        try:
                            firmware_tick = int(tick_str.strip())
                        except ValueError:
                            continue
                    else:
                        # Fallback: old firmware without TICK
                        rtc_str = payload
                        firmware_tick = None

                    if end_time is None:
                        end_time = time.monotonic() + LOG_DURATION_MINUTES * 60
                        mono_start = time.monotonic()
                        epoch_anchor = time.time()
                        print(
                            f"[INFO] First check received — logging for {LOG_DURATION_MINUTES} min from now."
                        )

                    real_timestamp = epoch_anchor + (time.monotonic() - mono_start)

                    measurements.append(
                        {
                            "firmware_tick": firmware_tick,
                            "real_time": real_timestamp,
                            "module_time_str": rtc_str,
                        }
                    )

                    real_time_hms = time_to_hms(real_timestamp)
                    output = f"[TRIGGER] Real: {real_time_hms}, Module: {rtc_str}, TICK={firmware_tick}"
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

        print(f"\n[INFO] {LOG_DURATION_MINUTES} min elapsed, stopping.")

except KeyboardInterrupt:
    print("\n[INFO] Stopped listening")
finally:
    if ser:
        ser.close()

print(f"[INFO] Logged {len(measurements)} measurements to {log_filename}")
