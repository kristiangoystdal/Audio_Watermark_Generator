from helper import *

file_names = find_txt_files("rtc_measurements")


def time_to_seconds(time_str):
    h, m, s = map(int, time_str.split(":"))
    return h * 3600 + m * 60 + s


# Compare the real time to the module time to calculate precision and drift
for file_name in file_names:
    with open(file_name, "r") as f:
        data = f.read()
    lines = data.strip().split("\n")
    real_start_time = None
    module_start_time = None
    real_end_time = None
    module_end_time = None

    for line in lines:
        if "Real:" in line and "Module:" in line:
            parts = line.split(",")
            real_time_part = parts[0].split("Real:")[1].strip()
            module_time_part = parts[1].split("Module:")[1].strip()

            if not real_start_time:
                real_start_time = real_time_part
                module_start_time = module_time_part
            else:
                real_end_time = real_time_part
                module_end_time = module_time_part

    real_elapsed = time_to_seconds(real_end_time) - time_to_seconds(real_start_time)

    # Handle next-day wraparound
    if real_elapsed < 0:
        real_elapsed += 86400

    module_elapsed = time_to_seconds(module_end_time) - time_to_seconds(
        module_start_time
    )

    # Handle next-day wraparound
    if module_elapsed < 0:
        module_elapsed += 86400

    drift = module_elapsed - real_elapsed
    print(f"File: {file_name}")
    print(f"First Measurement: Real={real_start_time}, Module={module_start_time}")
    print(f"Last Measurement: Real={real_end_time}, Module={module_end_time}")
    print(f"Real Elapsed Time: {real_elapsed} seconds")
    print(f"Module Elapsed Time: {module_elapsed} seconds")
    print(f"Drift over time: {drift} seconds over {real_elapsed} seconds")
    print(f"Drift rate: {drift / real_elapsed:.6f} seconds per second")
    print(f"Drift rate: {drift / (real_elapsed / 60):.6f} seconds per minute")
    print(f"Drift rate: {drift / (real_elapsed / 3600):.6f} seconds per hour")
    print(f"Drift rate: {(drift / real_elapsed) * 86400:.6f} seconds per day")
    print(f"Drift rate: {(drift / real_elapsed) * 604800:.6f} seconds per week")
    print(f"Drift rate: {(drift / real_elapsed) * 2_592_000:.6f} seconds per month")
