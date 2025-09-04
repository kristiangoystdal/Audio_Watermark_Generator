def calc_tim_settings(f_clk=80_000_000, f_target=1_000_000):
    """
    Calculate TIM2 prescaler and ARR values for a given target frequency.

    f_clk    : Timer input clock (Hz)
    f_target : Desired update frequency (Hz)

    Returns: (PSC, ARR, actual_freq, error)
    """
    best = None
    for psc in range(0, 65536):  # 16-bit prescaler
        arr = round(f_clk / ((psc + 1) * f_target) - 1)
        if 0 <= arr < 65536:
            actual = f_clk / ((psc + 1) * (arr + 1))
            error = abs(actual - f_target)
            if best is None or error < best[3]:
                best = (psc, arr, actual, error)
    return best  # (PSC, ARR, actual_freq, error)


def calc_table_length(f_sample, f_signal):
    """
    Calculate table length N for a desired output frequency.
    f_sample : sample clock (Hz)
    f_signal : desired output frequency (Hz)
    """
    return round(f_sample / f_signal)


if __name__ == "__main__":
    f_clk = 80_000_000  # TIM2 clock (Hz)
    f_target = 1_000_000  # Desired sample rate (Hz)
    f_signal1, f_signal2 = 1000, 100  # Example output frequencies (Hz)

    psc, arr, actual_fs, error = calc_tim_settings(f_clk, f_target)
    print(f"TIM2 settings for {f_target} Hz:")
    print(f"  Prescaler = {psc}, ARR = {arr}")
    print(f"  Actual Fs = {actual_fs:.2f} Hz (error {error:.2e})")

    print(f"\nSine table lengths for target frequencies:")
    N1 = calc_table_length(actual_fs, f_signal1)
    N2 = calc_table_length(actual_fs, f_signal2)
    print(f"  f={f_signal1} Hz -> N = {N1}")
    print(f"  f={f_signal2} Hz -> N = {N2}")
    print(f"\nActual output frequencies:")
    print(f"  N={N1} -> f = {actual_fs / N1:.2f} Hz")
    print(f"  N={N2} -> f = {actual_fs / N2:.2f} Hz")
