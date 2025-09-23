#!/usr/bin/env python3
"""
Calculate TIM8 pulse_time (CCR1, in ticks) needed to transmit your FSK frame,
mirroring your STM32 C code (calculate_pulse_time).

Edit USER_TEXT / clock settings below to match your firmware.
No arguments are used—everything is static.
"""

# ---------------------- User-editable "static" settings ----------------------

# Text payload (<= 48 chars). Will be wrapped with a 16-bit preamble at start/end.
USER_TEXT = "Hello World!"

# Clocks / prescaler (from your SystemClock_Config and TIM8 init)
# SystemClock_Config -> SYSCLK=64 MHz, AHBCLK=32 MHz, APB2=DIV1 => PCLK2=32 MHz
PCLK2_HZ = 32_000_000
APB2_DIV = 1  # if !=1, advanced timers get x2 clock
TIM8_PSC = 15999  # from MX_TIM8_Init

# ---------------------- Frame / waveform constants (match C) -----------------

NUM_CHARS = 48 + 4  # 48 data chars + 4 identifier chars
BITSTREAM_LENGTH = NUM_CHARS * 8

REPEAT_HALF = 4
PER_HALF_21K = 15
PER_HALF_22K = 16
PER_HALF_SIL = 18

PERIODS_PER_BIT_21K = PER_HALF_21K * REPEAT_HALF  # 60
PERIODS_PER_BIT_22K = PER_HALF_22K * REPEAT_HALF  # 64
PERIODS_PER_BIT_SIL = PER_HALF_SIL * REPEAT_HALF  # 72

FREQ_21K = 21000.0
FREQ_22K = 22000.0
FREQ_25K = 25000.0  # used for the 2 extra "silence bits" at end

# ---------------------- Bitstream helpers (mirrors your C) -------------------


def make_preamble(bits: list[int]) -> None:
    """Append 16-bit pattern 1010... (8 pairs of 1,0), exactly like your C."""
    for _ in range(8):
        bits.append(1)
        bits.append(0)


def make_bitstream_from_string(text: str) -> list[int]:
    """
    bitstream = [preamble16] + [text bytes MSB->LSB] + [preamble16], then pad with 2s
    to BITSTREAM_LENGTH. '2' means DC/mid marker for filling; it is ignored in timing sum.
    """
    if len(text) > 48:
        raise ValueError(
            "USER_TEXT longer than 48 chars (would overflow reserved frame)."
        )

    bits: list[int] = []
    make_preamble(bits)  # start identifier

    for ch in text:
        v = ord(ch) & 0xFF
        for b in range(7, -1, -1):
            bits.append((v >> b) & 1)

    make_preamble(bits)  # end identifier

    while len(bits) < BITSTREAM_LENGTH:
        bits.append(2)  # padding (ignored in timing sum)

    return bits


# ---------------------- Timing math (mirrors calculate_pulse_time) ----------


def compute_total_time_seconds(bits: list[int]) -> float:
    """
    Sum only 0/1 bits:
      0 -> PERIODS_PER_BIT_21K / 21k
      1 -> PERIODS_PER_BIT_22K / 22k
    Then add 2 * (PERIODS_PER_BIT_SIL / 25k).
    """
    if len(bits) < BITSTREAM_LENGTH:
        raise ValueError("Bitstream shorter than BITSTREAM_LENGTH.")
    total = 0.0
    for i in range(BITSTREAM_LENGTH):
        b = bits[i]
        if b == 0:
            total += PERIODS_PER_BIT_21K / FREQ_21K
        elif b == 1:
            total += PERIODS_PER_BIT_22K / FREQ_22K
        # b == 2 (padding) contributes nothing here
    total += 2.0 * (PERIODS_PER_BIT_SIL / FREQ_25K)
    return total


def tim8_tick_hz(pclk2_hz: int, apb2_div: int, tim8_psc: int) -> float:
    """
    Advanced timers double the timer clock when APB prescaler != 1.
    tick_hz = (PCLK2 * (2 if apb2_div!=1 else 1)) / (PSC+1)
    """
    tim8_clk = float(pclk2_hz) if apb2_div == 1 else float(pclk2_hz) * 2.0
    return tim8_clk / float(tim8_psc + 1)


def compute_pulse_ticks(total_time_s: float, tick_hz: float) -> int:
    """
    Convert seconds to ticks. C casts to uint32_t (truncate), so we mimic floor via int().
    Ensure minimum of 1 tick.
    """
    ticks = int(total_time_s * tick_hz)  # truncation like (uint32_t)(...)
    return max(1, ticks)


# ---------------------- Run ----------------------

if __name__ == "__main__":
    bits = make_bitstream_from_string(USER_TEXT)

    n0 = sum(1 for b in bits[:BITSTREAM_LENGTH] if b == 0)
    n1 = sum(1 for b in bits[:BITSTREAM_LENGTH] if b == 1)
    n2 = sum(1 for b in bits[:BITSTREAM_LENGTH] if b == 2)

    total_time_s = compute_total_time_seconds(bits)
    tick = tim8_tick_hz(PCLK2_HZ, APB2_DIV, TIM8_PSC)
    pulse_time = compute_pulse_ticks(total_time_s, tick)

    print("---- Frame / Timing Summary ----")
    print(f"USER_TEXT                    : {USER_TEXT!r}")
    print(f"BITSTREAM_LENGTH (bits)     : {BITSTREAM_LENGTH}")
    print(
        f"Contributing bits (0/1)     : n0={n0}, n1={n1}  (ignored/padding '2' bits={n2})"
    )
    print(
        f"Per-bit periods             : 21k={PERIODS_PER_BIT_21K}, 22k={PERIODS_PER_BIT_22K}, sil={PERIODS_PER_BIT_SIL}"
    )
    print(
        f"Frequencies (Hz)            : 21k={FREQ_21K}, 22k={FREQ_22K}, 25k={FREQ_25K}"
    )
    print(
        f"Total time (s)              : {total_time_s:.6f} s  ({total_time_s*1000:.2f} ms)"
    )
    print(
        f"TIM8 tick frequency         : {tick:.3f} Hz  (pclk2={PCLK2_HZ} Hz, apb2_div={APB2_DIV}, psc={TIM8_PSC})"
    )
    print(f"CCR1 (pulse_time in ticks)  : {pulse_time} tick(s)")
    print(f"Use: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, {pulse_time});")
    print(f"Then: __HAL_TIM_SET_COUNTER(&htim8, {pulse_time} - 10);")
