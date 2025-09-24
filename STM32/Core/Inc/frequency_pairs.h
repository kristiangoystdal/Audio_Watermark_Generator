#ifndef FREQUENCY_PAIRS_H
#define FREQUENCY_PAIRS_H

//-----------------------------------------------------------//
//------------ DO NOT MODIFY BELOW THIS FILE ----------------//
//-----------------------------------------------------------//

#define MAX_NUM_SAMPLES_BUFFER 720

// ---------------- Frequency Pair 1 ---------------- //
// 21.0 kHz (S=48, PER_HALF=15)
// 22.2 kHz (S=45, PER_HALF=16)
#define SAMPLES_21000 48
#define SAMPLES_22222 45
#define PER_HALF_21000 (MAX_NUM_SAMPLES_BUFFER / SAMPLES_21000) // 15
#define PER_HALF_22222 (MAX_NUM_SAMPLES_BUFFER / SAMPLES_22222) // 16

// ---------------- Frequency Pair 2 ---------------- //
// 12.5 kHz (S=80, PER_HALF=9)
// 11.1 kHz (S=90, PER_HALF=8)
#define SAMPLES_12500 80
#define SAMPLES_11111 90
#define PER_HALF_12500 (MAX_NUM_SAMPLES_BUFFER / SAMPLES_12500) // 9
#define PER_HALF_11111 (MAX_NUM_SAMPLES_BUFFER / SAMPLES_11111) // 8

// ---------------- Frequency Pair 3 ---------------- //
// 8.3 kHz (S=120, PER_HALF=6)
// 6.9 kHz (S=144, PER_HALF=5)
#define SAMPLES_8333 120
#define SAMPLES_6944 144
#define PER_HALF_8333 (MAX_NUM_SAMPLES_BUFFER / SAMPLES_8333) // 6
#define PER_HALF_6944 (MAX_NUM_SAMPLES_BUFFER / SAMPLES_6944) // 5

// ---------------- Frequency Pair 4 ---------------- //
// 2.8 kHz (S=360, PER_HALF=2)
// 1.4 kHz (S=720, PER_HALF=1)
#define SAMPLES_2778 360
#define SAMPLES_1389 720
#define PER_HALF_2778 (MAX_NUM_SAMPLES_BUFFER / SAMPLES_2778) // 2
#define PER_HALF_1389 (MAX_NUM_SAMPLES_BUFFER / SAMPLES_1389) // 1

// ---------------- Selection Logic ---------------- //

#if FSK_FREQUENCY_PAIR == 1
#define MAX_SAMPLES_LOW SAMPLES_21000
#define MAX_SAMPLES_HIGH SAMPLES_22222
#define PER_HALF_LOW PER_HALF_21000
#define PER_HALF_HIGH PER_HALF_22222
#elif FSK_FREQUENCY_PAIR == 2
#define MAX_SAMPLES_LOW SAMPLES_12500
#define MAX_SAMPLES_HIGH SAMPLES_11111
#define PER_HALF_LOW PER_HALF_12500
#define PER_HALF_HIGH PER_HALF_11111
#elif FSK_FREQUENCY_PAIR == 3
#define MAX_SAMPLES_LOW SAMPLES_8333
#define MAX_SAMPLES_HIGH SAMPLES_6944
#define PER_HALF_LOW PER_HALF_8333
#define PER_HALF_HIGH PER_HALF_6944
#elif FSK_FREQUENCY_PAIR == 4
#define MAX_SAMPLES_LOW SAMPLES_2778
#define MAX_SAMPLES_HIGH SAMPLES_1389
#define PER_HALF_LOW PER_HALF_2778
#define PER_HALF_HIGH PER_HALF_1389
#else
#define MAX_SAMPLES_LOW SAMPLES_21000
#define MAX_SAMPLES_HIGH SAMPLES_22222
#define PER_HALF_LOW PER_HALF_21000
#define PER_HALF_HIGH PER_HALF_22222
#endif

#endif // FREQUENCY_PAIRS_H
