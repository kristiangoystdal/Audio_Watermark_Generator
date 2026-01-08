#ifndef FREQUENCY_PAIRS_H
#define FREQUENCY_PAIRS_H

//-----------------------------------------------------------//
//------------ DO NOT MODIFY BELOW THIS FILE ----------------//
//-----------------------------------------------------------//

#define MAX_NUM_SAMPLES_BUFFER 720
#define FS_HZ 1000000u
#define MIN_BIT_US 3000u
#define MIN_BIT_SAMPLES ((FS_HZ * MIN_BIT_US) / 1000000u) // = 3000
#define MAX_BIT_DURATION_SAMPLES MIN_BIT_SAMPLES

#endif // FREQUENCY_PAIRS_H