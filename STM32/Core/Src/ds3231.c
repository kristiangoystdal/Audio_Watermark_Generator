#include "ds3231.h"
#include "main.h" // for I2C handle

extern I2C_HandleTypeDef hi2c1; // defined in i2c.c

#define DS3231_ADDR (0x68 << 1) // 7-bit addr shifted for HAL

// Convert normal decimal numbers to binary coded decimal
static uint8_t decToBcd(int val) {
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

// Convert binary coded decimal to normal decimal numbers
static int bcdToDec(uint8_t val) {
    return (int)((val / 16 * 10) + (val % 16));
}

// Set time/date into DS3231
void Set_Time(uint8_t sec, uint8_t min, uint8_t hour,
              uint8_t dow, uint8_t dom, uint8_t month, uint16_t year)
{
    uint8_t set_time[7];
    set_time[0] = decToBcd(sec);             // Seconds
    set_time[1] = decToBcd(min);             // Minutes
    set_time[2] = decToBcd(hour) & 0x3F;     // Hours (24h mode)
    set_time[3] = decToBcd(dow);             // Day of week (1–7)
    set_time[4] = decToBcd(dom);             // Date (1–31)
    set_time[5] = decToBcd(month);           // Month (1–12)
    set_time[6] = decToBcd(year % 100);      // Year (00–99)

    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, set_time, 7, HAL_MAX_DELAY);
}

// Read time/date from DS3231
void Get_Time(rtc_time_t *time)
{
    uint8_t get_time[7];
    HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, get_time, 7, HAL_MAX_DELAY);

    time->seconds = bcdToDec(get_time[0] & 0x7F);
    time->minutes = bcdToDec(get_time[1] & 0x7F);
    time->hours   = bcdToDec(get_time[2] & 0x3F); // 24h
    time->day     = bcdToDec(get_time[3] & 0x07); // Day of week (1–7)
    time->date    = bcdToDec(get_time[4] & 0x3F);
    time->month   = bcdToDec(get_time[5] & 0x1F);
    time->year    = 2000 + bcdToDec(get_time[6]);
}

void DS3231_Init(void)
{
    uint8_t sec;

    // Read current seconds register
    if (HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, 0x00,
                         I2C_MEMADD_SIZE_8BIT, &sec, 1, HAL_MAX_DELAY) != HAL_OK) {
        printf("⚠️ DS3231_Init: read failed!\r\n");
        return;
    }

    // Clear CH (bit 7)
    sec &= 0x7F;

    // Write it back
    if (HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, 0x00,
                          I2C_MEMADD_SIZE_8BIT, &sec, 1, HAL_MAX_DELAY) != HAL_OK) {
        printf("⚠️ DS3231_Init: write failed!\r\n");
        return;
    }

    printf("✅ DS3231 initialized, oscillator running.\r\n");
}

