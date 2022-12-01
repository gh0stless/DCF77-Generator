//
//  www.blinkenlight.net
//
//  Copyright 2014 Udo Klein
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see http://www.gnu.org/licenses/
 
 
 
// Please take care to NOT amplify the output of pin 3.
// Also do not connect pin 3 to a tuned antenna.
 
// Stay within in legal bounds. You have been warned.
 
// http://www.mikrocontroller.net/articles/Allgemeinzuteilung
// http://www.bundesnetzagentur.de/SharedDocs/Downloads/DE/Sachgebiete/Telekommunikation/Unternehmen_Institutionen/Frequenzen/Allgemeinzuteilungen/2014_04_InduktiveFunkanwendungen_pdf.pdf?__blob=publicationFile&v=5
 
// In Germany this implies that you must not exceed
// 42 dBμA/m in a distance of 10m of your transmitter
// If you connect a 1k resistor to the output pin then
// the current will stay below 5 mA which will keep
// the output well below regulatory limits.


// extended by Andreas Schumm (gh0stless) 2021

#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>  //für RGB Display Shield
#include "RTClib.h"

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

RTC_DS3231 rtc;

int menu = 0;
int menu2 = 0;
bool setNewTime = false;
bool setAbbruch = false;

bool buttonup = false;
bool buttondown = false;
bool buttonselect = false;
bool firstdisplay = true;

int timeout =0;
bool DisplayTimeOut = true;

uint8_t buttons = 0;


 
namespace BCD {
    typedef union {
        struct {
            uint8_t lo:4;
            uint8_t hi:4;
        } digit;
 
        struct {
            uint8_t b0:1;
            uint8_t b1:1;
            uint8_t b2:1;
            uint8_t b3:1;
            uint8_t b4:1;
            uint8_t b5:1;
            uint8_t b6:1;
            uint8_t b7:1;
        } bit;
 
        uint8_t val;
    } bcd_t;
 
    void increment(bcd_t &value);
 
    bcd_t int_to_bcd(const uint8_t value);
    uint8_t bcd_to_int(const bcd_t value);
}
 
namespace Debug {
    void bcddigit(uint8_t data);
    void bcddigits(uint8_t data);
}
 
typedef union {
    struct {
        uint8_t b0:2;
        uint8_t b1:2;
        uint8_t b2:2;
        uint8_t b3:2;
    } signal;
    uint8_t byte;
} tData;
 
namespace DCF77 {
    typedef enum {
        long_tick  = 3,
        short_tick = 2,
        undefined  = 1,
        sync_mark  = 0
    } tick_t;
 
    typedef struct {
        BCD::bcd_t year;     // 0..99
        BCD::bcd_t month;    // 1..12
        BCD::bcd_t day;      // 1..31
        BCD::bcd_t weekday;  // Mo = 1, So = 7
        BCD::bcd_t hour;     // 0..23
        BCD::bcd_t minute;   // 0..59
        uint8_t second;      // 0..60
        bool uses_summertime           : 1;  // false -> wintertime, true, summertime
        bool uses_backup_antenna       : 1;  // typically false
        bool timezone_change_scheduled : 1;
        bool leap_second_scheduled     : 1;
 
        bool undefined_minute_output                    : 1;
        bool undefined_uses_summertime_output           : 1;
        bool undefined_uses_backup_antenna_output       : 1;
        bool undefined_timezone_change_scheduled_output : 1;
    } time_data_t;
 
    typedef void (*output_handler_t)(const DCF77::time_data_t &decoded_time);
 
    typedef enum {
        useless  = 0,  // waiting for good enough signal
        dirty    = 1,  // time data available but unreliable
        free     = 2,  // clock was once synced but now may deviate more than 200 ms, must not re-lock if valid phase is detected
        unlocked = 3,  // lock was once synced, inaccuracy below 200 ms, may re-lock if a valid phase is detected
        locked   = 4,  // no valid time data but clock driven by accurate phase
        synced   = 5   // best possible quality, clock is 100% synced
    } clock_state_t;
}
 
namespace DCF77_Encoder {
    // What *** exactly *** is the semantics of the "Encoder"?
    // It only *** encodes *** whatever time is set
    // It does never attempt to verify the data
 
    void reset(DCF77::time_data_t &now);
 
    uint8_t weekday(const DCF77::time_data_t &now);  // sunday == 0
    BCD::bcd_t bcd_weekday(const DCF77::time_data_t &now);  // sunday == 7
 
    DCF77::tick_t get_current_signal(const DCF77::time_data_t &now);
 
    // This will advance the second. It will consider the control
    // bits while doing so. It will NOT try to properly set the
    // control bits. If this is desired "autoset" must be called in
    // advance.
    void advance_second(DCF77::time_data_t &now);
 
    // The same but for the minute
    void advance_minute(DCF77::time_data_t &now);
 
    // This will set the weekday by evaluating the date.
    void autoset_weekday(DCF77::time_data_t &now);
 
    // This will set the control bits, as a side effect it sets the weekday
    // It will generate the control bits exactly like DCF77 would.
    // Look at the leap second and summer / wintertime transistions
    // to understand the subtle implications.
    void autoset_control_bits(DCF77::time_data_t &now);
 
    void debug(const DCF77::time_data_t &clock);
    void debug(const DCF77::time_data_t &clock, const uint16_t cycles);
 
    // Bit      Bezeichnung     Wert    Pegel   Bedeutung
    // 0        M                       0       Minutenanfang (
 
    // 1..14    n/a                             reserviert
 
    // 15       R                               Reserveantenne aktiv (0 inaktiv, 1 aktiv)
    // 16       A1                              Ankündigung Zeitzonenwechsel (1 Stunde vor dem Wechsel für 1 Stunde, d.h ab Minute 1)
    // 17       Z1               2              Zeitzonenbit Sommerzeit (MEZ = 0, MESZ = 1); also Zeitzone = UTC + 2*Z1 + Z2
    // 18       Z2               1              Zeitzonenbit Winterzeit (MEZ = 1, MESZ = 0); also Zeitzone = UTC + 2*Z1 + Z2
    // 19       A2                              Ankündigung einer Schaltsekunde (1 Stunde vor der Schaltsekunde für 1 Stunde, d.h. ab Minute 1)
 
    // 20       S                       1       Startbit für Zeitinformation
 
    // 21                        1              Minuten  1er
    // 22                        2              Minuten  2er
    // 23                        4              Minuten  4er
    // 24                        8              Minuten  8er
    // 25                       10              Minuten 10er
    // 26                       20              Minuten 20er
    // 27                       40              Minuten 40er
    // 28       P1                              Prüfbit 1 (gerade Parität)
 
    // 29                        1              Stunden  1er
    // 30                        2              Stunden  2er
    // 31                        4              Stunden  4er
    // 32                        8              Stunden  8er
    // 33                       10              Stunden 10er
    // 34                       20              Stunden 20er
    // 35       P2                              Prüfbit 2 (gerade Parität)
 
    // 36                        1              Tag  1er
    // 37                        2              Tag  2er
    // 38                        4              Tag  4er
    // 39                        8              Tag  8er
    // 40                       10              Tag 10er
    // 41                       20              Tag 20er
 
    // 42                        1              Wochentag 1er (Mo = 1, Di = 2, Mi = 3,
    // 43                        2              Wochentag 2er (Do = 4, Fr = 5, Sa = 6,
    // 44                        4              Wochentag 4er (So = 7)
 
    // 45                        1              Monat  1er
    // 46                        2              Monat  2er
    // 47                        4              Monat  4er
    // 48                        8              Monat  8er
    // 49                       10              Monat 10er
 
    // 50                        1              Jahr  1er
    // 51                        2              Jahr  2er
    // 52                        4              Jahr  4er
    // 53                        8              Jahr  8er
    // 54                       10              Jahr 10er
    // 55                       20              Jahr 20er
    // 56                       40              Jahr 40er
    // 57                       80              Jahr 80er
 
    // 58       P3                              Prüftbit 3 (gerade Parität)
 
    // 59       sync                            Sync Marke, kein Impuls (übliches Minutenende)
    // 59                               0       Schaltsekunde (sehr selten, nur nach Ankündigung)
    // 60       sync                            Sync Marke, kein Impuls (nur nach Schaltsekunde)
 
    // Falls eine Schaltsekunde eingefügt wird, wird bei Bit 59 eine Sekundenmarke gesendet.
    // Der Syncimpuls erfolgt dann in Sekunde 60 statt 59. Üblicherweise wird eine 0 als Bit 59 gesendet
 
    // Üblicherweise springt die Uhr beim Wechsel Winterzeit nach Sommerzeit von 1:59:59 auf 3:00:00
    //                               beim Wechsel Sommerzeit nach Winterzeit von 2:59:59 auf 2:00:00
 
    // Die Zeitinformation wird immer 1 Minute im Vorraus übertragen. D.h. nach der Syncmarke hat
    // man die aktuelle Zeit
 
    // http://www.dcf77logs.de/SpecialFiles.aspx
 
    // Schaltsekunden werden in Deutschland von der Physikalisch-Technischen Bundesanstalt festgelegt,
    // die allerdings dazu nur die international vom International Earth Rotation and Reference Systems
    // Service (IERS) festgelegten Schaltsekunden übernimmt. Im Mittel sind Schaltsekunden etwa alle 18
    // Monate nötig und werden vorrangig am 31. Dezember oder 30. Juni, nachrangig am 31. März oder
    // 30. September nach 23:59:59 UTC (also vor 1:00 MEZ bzw. 2:00 MESZ) eingefügt. Seit der Einführung
    // des Systems 1972 wurden ausschließlich die Zeitpunkte im Dezember und Juni benutzt.
}
 
namespace Debug {
    void bcddigit(uint8_t data) {
        if (data <= 0x09) {
            Serial.print(data, HEX);
        } else {
            Serial.print('?');
        }
    }
 
    void bcddigits(uint8_t data) {
        bcddigit(data >>  4);
        bcddigit(data & 0xf);
    }
}
 
namespace BCD {
    void print(const bcd_t value) {
        Serial.print(value.val >> 4 & 0xF, HEX);
        Serial.print(value.val >> 0 & 0xF, HEX);
    }
 
    void increment(bcd_t &value) {
        if (value.digit.lo < 9) {
            ++value.digit.lo;
        } else {
            value.digit.lo = 0;
 
            if (value.digit.hi < 9) {
                ++value.digit.hi;
            } else {
                value.digit.hi = 0;
            }
        }
    }
 
    bcd_t int_to_bcd(const uint8_t value) {
        const uint8_t hi = value / 10;
 
        bcd_t result;
        result.digit.hi = hi;
        result.digit.lo = value-10*hi;
 
        return result;
    }
 
    uint8_t bcd_to_int(const bcd_t value) {
        return value.digit.lo + 10*value.digit.hi;
    }
}
 
namespace Arithmetic_Tools {
    template <uint8_t N> inline void bounded_increment(uint8_t &value) __attribute__((always_inline));
    template <uint8_t N>
    void bounded_increment(uint8_t &value) {
        if (value >= 255 - N) { value = 255; } else { value += N; }
    }
 
    template <uint8_t N> inline void bounded_decrement(uint8_t &value) __attribute__((always_inline));
    template <uint8_t N>
    void bounded_decrement(uint8_t &value) {
        if (value <= N) { value = 0; } else { value -= N; }
    }
 
    inline void bounded_add(uint8_t &value, const uint8_t amount) __attribute__((always_inline));
    void bounded_add(uint8_t &value, const uint8_t amount) {
        if (value >= 255-amount) { value = 255; } else { value += amount; }
    }
 
    inline void bounded_sub(uint8_t &value, const uint8_t amount) __attribute__((always_inline));
    void bounded_sub(uint8_t &value, const uint8_t amount) {
        if (value <= amount) { value = 0; } else { value -= amount; }
    }
 
    inline uint8_t bit_count(const uint8_t value) __attribute__((always_inline));
    uint8_t bit_count(const uint8_t value) {
        const uint8_t tmp1 = (value & 0b01010101) + ((value>>1) & 0b01010101);
        const uint8_t tmp2 = (tmp1  & 0b00110011) + ((tmp1>>2) & 0b00110011);
        return (tmp2 & 0x0f) + (tmp2>>4);
    }
 
    inline uint8_t parity(const uint8_t value) __attribute__((always_inline));
    uint8_t parity(const uint8_t value) {
        uint8_t tmp = value;
 
        tmp = (tmp & 0xf) ^ (tmp >> 4);
        tmp = (tmp & 0x3) ^ (tmp >> 2);
        tmp = (tmp & 0x1) ^ (tmp >> 1);
 
        return tmp;
    }
 
    void minimize(uint8_t &minimum, const uint8_t value) {
        if (value < minimum) {
            minimum = value;
        }
    }
 
    void maximize(uint8_t &maximum, const uint8_t value) {
        if (value > maximum) {
            maximum = value;
        }
    }
 
    uint8_t set_bit(const uint8_t data, const uint8_t number, const uint8_t value) {
        return value? data|(1<<number): data&~(1<<number);
    }
}
 
namespace DCF77_Encoder {
    using namespace DCF77;
 
    inline uint8_t days_per_month(const DCF77::time_data_t &now) __attribute__((always_inline));
    uint8_t days_per_month(const DCF77::time_data_t &now) {
        switch (now.month.val) {
            case 0x02:
                // valid till 31.12.2399
                // notice year mod 4 == year & 0x0d
                return 28 + ((now.year.val != 0) && ((bcd_to_int(now.year) & 0x03) == 0)? 1: 0);
            case 0x01: case 0x03: case 0x05: case 0x07: case 0x08: case 0x10: case 0x12: return 31;
            case 0x04: case 0x06: case 0x09: case 0x11:                                  return 30;
            default: return 0;
        }
    }
 
    void reset(DCF77::time_data_t &now) {
        now.second      = 0;
        now.minute.val  = 0x00;
        now.hour.val    = 0x00;
        now.day.val     = 0x01;
        now.month.val   = 0x01;
        now.year.val    = 0x00;
        now.weekday.val = 0x01;
        now.uses_summertime           = false;
        now.uses_backup_antenna       = false;
        now.timezone_change_scheduled = false;
        now.leap_second_scheduled     = false;
 
        now.undefined_minute_output                    = false;
        now.undefined_uses_summertime_output           = false;
        now.undefined_uses_backup_antenna_output       = false;
        now.undefined_timezone_change_scheduled_output = false;
    }
 
    uint8_t weekday(const DCF77::time_data_t &now) {  // attention: sunday will be ==0 instead of 7
        using namespace BCD;
 
        if (now.day.val <= 0x31 && now.month.val <= 0x12 && now.year.val <= 0x99) {
            // This will compute the weekday for each year in 2001-2099.
            // If you really plan to use my code beyond 2099 take care of this
            // on your own. My assumption is that it is even unclear if DCF77
            // will still exist then.
 
            // http://de.wikipedia.org/wiki/Gau%C3%9Fsche_Wochentagsformel
            const uint8_t  d = bcd_to_int(now.day);
            const uint16_t m = now.month.val <= 0x02? now.month.val + 10:
                                                      bcd_to_int(now.month) - 2;
            const uint8_t  y = bcd_to_int(now.year) - (now.month.val <= 0x02);
            // m must be of type uint16_t otherwise this will compute crap
            uint8_t day_mod_7 = d + (26*m - 2)/10 + y + y/4;
            // We exploit 8 mod 7 = 1
            while (day_mod_7 >= 7) {
                day_mod_7 -= 7;
                day_mod_7 = (day_mod_7 >> 3) + (day_mod_7 & 7);
            }
 
            return day_mod_7;  // attention: sunday will be == 0 instead of 7
        } else {
            return 0xff;
        }
    }
 
    BCD::bcd_t bcd_weekday(const DCF77::time_data_t &now) {
        BCD::bcd_t today;
 
        today.val = weekday(now);
        if (today.val == 0) {
            today.val = 7;
        }
 
        return today;
    }
 
    void autoset_weekday(DCF77::time_data_t &now) {
        now.weekday = bcd_weekday(now);
    }
 
    void autoset_timezone(DCF77::time_data_t &now) {
        // timezone change may only happen at the last sunday of march / october
        // the last sunday is always somewhere in [25-31]
 
        // Wintertime --> Summertime happens at 01:00 UTC == 02:00 CET == 03:00 CEST,
        // Summertime --> Wintertime happens at 01:00 UTC == 02:00 CET == 03:00 CEST
 
        if (now.month.val < 0x03) {
            // January or February
            now.uses_summertime = false;
        } else
        if (now.month.val == 0x03) {
            // March
            if (now.day.val < 0x25) {
                // Last Sunday of March must be 0x25-0x31
                // Thus still to early for summertime
                now.uses_summertime = false;
            } else
            if (uint8_t wd = weekday(now)) {
                // wd != 0 --> not a Sunday
                if (now.day.val - wd < 0x25) {
                    // early March --> wintertime
                    now.uses_summertime = false;
                } else {
                    // late march summertime
                    now.uses_summertime = true;
                }
            } else {
                // last sunday of march
                // decision depends on the current hour
                now.uses_summertime = (now.hour.val > 2);
            }
        } else
        if (now.month.val < 0x10) {
            // April - September
            now.uses_summertime = true;
        } else
        if (now.month.val == 0x10) {
            // October
            if (now.day.val < 0x25) {
                // early October
                now.uses_summertime = true;
            } else
            if (uint8_t wd = weekday(now)) {
                // wd != 0 --> not a Sunday
                if (now.day.val - wd < 0x25) {
                    // early October --> summertime
                    now.uses_summertime = true;
                } else {
                    // late October --> wintertime
                    now.uses_summertime = false;
                }
            } else {  // last sunday of october
                if (now.hour.val == 2) {
                    // can not derive the flag from time data
                    // this is the only time the flag is derived
                    // from the flag vector
 
                } else {
                    // decision depends on the current hour
                    now.uses_summertime = (now.hour.val < 2);
                }
            }
        } else {
            // November and December
            now.uses_summertime = false;
        }
    }
 
    void autoset_timezone_change_scheduled(DCF77::time_data_t &now) {
        // summer/wintertime change will always happen
        // at clearly defined hours
        // http://www.gesetze-im-internet.de/sozv/__2.html
 
        // in doubt have a look here: http://www.dcf77logs.de/
        if (now.day.val < 0x25 || weekday(now) != 0) {
            // timezone change may only happen at the last sunday of march / october
            // the last sunday is always somewhere in [25-31]
 
            // notice that undefined (==0xff) day/weekday data will not cause any action
            now.timezone_change_scheduled = false;
        } else {
            if (now.month.val == 0x03) {
                if (now.uses_summertime) {
                    now.timezone_change_scheduled = (now.hour.val == 0x03 && now.minute.val == 0x00); // wintertime to summertime, preparing first minute of summertime
                } else {
                    now.timezone_change_scheduled = (now.hour.val == 0x01 && now.minute.val != 0x00); // wintertime to summertime
                }
            } else if (now.month.val == 0x10) {
                if (now.uses_summertime) {
                    now.timezone_change_scheduled = (now.hour.val == 0x02 && now.minute.val != 0x00); // summertime to wintertime
                } else {
                    now.timezone_change_scheduled = (now.hour.val == 0x02 && now.minute.val == 0x00); // summertime to wintertime, preparing first minute of wintertime
                }
            } else if (now.month.val <= 0x12) {
                now.timezone_change_scheduled = false;
            }
        }
    }
 
    void verify_leap_second_scheduled(DCF77::time_data_t &now) {
        // If day or month are unknown we default to "no leap second" because this is alway a very good guess.
        // If we do not know for sure we are either acquiring a lock right now --> we will easily recover from a wrong guess
        // or we have very noisy data --> the leap second bit is probably noisy as well --> we should assume the most likely case
 
        now.leap_second_scheduled &= (now.day.val == 0x01);
 
        // leap seconds will always happen at 00:00 UTC == 01:00 CET == 02:00 CEST
        if (now.month.val == 0x01) {
            now.leap_second_scheduled &= ((now.hour.val == 0x00 && now.minute.val != 0x00) ||
                                          (now.hour.val == 0x01 && now.minute.val == 0x00));
        } else if (now.month.val == 0x07 || now.month.val == 0x04 || now.month.val == 0x10) {
            now.leap_second_scheduled &= ((now.hour.val == 0x01 && now.minute.val != 0x00) ||
                                          (now.hour.val == 0x02 && now.minute.val == 0x00));
        } else {
            now.leap_second_scheduled = false;
        }
    }
 
    void autoset_control_bits(DCF77::time_data_t &now) {
        autoset_weekday(now);
        autoset_timezone(now);
        autoset_timezone_change_scheduled(now);
        // we can not compute leap seconds, we can only verify if they might happen
        verify_leap_second_scheduled(now);
    }
 
    void advance_second(DCF77::time_data_t &now) {
        // in case some value is out of range it will not be advanced
        // this is on purpose
        if (now.second < 59) {
            ++now.second;
            if (now.second == 15) {
                autoset_control_bits(now);
            }
 
        } else if (now.leap_second_scheduled && now.second == 59 && now.minute.val == 0x00) {
            now.second = 60;
            now.leap_second_scheduled = false;
        } else if (now.second == 59 || now.second == 60) {
            now.second = 0;
            advance_minute(now);
        }
    }
 
    void advance_minute(DCF77::time_data_t &now) {
        if (now.minute.val < 0x59) {
            increment(now.minute);
        } else if (now.minute.val == 0x59) {
            now.minute.val = 0x00;
            // in doubt have a look here: http://www.dcf77logs.de/
            if (now.timezone_change_scheduled && !now.uses_summertime && now.hour.val == 0x01) {
                // Wintertime --> Summertime happens at 01:00 UTC == 02:00 CET == 03:00 CEST,
                // the clock must be advanced from 01:59 CET to 03:00 CEST
                increment(now.hour);
                increment(now.hour);
                now.uses_summertime = true;
            }  else if (now.timezone_change_scheduled && now.uses_summertime && now.hour.val == 0x02) {
                // Summertime --> Wintertime happens at 01:00 UTC == 02:00 CET == 03:00,
                // the clock must be advanced from 02:59 CEST to 02:00 CET
                now.uses_summertime = false;
            } else {
                if (now.hour.val < 0x23) {
                    increment(now.hour);
                } else if (now.hour.val == 0x23) {
                    now.hour.val = 0x00;
 
                    if (now.weekday.val < 0x07) {
                        increment(now.weekday);
                    } else if (now.weekday.val == 0x07) {
                        now.weekday.val = 0x01;
                    }
 
                    if (bcd_to_int(now.day) < days_per_month(now)) {
                        increment(now.day);
                    } else if (bcd_to_int(now.day) == days_per_month(now)) {
                        now.day.val = 0x01;
 
                        if (now.month.val < 0x12) {
                            increment(now.month);
                        } else if (now.month.val == 0x12) {
                            now.month.val = 0x01;
 
                            if (now.year.val < 0x99) {
                                increment(now.year);
                            } else if (now.year.val == 0x99) {
                                now.year.val = 0x00;
                            }
                        }
                    }
                }
            }
        }
    }
 
    DCF77::tick_t get_current_signal(const DCF77::time_data_t &now) {
        using namespace Arithmetic_Tools;
 
        if (now.second >= 1 && now.second <= 14) {
            // weather data or other stuff we can not compute
            return undefined;
        }
 
        bool result;
        switch (now.second) {
            case 0:  // start of minute
                return short_tick;
 
            case 15:
                if (now.undefined_uses_backup_antenna_output) { return undefined; }
                result = now.uses_backup_antenna; break;
 
            case 16:
                if (now.undefined_timezone_change_scheduled_output) { return undefined; }
                result = now.timezone_change_scheduled; break;
 
            case 17:
                if (now.undefined_uses_summertime_output) {return undefined; }
                result = now.uses_summertime; break;
 
            case 18:
                if (now.undefined_uses_summertime_output) {return undefined; }
                result = !now.uses_summertime; break;
 
            case 19:
                result = now.leap_second_scheduled; break;
 
            case 20:  // start of time information
                return long_tick;
 
            case 21:
                if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
                result = now.minute.digit.lo & 0x1; break;
            case 22:
                if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
                result = now.minute.digit.lo & 0x2; break;
            case 23:
                if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
                result = now.minute.digit.lo & 0x4; break;
            case 24:
                if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
                result = now.minute.digit.lo & 0x8; break;
 
            case 25:
                if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
                result = now.minute.digit.hi & 0x1; break;
            case 26:
                if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
                result = now.minute.digit.hi & 0x2; break;
            case 27:
                if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
                result = now.minute.digit.hi & 0x4; break;
 
            case 28:
                if (now.undefined_minute_output || now.minute.val > 0x59) { return undefined; }
                result = parity(now.minute.val); break;
 
 
            case 29:
                if (now.hour.val > 0x23) { return undefined; }
                result = now.hour.digit.lo & 0x1; break;
            case 30:
                if (now.hour.val > 0x23) { return undefined; }
                result = now.hour.digit.lo & 0x2; break;
            case 31:
                if (now.hour.val > 0x23) { return undefined; }
                result = now.hour.digit.lo & 0x4; break;
            case 32:
                if (now.hour.val > 0x23) { return undefined; }
                result = now.hour.digit.lo & 0x8; break;
 
            case 33:
                if (now.hour.val > 0x23) { return undefined; }
                result = now.hour.digit.hi & 0x1; break;
            case 34:
                if (now.hour.val > 0x23) { return undefined; }
                result = now.hour.digit.hi & 0x2; break;
 
            case 35:
                if (now.hour.val > 0x23) { return undefined; }
                result = parity(now.hour.val); break;
 
            case 36:
                if (now.day.val > 0x31) { return undefined; }
                result = now.day.digit.lo & 0x1; break;
            case 37:
                if (now.day.val > 0x31) { return undefined; }
                result = now.day.digit.lo & 0x2; break;
            case 38:
                if (now.day.val > 0x31) { return undefined; }
                result = now.day.digit.lo & 0x4; break;
            case 39:
                if (now.day.val > 0x31) { return undefined; }
                result = now.day.digit.lo & 0x8; break;
 
            case 40:
                if (now.day.val > 0x31) { return undefined; }
                result = now.day.digit.hi & 0x1; break;
            case 41:
                if (now.day.val > 0x31) { return undefined; }
                result = now.day.digit.hi & 0x2; break;
 
            case 42:
                if (now.weekday.val > 0x7) { return undefined; }
                result = now.weekday.val & 0x1; break;
            case 43:
                if (now.weekday.val > 0x7) { return undefined; }
                result = now.weekday.val & 0x2; break;
            case 44:
                if (now.weekday.val > 0x7) { return undefined; }
                result = now.weekday.val & 0x4; break;
 
            case 45:
                if (now.month.val > 0x12) { return undefined; }
                result = now.month.digit.lo & 0x1; break;
            case 46:
                if (now.month.val > 0x12) { return undefined; }
                result = now.month.digit.lo & 0x2; break;
            case 47:
                if (now.month.val > 0x12) { return undefined; }
                result = now.month.digit.lo & 0x4; break;
            case 48:
                if (now.month.val > 0x12) { return undefined; }
                result = now.month.digit.lo & 0x8; break;
 
            case 49:
                if (now.month.val > 0x12) { return undefined; }
                result = now.month.digit.hi & 0x1; break;
 
            case 50:
                if (now.year.val > 0x99) { return undefined; }
                result = now.year.digit.lo & 0x1; break;
            case 51:
                if (now.year.val > 0x99) { return undefined; }
                result = now.year.digit.lo & 0x2; break;
            case 52:
                if (now.year.val > 0x99) { return undefined; }
                result = now.year.digit.lo & 0x4; break;
            case 53:
                if (now.year.val > 0x99) { return undefined; }
                result = now.year.digit.lo & 0x8; break;
 
            case 54:
                if (now.year.val > 0x99) { return undefined; }
                result = now.year.digit.hi & 0x1; break;
            case 55:
                if (now.year.val > 0x99) { return undefined; }
                result = now.year.digit.hi & 0x2; break;
            case 56:
                if (now.year.val > 0x99) { return undefined; }
                result = now.year.digit.hi & 0x4; break;
            case 57:
                if (now.year.val > 0x99) { return undefined; }
                result = now.year.digit.hi & 0x8; break;
 
            case 58:
                if (now.weekday.val > 0x07 ||
                    now.day.val     > 0x31 ||
                    now.month.val   > 0x12 ||
                    now.year.val    > 0x99) { return undefined; }
 
                result = parity(now.day.digit.lo)   ^
                         parity(now.day.digit.hi)   ^
                         parity(now.month.digit.lo) ^
                         parity(now.month.digit.hi) ^
                         parity(now.weekday.val)    ^
                         parity(now.year.digit.lo)  ^
                         parity(now.year.digit.hi); break;
 
            case 59:
                // special handling for leap seconds
                if (now.leap_second_scheduled && now.minute.val == 0) { result = 0; break; }
                // standard case: fall through to "sync_mark"
            case 60:
                return sync_mark;
 
            default:
                return undefined;
        }
 
        return result? long_tick: short_tick;
    }
 
    void debug(const DCF77::time_data_t &clock) {
        using namespace Debug;
 
        Serial.print(F("  "));
        bcddigits(clock.year.val);
        Serial.print('.');
        bcddigits(clock.month.val);
        Serial.print('.');
        bcddigits(clock.day.val);
        Serial.print('(');
        bcddigit(clock.weekday.val);
        Serial.print(',');
        bcddigit(weekday(clock));
        Serial.print(')');
        bcddigits(clock.hour.val);
        Serial.print(':');
        bcddigits(clock.minute.val);
        Serial.print(':');
        if (clock.second < 10) {
            Serial.print('0');
        }
        Serial.print(clock.second, DEC);
        if (clock.uses_summertime) {
            Serial.print(F(" MESZ "));
        } else {
            Serial.print(F(" MEZ "));
        }
        if (clock.leap_second_scheduled) {
            Serial.print(F("leap second scheduled"));
        }
        if (clock.timezone_change_scheduled) {
            Serial.print(F("time zone change scheduled"));
        }
    }
}
 
namespace timer_2 {
    // 16000000 / 77500 = 206 + 70/155 = 206 + 14/31
    const uint8_t ticks_per_period = 206;
    // fractional ticks
    const uint8_t nominator   = 14;
    const uint8_t denominator = 31;
 
    const uint8_t pwm_full_carrier      = ticks_per_period / 2;   // ~50% duty cycle
    const uint8_t pwm_modulated_carrier = ticks_per_period / 44;  // 15% Amplitude ~ 1/44 Power
 
    void setup() {


        
         // disable timer2 interrupts during setup
        TIMSK2 = 0;
 
        // enable OC2B pin for output (digital pin 3)
        DDRD |= 1<<3;
 
        // The fast Pulse Width Modulation or fast PWM mode (WGM22:0 = 3 or 7) provides a high fre-
        // quency PWM waveform generation option. The fast PWM differs from the other PWM option by
        // its single-slope operation. The counter counts from BOTTOM to TOP then restarts from BOT-
        // TOM. TOP is defined as 0xFF when WGM2:0 = 3, and OCR2A when MGM2:0 = 7. In non-
        // inverting Compare Output mode, the Output Compare (OC2x) is cleared on the compare match
        // between TCNT2 and OCR2x, and set at BOTTOM. In inverting Compare Output mode, the out-
        // put is set on compare match and cleared at BOTTOM.
 
 
        TCCR2A = 1<< WGM20 | 1<<WGM21 | // Fast PWM
                 1<<COM2B1;             // Clear OC2B on Compare Match
 
        TCCR2B = 1<<CS20 |  // no Prescaler
                 1<<WGM22;  // Fast PWM
 
        OCR2A = ticks_per_period - 1;  // period length
 
        OCR2B = pwm_full_carrier;  // duty cycle
 
        TIMSK2 = 1<<OCIE2A;  // enable match interrupts
    }
}
 
ISR(TIMER2_COMPA_vect) {
    static uint8_t accumulated_fractional_ticks = 0;
 
    accumulated_fractional_ticks += timer_2::nominator;
    if (accumulated_fractional_ticks < timer_2::denominator) {
        OCR2A = timer_2::ticks_per_period - 1;
    } else {
        OCR2A = timer_2::ticks_per_period;
        accumulated_fractional_ticks -= timer_2::denominator;
    }
}
 
namespace timer_0 {
    void setup() {
        // disable timer 0 interrupts
        TIMSK0 = 0;
    }
}
 
namespace timer_1 {
    void setup() {
        TIMSK1 = 0;  // disable timer1 interrupts
 
        TCCR1A = 0;  // do not toggle or change timer IO pins
 
        // generate interrupts at 10 Hz
        OCR1A = 6250 - 1;  // (16000000 / 256 / 10) - 1
 
        TCCR1B = (1<<WGM12) |  // CTC using OCR1A
                 (1<<CS12);    // set prescaler to 256
 
        TIMSK1 = (1 << OCIE1A);  // enable OCR1A match interrupt
    }
}
 
uint8_t stop_modulation_after_times_100ms = 5;
DCF77::time_data_t now;
 
void modulate() {
    static uint8_t times_100ms = 0;
 
    if (times_100ms == 0) {
        // start of second
        if (times_100ms != stop_modulation_after_times_100ms) {
            OCR2B = timer_2::pwm_modulated_carrier;
        }
    }
 
    if (times_100ms == stop_modulation_after_times_100ms) {
        OCR2B = timer_2::pwm_full_carrier;
    }
 
    if (times_100ms < 9) {
        ++times_100ms;
    } else {
        // after 900ms)
        times_100ms = 0;
 
 
        DCF77::time_data_t l_now = now;
        DCF77_Encoder::advance_second(now);
        const DCF77::tick_t output_tick = DCF77_Encoder::get_current_signal(now);
        stop_modulation_after_times_100ms = output_tick == DCF77::long_tick ? 2:
                                            output_tick == DCF77::short_tick? 1:
                                            output_tick == DCF77::undefined?  1:
                                                                              0;
    }
}
 
ISR(TIMER1_COMPA_vect) {
    sei();
    // ensure that timer 2 can interrupt timer 1
    modulate();
}
 
namespace Parser {
    void show(const DCF77::time_data_t &now) {
          cli();
          const DCF77::time_data_t l_now = now;
          sei();
 
          Debug::bcddigits(l_now.year.val);
          Serial.print('.');
          Debug::bcddigits(l_now.month.val);
          Serial.print('.');
          Debug::bcddigits(l_now.day.val);
          Serial.print(' ');
 
          Debug::bcddigits(l_now.hour.val);
          Serial.print(':');
          Debug::bcddigits(l_now.minute.val);
          Serial.print(':');
          Serial.print(l_now.second / 10);
          Serial.print(l_now.second % 10);
          Serial.print(' ');
 
          Serial.print(l_now.weekday.val);
          Serial.print(' ');
          Serial.print(l_now.uses_summertime);
          Serial.print(l_now.uses_backup_antenna);
          Serial.print(l_now.timezone_change_scheduled);
          Serial.println(l_now.leap_second_scheduled);
 
    }
 
    char blocking_read() {
        while (!Serial.available()) {}
        const char c = Serial.read();
        /*
        Serial.print('(');
        Serial.print(c);
        Serial.print(')');
        */
        return c;
    }
 
    uint8_t get_flag(bool &valid) {
        if (valid) {
            const char c = blocking_read();
            if (c == '0' || c == '1') {
                return c - '0';
            } else {
                valid = false;
            }
        }
        return 0;
    }
 
    uint8_t get_digit(bool &valid) {
        if (valid) {
            const char c = blocking_read();
            if (c >= '0' && c <= '9') {
                return c - '0';
            } else {
                valid = false;
            }
        }
        return 0;
    }
 
    void assert_char(bool &valid, const char c) {
        valid = valid && blocking_read() == c;
    }
 
 
    bool parse_simple(DCF77::time_data_t &now) {
        bool valid = true;
 
        now.year.digit.hi = get_digit(valid);
        now.year.digit.lo = get_digit(valid);
        assert_char(valid, '.');
        now.month.digit.hi = get_digit(valid);
        now.month.digit.lo = get_digit(valid);
        assert_char(valid, '.');
        now.day.digit.hi = get_digit(valid);
        now.day.digit.lo = get_digit(valid);
        assert_char(valid, ' ');
 
        now.hour.digit.hi = get_digit(valid);
        now.hour.digit.lo = get_digit(valid);
        assert_char(valid, ':');
        now.minute.digit.hi = get_digit(valid);
        now.minute.digit.lo = get_digit(valid);
        assert_char(valid, ':');
        now.second  = get_digit(valid) * 10;
        now.second += get_digit(valid);
 
        return valid;
    }
 
    bool parse_extended(DCF77::time_data_t &now) {
        bool valid = parse_simple(now);
 
        assert_char(valid, ' ');
        now.weekday.val = get_digit(valid);
        assert_char(valid, ' ');
        now.uses_summertime = get_flag(valid);
        now.uses_backup_antenna = get_flag(valid);;
        now.timezone_change_scheduled = get_flag(valid);
        now.leap_second_scheduled = get_flag(valid);
 
        return valid;
    }
 
    void parse(DCF77::time_data_t &now) {
        
        const char mode = blocking_read();
 
        bool valid = false;
        DCF77::time_data_t new_time;
 
        switch (mode) {
            case 's':
                valid = parse_simple(new_time);
                if (valid) {
                    DCF77_Encoder::autoset_timezone(new_time);
                    DCF77_Encoder::autoset_weekday(new_time);
                }
                break;
            case 'x':
                valid = parse_extended(new_time);
                break;
        }
 
        if (valid) {
            cli();
            now = new_time;
            sei();
        } else {
            Serial.println();
            Serial.println();
            Serial.println(F("To set target time use one of the following formats"));
            Serial.println();
            Serial.println(F("simple mode:"));
            Serial.println(F("sYY.MM.DD hh:mm:ss"));
            Serial.println();
            Serial.println(F("extended mode:"));
            Serial.println(F("xYY.MM.DD hh:mm:ss w sbtl"));
            Serial.println();
        }
        Serial.println(F("current time setup (YY.MM.DD hh:mm.ss w sbtl)"));
        Serial.println(F("w = weekday, s = summertime, b = backup antenna, t = timzone change scheduled, l = leap second scheduled"));
        
        show(now);
    }
}

 void mybcddigit(uint8_t data) {
        if (data <= 0x09) {
            lcd.print(data, HEX);
        } else {
            lcd.print('?');
        }
 }
  
 void mybcddigits(uint8_t data) {
        mybcddigit(data >>  4);
        mybcddigit(data & 0xf);
}

unsigned char NumToBCD(unsigned char num)
{
      unsigned char a, b, bcd;
      
      a = (num % 10) & 0x0f;
      b = ((num / 10) << 4) & 0xf0;
      
      bcd = a | b;
      
      return bcd;
}

    unsigned char BCDToNum(unsigned char bcd)
    {
      unsigned char a, b;
      
      a = (bcd >> 4);
      b = bcd & 0x0f;
      
      return (a * 10 + b);
    }

 

// utility function for digital clock display: prints leading 0
void printLCDDigits(int digits){
    if(digits < 10)
      lcd.print('0');
    lcd.print(digits);
}



void LCDClockDisplay(){
     DCF77::time_data_t NewTime;
     cli();
     NewTime = now;
     sei();
    if (firstdisplay){
     lcd.setBacklight(0x7);
     timeout = (NewTime.second);
     timeout = timeout -1;
     if (timeout <= -1) timeout = 59;
     firstdisplay = false; 
    }
    else {
     int jetzt = (NewTime.second);
     if (timeout == jetzt && DisplayTimeOut){
        lcd.setBacklight(0x0);
     }
    }
    
    // digital clock display of the time on LCD
    lcd.setCursor(0, 0);
    mybcddigits(NewTime.hour.val);
    lcd.print(":");
    int korrekturminute = BCDToNum(NewTime.minute.val)-1;
    if (korrekturminute < 0) korrekturminute = 59;
    printLCDDigits(korrekturminute);
    //mybcddigits(NewTime.minute.val);
    lcd.print(":");
    printLCDDigits(NewTime.second);
    lcd.print("  ");
    lcd.print(rtc.getTemperature());
    lcd.print("\337");
    lcd.setCursor(0, 1);
    mybcddigits(NewTime.day.val);
    lcd.print(".");
    mybcddigits(NewTime.month.val);
    lcd.print(".20");
    mybcddigits(NewTime.year.val);
    lcd.print(" ");
      switch (NewTime.weekday.val){
        case 0:
          lcd.print("???");
          break;
        case 1:
          lcd.print("MON");
          break;
        case 2:
          lcd.print("DIE");
          break;
        case 3:
          lcd.print("MIT");
          break;
        case 4:
          lcd.print("DON");
          break;
        case 5:
          lcd.print("FRE");
          break;
        case 6:
          lcd.print("SAM");
          break;
        case 7:
          lcd.print("SON");
          break;
      }
      lcd.print(" ");
      if (NewTime.uses_summertime) lcd.print("S");
      else lcd.print("N");
}

void setDateAndTimeFromRTC(){
          DCF77::time_data_t NewTime;
          
          DateTime my_now = rtc.now();
          
          NewTime.year.val = NumToBCD(my_now.year()-2000);
          NewTime.month.val = NumToBCD(my_now.month());
          NewTime.day.val = NumToBCD(my_now.day());
          NewTime.hour.val = NumToBCD(my_now.hour());
          NewTime.minute.val = NumToBCD(my_now.minute());
          NewTime.second = my_now.second();
          NewTime.uses_backup_antenna = 0;
          NewTime.timezone_change_scheduled = 0;
          NewTime.leap_second_scheduled = 0;
          
          DCF77_Encoder::autoset_timezone(NewTime);
          DCF77_Encoder::autoset_weekday(NewTime);
    
          cli();
          now = NewTime;
          sei();
          
          menu = 0;
}
 
void setup() {

    DisplayTimeOut = (bool)EEPROM.read(0);
  
    timer_0::setup();
    timer_1::setup();
    timer_2::setup();
 
    Serial.begin(115200);
    #ifndef ESP8266
      while (!Serial); // wait for serial port to connect. Needed for native USB
    #endif

    if (! rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      abort();
    }

    if (rtc.lostPower()) {
      Serial.println("RTC lost power, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    // When time needs to be re-set on a previously configured device, the
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    
    // rtc.adjust(DateTime(2021, 10, 11, 16, 8, 0));
    
    // set up the LCD's number of columns and rows: 
    lcd.begin(16, 2);
    lcd.setBacklight(0x7);
 
    // compare to http://www.dcf77logs.de/ViewLog.aspx?mode=special&file=06%20-%20Schaltsekunde.log
    // set time to a leap second - because I always wanted to "see" one
    // - without staying up at late at night:)
    // geändert auf 7.7.21
    
    setDateAndTimeFromRTC();

    /*
    now.year.val = 0x21; 
    now.month.val = 0x8;
    now.day.val = 0x8;
    now.weekday.val = 7; 
    now.hour.val = 0x11;
    now.minute.val = 0x50;
    now.second = 0;
    now.uses_summertime = 1;
    now.uses_backup_antenna = 0;
    now.timezone_change_scheduled = 0;
    now.leap_second_scheduled = 0;
    */
}

void domenu() {
    buttons = lcd.readButtons();
    if (buttons) {
      lcd.setBacklight(0x7);
      firstdisplay = true;
      lcd.clear();
      lcd.setCursor(0,0);
      if (buttons & BUTTON_UP) {
        lcd.print("UP ");
        
      }
      if (buttons & BUTTON_DOWN) {
        lcd.print("DOWN ");
        
      }
      if (buttons & BUTTON_LEFT) {
        menu--;
        if (menu < 0) menu = 8;
        do {
          buttons = lcd.readButtons();
        }while (buttons & BUTTON_LEFT);
      }
      if (buttons & BUTTON_RIGHT) {
        menu++;
        if (menu > 8) menu = 0;
        do {
          buttons = lcd.readButtons();
        }while (buttons & BUTTON_RIGHT);
       
      }
      if (buttons & BUTTON_SELECT) {
        lcd.print("SELECT ");
      }
    }
}

void domenu2() {
      buttonup = false;
      buttondown = false;
      buttonselect = false;
      do {
          buttons = lcd.readButtons();
      }while (!buttons);
    if (buttons) {
      if (buttons & BUTTON_UP) {
        buttonup = true;
        buttondown = false;
      }
      if (buttons & BUTTON_DOWN) {
        buttonup = false;
        buttondown = true;
        
      }
      if (buttons & BUTTON_LEFT) {
        menu2--;
        if (menu2 < 0) menu2 = 6;
        do {
          buttons = lcd.readButtons();
        }while (buttons & BUTTON_LEFT);
      }
      if (buttons & BUTTON_RIGHT) {
        menu2++;
        if (menu2 > 6) menu2 = 0;
        do {
          buttons = lcd.readButtons();
        }while (buttons & BUTTON_RIGHT);
       
      }
      if (buttons & BUTTON_SELECT) {
        buttonselect = true;
      }
    }
}
    
void loop() {
    domenu();
    //Parser::parse(now);

    // in which subroutine should we go?
    switch (menu) {
      case 0: 
        LCDClockDisplay();
        break;
    
      case 1: 
        SetDateandTime();
        break;

      case 2: 
        LadeRTCZeit();
        break;
    
      case 3: 
       TestSchaltsekunde();
        break;
    
      case 4: 
        TestSommerzeit();
        break;
    
      case 5: 
        TestWinterzeit();
        break;
      case 6: 
        TestSchaltjahr();
        break;
      case 7: 
        EinstellungDisplay();
        break;
      case 8: 
        Ueber();
        break;
    }
}

void SetDateandTime(){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Zeit und Datum");
        lcd.setCursor(0,1);
        lcd.print("einstellen");
        do {
          buttons = lcd.readButtons();
        }while (!buttons);
        if(buttons & BUTTON_SELECT){

          DateTime my_now = rtc.now();
          int myYear = my_now.year()-2000;
          int myMonth= my_now.month();
          int myDay = my_now.day();
          int myHour = my_now.hour();
          int myMinute = my_now.minute();
          int mySecond = my_now.second();
          
          do {
              domenu2();
              lcd.clear();
              lcd.setCursor(0,0);
              switch (menu2) {
                  case 0: 
                    lcd.print("Eingabe Jahr:");
                    lcd.setCursor(0,1);
                    if (buttonup) myYear++;
                    if (buttondown) myYear--;
                    if (myYear > 99) myYear = 0;
                    if (myYear < 0) myYear = 99;
                    lcd.print(myYear);
                    break;
                
                  case 1: 
                    lcd.print("Eingabe Monat:");
                    lcd.setCursor(0,1);
                    if (buttonup) myMonth++;
                    if (buttondown) myMonth--;
                    if (myMonth > 12) myMonth = 1;
                    if (myMonth < 1) myMonth = 12;
                    lcd.print(myMonth);
                    break;
            
                  case 2: 
                    lcd.print("Eingabe Tag:");
                    lcd.setCursor(0,1);
                    if (buttonup) myDay++;
                    if (buttondown) myDay--;
                    if (myDay > 31) myDay = 1;
                    if (myDay < 1) myDay = 31;
                    lcd.print(myDay);
                    break;
                
                  case 3: 
                    lcd.print("Eingabe Stunde:");
                    lcd.setCursor(0,1);
                    if (buttonup) myHour++;
                    if (buttondown) myHour--;
                    if (myHour > 23) myHour = 0;
                    if (myHour < 0) myHour = 23;
                    lcd.print(myHour);
                    break;
                
                  case 4: 
                    lcd.print("Eingabe Minute:");
                    lcd.setCursor(0,1);
                    if (buttonup) myMinute++;
                    if (buttondown) myMinute--;
                    if (myMinute > 59) myMinute = 0;
                    if (myMinute < 0) myMinute = 59;
                    lcd.print(myMinute);
                    break;
                
                  case 5: 
                    lcd.print("Eingabe Sekunde:");
                    lcd.setCursor(0,1);
                    if (buttonup) mySecond++;
                    if (buttondown) mySecond--;
                    if (mySecond > 59) mySecond = 0;
                    if (mySecond < 0) mySecond = 59;
                    lcd.print(mySecond);
                    lcd.print("      <select>");
                    if (buttonselect) setNewTime = true;
                    break;

                  case 6: 
                    lcd.print("Abbruch");
                    lcd.setCursor(0,1);
                    lcd.print("        <select>");
                    if (buttonselect) setAbbruch = true;
              }          
          } while(!(setAbbruch||setNewTime));

           if (setNewTime){
              myMinute++;
              if (myMinute > 59){
                myMinute = 0;
                myHour ++;
                if (myHour > 23) {
                  myHour = 0;
                  myDay++;
                  if (myDay > 31) {
                    myDay = 1;
                    myMonth++;
                    if (myMonth > 12){
                      myMonth = 1;
                      myYear++;
                      if (myYear > 99) {
                        myYear = 0;
                      }
                    }
                  }
                }
             }
              DCF77::time_data_t NewTime;
              
              NewTime.year.val = NumToBCD(myYear);
              NewTime.month.val = NumToBCD(myMonth);
              NewTime.day.val = NumToBCD(myDay);
              NewTime.hour.val = NumToBCD(myHour);
              NewTime.minute.val = NumToBCD(myMinute);
              NewTime.second = mySecond; 
              NewTime.uses_backup_antenna = 0;
              NewTime.timezone_change_scheduled = 0;
              NewTime.leap_second_scheduled = 0;
              
              DCF77_Encoder::autoset_timezone(NewTime);
              DCF77_Encoder::autoset_weekday(NewTime);
              cli();
              now = NewTime;
              sei();
              
              rtc.adjust(DateTime(myYear, myMonth, myDay, myHour, myMinute, mySecond)); 
            }      
            menu = 0;
            menu2 = 0;
            setNewTime = false;
            setAbbruch = false;
        }
}

void LadeRTCZeit(){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Lade aktuelle");
        lcd.setCursor(0,1);
        lcd.print("Zeit und Datum");
        do {
          buttons = lcd.readButtons();
        }while (!buttons);
        if(buttons & BUTTON_SELECT){
          setDateAndTimeFromRTC();
          menu = 0;
        }
}

void TestSchaltsekunde(){
        DCF77::time_data_t NewTime;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Teste");
        lcd.setCursor(0,1);
        lcd.print("Schaltsekunde");
        do {
          buttons = lcd.readButtons();
        }while (!buttons);
        if(buttons & BUTTON_SELECT){
              NewTime.year.val = 0x09;
              NewTime.month.val = 0x01;
              NewTime.day.val = 0x01;
              NewTime.weekday.val = 4;
              NewTime.hour.val = 0x0;
              NewTime.minute.val = 0x52;
              NewTime.second = 0;
              NewTime.uses_summertime = 0;
              NewTime.uses_backup_antenna = 0;
              NewTime.timezone_change_scheduled = 0;
              NewTime.leap_second_scheduled = 1;
              cli();
              now = NewTime;
              sei();
              menu = 0;
        }
}

void TestSommerzeit(){
        DCF77::time_data_t NewTime;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Teste");
        lcd.setCursor(0,1);
        lcd.print("Sommerzeit");
        do {
          buttons = lcd.readButtons();
        }while (!buttons);
        if(buttons & BUTTON_SELECT){
              NewTime.year.val = 0x22;
              NewTime.month.val = 0x03;
              NewTime.day.val = 0x27;
              NewTime.weekday.val = 7;
              NewTime.hour.val = 0x1;
              NewTime.minute.val = 0x52;
              NewTime.second = 0;
              NewTime.uses_summertime = 0;
              NewTime.uses_backup_antenna = 0;
              NewTime.timezone_change_scheduled = 1;
              NewTime.leap_second_scheduled = 0;
              cli();
              now = NewTime;
              sei();
              menu = 0;
        }
}

void TestWinterzeit(){
        DCF77::time_data_t NewTime;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Teste");
        lcd.setCursor(0,1);
        lcd.print("Winterzeit");
        do {
          buttons = lcd.readButtons();
        }while (!buttons);
        if(buttons & BUTTON_SELECT){
              NewTime.year.val = 0x21;
              NewTime.month.val = 0x10;
              NewTime.day.val = 0x31;
              NewTime.weekday.val = 7;
              NewTime.hour.val = 0x02;
              NewTime.minute.val = 0x52;
              NewTime.second = 0;
              NewTime.uses_summertime = 1;
              NewTime.uses_backup_antenna = 0;
              NewTime.timezone_change_scheduled = 1;
              NewTime.leap_second_scheduled = 0;
              cli();
              now = NewTime;
              sei();
              menu = 0;
        }
}

void TestSchaltjahr(){
        DCF77::time_data_t NewTime;     
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Teste");
        lcd.setCursor(0,1);
        lcd.print("Schaltjahr");
        do {
          buttons = lcd.readButtons();
        }while (!buttons);
        if(buttons & BUTTON_SELECT){
              NewTime.year.val = 0x24;
              NewTime.month.val = 0x02;
              NewTime.day.val = 0x28;
              NewTime.weekday.val = 3;
              NewTime.hour.val = 0x23;
              NewTime.minute.val = 0x52;
              NewTime.second = 0;
              NewTime.uses_summertime = 0;
              NewTime.uses_backup_antenna = 0;
              NewTime.timezone_change_scheduled = 0;
              NewTime.leap_second_scheduled = 0;
              cli();
              now = NewTime;
              sei();
              menu = 0;
        }
}

void EinstellungDisplay(){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Einstellung");
        lcd.setCursor(0,1);
        lcd.print("Display");
        do {
          buttons = lcd.readButtons();
        }while (!buttons);
        if(buttons & BUTTON_SELECT){
          do{ buttons = 0;
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Display Timeout:");
              lcd.setCursor(0,1);
              if (DisplayTimeOut) lcd.print("Ein");
              else lcd.print("Aus");
              do {
                buttons = lcd.readButtons();
              }while (!buttons);
              if (buttons & BUTTON_UP)   DisplayTimeOut = !DisplayTimeOut;
              if (buttons & BUTTON_DOWN) DisplayTimeOut = !DisplayTimeOut;
          } while(!(buttons & BUTTON_SELECT));
          EEPROM.write(0,(int)DisplayTimeOut);
          menu = 0;
        }
        
}

void Ueber(){
   lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Ueber");
        lcd.setCursor(0,1);
        lcd.print("DCF77-Generator");
        buttons=0;
        do {
          buttons = lcd.readButtons();
        }while (!buttons);
        if(buttons & BUTTON_SELECT){
          
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("(c)2014 U.Klein");
              lcd.setCursor(0,1);
              lcd.print("blinkenlight.net");
              buttons=0;
              do {
                buttons = lcd.readButtons();
              }while (!buttons);
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("(c)2021 A.Schumm");
              lcd.setCursor(0,1);
              lcd.print("crazy-midi.de");
              buttons=0;
              do {
                buttons = lcd.readButtons();
              }while (!buttons);
              menu = 0;
        }
  }
