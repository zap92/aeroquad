/*
  AeroQuad v1.5 - Novmeber 2009
 www.AeroQuad.info
 Copyright (c) 2009 Chris Whiteford.  All rights reserved.
 An Open Source Arduino based quadrocopter.
 
 This program is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by 
 the Free Software Foundation, either version 3 of the License, or 
 (at your option) any later version. 
 
 This program is distributed in the hope that it will be useful, 
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 GNU General Public License for more details. 
 
 You should have received a copy of the GNU General Public License 
 along with this program. If not, see <http://www.gnu.org/licenses/>. 
 */

/*
  TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
 Copyright (C) 2008-9 Mikal Hart
 All rights reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "SubSystem.h"

#define _GPS_VERSION 9 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001

#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"

class GPS: 
public SubSystem
{
private:
  HardwareSerial *_serialPort;

  //GPS Related properties
  enum {
    _GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_OTHER        };

  unsigned long _time, _new_time;
  unsigned long _date, _new_date;
  long _latitude, _new_latitude;
  long _longitude, _new_longitude;
  long _altitude, _new_altitude;
  unsigned long  _speed, _new_speed;
  unsigned long  _course, _new_course;

  unsigned long _last_time_fix, _new_time_fix;
  unsigned long _last_position_fix, _new_position_fix;

  // parsing state variables
  byte _parity;
  bool _is_checksum_term;
  char _term[15];
  byte _sentence_type;
  byte _term_number;
  byte _term_offset;
  bool _gps_data_good;

  unsigned long _encoded_characters;
  unsigned short _good_sentences;
  unsigned short _failed_checksum;
  unsigned short _passed_checksum;

  int _from_hex(char a) 
  {
    if (a >= 'A' && a <= 'F')
      return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
      return a - 'a' + 10;
    else
      return a - '0';
  }

  bool _gpsisdigit(char c) 
  { 
    return c >= '0' && c <= '9'; 
  }

  long _gpsatol(const char *str)
  {
    long ret = 0;
    while (_gpsisdigit(*str))
      ret = 10 * ret + *str++ - '0';
    return ret;
  }

  int _gpsstrcmp(const char *str1, const char *str2)
  {
    while (*str1 && *str1 == *str2)
      ++str1, ++str2;
    return *str1;
  }

  unsigned long _parse_decimal()
  {
    char *p = _term;
    bool isneg = *p == '-';
    if (isneg) ++p;
    unsigned long ret = 100UL * _gpsatol(p);
    while (_gpsisdigit(*p)) ++p;
    if (*p == '.')
    {
      if (_gpsisdigit(p[1]))
      {
        ret += 10 * (p[1] - '0');
        if (_gpsisdigit(p[2]))
          ret += p[2] - '0';
      }
    }
    return isneg ? -ret : ret;
  }

  unsigned long _parse_degrees()
  {
    char *p;
    unsigned long left = _gpsatol(_term);
    unsigned long tenk_minutes = (left % 100UL) * 10000UL;
    for (p=_term; _gpsisdigit(*p); ++p);
    if (*p == '.')
    {
      unsigned long mult = 1000;
      while (_gpsisdigit(*++p))
      {
        tenk_minutes += mult * (*p - '0');
        mult /= 10;
      }
    }
    return (left / 100) * 100000 + tenk_minutes / 6;
  }


  bool _term_complete()
  {
    if (_is_checksum_term)
    {
      byte checksum = 16 * _from_hex(_term[0]) + _from_hex(_term[1]);
      if (checksum == _parity)
      {
        if (_gps_data_good)
        {
          ++_good_sentences;

          _last_time_fix = _new_time_fix;
          _last_position_fix = _new_position_fix;

          switch(_sentence_type)
          {
          case _GPS_SENTENCE_GPRMC:
            _time      = _new_time;
            _date      = _new_date;
            _latitude  = _new_latitude;
            _longitude = _new_longitude;
            _speed     = _new_speed;
            _course    = _new_course;
            break;
          case _GPS_SENTENCE_GPGGA:
            _altitude  = _new_altitude;
            _time      = _new_time;
            _latitude  = _new_latitude;
            _longitude = _new_longitude;
            break;
          }

          return true;
        }
      }
      else
        ++_failed_checksum;
      return false;
    }

    // the first term determines the sentence type
    if (_term_number == 0)
    {
      if (!_gpsstrcmp(_term, _GPRMC_TERM))
        _sentence_type = _GPS_SENTENCE_GPRMC;
      else if (!_gpsstrcmp(_term, _GPGGA_TERM))
        _sentence_type = _GPS_SENTENCE_GPGGA;
      else
        _sentence_type = _GPS_SENTENCE_OTHER;
      return false;
    }

    if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
      switch((_sentence_type == _GPS_SENTENCE_GPGGA ? 200 : 100) + _term_number)
      {
      case 101: // Time in both sentences
      case 201:
        _new_time = _parse_decimal();
        _new_time_fix = millis();
        break;
      case 102: // GPRMC validity
        _gps_data_good = _term[0] == 'A';
        break;
      case 103: // Latitude
      case 202:
        _new_latitude = _parse_degrees();
        _new_position_fix = millis();
        break;
      case 104: // N/S
      case 203:
        if (_term[0] == 'S')
          _new_latitude = -_new_latitude;
        break;
      case 105: // Longitude
      case 204:
        _new_longitude = _parse_degrees();
        break;
      case 106: // E/W
      case 205:
        if (_term[0] == 'W')
          _new_longitude = -_new_longitude;
        break;
      case 107: // Speed (GPRMC)
        _new_speed = _parse_decimal();
        break;
      case 108: // Course (GPRMC)
        _new_course = _parse_decimal();
        break;
      case 109: // Date (GPRMC)
        _new_date = _gpsatol(_term);
        break;
      case 206: // Fix data (GPGGA)
        _gps_data_good = _term[0] > '0';
        break;
      case 209: // Altitude (GPGGA)
        _new_altitude = _parse_decimal();
        break;
      }

    return false;
  }

  bool _encode(char c)
  {
    bool valid_sentence = false;

    ++_encoded_characters;
    switch(c)
    {
    case ',': // term terminators
      _parity ^= c;
    case '\r':
    case '\n':
    case '*':
      if (_term_offset < sizeof(_term))
      {
        _term[_term_offset] = 0;
        valid_sentence = _term_complete();
      }
      ++_term_number;
      _term_offset = 0;
      _is_checksum_term = c == '*';
      return valid_sentence;

    case '$': // sentence begin
      _term_number = _term_offset = 0;
      _parity = 0;
      _sentence_type = _GPS_SENTENCE_OTHER;
      _is_checksum_term = false;
      _gps_data_good = false;
      return valid_sentence;
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
      _term[_term_offset++] = c;
    if (!_is_checksum_term)
      _parity ^= c;

    return valid_sentence;
  }

public:
  enum {
    GPS_INVALID_AGE = 0xFFFFFFFF, GPS_INVALID_ANGLE = 999999999, GPS_INVALID_ALTITUDE = 999999999, GPS_INVALID_DATE = 0,
    GPS_INVALID_TIME = 0xFFFFFFFF, GPS_INVALID_SPEED = 999999999, GPS_INVALID_FIX_TIME = 0xFFFFFFFF                                                      };

  GPS():
  SubSystem()
  {
    _time = GPS_INVALID_TIME;
    _date = GPS_INVALID_DATE;
    _latitude = GPS_INVALID_ANGLE;
    _longitude = GPS_INVALID_ANGLE;
    _altitude = GPS_INVALID_ALTITUDE;
    _speed = GPS_INVALID_SPEED;
    _course = GPS_INVALID_ANGLE;
    _last_time_fix = GPS_INVALID_FIX_TIME;
    _last_position_fix = GPS_INVALID_FIX_TIME;
    _parity = 0;
    _is_checksum_term = false;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _term_number = 0;
    _term_offset = 0;
    _gps_data_good = false;
    _encoded_characters = 0;
    _good_sentences = 0;
    _failed_checksum = 0;
    _term[0] = '\0';
  }

  void assignSerialPort(HardwareSerial *serialPort)
  {
    _serialPort = serialPort;
  }

  void initialize(unsigned int frequency, unsigned int offset = 0) 
  { 
    this->_initialize(frequency, offset); 

    if (!_serialPort)
    {
      //No Serial port assigned.  Just disable this sub system
      this->disable();
    }
    else
    {
      _serialPort->begin(4800);
    }
  }

  void process(unsigned long currentTime)
  {
    if (this->_canProcess(currentTime))
    {
      //We are enabled and have a valid serial port and our time contraint has been met.  So lets process some data
      int charCounter = 0;
      while (_serialPort->available() && charCounter < 100)
      {
        _encode(_serialPort->read());
        charCounter++;
      }
    }
  }

  void get_position(long *latitude, long *longitude, long *altitude, unsigned long *course, long *speed, unsigned long *fix_age = 0)
  {
    if (latitude) *latitude = _latitude;
    if (longitude) *longitude = _longitude;
    if (altitude) *altitude = _altitude;
    if (course) *course = _course;
    if (speed) *speed = _speed * _GPS_MPS_PER_KNOT;
    if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? GPS_INVALID_AGE : millis() - _last_position_fix;
  }

  void get_datetime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
  {
    if (date) *date = _date;
    if (time) *time = _time;
    if (fix_age) *fix_age = _last_time_fix == GPS_INVALID_FIX_TIME ? GPS_INVALID_AGE : millis() - _last_time_fix;
  }

};


































