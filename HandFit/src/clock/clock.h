//
//  clock.h
//
//  Created by Jonathan Tompson on 3/30/12.
//  Copyright (c) 2012 NYU. All rights reserved.
//

#ifndef clock_h
#define clock_h

#ifdef __APPLE_CC__
#include <mach/mach.h>
#include <mach/mach_time.h>


class Clock {
 public:
  Clock() {
    time = mach_absolute_time();
    time_start = time;
    (void) mach_timebase_info(&sTimebaseInfo);
  }
  
  void Update() {
    time_old = time;
    time = mach_absolute_time();
  }
  
  double getDeltaTime() {
    elapsedNano = (time-time_old) * sTimebaseInfo.numer / sTimebaseInfo.denom;
    return (static_cast<double>(elapsedNano) / 1000000000.0);
  }

  double getTime() {
    uint64_t cur_time = mach_absolute_time();
    uint64_t tNano = (cur_time-time_start) * sTimebaseInfo.numer / sTimebaseInfo.denom;
    return (static_cast<double>(tNano) / 1000000000.0);
  }
  
  uint64_t getAbsoluteTimeNano() {
    uint64_t cur_time = mach_absolute_time();
    uint64_t tNano = (cur_time) * sTimebaseInfo.numer / sTimebaseInfo.denom;
    return tNano;
  }

 private:
  uint64_t time_start;
  uint64_t time;
  uint64_t time_old;
  uint64_t elapsedNano;
  mach_timebase_info_data_t sTimebaseInfo;
};

#elif defined( __WIN32__ ) || defined( _WIN32 ) || defined( WIN32 )
#include <windows.h>
#include <stdexcept>

class Clock {
 public:
  Clock() {
    if (!QueryPerformanceFrequency(
                                reinterpret_cast<LARGE_INTEGER*>(&m_cntFreq))) {
      throw std::runtime_error("Clock::Clock() - Failed init counter!");
    }
    m_secsPerCnt = 1.0 / static_cast<double>(m_cntFreq);
    QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&m_cntCurrent));
    m_cntStart = m_cntCurrent;
  }

  void Update() {
    m_cntOld = m_cntCurrent;
    QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&m_cntCurrent));
  }
  
  double getDeltaTime() {
    double elapsedSec = static_cast<double>(m_cntCurrent-m_cntOld) * 
                        static_cast<double>(m_secsPerCnt);
    return elapsedSec;
  }

  double getTime() {
    __int64 m_cnt;
    QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&m_cnt));
    return static_cast<double>(m_cnt-m_cntStart) * 
      static_cast<double>(m_secsPerCnt);
  }

  uint64_t getAbsoluteTimeNano() {
    double time = getTime();
    return static_cast<uint64_t>(floor(time * 1000000000.0));
  }

 private:
  double elapsedSec;
  __int64 m_cntFreq,  // Frequency of the precision counter
          m_cntCurrent,  // Current counter value
          m_cntOld,
          m_cntStart;
  double  m_secsPerCnt;  // counter period
  __int64 m_clockPauseTime;  // record when the clock was paused
};
#else  // We must be on linux

#endif

#endif
