#pragma once
// -----------------------------------------------------------------------
// This is NOT mbed-os. It's a host-compile stand-in for just the pieces of
// the mbed API that this training slice touches directly. It exists so a
// trainee's .cpp can be TYPE-CHECKED with a normal desktop compiler
// (g++ / clang++ / MSVC) without needing the ARM toolchain, a real STM32
// target definition, or the full mbed-os source tree.
//
// It intentionally does nothing electrically real. Function bodies return
// harmless placeholder values. Do not use this to validate timing,
// electrical behavior, or runtime correctness -- only that the code
// compiles and the types line up. Real behavior is only verified once this
// file is dropped into the actual repo and built against real mbed-os.
//
// Extending this file: if a trainee's code uses an mbed class/function not
// declared here, the host compiler error will name it exactly
// (e.g. "no member named 'read' in 'DigitalIn'") -- add just enough of a
// declaration to satisfy that error, matching the real mbed-os signature.
// -----------------------------------------------------------------------

#include <cstdint>
#include <cstddef>
#include <chrono>
#include "drivers/Timer.h"
#include "platform/Callback.h"
#include "platform/mbed_chrono.h"
#include "drivers/InterruptIn.h"
#include "rtos/Mutex.h"
#include "rtos/ThisThread.h"

#ifndef MBED_H
#define MBED_H

#if defined(_MSC_VER)
    #include <basetsd.h>
    using ssize_t = SSIZE_T; // MSVC has no POSIX ssize_t
#else
    #include <sys/types.h> // provides ssize_t on Linux/macOS/MinGW
#endif

#include "PinNames.h"
#include "us_ticker_api.h"

using namespace std::chrono_literals; // so `100ms` literals work, as they do via real mbed.h

namespace mbed {

// class Timer {
// public:
//     void start() {}
//     void stop() {}
//     void reset() {}
//     std::chrono::microseconds elapsed_time() const { return std::chrono::microseconds(0); }
// };

// class InterruptIn {
// public:
//     explicit InterruptIn(PinName) {}
//     int read() const { return 0; }
//     operator int() const { return read(); }
//     template <typename Func>
//     void rise(Func /*func*/) {}
//     template <typename Func>
//     void fall(Func /*func*/) {}
// };

} // namespace mbed

// ---- I2C ----------------------------------------------------------------
// Real signature: mbed I2C read/write take (address, data, length, repeated)
class I2C {
public:
    I2C(PinName sda, PinName scl) {}
    void frequency(int) {}
    int write(int /*address*/, const char* /*data*/, int /*length*/, bool /*repeated*/ = false) { return 0; }
    int read(int /*address*/, char* /*data*/, int /*length*/, bool /*repeated*/ = false) { return 0; }
};

// ---- BufferedSerial -------------------------------------------------------
class BufferedSerial {
public:
    BufferedSerial(PinName /*tx*/, PinName /*rx*/, int /*baud*/ = 9600) {}
    ssize_t read(void*, size_t) { return 0; }
    ssize_t write(const void*, size_t n) { return static_cast<ssize_t>(n); }
    bool readable() const { return false; }
    bool writable() const { return true; }
};

// ---- DigitalOut / DigitalIn -----------------------------------------------
// Not seen directly in the shared files, but common enough elsewhere in
// mbed projects that it's cheap to include preemptively.
class DigitalOut {
public:
    explicit DigitalOut(PinName, int value = 0) : _v(value) {}//constructor
    void write(int v) { _v = v; }
    int read() const { return _v; }
    DigitalOut& operator=(int v) { write(v); return *this; }
    operator int() const { return read(); }
private:
    int _v;
};

class DigitalIn {
public:
    explicit DigitalIn(PinName) {}
    int read() const { return 0; }
    operator int() const { return read(); }
};

// ---- ThisThread -----------------------------------------------------------
namespace ThisThread {
    inline void sleep_for(std::chrono::milliseconds) { /* no-op on host */ }
}

#endif // MBED_H
