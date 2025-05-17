
//////////////////////////////////////////////////////////////////////////////
/// \file main.cpp
/// \author Kai R. ()
/// \brief  Program for a HIIT (high-intensity interval training) stopwatch.
///         An acoustic signal sounds at the end of each of the two possible countdowns.
///         This indicates the change between the active and regeneration phase of the training.
///
/// \date 2025-05-17
/// \version 1.0
///
///                 Controller TinyAVR 1614
///                          -----
///                    VCC -|     |- GND
///      Buzzer Pin ~    0 -|     |- NC
///      Encoder Pin CW  1 -|     |- NC
///      Encoder Pin CCW 2 -|     |- NC
///                     NC -|     |- NC
///                  TOSC1 -|     |- SCL Oled
///                  TOSC2 -|     |- SDA Oled
///                          -----
///
//////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <avr/sleep.h>
#include <U8g2lib.h>
#include <RotaryEncoder.h>
#include <Button_SL.hpp>
#include "SimpleCounter.hpp"
#include "ToneSequence.hpp"
#include <EEPROM.h>

//
// Global constants
//
namespace gc {
constexpr byte pin_in1 {1};       // Encoder Pin cw
constexpr byte pin_in2 {2};       // Encoder Pin ccw
constexpr byte pin_btn {3};       // Encoder Button
constexpr byte pin_signal {0};    // Buzzer pin for acoustic signal output
constexpr byte indicator {'@'};   // Indicator of whether a memory location has ever been written to (must be != 0xFF).
constexpr byte eep_address {0};   // Startaddress for saving data in the eeprom

constexpr unsigned long time_until_sleep {60 * 1000 * 30};   // 30 Seconds
// Notes for the acoustic signal.
// Used buzzer as a resonance frequency from 4000Hz +- 500Hz
// Use a different note if the buzzer has a different resonance frequency to achieve maximum volume.
// Note definitions are placed in ToneSequence.hpp
// as7 has 3729 Hz
constexpr Note melody[] {
    {note::as7, 1000 / 4 },
    {0,         1000 / 20},
    {note::as7, 1000 / 8 },
    {0,         1000 / 20},
    {note::as7, 1000 / 4 },
};
}   // namespace gc

//
// ----------------- Class / struct definitions begin-----------------------------------------------
//

//////////////////////////////////////////////////////////////////////////////
/// \brief Helperclass for a non blocking delay
///
//////////////////////////////////////////////////////////////////////////////
class NbDelay {
public:
  void start() { timestamp = millis(); }
  boolean operator()(const unsigned long duration) { return millis() - timestamp >= duration; }

private:
  unsigned long timestamp {0};
};

//////////////////////////////////////////////////////////////////////////////
/// \brief Enum for a state machine
///
//////////////////////////////////////////////////////////////////////////////
enum class State : byte { input, check, run, alarm };

//////////////////////////////////////////////////////////////////////////////
/// \brief  Structure for representing minutes and seconds with the help
///         of SimpleCounter objects.
///
//////////////////////////////////////////////////////////////////////////////
struct Timer {
  Timer(unsigned int max_m = 0, unsigned int max_s = 0) : minutes {max_m}, seconds {max_s} {}
  // bool isZero() { return not(minutes() + seconds() > 0); }
  bool isZero() { return minutes() + seconds() == 0; }
  unsigned long sumSeconds() { return minutes() * seconds.scMax() + seconds(); }
  SimpleCounter minutes;
  SimpleCounter seconds;
};

//////////////////////////////////////////////////////////////////////////////
/// \brief Coordinates on a display.
///
//////////////////////////////////////////////////////////////////////////////
struct Pos {
  int x;
  int y;
};

//////////////////////////////////////////////////////////////////////////////
/// \brief Auxiliary structure for displaying text on a display.
///
//////////////////////////////////////////////////////////////////////////////
struct Row {
  Pos pos;
  char text[6];
};
//
// ----------------- Class / struct definitions end ------------------------------------------------
//

//
// Global variables
//
Row rows[] {
    // logisoso26_tn - 16 Width 32 Height
    {{28, 28}, "00:00"},
    {{28, 60}, "00:00"},
};

Timer time[2] {
    {60, 60},
    {60, 60}
};

using Display = U8G2_SSD1306_128X64_NONAME_1_HW_I2C;
Display u8g2(U8G2_R0);
Btn::ButtonSL btn {gc::pin_btn};
RotaryEncoder encoder {gc::pin_in1, gc::pin_in2, RotaryEncoder::LatchMode::FOUR3};
ToneSequence<gc::pin_signal> signal;
NbDelay wait_for_sleep;

bool is_second_over {false};
//
// Functions
//

//////////////////////////////////////////////////////////////////////////////
/// \brief Set up internal Real Timer Clock
///
//////////////////////////////////////////////////////////////////////////////
void rtcInit() {
  uint8_t temp;
  // Initialize 32.768kHz Oscillator:
  // Disable oscillator:
  temp = CLKCTRL.XOSC32KCTRLA & ~CLKCTRL_ENABLE_bm;
  // Enable writing to protected register
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm) { ; }   // Wait until XOSC32KS is 0
  temp = CLKCTRL.XOSC32KCTRLA & ~CLKCTRL_SEL_bm;           // Use External Crystal
  // Enable writing to protected register
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  temp = CLKCTRL.XOSC32KCTRLA | CLKCTRL_ENABLE_bm;   // Enable oscillator
  // Enable writing to protected register
  CPU_CCP = CCP_IOREG_gc;
  CLKCTRL.XOSC32KCTRLA = temp;

  // Initialize RTC
  while (RTC.STATUS > 0) { ; }   // Wait until synchronized
  // 32.768kHz External Crystal Oscillator (XOSC32K)
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;

  // RTC Clock Cycles 32768, enabled ie 1Hz interrupt
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc;
  RTC.PITINTCTRL = RTC_PI_bm;   // Periodic Interrupt: enabled
  // pinMode(PIN_A3, OUTPUT);      // For debugging purposes
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Enable RTC
///
//////////////////////////////////////////////////////////////////////////////
inline void rtcEnable() { RTC.PITCTRLA |= RTC_PITEN_bm; }

//////////////////////////////////////////////////////////////////////////////
/// \brief disable RTC
///
//////////////////////////////////////////////////////////////////////////////
inline void rtcDisable() { RTC.PITCTRLA &= ~(RTC_PITEN_bm); }

//////////////////////////////////////////////////////////////////////////////
/// \brief Interrupt function of the RTC
///
//////////////////////////////////////////////////////////////////////////////
ISR(RTC_PIT_vect) {   // Called every second
  // digitalWrite(PIN_PA3, CHANGE);   // For debugging purposes
  RTC.PITINTFLAGS = RTC_PI_bm;   // Clear interrupt flag
  is_second_over = true;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Check if a button has been short or long pressed
///
/// \param b      Reference to Button Object
/// \param value  Marker for how often the button was pressed.
/// \param max    Upper value of button presses. Once this is reached, it is reset to zero.
/// \return byte  value
//////////////////////////////////////////////////////////////////////////////
byte askButton(Btn::ButtonSL& b, byte value, byte max) {
  switch (b.tick()) {
    case Btn::ButtonState::shortPressed: ((value + 1) > max) ? value = 1 : ++value; break;
    case Btn::ButtonState::longPressed: value = 0; break;
    default: break;
  }
  return value;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Set the Timer object
///
/// \tparam N     Number of timer objects
/// \param enc    Reference to encoder object
/// \param t      Reference to timer object array
/// \param marker variable for determining the index to the timer object that is to be changed.
/// \return true  when the encoder has been actuated.
/// \return false When the encoder has not been actuated.
//////////////////////////////////////////////////////////////////////////////
template <size_t N> bool setTime(RotaryEncoder& enc, Timer (&t)[N], byte marker) {
  if (!marker) { return false; }

  byte idx = (marker < 3) ? 0 : 1;   // determine timer index from marker value. Two marker values per timer.
  enc.tick();
  switch (enc.getDirection()) {
    case RotaryEncoder::Direction::CLOCKWISE:
      if (!(marker % 2)) {
        ++t[idx].minutes;
      } else {
        ++t[idx].seconds && ++t[idx].minutes;
      }
      return true;
    case RotaryEncoder::Direction::COUNTERCLOCKWISE:
      if (!(marker % 2)) {
        --t[idx].minutes;
      } else {
        --t[idx].seconds && --t[idx].minutes;
      }
      return true;
    default: break;
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief  Determine the screen position of the marker line for
///         the active time object.
///
/// \param r      row
/// \param width  of indicator
/// \param marker to determine the screen position for the marker line
/// \return Pos   Position of marker line on the screen
//////////////////////////////////////////////////////////////////////////////
inline Pos markerPos(const Row* r, int width, byte marker) {
  switch (marker) {
    case 1: return {r[0].pos.x + width * 3, r[0].pos.y + 1};   // Seconds line 1
    case 2: return {r[0].pos.x, r[0].pos.y + 1};               // Minutes line 1
    case 3: return {r[1].pos.x + width * 3, r[1].pos.y + 1};   // Seconds line 2
    case 4: return {r[1].pos.x, r[1].pos.y + 1};               // Minutes line 2
  }
  return {0, 0};
}

//////////////////////////////////////////////////////////////////////////////
/// \brief  Display all data on the screen
///
/// \tparam N      Number of Elements in the given arrays (row and time objekts)
/// \param dp      Reference to display object
/// \param marker  indicator for actual choosen timer object
//////////////////////////////////////////////////////////////////////////////
template <size_t N> void displayTime(Display& dp, Row (&r)[N], const Timer (&t)[N], byte marker = 0) {
  sprintf(r[0].text, "%02d:%02d", t[0].minutes(), t[0].seconds());
  sprintf(r[1].text, "%02d:%02d", t[1].minutes(), t[1].seconds());
  dp.firstPage();
  do {
    dp.drawStr(r[0].pos.x, r[0].pos.y, r[0].text);
    dp.drawStr(r[1].pos.x, r[1].pos.y, r[1].text);
    if (marker) {
      int width {dp.getMaxCharWidth()};
      Pos mPos = markerPos(r, width, marker);
      dp.drawHLine(mPos.x, mPos.y, width * 2);
    }
  } while (dp.nextPage());
}

//////////////////////////////////////////////////////////////////////////////
/// \brief  Reading data from eeprom
///
/// \param eeAddr     flash address.
/// \param indicator  Indicator of whether data has ever been stored.
/// \param t          Timer data to be read.
//////////////////////////////////////////////////////////////////////////////
void readEEPROM(int eeAddr, byte indicator, Timer& t) {
  if (EEPROM.read(eeAddr) != indicator) {   // Executed if no data has ever been saved.
    EEPROM.put(eeAddr, indicator);
    EEPROM.put(eeAddr + 1, t);
  } else {
    EEPROM.get(eeAddr + 1, t);   // If data has been saved, read it out.
  }
}

//////////////////////////////////////////////////////////////////////////////
/// \brief  Write pwm data to EEProm
///
/// \param eeAddr       flash address.
/// \param indicator    Indicator of whether data has ever been stored.
/// \param t            Timer data to be saved.
//////////////////////////////////////////////////////////////////////////////
void writeEEPROM(int eeAddr, byte indicator, Timer& t) {
  EEPROM.put(eeAddr, indicator);
  EEPROM.put(eeAddr + 1, t);   // eeAddr + 1 -> don't override indicator char
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Interrupt service routine for wake up
///
//////////////////////////////////////////////////////////////////////////////
void intWakeup() { detachInterrupt(gc::pin_btn); }

//////////////////////////////////////////////////////////////////////////////
/// \brief  Start (and stop) the sleep mode
///
/// \param wakeupPin     Number of the wake up interrupt pin
//////////////////////////////////////////////////////////////////////////////
void powerDown(uint8_t wakeupPin, Display& dp) {
  // go into deep Sleep
  attachInterrupt(digitalPinToInterrupt(wakeupPin), intWakeup, LOW);
  dp.setPowerSave(true);
  delay(20);
  sleep_cpu();   // sleep
  // switch anything on
  delay(20);
  dp.setPowerSave(false);
}

//
// Main program
//
void setup() {
  // Turn on all the pullups for minimal power in sleep
  PORTA.DIR = 0;                           // All PORTA pins inputs
  for (uint8_t pin = 0; pin < 8; ++pin) { (&PORTA.PIN0CTRL)[pin] = PORT_PULLUPEN_bm; }
  PORTB.DIR = 0;                           // All PORTB pins inputs
  for (uint8_t pin = 0; pin < 4; ++pin) { (&PORTB.PIN0CTRL)[pin] = PORT_PULLUPEN_bm; }

  ADC0.CTRLA &= ~ADC_ENABLE_bm;            // ADC is not required so switch it off
  rtcInit();                               // Initialize the real time clock
  u8g2.begin();
  u8g2.setBusClock(400000);
  u8g2.setFont(u8g2_font_logisoso26_tn);   // 16 Width 32 Height

  btn.begin();
  btn.releaseOn();

  readEEPROM(gc::eep_address, gc::indicator, time[0]);
  readEEPROM(gc::eep_address + sizeof(Timer) + 1, gc::indicator, time[1]);

  displayTime(u8g2, rows, time, 1);

  wait_for_sleep.start();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // Set sleep mode to POWER DOWN mode
  sleep_enable();                        // Enable sleep mode, but not yet
}

void loop() {
  constexpr byte max_marker {4};       // maximum value vor marker. the value 0 indicates a long button press.
  static byte marker {1};              // Indicator for which time is to be set (minute/second per timer).
  static Timer save_time[2];           // Timer array / only two timers are possible.
  static size_t idx {0};               // Index for timer array.
  static State state {State::input};   // initial state for the state machine
  static bool is_signal {false};       // Flag for whether an acoustic signal should be emitted or not.

  byte new_marker = askButton(btn, marker, max_marker);
  // new_marker != 0 then Button short pressed. If clock is running stop it.
  // If the clock is not runnig set marker line to the next position.
  // new_marker == 0 then Button is long pressed -> start Clock.
  if (new_marker && state == State::run) {
    // count down has been stopped. Recover the initial time values for display.
    wait_for_sleep.start();   // Start wait for sleep mode. Becomes active if no actions will be done.
    time[0] = save_time[0];
    time[1] = save_time[1];
    state = State::input;
    idx = 0;                     // Set index to the first timer
    is_signal = signal.stop();   // signal.stop() returns false = signal off
    rtcDisable();
  } else if (!new_marker && state != State::run) {
    // Start count down if button was long pressed (new_marker = 0). Save the set time values into the flash memory.
    // If the time values have not been adjusted, nothing is written to the memory.
    writeEEPROM(gc::eep_address, gc::indicator, time[0]);
    writeEEPROM(gc::eep_address + sizeof(Timer) + 1, gc::indicator, time[1]);
    state = State::check;
  }

  // set time by rotating the encoder
  if (setTime(encoder, time, marker) || new_marker != marker) {
    marker = new_marker;
    wait_for_sleep.start();
    displayTime(u8g2, rows, time, marker);
  }

  if (is_signal) { is_signal = signal(gc::melody); }   // plays a melody as long as signal returns true

  switch (state) {
    case State::check:
      if (time[0].isZero() && time[1].isZero()) {
        state = State::input;
        marker = 1;
        displayTime(u8g2, rows, time, marker);
        break;   // no timevalues to count down -> quit check, don't run anything.
      }
      // Safe initial time values for the next run
      save_time[0] = time[0];
      save_time[1] = time[1];
      if (time[0].isZero()) { idx = 1; }
      state = State::run;
      tone(gc::pin_signal, note::as7, 125);   // 1000ms / 8 = 125 (eighth note)
      delay(125);
      rtcEnable();
      [[fallthrough]];
    case State::run:
      if (is_second_over) {                                                // is_second_over is switched by ISR of RTC
        is_second_over = false;
        if (time[idx].sumSeconds() == 1) { is_signal = signal.reset(); }   // signal.reset() returns true = signal on
        if (time[idx].isZero()) {                                          // Timer is zero -> switch to the next timer
          time[idx] = save_time[idx];                                      // recover time value for the next run.
          decltype(idx) tmp_idx = (idx + 1) % 2;
          idx = (!save_time[tmp_idx].isZero()) ? tmp_idx : idx;
        } else {
          // do count down
          !time[idx].minutes() ? static_cast<bool>(--time[idx].seconds) : --time[idx].seconds && --time[idx].minutes;
        }
        // !time[idx].minutes() ? (bool)--time[idx].seconds : --time[idx].seconds && --time[idx].minutes;
        displayTime(u8g2, rows, time, marker);
      }
      break;
    case State::input:
      // Put the microcontroller into deep sleep if no actions are performed.
      if (wait_for_sleep(gc::time_until_sleep)) {
        pinMode(gc::pin_signal, OUTPUT);   // Saves power
        powerDown(gc::pin_btn, u8g2);      // Go to deep sleep here
        wait_for_sleep.start();            // Start timer so that the display does not go off immediately after wake up
        // A delay so that the minute/second changeover is not triggered immediately after waking up.
        delay(200);
      }
      break;
    default: break;
  }
}
