//
// Example of sleep and deepsleep modes
//
//

#define BOARD_LED_PIN PB0
//#define BOARD_LED_PIN PC13

// Strictly speaking only the RTC crystal is necessary for this code to work.
#define BOARD_HAS_WORKING_RTC_CRYSTAL true
#define BOARD_HAS_WORKING_RTC_BATTERY true

// Define the Base address of the RTC registers (battery backed up CMOS Ram), so we can use them for config of touch screen or whatever.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.
#define BKP_REG_BASE   (uint32_t *)(0x40006C00 +0x04)

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/pwr.h>
#include <libmaple/scb.h>

// These are possibly defined somewhere but I couldn't find them.
// System Control Register Bits. See...
// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihhjgdh.html
#define SCB_SCR_SLEEPDEEP 4       // This register bit controls deepsleep(1) or sleep(0)
#define SCB_SCR_SLEEPONEXIT 2     // This register bit controls sleeponexit. i.e. do we go back to sleep immediately after serviceing the interrupt, or go back to the main loop. 
#define SCB_SCR_SEVONPEND 16      // Controls Send Event on Pending bit:

// Set up RTC and choose one of the available clocks.
// The HSE/128 works in all boards
// Boards with a 32kHz RTC crystal fitted can choose any clock (but I suggest using LSE as this is the 32kHz clock).
//
#include <RTClock.h>

#ifdef BOARD_HAS_WORKING_RTC_CRYSTAL true
RTClock rt(RTCSEL_LSE);
#else
RTClock rt(RTCSEL_HSE);
// Alternatives...
// RTClock rt(RTCSEL_LSI);
// RTClock rt; // this starts HSE clock as a default.
#endif


bool ledOff = true;
bool alarmTriggeredFlag = false;
bool secInterruptFlag = false;

bool deepSleepFlag = false;



long int globSecCnt = 0;    // Number of secondsInterrupts this session.
// NOTE: globSecCnt is *not* preserved in deepsleep as the system restarts (ram contents are lost) after deepsleep.
// The RTC of course does keep an accurate count of seconds.
// Caveat, RTC time, and the RTC battery backed registers are preserved in deepsleep only if there is a voltage source (generally a battery) connected to vBat.
// Some boards have vBat connected to VDD,the main STM32FXX 3V3 power supply so their RTC will keep working so long as the main power is preserved.
// Flash is always preserved in deepsleeep, so writing to flash would be the best way to preserve config in excess of the
// 14 bytes offered by the RTC when in deepsleep.

long int alarmDelay = 15;   // Seconds between restarts if using deepsleep, or time of first alarm if using sleep (subsequent sleeps can be set in the loop() section.

struct tm time_tm, *tm_ptr; // This is a structure with date and time fields, used to set the time if BOARD_HAS_WORKING_RTC_CRYSTAL is false.

int numFlashes = 1;



// Create USB serial_debug port
USBSerial serial_debug;


void setup() {

  // This may be needed if we find the sys clock is incorrect after wakeup
  // Set up clock, PLL an multiplier
  // Chose RCC_PLLMUL_9 and RCC_PRESCALER_USB, 1.5 for 72MHz fom 8MHz crystal (standard STM32F103 speeds on most clone boards).
  // Chose RCC_PLLMUL_6 and RCC_PRESCALER_USB, 1 for 48MHz lower power from 8MHz crystal (standard STM32F103 speeds on most clone boards)
  // rcc_clk_init(rcc_sysclk_src sysclk_src, rcc_pllsrc pll_src, rcc_pll_multiplier pll_mul)
  // rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);
  // rcc_switch_sysclk(rcc_sysclk_src sysclk_src)

  // DANGER - here be dragons.
  // Switch to external 8MHz clock, set multipler, switch back to internal
  // rcc_switch_sysclk(RCC_CLKSRC_HSE);
  // rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_2);
  // rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_16);
  // rcc_switch_sysclk(RCC_CLKSRC_HSI);
  //
  // rcc_clk_init(RCC_CLKSRC_HSE, RCC_PLLSRC_HSE , RCC_PLLMUL_16);
  // rcc_switch_sysclk(RCC_CLKSRC_HSE);


  // Likewise for the USB prescaler
  // rcc_set_prescaler(rcc_prescaler prescaler, uint32 divider)
  // rcc_set_prescaler(RCC_PRESCALER_USB, 1.5);
  // rcc_set_prescaler(RCC_PRESCALER_USB, 1.5);

  pinMode(BOARD_LED_PIN, OUTPUT);//setup our alive pin.
  digitalWrite(BOARD_LED_PIN, ledOff);


  // If we have a working RTC crystal, we do nothing, this will simply preserve the current time, otherwise with no working RTC crystam we set something sane in the RTC instead.
  if (!BOARD_HAS_WORKING_RTC_CRYSTAL)
  {
    setSaneRTC();
  }

  // Attempt to set up 48MHz and 1x USB clock
  clockARM48();

  numFlashes = 1;
  for ( int x = 0; x < 10; x++ )
  {
    serial_debug.println("Flash48!");
    flashLED(numFlashes);
  }

  // Back to 72MHz and 1.5x USB clock
  clockARM72();
;
  
  numFlashes = 1;
  for ( int x = 0; x < 10; x++ )
  {
    serial_debug.println("Flash72!");
    flashLED(numFlashes);
  }
  
  delay(5000);

  // If not using deepsleep, you can enable the secondsInterrupt ISR, and wakeup every second. This is less energy efficent obviously.
  // It has an effect on deepsleep in this sketch, so can not be left configured.
  // rt.attachSecondsInterrupt(&serviceSecondsInterrupt);  //Set the SecondsInterrupt  ISR to fire (every second obviously).

  // Set the inital alarm timer to wakeup now + alarmDelay seconds
  time_t nextAlarm = (rt.getTime() + alarmDelay); // Calculate from time now.
  rt.createAlarm(&AlarmFunction, nextAlarm);      // Set the alarm to trigger (alarmDelay) later...

}

void loop() {
  // Set loop alarm delay (seconds).
  alarmDelay = 20;

  //Note: If we use deepsleep, the rest of this code will do nothing since we never actually get the alarmTriggeredFlag set.
  // The perhaps less than obvious reason for this is that deepsleep restarts the CPU on wakeup, so the flag is cleared.

  // Check Alarm triggered flag
  if (alarmTriggeredFlag)
  {
    // 3 flashes.. we came back from low power mode without restarting.
    numFlashes = 3;
    flashLED(numFlashes);
    delay(2000);
  } else {
    // 8 quick flashes to prove we got here, either at power on or deepsleep power on.
    numFlashes = 8;
    flashLED(numFlashes);
    delay(1000);
  }

  // Check seconds interrupt flag
  if (secInterruptFlag)
  {
    // We dont have much time here.. the next interrup is due in 1 second
    // Do *something* to prove we were here.
    numFlashes = 1;
    flashLED(numFlashes);
  }

  // rcc_switch_sysclk(RCC_CLKSRC_HSE);
  // delay(1);
  // Sleep mode - save some power while we wait for the next alarm.
  // if deepSleepFlag=true we deepsleep, otherwise, we just sleep

  sleepMode(deepSleepFlag);
  deepSleepFlag = !deepSleepFlag;
  delay(1000);
}

void serviceSecondsInterrupt()
{
  if (rtc_is_second())
  {
    globSecCnt++;
    secInterruptFlag = true;
    toggleLED();
  }
}

void AlarmFunction () {

  // This appears to be needed here, We find the sys clock is incorrect (slow) after wakeup.
  // Set up clock, PLL and multiplier anything from RCC_PLLMUL_2..RCC_PLLMUL_16

  // Chose RCC_PLLMUL_9 and RCC_PRESCALER_USB, 1.5 for 72MHz fom 8MHz crystal (standard STM32F103 speeds on most clone boards).
  // Chose RCC_PLLMUL_6 and RCC_PRESCALER_USB, 1 for 48MHz lower power from 8MHz crystal (standard STM32F103 speeds on most clone boards)

  // rcc_clk_init(rcc_sysclk_src sysclk_src, rcc_pllsrc pll_src, rcc_pll_multiplier pll_mul)

  // Experimental current draw results typical valuse with a power and blink LEDs lit subtract approx 4mA for the LED, 2mA for the regulator, and say 1mA for the resistance of the leads. .
  // The halted CPU should be drawing <300uA With the processor halted, the meter was reading 7.9mA, so allowing for the quality of the meter, and other environmental issues.
  // the results below look pretty accurate.. to within 1mA. Close enough for government work :¬)

  // These look similar to the ST datasheet http://www.st.com/web/en/resource/technical/document/datasheet/CD00161566.pdf
  // (Page 42 & 43)

  // External (RCC_CLKSRC_HSE) 8MHz, no multiplier.
  //  rcc_switch_sysclk(RCC_CLKSRC_HSE);                             // 8MHz => 16mA    -- datasheet value @20 deg. C  => between 5.5 and 6mA

  // Internal (RCC_CLKSRC_HSI) 8MHz x RCC_PLLMUL_x multiplier.
  // rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_2);   // 16MHz  => 21 mA  -- datasheet value           => between 10 and 11mA

  // Default configuration 72MHz
  // rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);   // 72MHz  => 48 mA  -- datasheet value           => between 40 and 41mA

  // rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_16);  // 128MHz => 69 mA  -- no data.

  // rcc_switch_sysclk(RCC_CLKSRC_HSE);

  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_16);  // 128MHz => 69 mA      -- Overclocked no STM data available.

  //rcc_clk_init(RCC_CLKSRC_HSE, RCC_PLLSRC_HSE , RCC_PLLMUL_2);  //

  //rcc_switch_sysclk(RCC_CLKSRC_HSI);


  // Likewise for the USB prescaler
  // rcc_set_prescaler(rcc_prescaler prescaler, uint32 divider)
  // rcc_set_prescaler(RCC_PRESCALER_USB, 1.5);
  // rcc_set_prescaler(RCC_PRESCALER_USB, 1.5);

  alarmTriggeredFlag = true;
  // We get here when the alarm triggers.
  // We have all the time in the world in here, till we set the next alarm and exit.
  numFlashes = 4;
  flashLED(numFlashes);
  //Note: these flashes are at whichever clock speed we chose above, The milis() will therefore only be millis() if the clock is 72 mHz
  delay(3000); // Wait long enough to see the current reading on the multimeter. At 128MHz this will be pretty brief, but at 8MHz it will take forever :¬)

  //Reset alarm to trigger another interrupt in alarmDelay seconds...
  time_t now = rt.getTime();
  rt.setAlarmTime(now + alarmDelay);
}


void toggleLED ()
{
  ledOff = !ledOff;
  digitalWrite(BOARD_LED_PIN, ledOff);
}


// Set a relatively sane RTC time to work from if there is no battery or RTC crystal fitted.
void setSaneRTC()
{
  // Set the RTC time to 12:45:00 18th Oct 2015 using a tm struct.
  //    see http://www.cplusplus.com/reference/ctime/tm/

  time_tm.tm_hour = 12;
  time_tm.tm_min = 45;
  time_tm.tm_sec = 0;

  time_tm.tm_mday =  18;
  time_tm.tm_mon = 10;
  time_tm.tm_year = 115;//  Per standard C "struct tm" ... years since 1900

  rt.setTime(&time_tm);

}

// Quick (N times) flashes on the LED
// Timing will be in mS *only* if the processor clock is running at the correct rate.
void flashLED(int numFlashes)
{
  int flashLen = 80; //mS ?
  ledOff = true;
  if (numFlashes > 0)
  {
    for (int i = 0 ; i < numFlashes; i++ )
    {
      for (int j = 0; j < 2; j++)
      {
        toggleLED();
        delay(flashLen);
      }
    }
  }
}


void sleepMode(bool deepSleepFlag)
{
  // Clear PDDS and LPDS bits
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF;

  // Set PDDS and LPDS bits for standby mode, and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR |= PWR_CR_CWUF;
  PWR_BASE->CR |= PWR_CR_PDDS;
  // Enable wakeup pin bit.
  PWR_BASE->CR |=  PWR_CSR_EWUP;



  // Low-power deepsleep bit.
  // PWR_BASE->CR |= PWR_CR_LPDS;

  // System Control Register Bits. See...
  // http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihhjgdh.html
  if (deepSleepFlag)
  { // Experimental
    // Set Power down deepsleep bit.
    PWR_BASE->CR |= PWR_CR_PDDS;
    // Unset Low-power deepsleep.
    PWR_BASE->CR &= ~PWR_CR_LPDS;
    // Set sleepdeep in the system control register - if set, we deepsleep and coldstart from RTC or pin interrupts.
    SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;
  } else {
    //  Unset Power down deepsleep bit.
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    // set Low-power deepsleep.
    PWR_BASE->CR |= PWR_CR_LPDS;
    /*
     * PWR_CR_PDDS
        Power down deepsleep.
       PWR_CR_LPDS
        Low-power deepsleep.
     */
    // Unset sleepdeep in the system control register - if not set then we only sleep and can wake from RTC or pin interrupts.
    SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;
    // Low-power deepsleep bit.


  }

  // Set end Event on Pending bit: - enabled events and all interrupts, including disabled interrupts, can wakeup the processor.
  // SCB_BASE->SCR |= SCB_SCR_SEVONPEND;

  // Set SLEEPONEXIT -Indicates sleep-on-exit when returning from Handler mode to Thread mode -
  // if enabled, we will effectively sleep from the end of  one interrupt till the start of the next.
  // SCB_BASE->SCR |= SCB_SCR_SLEEPONEXIT;

  // Now go into stop mode, wake up on interrupt
  asm("    wfi");

}

void clockARM48()
{
  // Attempt to set up 48MHz and 1x USB clock
  rcc_switch_sysclk(RCC_CLKSRC_HSE);
  rcc_set_prescaler(RCC_PRESCALER_USB, 1);
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_6);
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_6);
  rcc_switch_sysclk(RCC_CLKSRC_HSI);
  //rcc_switch_sysclk(RCC_CLKSRC_PLL);
}

void clockARM72()
{
  // Attempt to set up 72MHz and 1x USB clock
  rcc_switch_sysclk(RCC_CLKSRC_HSE);
  rcc_set_prescaler(RCC_PRESCALER_USB, 1.5);
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);
  // rcc_switch_sysclk(RCC_CLKSRC_HSI);
  rcc_switch_sysclk(RCC_CLKSRC_PLL);
}
