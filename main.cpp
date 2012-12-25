#include <avr/io.h>
#include <avr/sleep.h>          // definitions for power-down modes
#include <avr/wdt.h>
#include <Arduino.h>
#include <EEPROM.h>

#define MIN_TEMP 80.0
#define MAX_TEMP 81.0

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// macros from tvbgone:
#define DELAY_CNT 11
#define freq_to_timerval(x) ((F_CPU / x - 1)/ 2)
#define NOP __asm__ __volatile__ ("nop")

// IR led is on pin 0, indicator light on pin 1
#define IRLED PB0
#define LED PB1

uint16_t power_button[] = {47, 443, 47, 443, 15, 1292, 47, 446, 47, 454, 15, 1301, 15, 1301, 15, 1301, 15, 1301, 15, 1301, 15, 1299, 47, 7154};

void delay_ten_us(uint16_t us) {
  uint8_t timer;
  while (us != 0) {
    // for 8MHz we want to delay 80 cycles per 10 microseconds
    // this code is tweaked to give about that amount.
    for (timer=0; timer <= DELAY_CNT; timer++) {
      NOP;
      NOP;
    }
    NOP;
    us--;
  }
}

// xmitCodeElement from tvbgone
void xmitCodeElement(uint16_t ontime, uint16_t offtime, uint8_t PWM_code )
{
  // start Timer0 outputting the carrier frequency to IR emitters on and OC0A 
  // (PB0, pin 5)
  TCNT0 = 0; // reset the timers so they are aligned
  TIFR = 0;  // clean out the timer flags

  if(PWM_code) {
    // 99% of codes are PWM codes, they are pulses of a carrier frequecy
    // Usually the carrier is around 38KHz, and we generate that with PWM
    // timer 0
    TCCR0A =_BV(COM0A0) | _BV(WGM01);          // set up timer 0
    TCCR0B = _BV(CS00);
  } else {
    // However some codes dont use PWM in which case we just turn the IR
    // LED on for the period of time.
    PORTB &= ~_BV(IRLED);
  }

  // Now we wait, allowing the PWM hardware to pulse out the carrier 
  // frequency for the specified 'on' time
  delay_ten_us(ontime);
  
  // Now we have to turn it off so disable the PWM output
  TCCR0A = 0;
  TCCR0B = 0;
  
  // And make sure that the IR LED is off too (since the PWM may have 
  // been stopped while the LED is on!)
  PORTB |= _BV(IRLED);           // turn off IR LED
}

void play(uint16_t sequence[], int length) {
  int i, count;
  uint16_t processed[length];
  for(i=0; i<length; i+=2) {
    processed[i] = (sequence[i] + 1) * 27.0f / 10.0f;
    processed[i+1] = sequence[i+1] / 9.5f;
  }

  wdt_reset();
  for(count=0; count<5; count++) {
    for(i=0; i<length; i+=2) {
      xmitCodeElement(processed[i], 0, 1);
      xmitCodeElement(processed[i+1], 0, 0);
    }
    PORTB |= _BV(IRLED);
  }
}

// sleep from tvbgone
void sleep( void )
{
  // Shut down everything and put the CPU to sleep
  TCCR0A = 0;           // turn off frequency generator (should be off already)
  TCCR0B = 0;           // turn off frequency generator (should be off already)
  PORTB |= _BV(LED) |       // turn off visible LED
           _BV(IRLED);     // turn off IR LED

  delay_ten_us(1000);      // wait 10 millisec

  MCUCR = _BV(SM1) |  _BV(SE);    // power down mode,  SE enables Sleep Modes
  sleep_cpu();                    // put CPU into Power Down Sleep Mode
  
  sleep_disable();
  sbi(ADCSRA,ADEN);
}

// basic temperature reading using 10k thermistor
double temperature( void ) {
  double Temp, reading;
  int i;
  
  // enable ADC
  ADCSRA |= (1 << ADEN); 
  ADCSRA |= (1 << ADSC);      
  
  // get a few crappy readings first
  for(i=0; i<10; i++) {
    reading = analogRead(3);
  }
  Temp = log(((10240000/reading) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
  Temp = Temp - 273.15;
  Temp = (Temp * 9.0)/ 5.0 + 32.0;
  
  return Temp;
}

void blink(uint8_t count) {
  uint8_t i;
  for(i=0; i<count; i++) {
    PORTB &= ~_BV(LED);
    delay_ten_us(10000);
    PORTB |= _BV(LED);
    delay_ten_us(10000);
  }
}

int __attribute__((noreturn)) main(void) {
  bool heaterOn = false;
  int i;
  
  TCCR1 = 0;
  TCCR0A = 0;
  TCCR0B = 0;

  MCUSR = 0;
  WDTCR = _BV(WDCE) | _BV(WDE);
  
  WDTCR = 0;
  
  // set pwm frequency
  OCR0A = freq_to_timerval(36800);
  
  // configure pins for i/o
  DDRB = _BV(PB0) | _BV(LED) | _BV(IRLED) | ~_BV(PB3);
  PORTB = _BV(LED) | _BV(IRLED);

  // setup ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);
  ADMUX |= (0 << REFS0) | (0 << REFS1);
  ADMUX |= (1 << ADLAR);

  // wake up every 2 seconds to check the temperature
  wdt_enable(WDTO_2S);
  
  while(1) {
    if (temperature() < MIN_TEMP) {
      blink(5);
      if (!heaterOn) {
        heaterOn = true;
        play(power_button, 24);
      }
    } else {
      blink(2);
      if (heaterOn && temperature() > MAX_TEMP) {
        heaterOn = false;
        play(power_button, 24);
      }
    }
    sleep();
  }
}