/*
Chip type               : ATtiny13A
AVR Core Clock frequency: 1,200000 MHz
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#define _IN(bit) (PINB & _BV(bit))
#define _SET(bit) (PORTB |= _BV(bit))
#define _CLEAR(bit) (PORTB &= ~_BV(bit))

#define STOP (!(_IN(PB1)))

#define MOTOR_DOWN PB0
#define MOTOR_UP PB2

#define KEY_UP (_IN(PB3))
#define KEY_DOWN (_IN(PB4))

// Bandgap Voltage Reference: Off
#define ADC_VREF_TYPE ((0 << REFS0) | (0 << ADLAR))

// timer resolution, ms
#define TIMER_RESOLUTION 4

// time constants
#define MOTOR_LIMIT (8000 / TIMER_RESOLUTION)
#define STOP_SWITCH_LIMIT (250 / TIMER_RESOLUTION)
#define KEY_DEBOUNCE (40 / TIMER_RESOLUTION)
#define KEY_MANUAL (200 / TIMER_RESOLUTION)

#define ADC_AVERAGE_SAMPLES 4
#define AUTO_CLOSE_THRESHOLD (1000 / ADC_AVERAGE_SAMPLES / TIMER_RESOLUTION)

volatile bool close_fl;
bool close_state_fl;

uint16_t adc_tmp;
uint8_t auto_close_counter;
uint8_t adc_counter = ADC_AVERAGE_SAMPLES;

// ADC interrupt service routine
ISR(ADC_vect, ISR_NAKED)
{

  adc_tmp += ADCW;

  if (--adc_counter == 0)
  {
    adc_counter = ADC_AVERAGE_SAMPLES;

    // check if ADC < ~4,5V (resistor divider 10k to close signal and 2,2k to +5v)
    if ((adc_tmp / ADC_AVERAGE_SAMPLES) < 920)
    {
      if (auto_close_counter >= AUTO_CLOSE_THRESHOLD)
      {
        close_fl = true;
      }
      else
      {
        auto_close_counter++;
      }
    }
    else
    {
      auto_close_counter = 0;
      close_fl = false;
    }

    adc_tmp = 0;
  }

  reti();
}

uint8_t stop_switch_counter;
uint8_t key_up_counter, key_down_counter;
uint16_t motor_timer;

bool auto_up, auto_down;

// Timer 0 overflow interrupt service routine
ISR(TIM0_OVF_vect, ISR_NAKED)
{

  // Reinitialize Timer 0 value
  TCNT0 = 0xB5;

  // Place your code here
  if (close_fl)
  {
    if (!close_state_fl)
    {
      close_state_fl = true;

      key_up_counter = 0;
      key_down_counter = 0;
      auto_up = false;
      auto_down = false;

      // stop motor when motor was on in any direction
      _CLEAR(MOTOR_UP);
      _CLEAR(MOTOR_DOWN);
      motor_timer = 0;

      // start motor up
      _SET(MOTOR_UP);
      stop_switch_counter = 0;
    }
  }
  else
  {
    if (close_state_fl)
    {
      // stop motor when motor was on in any direction
      _CLEAR(MOTOR_UP);
      _CLEAR(MOTOR_DOWN);
      motor_timer = 0;

      close_state_fl = false;
      // set stop switch gpio default state
      DDRB &= ~(1 << DDB1);
      _SET(PB1);
    }
    else
    {
      if (KEY_UP)
      {
        if (key_up_counter <= KEY_MANUAL)
        {
          key_up_counter++;
        }

        if (!auto_up && key_up_counter > KEY_DEBOUNCE && key_up_counter < KEY_MANUAL)
        {
          if (_IN(MOTOR_UP) || _IN(MOTOR_DOWN))
          {
            // stop motor when motor was on in any direction
            _CLEAR(MOTOR_UP);
            _CLEAR(MOTOR_DOWN);
            key_up_counter = KEY_MANUAL;
          }
          else
          {
            // start motor up
            _SET(MOTOR_UP);
            stop_switch_counter = 0;
            auto_up = true;
          }
        }
      }
      else
      {
        if (key_up_counter >= KEY_MANUAL)
        {
          // stop motor up (manual mode)
          _CLEAR(MOTOR_UP);
        }
        auto_up = false;
        key_up_counter = 0;
      }

      if (KEY_DOWN)
      {
        if (key_down_counter <= KEY_MANUAL)
        {
          key_down_counter++;
        }

        if (!auto_down && key_down_counter > KEY_DEBOUNCE && key_down_counter < KEY_MANUAL)
        {
          if (_IN(MOTOR_UP) || _IN(MOTOR_DOWN))
          {
            // stop motor when motor was on in any direction
            _CLEAR(MOTOR_UP);
            _CLEAR(MOTOR_DOWN);
            key_down_counter = KEY_MANUAL;
          }
          else
          {
            // start motor down
            _SET(MOTOR_DOWN);
            stop_switch_counter = 0;
            auto_down = true;
          }
        }
      }
      else
      {
        if (key_down_counter >= KEY_MANUAL)
        {
          // stop motor down (manual mode)
          _CLEAR(MOTOR_DOWN);
        }
        auto_down = false;
        key_down_counter = 0;
      }
    }
  }

  if (_IN(MOTOR_UP) || _IN(MOTOR_DOWN))
  {
    if (stop_switch_counter < STOP_SWITCH_LIMIT)
    {
      stop_switch_counter++;
    }
    else
    {
      if (STOP)
      {
        // stop motor by switch
        motor_timer = MOTOR_LIMIT;
      }
    }

    if (motor_timer < MOTOR_LIMIT)
      motor_timer++;
  }
  else
    motor_timer = 0;

  if (motor_timer >= MOTOR_LIMIT)
  {
    // stop motor by time limit
    _CLEAR(MOTOR_UP);
    _CLEAR(MOTOR_DOWN);
    motor_timer = 0;
  }

  if (motor_timer == 0 && close_state_fl)
  {
    // set stop switch gpio to 0 output (cascade closing)
    DDRB |= (1 << DDB1);
    _CLEAR(PB1);
  }

  reti();
}

int main(void)
{
  // Declare your local variables here

  // Crystal Oscillator division factor: 1
  //CLKPR = (1 << CLKPCE);
  //CLKPR = (0 << CLKPCE) | (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0);

  // Input/Output Ports initialization
  // Port B initialization
  // Function: Bit5=In Bit4=In Bit3=In Bit2=Out Bit1=In Bit0=Out
  DDRB = (0 << DDB5) | (0 << DDB4) | (0 << DDB3) | (1 << DDB2) | (0 << DDB1) | (1 << DDB0);
  // State: Bit5=T Bit4=T Bit3=T Bit2=0 Bit1=P Bit0=0
  PORTB = (0 << PORTB5) | (0 << PORTB4) | (0 << PORTB3) | (0 << PORTB2) | (1 << PORTB1) | (0 << PORTB0);

  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: 18,750 kHz
  // Mode: Normal top=0xFF
  // OC0A output: Disconnected
  // OC0B output: Disconnected
  // Timer Period: 4 ms
  TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (0 << WGM01) | (0 << WGM00);
  TCNT0 = 0xB5;
  OCR0A = 0x00;
  OCR0B = 0x00;

  // Timer/Counter 0 Interrupt(s) initialization
  TIMSK0 = (0 << OCIE0B) | (0 << OCIE0A) | (1 << TOIE0);

  // External Interrupt(s) initialization
  // INT0: Off
  // Interrupt on any change on pins PCINT0-5: Off
  GIMSK = (0 << INT0) | (0 << PCIE);
  MCUCR = (0 << ISC01) | (0 << ISC00);

  // Analog Comparator initialization
  // Analog Comparator: Off
  // The Analog Comparator's positive input is
  // connected to the AIN0 pin
  // The Analog Comparator's negative input is
  // connected to the AIN1 pin
  ACSR = (1 << ACD) | (0 << ACBG) | (0 << ACO) | (0 << ACI) | (0 << ACIE) | (0 << ACIS1) | (0 << ACIS0);
  ADCSRB = (0 << ACME);
  // Digital input buffer on AIN0: On
  // Digital input buffer on AIN1: On
  DIDR0 = (0 << AIN0D) | (0 << AIN1D);

  // ADC initialization
  // ADC Clock frequency: 75,000 kHz
  // ADC Bandgap Voltage Reference: Off
  // ADC Auto Trigger Source: Timer/Counter Overflow
  // ADC channel ADC0 (PB5/RESET pin)
  // Digital input buffers on ADC0: Off, ADC1: On, ADC2: On, ADC3: On
  DIDR0 |= (1 << ADC0D) | (0 << ADC2D) | (0 << ADC3D) | (0 << ADC1D);
  ADMUX = ADC_VREF_TYPE | (0 << MUX1) | (0 << MUX0);
  ADCSRA = (1 << ADEN) | (0 << ADSC) | (1 << ADATE) | (0 << ADIF) | (1 << ADIE) | (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);
  ADCSRB = (1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);

  // start timer0
  TCCR0B = (0 << WGM02) | (0 << CS02) | (1 << CS01) | (1 << CS00);
  // Global enable interrupts
  sei();

  while (1)
  {
  }
}
