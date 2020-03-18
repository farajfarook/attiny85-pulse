// Requires headers for AVR defines and ISR function
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define DEBOUNCE_TIME 1000
#define CLOCK_MAX 10000
#define CLOCK_MIN 50

#define INTERRUPT_PIN_INCR PCINT3 // Pin 3
#define INTERRUPT_PIN_DECR PCINT4 // Pin 4
#define INT_PIN_INCR PB3          // Interrupt pin of choice: Pin 3
#define INT_PIN_DECR PB4          // Interrupt pin of choice: Pin 3
#define MODE_PIN PB2              // Pin2
#define LED_PIN PB1               // Pin 1
#define SIG_PIN PB0               // Pin 0
#define PCINT_VECTOR PCINT0_vect  // This step is not necessary - it's a naming thing for clarit

bool auto_mode = true;

unsigned int auto_clock = 1000;
unsigned int current_auto_clock = 0;

unsigned int manual_clock = 1000;
unsigned int current_manual_clock = 0;

bool auto_tick = false;
bool manual_tick = false;

int mode_time = 0;
int incr_time = 0;
int decr_time = 0;

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(SIG_PIN, OUTPUT);

    // Disable interrupts during setup
    cli();

    // Enable interrupt handler (ISR) for our chosen interrupt pin
    PCMSK |= (1 << INTERRUPT_PIN_INCR);
    PCMSK |= (1 << INTERRUPT_PIN_DECR);

    // Enable PCINT interrupt in the general interrupt mask
    GIMSK |= (1 << PCIE);

    // Set our interrupt pin as input with a pullup to keep it stable
    pinMode(INT_PIN_INCR, INPUT_PULLUP);
    pinMode(INT_PIN_DECR, INPUT_PULLUP);

    // Setup mode pin
    pinMode(MODE_PIN, INPUT_PULLUP);

    // Enable interrupts after setup
    sei();
}

ISR(PCINT_VECTOR)
{
    int current_time = millis();

    int current_incr = digitalRead(INT_PIN_INCR);
    int current_incr_time_delta = current_time - incr_time;
    if (current_incr == LOW && current_incr_time_delta > DEBOUNCE_TIME)
    {
        if (auto_mode)
        {
            incr_time = current_time;
            auto_clock += auto_clock / 4;
            if (auto_clock > CLOCK_MAX)
            {
                auto_clock = CLOCK_MAX;
            }
        }
    }

    int current_decr = digitalRead(INT_PIN_DECR);
    int current_decr_time_delta = current_time - decr_time;
    if (current_decr == LOW && current_decr_time_delta > DEBOUNCE_TIME)
    {
        if (auto_mode)
        {
            decr_time = current_time;
            auto_clock -= auto_clock / 4;
            if (auto_clock < CLOCK_MIN)
            {
                auto_clock = CLOCK_MIN;
            }
        }
    }
}

void loop()
{
    bool current_mode = digitalRead(MODE_PIN) == HIGH;
    int current_time = millis();
    int mode_time_delta = current_time - mode_time;
    if (current_mode != auto_mode && mode_time_delta > DEBOUNCE_TIME)
    {
        current_auto_clock = 0;
        current_manual_clock = 0;
        auto_mode = current_mode;
        mode_time = current_time;
    }

    if (current_auto_clock >= auto_clock)
    {
        auto_tick = !auto_tick;
        current_auto_clock = 0;
    }
    else
    {
        current_auto_clock++;
    }

    if (current_manual_clock >= manual_clock)
    {
        manual_tick = false;
    }
    else
    {
        manual_tick = true;
        current_manual_clock++;
    }

    int tick = auto_mode ? auto_tick : manual_tick;
    digitalWrite(SIG_PIN, tick);
    digitalWrite(LED_PIN, auto_tick);
    delay(1);
}