// Requires headers for AVR defines and ISR function
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define DEBOUNCE_TIME 1000
#define CLOCK_MAX 10000
#define CLOCK_MIN 50

#define PIN_INCR PB3 // Interrupt pin of choice: Pin 3
#define PIN_DECR PB4 // Interrupt pin of choice: Pin 3
#define PIN_MODE PB2 // Pin2
#define PIN_LED PB1  // Pin 1
#define PIN_SIG PB0  // Pin 0

typedef bool ExecMode;
#define MODE_AUTO 1
#define MODE_MANUAL 0

unsigned int clk_auto = 1000;
unsigned int clk_auto_exec = 0;

unsigned int clk_manual = 1000;
unsigned int clk_manual_exec = 0;

int tick_auto = LOW;
int tick_manual = LOW;

int time_mode = 0, time_incr = 0, time_decr = 0;
int data_mode, data_incr, data_decr;

void setup()
{
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_SIG, OUTPUT);

    pinMode(PIN_MODE, INPUT_PULLUP);
    pinMode(PIN_INCR, INPUT_PULLUP);
    pinMode(PIN_DECR, INPUT_PULLUP);
}

ExecMode fetch_mode(int time)
{
    int data_exec = digitalRead(PIN_MODE);
    int time_diff = time - time_mode;
    if (data_exec != data_mode && time_diff > DEBOUNCE_TIME)
    {
        data_mode = data_exec;
        time_mode = time;
    }
    return data_mode == LOW ? MODE_MANUAL : MODE_AUTO;
}

bool is_incr_clicked(int time)
{
    int data_exec = digitalRead(PIN_INCR);
    int time_diff = time - time_incr;
    if (data_exec != data_incr && time_diff > DEBOUNCE_TIME)
    {
        data_incr = data_exec;
        time_incr = time;
    }
    return data_incr == LOW;
}

bool is_decr_clicked(int time)
{
    int data_exec = digitalRead(PIN_DECR);
    int time_diff = time - time_decr;
    if (data_exec != data_decr && time_diff > DEBOUNCE_TIME)
    {
        data_decr = data_exec;
        time_decr = time;
    }
    return data_decr == LOW;
}

void manual_clk_exec()
{
    if (clk_manual_exec >= clk_manual)
    {
        tick_manual = false;
    }
    else
    {
        tick_manual = true;
        clk_manual_exec++;
    }
}

void manual_clk_evt(bool incr, bool decr)
{
    if (decr || incr)
    {
        clk_manual_exec = 0;
    }
}

void auto_clk_exec()
{
    if (clk_auto_exec >= clk_auto)
    {
        tick_auto = !tick_auto;
        clk_auto_exec = 0;
    }
    else
    {
        clk_auto_exec++;
    }
}

void auto_clk_evt(bool incr, bool decr)
{
    if (incr)
    {
        clk_auto_exec = 0;
        clk_auto += clk_auto / 4;
        if (clk_auto > CLOCK_MAX)
        {
            clk_auto = CLOCK_MAX;
        }
    }
    if (decr)
    {
        clk_auto_exec = 0;
        clk_auto -= clk_auto / 4;
        if (clk_auto < CLOCK_MIN)
        {
            clk_auto = CLOCK_MIN;
        }
    }
}

void loop()
{
    int time = millis();
    ExecMode mode = fetch_mode(time);
    bool decr = is_decr_clicked(time);
    bool incr = is_incr_clicked(time);

    if (mode == MODE_AUTO)
        auto_clk_evt(incr, decr);
    else
        manual_clk_evt(incr, decr);

    auto_clk_exec();
    manual_clk_exec();

    digitalWrite(PIN_SIG, mode == MODE_AUTO ? tick_auto : tick_manual);
    digitalWrite(PIN_LED, tick_auto);
    delay(1);
}