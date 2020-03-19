// Requires headers for AVR defines and ISR function
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define DEBOUNCE_TIME 1000
#define CLOCK_MAX 10000
#define CLOCK_MIN 50

#define PIN_SPEED PB3  // Pin 3
#define PIN_MANUAL PB4 // Pin 4
#define PIN_MODE PB2   // Pin 2
#define PIN_LED PB1    // Pin 1
#define PIN_SIG PB0    // Pin 0

typedef bool ExecMode;
#define MODE_AUTO 1
#define MODE_MANUAL 0

unsigned int clk_auto = 1000;
unsigned int clk_auto_exec = 0;

unsigned int clk_manual = 1000;
unsigned int clk_manual_exec = 0;

int tick_auto = LOW;
int tick_manual = LOW;

int time_mode = 0, time_manual = 0;
int data_mode, data_manual, data_speed;

void setup()
{
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_SIG, OUTPUT);

    pinMode(PIN_MODE, INPUT_PULLUP);
    pinMode(PIN_MANUAL, INPUT_PULLUP);
    pinMode(PIN_SPEED, INPUT);
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

void fetch_clk_manual(int time)
{
    int data_exec = digitalRead(PIN_MANUAL);
    int time_diff = time - time_manual;
    if (data_exec != data_manual && time_diff > DEBOUNCE_TIME)
    {
        data_manual = data_exec;
        time_manual = time;
        clk_manual_exec = 0;
    }
}

void fetch_clk_auto(int time)
{
    int value = analogRead(PIN_SPEED);
    clk_auto = value * 5;
}

void run_clk_manual()
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

void run_clk_auto()
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

void loop()
{
    int time = millis();

    fetch_clk_auto(time);
    fetch_clk_manual(time);

    run_clk_auto();
    run_clk_manual();

    ExecMode mode = fetch_mode(time);
    digitalWrite(PIN_SIG, mode == MODE_AUTO ? tick_auto : tick_manual);
    digitalWrite(PIN_LED, tick_auto);

    delay(1);
}