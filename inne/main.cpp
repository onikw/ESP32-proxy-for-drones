#include <Arduino.h>
#include "sbus.h"

// te wartości można dostosować według potrzeb

// piny
const int RX_PIN = D2;
const int TX_PIN = D3;
const int SW_PIN = D7;

// throttle
const int THR_CH = 2;
const int THR_0 = 200;
const int THR_60 = 1200;

// arm
const int ARM_CH = 4;
const int ARM_FALSE = 200;
const int ARM_TRUE = 1800;

// mode
const int MODE_CH = 5;
const int MODE_1 = 200;
const int MODE_2 = 1000;
const int MODE_3 = 1800;

// czas spadku [ms]
const int TIME_FALL = 500;
// czas 60% throttle [ms]
const int TIME_60 = 4500;

// poniżej już nic nie zmieniaj, chyba że wiesz co robisz

const int REPEAT = 20;

int thr_val = THR_0;
int arm_val = ARM_FALSE;
int mode_val = MODE_1;

boolean is_mounted = false;
boolean is_released = true;
boolean is_auto = false;

bfs::SbusRx sbus_rx(&Serial1, RX_PIN, TX_PIN, true);
bfs::SbusTx sbus_tx(&Serial1, RX_PIN, TX_PIN, true);

bfs::SbusData data;

boolean fetch_and_read()
{
    if (sbus_rx.Read())
    {
        data = sbus_rx.data();
        if (!is_mounted && data.ch[ARM_CH] >= ARM_TRUE - 100)
            is_auto = false;
        return true;
    }
    return false;
}

void send()
{
    if (is_auto)
    {
        if (!is_released)
            data.ch[THR_CH] = thr_val;
        data.ch[ARM_CH] = arm_val;
        data.ch[MODE_CH] = mode_val;
    }
    sbus_tx.data(data);
    sbus_tx.Write();
}

void setup()
{
    pinMode(SW_PIN, INPUT_PULLUP);
    sbus_rx.Begin();
    sbus_tx.Begin();
}

void loop()
{
    if (is_released && digitalRead(SW_PIN) == LOW)
    {
        is_mounted = true;
        is_released = false;
    }
    if (!is_released && digitalRead(SW_PIN) == HIGH)
    {
        is_auto = true;
        is_mounted = false;

        thr_val = THR_0;
        arm_val = ARM_TRUE;
        mode_val = MODE_2;

        for (int i = 0; i < TIME_FALL; i += REPEAT)
        {
            fetch_and_read();
            send();
            delay(REPEAT);
        }

        thr_val = THR_60;

        for (int i = 0; i < TIME_60; i += REPEAT)
        {
            fetch_and_read();
            send();
            delay(REPEAT);
        }

        fetch_and_read();
        mode_val = MODE_3;
        send();

        is_released = true;
    }
    if (fetch_and_read())
        send();
}