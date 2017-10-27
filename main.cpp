#include "mbed.h"
#include "BufferedSerial.h"
#include "QEI.h"
#include "PID.h"

#define MAXSPEED 100
#define MAXSPEEDINPUT 100
#define STANDBYSPEED 0.1
#define KP 2
#define KI 0.25
#define KD 0
#define DELTA_T 0.02

#define M0_PWM PC_6
#define M0_DIR1 PC_8
#define M0_DIR2 PB_8
#define M0_FAULT PC_9
#define M0_ENCA PB_9
#define M0_ENCB PC_5

#define M1_PWM PB_1
#define M1_DIR1 PB_2
#define M1_DIR2 PA_8
#define M1_FAULT PA_9
#define M1_ENCA PB_10
#define M1_ENCB PB_15

#define M2_PWM PA_11
#define M2_DIR1 PA_12
#define M2_DIR2 PA_6
#define M2_FAULT PA_5
#define M2_ENCA PA_7
#define M2_ENCB PB_12

class Motor
{

private:
    volatile int currentSpeed;
    int expectedSpeed;
    DigitalOut dir1, dir2;
    PwmOut pwm;
    DigitalIn fault;
    QEI enc;
    PID pid;

public:

    Motor(PinName dir1pin, PinName dir2pin, PinName pwmpin, PinName encapin, PinName encbpin, PinName faultpin):
        dir1(dir1pin),
        dir2(dir2pin),
        pwm(pwmpin),
        fault(faultpin),
        enc(encapin, encbpin, NC, 1200, QEI::X4_ENCODING),
        pid(KP, KI, KD, DELTA_T) {
        pwm.period_ms(3);
        pid.setInputLimits(-1.0, 1.0);
        pid.setOutputLimits(-1.0, 1.0);
        pid.setBias(0.0);
        pid.setMode(1);
    }

    void stop() {
        dir1 = 0;
        dir2 = 0;
    }

    void setSpeed(double spd) {
        if (spd > 1.0) {
            spd = 1.0;
        } else if (spd < -1.0) {
            spd = -1.0;
        }

        if (spd == 0) {
            stop();
        } else if (spd > 0) {
            dir1 = 1;
            dir2 = 0;
            pwm = spd;
        } else {
            dir1 = 0;
            dir2 = 1;
            pwm = -spd;
        }
    }

    void setExpectedSpeed(int speed) {
        expectedSpeed = speed;
    }

    int getSpeed() {
        return currentSpeed;
    }

    void processPID() {
        currentSpeed = enc.getPulses();
        enc.reset();

        if (expectedSpeed == 0 && currentSpeed == 0) {
            setSpeed(0);
            return;
        }

        pid.setProcessValue((float) currentSpeed / MAXSPEED);
        pid.setSetPoint((float) expectedSpeed / MAXSPEEDINPUT);
        setSpeed(pid.compute());
    }
};

BufferedSerial serial(USBTX, USBRX);
char buffer[32];

Motor m[3] = {
    Motor(M0_DIR1, M0_DIR2, M0_PWM, M0_ENCA, M0_ENCB, M0_FAULT),
    Motor(M1_DIR1, M1_DIR2, M1_PWM, M1_ENCA, M1_ENCB, M1_FAULT),
    Motor(M2_DIR1, M2_DIR2, M2_PWM, M2_ENCA, M2_ENCB, M2_FAULT)
};


DigitalOut LED1R(LED1);
Ticker pidTicker;

void processPID()
{
    for (uint8_t i = 0; i < 3; i++) {
        m[i].processPID();
    }
}

char* readNumStringUntil(char c)
{
    uint8_t i;
    for(i = 0; i < 31; i++) {
        buffer[i] = serial.getc();
        if (buffer[i] == c || ((buffer[i] < '0' || buffer[i] > '9') && buffer[i] != '-'))
            break;
    }
    buffer[i] = 0;
    return buffer;
}

int main()
{
    pidTicker.attach(&processPID, DELTA_T);
    serial.baud(115200);

    int speed[3];

    while (true) {
        serial.printf("%d:%d:%d\n",  m[0].getSpeed(), m[1].getSpeed(), m[2].getSpeed());
        if (serial.readable()) {
            speed[0] = atoi(readNumStringUntil(':'));
            speed[1] = atoi(readNumStringUntil(':'));
            speed[2] = atoi(readNumStringUntil(';'));

            for (uint8_t i = 0; i < 3; i++) {
                m[i].setExpectedSpeed(speed[i]);
            }
        }
        wait(DELTA_T);
    }
}