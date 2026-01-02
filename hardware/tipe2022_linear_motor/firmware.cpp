#include <Arduino.h>
//##### pin Arduino #####
// L6234:
const int PIN_PWM_U = 46; // U = green / brown_white
const int PIN_EN_U = 47;
const int PIN_PWM_V = 44; // V = brown / blue_white
const int PIN_EN_V = 42;
const int PIN_PWM_W = 45; // W = blue / green_white
const int PIN_EN_W = 43;

// Linear optical encoder:
const int PIN_ENCODER_A = 3; // link to an interrupt
const int PIN_ENCODER_B = 4;

// Command potentiometer:
const int PIN_POT = A0;

// Limit Switch:
const int PIN_END_L = 18;
const int PIN_END_R = 19;

//##### Parameters #####
// Multitasking:
const int INTERVAL_SERIAL = 1000; // interval between two serial communication default= 1000 (ms)
const int INTERVAL_MOVE = 100; //interval between each PI compute

// Pre calculate sin:
const int N_SIN = 192; // size of pre compute sinus
const int PHASE_N90 = (3 * N_SIN) / 4;
const int PHASE_90 = N_SIN / 4;
const int PHASE_120 = N_SIN / 3;
const int PHASE_240 = (2 * N_SIN) / 3;

// Control engineering:
const int KP = 1; // Proportional coefficient
const int KI = 1; // Integral coefficient
const int KD = 0; // Deriavative coefficient

// Linear optical encoder:
const int OFFSET = 30;
const int TOTAL_LENGTH = 3400;
const int PERIOD_LENGTH = 340;



//##### Variables #####
// Pre calculate sin:
int pre_sin[192] = {128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 187,
                    191, 195, 198, 201, 205, 208, 211, 214, 217, 220, 223, 226, 228, 231, 233, 235, 237, 240, 241,
                    243, 245, 246, 248, 249, 250, 251, 252, 253, 253, 254, 254, 254, 255, 254, 254, 254, 253, 253,
                    252, 251, 250, 249, 248, 246, 245, 243, 241, 240, 237, 235, 233, 231, 228, 226, 223, 220, 217,
                    214, 211, 208, 205, 201, 198, 195, 191, 187, 184, 180, 176, 172, 168, 164, 160, 156, 152, 148,
                    144, 140, 136, 132, 128, 123, 119, 115, 111, 107, 103, 99, 95, 91, 87, 83, 79, 75, 71, 68, 64,
                    60, 57, 54, 50, 47, 44, 41, 38, 35, 32, 29, 27, 24, 22, 20, 18, 15, 14, 12, 10, 9, 7, 6, 5, 4,
                    3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 5, 6, 7, 9, 10, 12, 14, 15, 18, 20, 22, 24, 27, 29,
                    32, 35, 38, 41, 44, 47, 50, 54, 57, 60, 64, 68, 71, 75, 79, 83, 87, 91, 95, 99, 103, 107, 111,
                    115, 119, 123
                   };

int index_U = 0; // index coil U
int index_V = index_U + PHASE_120; //index coil V 120°
int index_W = index_U + PHASE_240; //index coil W 240°
int vec = 0;
int amplitude = 0;

// Linear optical encoder:
volatile int pos = 0;  //step number of linear encoder

// Multitasking:
unsigned int prev_millis_serial = 0;
unsigned int prev_millis_move = 0;

// Control engineering:
int set_point = 0;
int error = 0;
int last_error = 0;
long sum_error = 0;
float cmd = 0.0;

void setup() {
    Serial.begin(115200); //default= 115200
    Serial.println(F("Linear Motor TIPE Antonin Pivard 2021-2022"));

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder, CHANGE);  //encoder interruption
    pinMode(PIN_ENCODER_B, INPUT);
    TCCR5B = TCCR5B & 0b11111000 | 0x01; // set pwm frequency to ~32kHz

    pinMode(PIN_PWM_U, OUTPUT); // U = green / brown_white
    pinMode(PIN_EN_U, OUTPUT);
    pinMode(PIN_PWM_V, OUTPUT); // V = brown / blue_white
    pinMode(PIN_EN_V, OUTPUT);
    pinMode(PIN_PWM_W, OUTPUT); // W = blue / green_white
    pinMode(PIN_EN_W, OUTPUT);

    int v = 0;
    digitalWrite(PIN_EN_U, v);// Disable coil before start control loop
    digitalWrite(PIN_EN_V, v);
    digitalWrite(PIN_EN_W, v);
    analogWrite(PIN_PWM_U, 0);
    analogWrite(PIN_PWM_V, 0);
    analogWrite(PIN_PWM_W, 0);
}

void loop() {
    // Serial communication:
    unsigned int cur_millis_serial = millis();

    if (cur_millis_serial - prev_millis_serial >= INTERVAL_SERIAL) {
        prev_millis_serial = cur_millis_serial;
        Serial.print("set point: ");
        Serial.println(set_point);
        Serial.print("pos: ");
        Serial.println(pos);
        Serial.print("error: ");
        Serial.println(error);
        Serial.print("cmd: ");
        Serial.println(cmd);
    }

    // Control loop:
    unsigned int cur_millis_move = millis();
    unsigned int delta_move = cur_millis_move - prev_millis_move;

    if (delta_move >= INTERVAL_MOVE) {
        prev_millis_move = cur_millis_move;
        set_point = analogRead(PIN_POT);
        set_point = map(set_point, 0, 1023, 0, 3000);
        
        error = set_point - pos;
        sum_error += error;

        if (error > 0) {
            vec = PHASE_N90;
        } else if (error < 0) {
            vec = PHASE_90;
        }

        cmd = error * KP + (sum_error * delta_move * KI + ((error - last_error) / (delta_move)) * KD) / 1000;
        last_error = error;
        move(cmd, vec);
    }
}
void move(float cmd, int vec) {
    index_U = int(cmd) + vec;
    index_V = index_U + PHASE_120;
    index_W = index_U + PHASE_240;

    index_U %= N_SIN;
    index_V %= N_SIN;
    index_W %= N_SIN;

    analogWrite(PIN_PWM_U, pre_sin[index_U]);
    analogWrite(PIN_PWM_V, pre_sin[index_V]);
    analogWrite(PIN_PWM_W, pre_sin[index_W]);
}
void encoder() {
    //19 cycles
    if ((PINE & B00100000) == (PING & B00100000)) {
        ++pos;
    } else {
        --pos;
    }
}