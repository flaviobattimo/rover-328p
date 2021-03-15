#include <Arduino.h>

#define MASSAGE_ASCII 1
#define STOP_MOTORS_IF_NO_COMMAND_RECEIVED 1
#define USE_FAST_START 0

#include <Wire.h>
#include <PCA9685.h>
#include <PID_v1.h>

#if MASSAGE_ASCII
    #include <AsciiMassagePacker.h>
    #include <AsciiMassageParser.h>
    AsciiMassagePacker massageOutbound;
    AsciiMassageParser massageInbound;
#else
    #include <SlipMassagePacker.h>
    #include <SlipMassageParser.h>
    SlipMassagePacker massageOutbound;
    SlipMassageParser massageInbound;
#endif

/*
 * D0: RX
 * D1: TX
 * D2 D3 D4 D5 D6 D7: motor sense probes 0-5
 * D8: motor dir 0
 * D9: motor dir 1
 * D10: motor dir 2
 * D11: motor dir 3
 * D12: motor dir 4
 * D13: LED
 * A0: power meter
 * A1: turret laser
 * A2: buzzer
 * A3: motor dir 5
 * A4: SDA
 * A5: SCL
 */

//motors are rated for 30*200=6000 RPM or 100 RPS at 12 volt with no load -> speed is 200 RPM

#define MIN_UPDATE_DELAY (50)

#define MOTORS_SPEED_CHECK_MIN_INTERVAL (50)
#define MOTORS_PROBE_RATED_RPS (2000.0f)
#define MOTORS_MAX_PWM (4095)

#define INFO_SEND_INTERVAL (100)

#define MOTORS_MODE_STOP (0)
#define MOTORS_MODE_SPEED (1)
#define MOTORS_MODE_PWM (2)

#define POWER_METER_MULTIPLIER (1.0f * (5.0f/1024.0f) / ( 185.0f / 1000.0f))
#define MAX_TIME_WITH_NO_DATA (2000)

#define MOTORS_PID_DEFAULT_KP (0.5)
#define MOTORS_PID_DEFAULT_KI (10.0)
#define MOTORS_PID_DEFAULT_KD (0.01)

#define PWMC_LOW_SERVOS_START_CHANNEL 0
#define PWMC_LOW_FREQUENCY 50
#define PWMC_LOW_ADDRESS (B000000)
//#define PWMC_LOW_MOTORS_START_CHANNEL 8
//#define PWMC_LOW_BOTTOM_LED_CHANNEL 15

#define PWMC_HIGH_MOTORS_START_CHANNEL 8
#define PWMC_HIGH_FREQUENCY 1000
#define PWMC_HIGH_ADDRESS (B000001)
#define PWMC_HIGH_BOTTOM_LED_CHANNEL 15

#define MOTORS_DIRECTION_0_PIN 8
#define MOTORS_DIRECTION_1_PIN 9
#define MOTORS_DIRECTION_2_PIN 10
#define MOTORS_DIRECTION_3_PIN 11
#define MOTORS_DIRECTION_4_PIN 12
#define MOTORS_DIRECTION_5_PIN A3

#define TURRET_LASER_PIN A1

#define BUZZER_PIN A2

PCA9685 pwmControllerLow;
#ifdef PWMC_HIGH_ADDRESS
PCA9685 pwmControllerHigh;
#endif
PCA9685_ServoEvaluator pwmServo1;

typedef struct Motor_S{
    uint16_t* pwmValue;
    uint8_t direction;
#if USE_FAST_START
    uint8_t stopped;
#endif
    double desiredSpeed;
    double actualSpeed;
    double calculatedPwm;
    PID* pid;
} Motor;

typedef struct motorsProbe_S{
    uint16_t probe[6];
    uint8_t last;
    uint8_t flag;
    uint8_t counter;
} motorsProbe_T;

float motorsKp = MOTORS_PID_DEFAULT_KP;
float motorsKi = MOTORS_PID_DEFAULT_KI;
float motorsKd = MOTORS_PID_DEFAULT_KD;

uint32_t motorsLastCommandReceived = 0;
uint32_t motorsLastUpdate = 0;
uint8_t motorsUpdateRequested = 0;
volatile motorsProbe_T motorsProbe;
uint32_t motorsProbeLastInterval;
uint32_t motorsLastSpeedCheckTime = 0;
uint8_t motorsMode = MOTORS_MODE_STOP;

Motor motors[6];
uint16_t motors_pwm[6];

uint32_t servosLastUpdate = 0;
uint8_t servosUpdateRequested = 0;
uint16_t servos_pwm[6];

uint8_t led = 0;

uint32_t infoLastSendTime = 0;

void pciSetup(uint8_t pin)
{
    pinMode(pin ,INPUT);
    digitalWrite(pin,HIGH);
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

float mapfq(float x, float in_max, float out_max) {
  return (x) * (out_max) / (in_max);
}
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// handle pin change interrupt for D0 to D7 here
// probes are attached to Motor0:D2 - Motor5:D7
ISR(PCINT2_vect) 
{
    uint8_t values = PIND & B11111100;

    if (motorsProbe.flag){
        motorsProbe.last=values;
        motorsProbe.flag=0;
        motorsProbe.probe[0]=0;
        motorsProbe.probe[1]=0;
        motorsProbe.probe[2]=0;
        motorsProbe.probe[3]=0;
        motorsProbe.probe[4]=0;
        motorsProbe.probe[5]=0;
    }

    uint8_t changed = motorsProbe.last ^ values;
    motorsProbe.last = values;

    if (changed & B00000100) motorsProbe.probe[0]++;
    if (changed & B00001000) motorsProbe.probe[1]++;
    if (changed & B00010000) motorsProbe.probe[2]++;
    if (changed & B00100000) motorsProbe.probe[3]++;
    if (changed & B01000000) motorsProbe.probe[4]++;
    if (changed & B10000000) motorsProbe.probe[5]++;

    motorsProbe.counter++;
    
}

void stopMotors(){
    memset(motors_pwm,0,sizeof(motors_pwm));
#if defined(PWMC_HIGH_ADDRESS) && defined(PWMC_HIGH_MOTORS_START_CHANNEL)
    pwmControllerHigh.setChannelsPWM(PWMC_HIGH_MOTORS_START_CHANNEL, sizeof(motors_pwm)/sizeof(uint16_t), motors_pwm);
#elif defined(PWMC_LOW_MOTORS_START_CHANNEL)
    pwmControllerLow.setChannelsPWM(PWMC_LOW_MOTORS_START_CHANNEL, sizeof(motors_pwm)/sizeof(uint16_t), motors_pwm);
#endif
    motorsMode=MOTORS_MODE_STOP;
}

void setup()
{
	Serial.begin(115200);

    Wire.begin();                      // Wire must be started first
    Wire.setClock(400000);             // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmControllerLow.resetDevices();       // Software resets all PCA9685 devices on Wire line

    pwmControllerLow.init(PWMC_LOW_ADDRESS);        // Address pins A5-A0 set to B000000, default mode settings
    pwmControllerLow.setPWMFrequency(PWMC_LOW_FREQUENCY);  // 50Hz provides 20ms standard servo phase length

#ifdef PWMC_HIGH_ADDRESS
    pwmControllerHigh.init(PWMC_HIGH_ADDRESS);
    pwmControllerHigh.setPWMFrequency(PWMC_HIGH_FREQUENCY);
#endif

    //set default motor values
    for (int i=0;i<6;i++){
        Motor *m = &motors[i];
        m->pwmValue = &motors_pwm[i];
        m->direction = 0;
#if USE_FAST_START
        m->stopped = 1;
#endif
        m->desiredSpeed = 0;
        m->actualSpeed = 0;
        m->calculatedPwm = 0;
        m->pid = new PID(&m->actualSpeed,&m->calculatedPwm,&m->desiredSpeed,motorsKp,motorsKi,motorsKd,DIRECT);
        m->pid->SetOutputLimits(0,4095);
        m->pid->SetMode(AUTOMATIC);
        m->pid->SetSampleTime(MOTORS_SPEED_CHECK_MIN_INTERVAL);
    }

    //Motor direction output
    pinMode(MOTORS_DIRECTION_0_PIN,OUTPUT); digitalWrite(MOTORS_DIRECTION_0_PIN,0);
    pinMode(MOTORS_DIRECTION_1_PIN,OUTPUT); digitalWrite(MOTORS_DIRECTION_1_PIN,0);
    pinMode(MOTORS_DIRECTION_2_PIN,OUTPUT); digitalWrite(MOTORS_DIRECTION_2_PIN,0);
    pinMode(MOTORS_DIRECTION_3_PIN,OUTPUT); digitalWrite(MOTORS_DIRECTION_3_PIN,0);
    pinMode(MOTORS_DIRECTION_4_PIN,OUTPUT); digitalWrite(MOTORS_DIRECTION_4_PIN,0);
    pinMode(MOTORS_DIRECTION_5_PIN,OUTPUT); digitalWrite(MOTORS_DIRECTION_5_PIN,0);

    memset(&motorsProbe,0,sizeof(motorsProbe_T));

    stopMotors();

    servos_pwm[0] = servos_pwm[1] = servos_pwm[2] = servos_pwm[3] = servos_pwm[4] = servos_pwm[5] = pwmServo1.pwmForAngle(0);
    pwmControllerLow.setChannelsPWM(PWMC_LOW_SERVOS_START_CHANNEL, sizeof(servos_pwm)/sizeof(uint16_t), servos_pwm);

    //reset lights
#if defined(PWMC_LOW_BOTTOM_LED_CHANNEL)
    pwmControllerLow.setChannelPWM(PWMC_LOW_BOTTOM_LED_CHANNEL,0);
#elif defined(PWMC_HIGH_BOTTOM_LED_CHANNEL) && defined(PWMC_HIGH_ADDRESS)
    pwmControllerHigh.setChannelPWM(PWMC_HIGH_BOTTOM_LED_CHANNEL,0);
#endif
    pinMode(TURRET_LASER_PIN, OUTPUT);
    digitalWrite(TURRET_LASER_PIN,LOW);

    pciSetup(2);
    pciSetup(3);
    pciSetup(4);
    pciSetup(5);
    pciSetup(6);
    pciSetup(7);

    pinMode(BUZZER_PIN, OUTPUT);
    
}

void loop()
{
    
    if ( massageInbound.parseStream( &Serial ) ) {

        if ( massageInbound.fullMatch ("sMS") ) {
            Motor* m = motors;
            for (uint8_t i=0;i<6;i++){
                int16_t v = massageInbound.nextInt();
                if (v<0){
                    m->desiredSpeed = -v;
                    m->direction = 1;
                }
                else{
                    m->desiredSpeed = v;
                    m->direction = 0;
                }

                m++;
            }
            motorsMode=MOTORS_MODE_SPEED;
            motorsLastCommandReceived = millis();
        }

        else if ( massageInbound.fullMatch ("sMP") ) {
            Motor* m = motors;
            for (uint8_t i=0;i<6;i++){
                int16_t v = massageInbound.nextInt();
                if (v<0){
                    *m->pwmValue = (uint16_t)(-v);
                    m->direction = 1;
                }
                else{
                    *m->pwmValue = (uint16_t)(v);
                    m->direction = 0;
                }

                m++;
            }
            motorsMode=MOTORS_MODE_PWM;
            motorsUpdateRequested=1;
            motorsLastCommandReceived = millis();
        }

        else if ( massageInbound.fullMatch ("sMK") ) {
            float Kp,Ki,Kd;
            Kp = massageInbound.nextFloat();
            Ki = massageInbound.nextFloat();
            Kd = massageInbound.nextFloat();

            Motor* m = motors;
            for (uint8_t i=0;i<6;i++){
                m->pid->SetTunings(Kp,Ki,Kd,1);
                m++;
            }

        }

        else if ( massageInbound.fullMatch ("sSA") ) {
            for (uint8_t i = 0; i<sizeof(servos_pwm)/sizeof(uint16_t); i++){
                servos_pwm[i]=pwmServo1.pwmForAngle(massageInbound.nextByte());
            }
            servosUpdateRequested=1;
        }

        else if ( massageInbound.fullMatch ("sSP") ) {
            for (uint8_t i = 0; i<sizeof(servos_pwm)/sizeof(uint16_t); i++){
                servos_pwm[i]=massageInbound.nextInt();
            }
            servosUpdateRequested=1;
        }

        else if ( massageInbound.fullMatch("sLP")){
            int bottomLed = massageInbound.nextInt(); //used by ECU
            massageInbound.nextInt(); //used by ESP32CAM
            int turretLaser = massageInbound.nextInt(); //used by UNO
            //set LED light
#if defined(PWMC_LOW_BOTTOM_LED_CHANNEL)
            pwmControllerLow.setChannelPWM(PWMC_LOW_BOTTOM_LED_CHANNEL,bottomLed);
#elif defined(PWMC_HIGH_BOTTOM_LED_CHANNEL)
            pwmControllerHigh.setChannelPWM(PWMC_HIGH_BOTTOM_LED_CHANNEL,bottomLed);
#endif
            digitalWrite(TURRET_LASER_PIN,turretLaser ? HIGH : LOW);
        }

        else if ( massageInbound.fullMatch("sTN")){
            int buzzerFrequency = massageInbound.nextInt();
            long buzzerDuration = massageInbound.nextLong();
            if (buzzerFrequency==0){
                noTone(BUZZER_PIN);
            }
            else if (buzzerDuration>0){
                tone(BUZZER_PIN,buzzerFrequency,buzzerDuration);
            }
            else{
                tone(BUZZER_PIN,buzzerFrequency);
            }
        }

    }

    uint32_t time = millis();

    if (time > motorsLastSpeedCheckTime + MOTORS_SPEED_CHECK_MIN_INTERVAL){
        motorsProbe_T motorsProbeCopy;
        memcpy(&motorsProbeCopy,&motorsProbe,sizeof(motorsProbe_T));

        if (motorsProbeCopy.counter!=motorsProbe.counter){
            //abort read, the ISR has changed values
        }
        else{
            motorsProbe.flag=1;

            //calculate the multiplier to get a speed in probes per second
            float multiplier = 1000.0f / ((float)(time - motorsLastSpeedCheckTime));

            motors[0].actualSpeed = motorsProbeCopy.probe[0] * multiplier;
            motors[1].actualSpeed = motorsProbeCopy.probe[1] * multiplier;
            motors[2].actualSpeed = motorsProbeCopy.probe[2] * multiplier;
            motors[3].actualSpeed = motorsProbeCopy.probe[3] * multiplier;
            motors[4].actualSpeed = motorsProbeCopy.probe[4] * multiplier;
            motors[5].actualSpeed = motorsProbeCopy.probe[5] * multiplier;

            if (motorsMode==MOTORS_MODE_SPEED){
                //multiply every speed
                for (size_t i = 0; i < 6; i++)
                {
                    Motor *m = &motors[i];
                    
                    if (m->desiredSpeed == 0.0f){
                        //fast stop!
                        m->calculatedPwm = 0;
                    }
#if USE_FAST_START
                    else if (m->actualSpeed == 0.0f && m->stopped){
                        //fast start!

                        //precalculate a pwm speed based on desired speed
                        //desiredPwm = mapf(desiredSpeed,0,MOTORS_PROBE_RATED_RPS,0,MOTORS_MAX_PWM);
                        //equivalent to mapf but faster

                        m->calculatedPwm = mapfq(m->desiredSpeed,MOTORS_PROBE_RATED_RPS,MOTORS_MAX_PWM);
                    }
#endif
                    else{
                        m->pid->Compute();
                    }

                    if (m->calculatedPwm >= 4095.0f){
                        *m->pwmValue = 4095;
                    }
                    else if (m->calculatedPwm < 0.0f){
                        *m->pwmValue = 0;
#if USE_FAST_START
                        m->stopped = 1;
#endif
                    }
                    else{
                        *m->pwmValue = m->calculatedPwm;
                    }
                    
                }
                motorsUpdateRequested = 1;
            }

            motorsLastSpeedCheckTime = time;

        }

    }

    if (motorsUpdateRequested && time > motorsLastUpdate + MIN_UPDATE_DELAY){
        motorsUpdateRequested = 0;
        motorsLastUpdate = time;
#ifdef PWMC_LOW_MOTORS_START_CHANNEL
        pwmControllerLow.setChannelsPWM(PWMC_LOW_MOTORS_START_CHANNEL, sizeof(motors_pwm)/sizeof(uint16_t), motors_pwm);
#elif defined(PWMC_HIGH_MOTORS_START_CHANNEL)
        pwmControllerHigh.setChannelsPWM(PWMC_HIGH_MOTORS_START_CHANNEL, sizeof(motors_pwm)/sizeof(uint16_t), motors_pwm);
#endif
        digitalWrite(MOTORS_DIRECTION_0_PIN,motors[0].direction);
        digitalWrite(MOTORS_DIRECTION_1_PIN,motors[1].direction);
        digitalWrite(MOTORS_DIRECTION_2_PIN,motors[2].direction);
        digitalWrite(MOTORS_DIRECTION_3_PIN,motors[3].direction);
        digitalWrite(MOTORS_DIRECTION_4_PIN,motors[4].direction);
        digitalWrite(MOTORS_DIRECTION_5_PIN,motors[5].direction);

    }
#if STOP_MOTORS_IF_NO_COMMAND_RECEIVED
    else if (motorsMode!=MOTORS_MODE_STOP && time > motorsLastCommandReceived + MAX_TIME_WITH_NO_DATA){
        stopMotors();
    }
#endif


    if (servosUpdateRequested && time > servosLastUpdate + MIN_UPDATE_DELAY ){
        servosUpdateRequested = 0;
        servosLastUpdate = time;

        pwmControllerLow.setChannelsPWM(PWMC_LOW_SERVOS_START_CHANNEL, sizeof(servos_pwm)/sizeof(uint16_t), servos_pwm);
    }

    if (time > infoLastSendTime + INFO_SEND_INTERVAL){

        float power = ((float)(512 - analogRead(0))) * POWER_METER_MULTIPLIER;

        massageOutbound.beginPacket("stats");
        massageOutbound.addLong( time );

        massageOutbound.addByte( motorsMode );

        massageOutbound.addInt(motors[0].desiredSpeed);
        massageOutbound.addInt(motors[1].desiredSpeed);
        massageOutbound.addInt(motors[2].desiredSpeed);
        massageOutbound.addInt(motors[3].desiredSpeed);
        massageOutbound.addInt(motors[4].desiredSpeed);
        massageOutbound.addInt(motors[5].desiredSpeed);

        massageOutbound.addInt(motors[0].actualSpeed);
        massageOutbound.addInt(motors[1].actualSpeed);
        massageOutbound.addInt(motors[2].actualSpeed);
        massageOutbound.addInt(motors[3].actualSpeed);
        massageOutbound.addInt(motors[4].actualSpeed);
        massageOutbound.addInt(motors[5].actualSpeed);

        massageOutbound.addInt(motors_pwm[0]);
        massageOutbound.addInt(motors_pwm[1]);
        massageOutbound.addInt(motors_pwm[2]);
        massageOutbound.addInt(motors_pwm[3]);
        massageOutbound.addInt(motors_pwm[4]);
        massageOutbound.addInt(motors_pwm[5]);

        massageOutbound.addInt(servos_pwm[0]);
        massageOutbound.addInt(servos_pwm[1]);
        massageOutbound.addInt(servos_pwm[2]);
        massageOutbound.addInt(servos_pwm[3]);
        massageOutbound.addInt(servos_pwm[4]);
        massageOutbound.addInt(servos_pwm[5]);

        massageOutbound.addFloat(power);

        massageOutbound.streamPacket(&Serial);

        infoLastSendTime=millis();

    }

    led = !led;
    digitalWrite(LED_BUILTIN,led);

}
