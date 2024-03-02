#include <Arduino.h>
#include <motor_parameters.hpp>
#include <SimpleFOC.h>

// AS5600 magnetic encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// SimpleFOCMini driver
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3, EN);

// GBM5208-75T motor
BLDCMotor motor = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_);  // NB: for pure voltage control, do not pass the phase resistance!

// Commander
float target = 0;
Commander commander = Commander(Serial, '\n');
void doTarget(char* cmd) { commander.scalar(&target, cmd); }
void onMotion(char* cmd){ commander.motion(&motor,cmd); }

// Control button
#define BUTTON 23

// Starting time of the experiment
uint32_t t_start = 0;
uint32_t t_loop = 0;

// Offset of the encoder with respect to the vertical
float offset = -0.35;   // position = measurement - offset

void setup() {
    Serial.begin(115200);

    // Sensor setup 
    Wire.setClock(400000);
    sensor.init();

    // Driver setup
    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver.init();

    // Motor setup
        // Set the current limit to avoid overheating
        motor.current_limit = CURRENT_LIMIT;
        
        // Initial position detection (I think) 
        motor.voltage_sensor_align = 3;

        // Set modulation to space vectors (why not? that's cool)
        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

        // Low pass filters
        motor.LPF_velocity = 0.01;
        //motor.LPF_angle = 0.01;

        // Control type
        motor.controller = MotionControlType::velocity;
        //motor.torque_controller = TorqueControlType::voltage;
        motor.target = target;

        // Control gains
        motor.PID_velocity.P = 0.05;
        motor.PID_velocity.I = 5;
        motor.PID_velocity.D = 0;
        motor.P_angle.P = 10;

        // Limits
        motor.velocity_limit = 0.5;

        // Link the sensor and the driver 
        motor.linkSensor(&sensor);
        motor.linkDriver(&driver);

        // Setup monitoring
        motor.useMonitoring(Serial);
        motor.monitor_downsample = 100;

        // Start the motor
        motor.init();
        motor.initFOC();

        motor.sensor_direction = CCW;
        motor.sensor_offset = offset;
        //motor.zero_electric_angle = 5.68;

    _delay(1000);

    // Commander
    commander.add('T',doTarget,"target");
    commander.add('M',onMotion,"motion control");

    // Wait for button press
    while (!digitalRead(BUTTON));
    _delay(1000);

    // Start the stopwatch
    t_start = micros();
    t_loop = t_start;
}

void loop() {
    // Control loop
    motor.loopFOC();
    motor.move();
    //motor.monitor();
    commander.run();

    //Serial.println(target);
}
