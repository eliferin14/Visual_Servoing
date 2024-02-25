// Power supply
#define POWER_SUPPLY_VOLTAGE 15.1

// GBM5208-75T Motor
#define POLE_PAIRS 11
#define PHASE_RESISTANCE 7.5
#define KV_ 80 // https://www.rctimer.com/rctimer-gimbal-motor-gbm5208-75t-p0448.html

// On DYS site (http://www.dys.hk/product/BGM5208-75.html) it is reported a working point
// torque: 1950g at 5V, 0.47A
// So I guess 0.47A is the limit?
#define CURRENT_LIMIT 1 

// SimpleFOCMini driver
#define IN1 2
#define IN2 4
#define IN3 5
#define EN 15