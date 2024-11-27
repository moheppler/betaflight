/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/gimbal.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

// TODO check if needed
#include "drivers/dshot.h"

// TODO remove if unused
#include "build/debug.h"


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoCenterPulse = 1500;
    servoConfig->dev.servoPwmRate = 50;
    servoConfig->tri_unarmed_servo = 1;
    servoConfig->servo_lowpass_freq = 0;
    servoConfig->channelForwardingStartChannel = AUX1;

#ifdef SERVO1_PIN
    servoConfig->dev.ioTags[0] = IO_TAG(SERVO1_PIN);
#endif
#ifdef SERVO2_PIN
    servoConfig->dev.ioTags[1] = IO_TAG(SERVO2_PIN);
#endif
#ifdef SERVO3_PIN
    servoConfig->dev.ioTags[2] = IO_TAG(SERVO3_PIN);
#endif
#ifdef SERVO4_PIN
    servoConfig->dev.ioTags[3] = IO_TAG(SERVO4_PIN);
#endif
}

PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
            .min = DEFAULT_SERVO_MIN,
            .max = DEFAULT_SERVO_MAX,
            .middle = DEFAULT_SERVO_MIDDLE,
            .rate = 100,
            .forwardFromChannel = CHANNEL_FORWARDING_DISABLED
        );
    }
}

// no template required since default is zero
PG_REGISTER(gimbalConfig_t, gimbalConfig, PG_GIMBAL_CONFIG, 0);

int16_t servo[MAX_SUPPORTED_SERVOS];

static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];
static int useServo;

#define COUNT_SERVO_RULES(rules) (sizeof(rules) / sizeof(servoMixer_t))
// mixer rule format servo, input, rate, speed, min, max, box
static const servoMixer_t servoMixerAirplane[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_RUDDER,      INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_ELEVATOR,    INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE,    INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerFlyingWing[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL, -100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE,    INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerTri[] = {
    { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
};

#if defined(USE_UNCOMMON_MIXERS)
static const servoMixer_t servoMixerBI[] = {
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_PITCH, -100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerDual[] = {
    { SERVO_DUALCOPTER_LEFT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_DUALCOPTER_RIGHT, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerSingle[] = {
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerHeli[] = {
    { SERVO_HELI_LEFT, INPUT_STABILIZED_PITCH,   -50, 0, 0, 100, 0 },
    { SERVO_HELI_LEFT, INPUT_STABILIZED_ROLL,    -87, 0, 0, 100, 0 },
    { SERVO_HELI_LEFT, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_STABILIZED_PITCH,  -50, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_STABILIZED_ROLL,  87, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_TOP, INPUT_STABILIZED_PITCH,   100, 0, 0, 100, 0 },
    { SERVO_HELI_TOP, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_RUD, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
};
#else
#define servoMixerBI NULL
#define servoMixerDual NULL
#define servoMixerSingle NULL
#define servoMixerHeli NULL
#endif // USE_UNCOMMON_MIXERS

static const servoMixer_t servoMixerGimbal[] = {
    { SERVO_GIMBAL_PITCH, INPUT_GIMBAL_PITCH, 125, 0, 0, 100, 0 },
    { SERVO_GIMBAL_ROLL, INPUT_GIMBAL_ROLL,  125, 0, 0, 100, 0 },
};

const mixerRules_t servoMixers[] = {
    { 0, NULL },                // entry 0
    { COUNT_SERVO_RULES(servoMixerTri), servoMixerTri },       // MULTITYPE_TRI
    { 0, NULL },                // MULTITYPE_QUADP
    { 0, NULL },                // MULTITYPE_QUADX
    { COUNT_SERVO_RULES(servoMixerBI), servoMixerBI },        // MULTITYPE_BI
    { COUNT_SERVO_RULES(servoMixerGimbal), servoMixerGimbal },    // * MULTITYPE_GIMBAL
    { 0, NULL },                // MULTITYPE_Y6
    { 0, NULL },                // MULTITYPE_HEX6
    { COUNT_SERVO_RULES(servoMixerFlyingWing), servoMixerFlyingWing },// * MULTITYPE_FLYING_WING
    { 0, NULL },                // MULTITYPE_Y4
    { 0, NULL },                // MULTITYPE_HEX6X
    { 0, NULL },                // MULTITYPE_OCTOX8
    { 0, NULL },                // MULTITYPE_OCTOFLATP
    { 0, NULL },                // MULTITYPE_OCTOFLATX
    { COUNT_SERVO_RULES(servoMixerAirplane), servoMixerAirplane },  // * MULTITYPE_AIRPLANE
    { COUNT_SERVO_RULES(servoMixerHeli), servoMixerHeli },                // * MULTITYPE_HELI_120_CCPM
    { 0, NULL },                // * MULTITYPE_HELI_90_DEG
    { 0, NULL },                // MULTITYPE_VTAIL4
    { 0, NULL },                // MULTITYPE_HEX6H
    { 0, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { COUNT_SERVO_RULES(servoMixerDual), servoMixerDual },      // MULTITYPE_DUALCOPTER
    { COUNT_SERVO_RULES(servoMixerSingle), servoMixerSingle },    // MULTITYPE_SINGLECOPTER
    { 0, NULL },                // MULTITYPE_ATAIL4
    { 0, NULL },                // MULTITYPE_CUSTOM
    { 0, NULL },                // MULTITYPE_CUSTOM_PLANE
    { 0, NULL },                // MULTITYPE_CUSTOM_TRI
    { 0, NULL },
};

int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    const uint8_t channelToForwardFrom = servoParams(servoIndex)->forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeState.channelCount) {
        return scaleRangef(constrainf(rcData[channelToForwardFrom], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, servoParams(servoIndex)->min, servoParams(servoIndex)->max);
    }

    return servoParams(servoIndex)->middle;
}

int servoDirection(int servoIndex, int inputSource)
{
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (servoParams(servoIndex)->reversedSources & (1 << inputSource)) {
        return -1;
    } else {
        return 1;
    }
}

void loadCustomServoMixer(void)
{
    // reset settings
    servoRuleCount = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (customServoMixers(i)->rate == 0) {
            break;
        }
        currentServoMixer[i] = *customServoMixers(i);
        servoRuleCount++;
    }
}

static void servoConfigureOutput(void)
{
    if (useServo) {
        servoRuleCount = servoMixers[getMixerMode()].servoRuleCount;
        if (servoMixers[getMixerMode()].rule) {
            for (int i = 0; i < servoRuleCount; i++)
                currentServoMixer[i] = servoMixers[getMixerMode()].rule[i];
        }
    }

    switch (getMixerMode()) {
    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_CUSTOM_TRI:
        loadCustomServoMixer();
        break;
    default:
        break;
    }
}

void servosInit(void)
{
    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = mixers[getMixerMode()].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (featureIsEnabled(FEATURE_SERVO_TILT) || featureIsEnabled(FEATURE_CHANNEL_FORWARDING)) {
        useServo = 1;
    }

    // give all servos a default command
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }

    if (mixerIsTricopter()) {
        servosTricopterInit();
    }

    servoConfigureOutput();
}

void servoMixerLoadMix(int index)
{
    // we're 1-based
    index++;
    // clear existing
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        customServoMixersMutable(i)->targetChannel = customServoMixersMutable(i)->inputSource = customServoMixersMutable(i)->rate = customServoMixersMutable(i)->box = 0;
    }
    for (int i = 0; i < servoMixers[index].servoRuleCount; i++) {
        *customServoMixersMutable(i) = servoMixers[index].rule[i];
    }
}

// STATIC_UNIT_TESTED void forwardAuxChannelsToServos(uint8_t firstServoIndex)
// {
//     // start forwarding from this channel
//     int channelOffset = servoConfig()->channelForwardingStartChannel;
//     const int maxAuxChannelCount = MIN(MAX_AUX_CHANNEL_COUNT, rxConfig()->max_aux_channel);
//     for (int servoOffset = 0; servoOffset < maxAuxChannelCount && channelOffset < MAX_SUPPORTED_RC_CHANNEL_COUNT; servoOffset++) {
//         pwmWriteServo(firstServoIndex + servoOffset, rcData[channelOffset++]);
//     }
// }

// Write and keep track of written servos

static uint32_t servoWritten;

STATIC_ASSERT(sizeof(servoWritten) * 8 >= MAX_SUPPORTED_SERVOS, servoWritten_is_too_small);

static void writeServoWithTracking(uint8_t index, servoIndex_e servoname)
{
    pwmWriteServo(index, servo[servoname]);
    servoWritten |= (1 << servoname);
}

// static void updateGimbalServos(uint8_t firstServoIndex)
// {
//     writeServoWithTracking(firstServoIndex + 0, SERVO_GIMBAL_PITCH);
//     writeServoWithTracking(firstServoIndex + 1, SERVO_GIMBAL_ROLL);
// }

// static void servoTable(void);
static void filterServos(void);

static void rocketmixer(double timeSinceBoot_tS)
{
    // Need persistent bookkeeping variable for motor speeds
    static float motor_prev[2] = {1.0f, 1.0f};
    
    float r_g = 0.6744; // distance from CoM to center of thrust [m] 
    
    // inverse kinematics constants
    float k_1 = 0.98480775301;
    float k_2 = 0.17364817766;

    // define torque and thrust saturations
    float max_Mx = 10000; 
    float max_My = 10000;
    float max_Mz = 10000;
    float max_Tx = 10000;
    float min_Tx = 150; // need this because inverse kinematics breaks when Tx <= 0
    
    // get command torques & thrust
    float des_Mx = pidData[FD_ROLL].Sum/100; // TODO check axes, remove scaling
    float des_My = pidData[FD_PITCH].Sum/100;
    float des_Mz = pidData[FD_YAW].Sum/100;
    
    des_Mx = constrainf(des_Mx, -max_Mx, max_Mx);
    des_My = constrainf(des_My, -max_My, max_My);
    des_Mz = constrainf(des_Mz, -max_Mz, max_Mz);

    // float des_Tx = 1000 + 1500 * sin(timeSinceBoot_tS/10); // need to figure out how to get this
    float des_Tx = 10;
    des_Tx = constrainf(des_Tx, min_Tx, max_Tx);
    // debug[1] = des_Tx;
    // debug[2] = des_Mx;
    // debug[3] = des_My;
    // debug[4] = des_Mz;
    // UNUSED(timeSinceBoot_tS);


    // calculate desired thrust vector
    float des_thrust_vector[3] = {des_Tx, -des_Mz / r_g, des_My / r_g};

    // debug[0] = des_thrust_vector[0] * 10;
    // debug[1] = des_thrust_vector[1] * 10;
    // debug[2] = des_thrust_vector[2] * 10;

    // find normalized thrust vector and extract y- and z-components
    float norm = sqrtf(des_thrust_vector[0] * des_thrust_vector[0] +
                       des_thrust_vector[1] * des_thrust_vector[1] +
                       des_thrust_vector[2] * des_thrust_vector[2]);


    float normed_y = des_thrust_vector[1] / norm;
    float normed_z = des_thrust_vector[2] / norm;

    // calculate desired servo angles
    float arg_2 = (-(k_1 * normed_y) + (k_2 * normed_z)) / (k_1 * k_1 + k_2 * k_2);
    float phi_2 = asin(arg_2);

    float arg_1 = -((k_1 * normed_z) + (k_2 * normed_y)) / ((k_1 * k_1 + k_2 * k_2) * (float)cos(phi_2));
    float phi_1 = asin(arg_1);

    float pwm_1 = 920 + 1200 *((phi_1 + 1.5708)/3.1416); // convert to pwm
    float pwm_2 = 920 + 1200 *((phi_2 + 1.5708)/3.1416);


    // Calculate command RPM based on desired thrust and torque
    float thrust_constant = 2e-8; // thrust constant [N/(revol/min)^2]
    float torque_constant = 3e-10; // torque constant [Nm/(revol/min)^2]
    double min_RPMs = 1000*1000; // minimum RPM squared for motors
    double max_RPMs = 14000*14000; // maximum RPM squared for motors

    double des_common_RPMs = des_Tx / (2 * thrust_constant);
    // debug[2] = sqrt(des_common_RPMs);
    des_common_RPMs = constraind(des_common_RPMs, min_RPMs, max_RPMs);
    // debug[3] = sqrt(des_common_RPMs);

    double des_RPMs_diff = des_Mx / torque_constant;
    // debug[4] = des_RPMs_diff;
    double max_RPMs_diff = mind(des_common_RPMs, max_RPMs - des_common_RPMs); // maximum RPMs difference between motors
    des_RPMs_diff = constraind(des_RPMs_diff, -max_RPMs_diff, max_RPMs_diff);
    // debug[5] = des_RPMs_diff;

    // double des_RPM_1 = sqrt(des_common_RPMs + des_RPMs_diff);
    // double des_RPM_2 = sqrt(des_common_RPMs - des_RPMs_diff);
    // debug[6] = des_RPM_1;
    // debug[7] = des_RPM_2;
    // UNUSED(des_RPM_1);
    // UNUSED(des_RPM_2);

    // TODO testing remove
    double des_RPM_1 = 4500;
    double des_RPM_2 = 2775;

    // Proportional RPM control

    double K_p = 0.00005; // TODO probably need to tune this

    // Get motor speeds (use doubles for increased precision)
    double motor_RPM_1 = getDshotRpm(0); // this is revolutions per second, not radians per second
    double motor_RPM_2 = getDshotRpm(1);

    // Calculate error
    double error_RPM_1 = des_RPM_1 - motor_RPM_1;
    double error_RPM_2 = des_RPM_2 - motor_RPM_2;


    // Calculate control signal
    float control_correction_1 = K_p * error_RPM_1;
    float control_correction_2 = K_p * error_RPM_2;

    debug[3] = control_correction_1 * 100;
    debug[4] = control_correction_2 * 100;

    // Calculate new control signals
    debug[1] = motor_prev[0];
    debug[2] = motor_prev[1];

    motor[0] = motor_prev[0] + control_correction_1;
    motor[1] = motor_prev[1] + control_correction_2;

    debug[5] = motor[0];
    debug[6] = motor[1];


  
    // pass angles and speeds to servos & motors (TODO might need to do some conversion here?)
    servo[0] = 0;
    servo[1] = 0;
    servo[2] = 0;
    servo[3] = 0;
    servo[4] = pwm_1;
    servo[5] = pwm_2;
    servo[6] = 0;
    servo[7] = 0;

    // TODO need this so the ESC starts with a zero command can maybe fix this later
    double motorTimer_tS = 300;
    if (timeSinceBoot_tS < motorTimer_tS) {
        motor[0] = 0;
        motor[1] = 0;
        motor_prev[0] = 1.0;
        motor_prev[1] = 1.0;
    }

    motor_prev[0] = motor[0];
    motor_prev[1] = motor[1];
}

void writeServos(double timeSinceBoot_tS)
{
    // TODO get rid of servoTable() if things work with rocketmixer()
    
    // servoTable();
    rocketmixer(timeSinceBoot_tS);
    filterServos();



    uint8_t servoIndex = 0;
    switch (getMixerMode()) {
    case MIXER_TRI:
    // case MIXER_CUSTOM_TRI:
    //     // We move servo if unarmed flag set or armed
    //     if (!(servosTricopterIsEnabledServoUnarmed() || ARMING_FLAG(ARMED))) {
    //         servo[SERVO_RUDDER] = 0; // kill servo signal completely.
    //     }
    //     writeServoWithTracking(servoIndex++, SERVO_RUDDER);
    //     break;

    // case MIXER_FLYING_WING:
    //     writeServoWithTracking(servoIndex++, SERVO_FLAPPERON_1);
    //     writeServoWithTracking(servoIndex++, SERVO_FLAPPERON_2);
    //     break;

    // case MIXER_CUSTOM_AIRPLANE:
    case MIXER_AIRPLANE:
        for (int i = SERVO_PLANE_INDEX_MIN; i <= SERVO_PLANE_INDEX_MAX; i++) {
            writeServoWithTracking(servoIndex++, i);
        }
        break;

#ifdef USE_UNCOMMON_MIXERS
    case MIXER_BICOPTER:
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_RIGHT);
        break;

    case MIXER_HELI_120_CCPM:
        writeServoWithTracking(servoIndex++, SERVO_HELI_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_HELI_RIGHT);
        writeServoWithTracking(servoIndex++, SERVO_HELI_TOP);
        writeServoWithTracking(servoIndex++, SERVO_HELI_RUD);
        break;

    case MIXER_DUALCOPTER:
        writeServoWithTracking(servoIndex++, SERVO_DUALCOPTER_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_DUALCOPTER_RIGHT);
        break;

    case MIXER_SINGLECOPTER:
        for (int i = SERVO_SINGLECOPTER_INDEX_MIN; i <= SERVO_SINGLECOPTER_INDEX_MAX; i++) {
            writeServoWithTracking(servoIndex++, i);
        }
        break;
#endif // USE_UNCOMMON_MIXERS

    default:
        break;
    }

    // Two servos for SERVO_TILT, if enabled
    // if (featureIsEnabled(FEATURE_SERVO_TILT) || getMixerMode() == MIXER_GIMBAL) {
    //     updateGimbalServos(servoIndex);
    //     servoIndex += 2;
    // }

    // Scan servos and write those marked forwarded and not written yet
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const uint8_t channelToForwardFrom = servoParams(i)->forwardFromChannel;
        if ((channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) && !(servoWritten & (1 << i))) {
            pwmWriteServo(servoIndex++, servo[i]);
        }
    }

    // forward AUX to remaining servo outputs (not constrained)
    // if (featureIsEnabled(FEATURE_CHANNEL_FORWARDING)) {
    //     forwardAuxChannelsToServos(servoIndex);
    //     servoIndex += MAX_AUX_CHANNEL_COUNT;
    // }
}

void servoMixer(void)
{
    int16_t input[INPUT_SOURCE_COUNT]; // Range [-500:+500]
    static int16_t currentOutput[MAX_SERVO_RULES];

    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        // Direct passthru from RX
        input[INPUT_STABILIZED_ROLL] = rcCommand[ROLL];
        input[INPUT_STABILIZED_PITCH] = rcCommand[PITCH];
        input[INPUT_STABILIZED_YAW] = rcCommand[YAW];
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        input[INPUT_STABILIZED_ROLL] = pidData[FD_ROLL].Sum * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_PITCH] = pidData[FD_PITCH].Sum * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_YAW] = pidData[FD_YAW].Sum * PID_SERVO_MIXER_SCALING;

        // Reverse yaw servo when inverted in 3D mode
        if (featureIsEnabled(FEATURE_3D) && (rcData[THROTTLE] < rxConfig()->midrc)) {
            input[INPUT_STABILIZED_YAW] *= -1;
        }
    }

    input[INPUT_GIMBAL_PITCH] = scaleRange(attitude.values.pitch, -1800, 1800, -500, +500);
    input[INPUT_GIMBAL_ROLL] = scaleRange(attitude.values.roll, -1800, 1800, -500, +500);

    input[INPUT_STABILIZED_THROTTLE] = motor[0] - 1000 - 500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

    // center the RC input value around the RC middle value
    // by subtracting the RC middle value from the RC input value, we get:
    // data - middle = input
    // 2000 - 1500 = +500
    // 1500 - 1500 = 0
    // 1000 - 1500 = -500
    input[INPUT_RC_ROLL]     = rcData[ROLL]     - rxConfig()->midrc;
    input[INPUT_RC_PITCH]    = rcData[PITCH]    - rxConfig()->midrc;
    input[INPUT_RC_YAW]      = rcData[YAW]      - rxConfig()->midrc;
    input[INPUT_RC_THROTTLE] = rcData[THROTTLE] - rxConfig()->midrc;
    input[INPUT_RC_AUX1]     = rcData[AUX1]     - rxConfig()->midrc;
    input[INPUT_RC_AUX2]     = rcData[AUX2]     - rxConfig()->midrc;
    input[INPUT_RC_AUX3]     = rcData[AUX3]     - rxConfig()->midrc;
    input[INPUT_RC_AUX4]     = rcData[AUX4]     - rxConfig()->midrc;

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = 0;
    }

    // mix servos according to rules
    for (int i = 0; i < servoRuleCount; i++) {
        // consider rule if no box assigned or box is active
        if (currentServoMixer[i].box == 0 || IS_RC_MODE_ACTIVE(BOXSERVO1 + currentServoMixer[i].box - 1)) {
            uint8_t target = currentServoMixer[i].targetChannel;
            uint8_t from = currentServoMixer[i].inputSource;
            uint16_t servo_width = servoParams(target)->max - servoParams(target)->min;
            int16_t min = currentServoMixer[i].min * servo_width / 100 - servo_width / 2;
            int16_t max = currentServoMixer[i].max * servo_width / 100 - servo_width / 2;

            if (currentServoMixer[i].speed == 0)
                currentOutput[i] = input[from];
            else {
                if (currentOutput[i] < input[from])
                    currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
                else if (currentOutput[i] > input[from])
                    currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
            }

            servo[target] += servoDirection(target, from) * constrain(((int32_t)currentOutput[i] * currentServoMixer[i].rate) / 100, min, max);
        } else {
            currentOutput[i] = 0;
        }
    }

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoParams(i)->rate * servo[i]) / 100L;
        servo[i] += determineServoMiddleOrForwardFromChannel(i);
    }
}

// static void servoTable(void)
// {
//     // airplane / servo mixes
//     switch (getMixerMode()) {
//     case MIXER_CUSTOM_TRI:
//     case MIXER_TRI:
//         servosTricopterMixer();
//         break;
//     case MIXER_CUSTOM_AIRPLANE:
//     case MIXER_FLYING_WING:
//     case MIXER_AIRPLANE:
//     case MIXER_BICOPTER:
//     case MIXER_DUALCOPTER:
//     case MIXER_SINGLECOPTER:
//     case MIXER_HELI_120_CCPM:
//     case MIXER_GIMBAL:
//         servoMixer();
//         break;

//     /*
//     case MIXER_GIMBAL:
//         servo[SERVO_GIMBAL_PITCH] = (((int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate * attitude.values.pitch) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
//         servo[SERVO_GIMBAL_ROLL] = (((int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);
//         break;
//     */

//     default:
//         break;
//     }

//     // camera stabilization
//     if (featureIsEnabled(FEATURE_SERVO_TILT)) {
//         // center at fixed position, or vary either pitch or roll by RC channel
//         servo[SERVO_GIMBAL_PITCH] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
//         servo[SERVO_GIMBAL_ROLL] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);

//         if (IS_RC_MODE_ACTIVE(BOXCAMSTAB)) {
//             if (gimbalConfig()->mode == GIMBAL_MODE_MIXTILT) {
//                 servo[SERVO_GIMBAL_PITCH] -= (-(int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate) * attitude.values.pitch / 50 - (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll / 50;
//                 servo[SERVO_GIMBAL_ROLL] += (-(int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate) * attitude.values.pitch / 50 + (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll / 50;
//             } else {
//                 servo[SERVO_GIMBAL_PITCH] += (int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate * attitude.values.pitch / 50;
//                 servo[SERVO_GIMBAL_ROLL] += (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll  / 50;
//             }
//         }
//     }

//     // constrain servos
//     for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
//         servo[i] = constrain(servo[i], servoParams(i)->min, servoParams(i)->max); // limit the values
//     }
// }

bool isMixerUsingServos(void)
{
    return useServo;
}

static biquadFilter_t servoFilter[MAX_SUPPORTED_SERVOS];

void servosFilterInit(void)
{
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            biquadFilterInitLPF(&servoFilter[servoIdx], servoConfig()->servo_lowpass_freq, targetPidLooptime);
        }
    }

}
static void filterServos(void)
{
#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();
#endif
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = lrintf(biquadFilterApply(&servoFilter[servoIdx], (float)servo[servoIdx]));
            // Sanity check
            servo[servoIdx] = constrain(servo[servoIdx], servoParams(servoIdx)->min, servoParams(servoIdx)->max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif
}
#endif // USE_SERVOS
