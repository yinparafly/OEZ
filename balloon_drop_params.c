/**
 * Balloon Drop Parameters
 *
 * ===== 新增文件 =====
 */

/**
 * Trigger airspeed ratio for balloon drop pull-up
 *
 * Ratio of cruise airspeed at which pull-up is triggered
 *
 * @min 0.3
 * @max 0.9
 * @decimal 2
 * @increment 0.05
 * @group Balloon Drop
 */
PARAM_DEFINE_FLOAT(BD_TRIG_RATIO, 0.7f);

/**
 * Pull-up load factor for balloon drop
 *
 * Load factor in g units (e.g. 1.7 means 1.7g total)
 *
 * @min 1.1
 * @max 3.0
 * @decimal 1
 * @increment 0.1
 * @group Balloon Drop
 */
PARAM_DEFINE_FLOAT(BD_LOAD_FACTOR, 1.7f);

/**
 * Cruise airspeed for balloon drop
 *
 * Large cruise airspeed used to calculate trigger speed
 *
 * @unit m/s
 * @min 5
 * @max 50
 * @decimal 1
 * @increment 1
 * @group Balloon Drop
 */
PARAM_DEFINE_FLOAT(BD_CRUISE_SPD, 25.0f);

/**
 * Motor ramp-up time for balloon drop
 *
 * Time in seconds to ramp throttle from 0 to cruise
 *
 * @unit s
 * @min 0.5
 * @max 5.0
 * @decimal 1
 * @increment 0.5
 * @group Balloon Drop
 */
PARAM_DEFINE_FLOAT(BD_MOTOR_RAMP, 2.0f);

/**
 * Cruise throttle for balloon drop
 *
 * Normalized throttle (0-1) for cruise flight
 *
 * @min 0.2
 * @max 0.9
 * @decimal 2
 * @increment 0.05
 * @group Balloon Drop
 */
PARAM_DEFINE_FLOAT(BD_CRUISE_THR, 0.6f);

/**
 * Pitch angle threshold for pull-up completion
 *
 * Pitch angle in degrees at which pull-up is considered complete
 *
 * @unit deg
 * @min -20
 * @max 10
 * @decimal 0
 * @increment 5
 * @group Balloon Drop
 */
PARAM_DEFINE_FLOAT(BD_PITCH_THRESH, -10.0f);
