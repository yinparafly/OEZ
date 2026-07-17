/**
 * Minimal MC parameters for CollisionPrevention compatibility
 * in fixed-wing builds where MC modules are disabled.
 *
 * ===== 新增文件 =====
 */

/**
 * Position controller proportional gain
 * @group Collision Prevention
 */
PARAM_DEFINE_FLOAT(MPC_XY_P, 0.95f);

/**
 * Maximum horizontal jerk
 * @group Collision Prevention
 */
PARAM_DEFINE_FLOAT(MPC_JERK_MAX, 8.0f);

/**
 * Maximum horizontal acceleration
 * @group Collision Prevention
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR, 3.0f);

/**
 * Velocity P gain for horizontal position control
 * @group Collision Prevention
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P_ACC, 1.8f);

/**
 * Maximum horizontal velocity in manual flight
 * @group Collision Prevention
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MANUAL, 10.0f);
