/**
 * Stub MC parameters for fixed-wing builds.
 * These are NOT used at runtime - only needed for compilation.
 *
 * ===== New file =====
 */

/**
 * Maximum horizontal velocity.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 12.0f);

/**
 * Maximum downwards velocity.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN, 3.0f);

/**
 * Maximum upwards velocity.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP, 3.0f);

/**
 * Maximum yaw speed.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_MAX, 200.0f);

/**
 * Yaw speed filter time constant.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_TAU, 0.08f);

/**
 * Maximum manual tilt angle.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_MAN_TILT_MAX, 35.0f);

/**
 * Position P gain.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_XY_P, 0.95f);

/**
 * Velocity P gain.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P_ACC, 1.8f);

/**
 * Maximum manual velocity.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MANUAL, 10.0f);

/**
 * Maximum jerk.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_JERK_MAX, 8.0f);

/**
 * Maximum horizontal acceleration.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR, 3.0f);

/**
 * Landing speed.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_LAND_SPEED, 0.5f);

/**
 * Landing crawl speed.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_LAND_CRWL, 0.3f);

/**
 * Landing radius.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_LAND_RADIUS, 10.0f);

/**
 * Landing RC help.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_LAND_RC_HELP, 0.0f);

/**
 * Landing altitude.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT, 10.0f);

/**
 * Hover throttle.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 0.35f);

/**
 * Takeoff ramp time.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_TKO_RAMP_T, 1.0f);

/**
 * Takeoff speed.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_TKO_SPEED, 1.5f);

/**
 * Altitude P gain.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_Z_P, 1.0f);

/**
 * Auto downwards velocity.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_DN, 1.5f);

/**
 * Auto upwards velocity.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_UP, 1.5f);

/**
 * Maximum downwards acceleration.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_ACC_DOWN_MAX, 3.0f);

/**
 * Maximum upwards acceleration.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX, 4.0f);

/**
 * Maximum horizontal acceleration auto.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX, 5.0f);

/**
 * Altitude control mode.
 * @group MC stubs
 */
PARAM_DEFINE_INT32(MPC_ALT_MODE, 2);

/**
 * Maximum horizontal hold velocity.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_XY, 0.8f);

/**
 * Maximum vertical hold velocity.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_Z, 0.6f);

/**
 * Auto jerk limit.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_JERK_AUTO, 1.0f);

/**
 * Cruise speed.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_XY_CRUISE, 10.0f);

/**
 * Maximum position error.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_XY_ERR_MAX, 2.0f);

/**
 * Trajectory P gain.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_XY_TRAJ_P, 0.5f);

/**
 * Yaw auto acceleration.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_YAWRAUTO_ACC, 30.0f);

/**
 * Yaw auto maximum rate.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_YAWRAUTO_MAX, 45.0f);

/**
 * Yaw control mode.
 * @group MC stubs
 */
PARAM_DEFINE_INT32(MPC_YAW_MODE, 0);

/**
 * Maximum backward velocity manual.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_BACK, 3.0f);

/**
 * Maximum sideways velocity manual.
 * @group MC stubs
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_SIDE, 3.0f);
