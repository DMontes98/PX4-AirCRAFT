
/**
 * Enable Modular Sensor and Computing System on Startup
 *
 * If set to true, the MSCS will run on system startup
 *
 * @value 0 Disable MSCS on Startup
 * @value 1 Enable MSCS on Startup
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_STARTUP, 0);

/**
 * Define Cyphal Node ID for Left ADP
 *
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ID_ADP_L, 22);

/**
 * Define Cyphal Node ID for Right ADP
 *
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ID_ADP_R, 23);
