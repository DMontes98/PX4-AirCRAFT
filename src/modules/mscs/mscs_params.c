
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

/**
 * ADP Data Publishing Mode Raw
 *
 *
 * @value 0 Raw Data Mode Deactivated
 * @value 1 Raw Data Mode Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_RAW, 0);

/**
 * ADP Data Publishing Mode Calibrated
 *
 *
 * @value 0 Calibrated Data Mode Deactivated
 * @value 1 Calibrated Data Mode Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_CAL, 1);

/**
 * ADP Data Publishing Rate
 *
 * @min 0
 * @max 7
 *
 * @value 0  1000Hz
 * @value 1  500Hz
 * @value 2  250Hz
 * @value 3  100Hz
 * @value 4  50Hz
 * @value 5  25Hz
 * @value 6  10Hz
 * @value 7  1Hz
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_RT, 3);

/**
 * ADP Angle Sensor Rate
 *
 * @min 0
 * @max 7
 *
 * @value 0  500Hz
 * @value 1  250Hz
 * @value 2  100Hz
 * @value 3  50Hz
 * @value 4  25Hz
 * @value 5  10Hz
 * @value 6  5Hz
 * @value 7  1Hz
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_ANG_RT, 2);


/**
 * ADP Pressure Sensor Rate
 *
 * @min 0
 * @max 7
 *
 * @value 0  1000Hz
 * @value 1  500Hz
 * @value 2  250Hz
 * @value 3  100Hz
 * @value 4  50Hz
 * @value 5  25Hz
 * @value 6  10Hz
 * @value 7  1Hz
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_PRS_RT, 3);

/**
 * ADP Publish Differential Pressure Sensor
 *
 *
 * @value 0 Differential Pressure Sensor Publishing Deactivated
 * @value 1 Differential Pressure Sensor Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_DIF_PRS, 1);

/**
 * ADP Publish Static Pressure Sensor
 *
 *
 * @value 0 Static Pressure Sensor Publishing Deactivated
 * @value 1 Static Pressure Sensor Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_STA_PRS, 1);

/**
 * ADP Publish Angle of Attack Sensor
 *
 *
 * @value 0 Angle of Attack Sensor Publishing Deactivated
 * @value 1 Angle of Attack Sensor Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_AOA, 1);

/**
 * ADP Publish Angle of Sideslip Sensor
 *
 *
 * @value 0 Angle of Sideslip Sensor Publishing Deactivated
 * @value 1 Angle of Sideslip Sensor Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_BETA, 1);

/**
 * ADP Publish Temperature Sensor
 *
 *
 * @value 0 Temperature Sensor Publishing Deactivated
 * @value 1 Temperature Sensor Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_TEMP, 0);

/**
 * ADP Publish Airspeed Rate Divisor
 *
 * @min 1
 * @max 16
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_ASPD_DV, 1);

/**
 * ADP Publish Altitude Rate Divisor
 *
 * @min 1
 * @max 16
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_ALT_DV, 1);


/**
 * ADP Publish Indicated Airspeed Reading
 *
 *
 * @value 0 IAS Publishing Deactivated
 * @value 1 IAS Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_IAS, 1);


/**
 * ADP Publish Calibrated Airspeed Reading
 *
 *
 * @value 0 CAS Publishing Deactivated
 * @value 1 CAS Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_CAS, 0);


/**
 * ADP Publish True Airspeed Reading
 *
 *
 * @value 0 TAS Publishing Deactivated
 * @value 1 TAS Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_TAS, 0);

/**
 * ADP Publish Pressure Altitude Reading
 *
 *
 * @value 0 Pressure Altitude Publishing Deactivated
 * @value 1 Pressure Altitude Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_PRS_ALT, 0);

/**
 * ADP Publish Density Altitude Reading
 *
 *
 * @value 0 Density Altitude Publishing Deactivated
 * @value 1 Density Altitude Publishing Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_DEN_ALT, 0);

/**
 * ADP Publish MSL Altitude Reading
 *
 *
 * @value 0 MSL Altitude Reading Deactivated
 * @value 1 MSL Altitude Reading Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_MSL_ALT, 0);

/**
 * ADP Publish AGL Altitude Reading
 *
 *
 * @value 0 AGL Altitude Reading Deactivated
 * @value 1 AGL Altitude Reading Activated
 * @boolean
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_ADP_AGL_ALT, 0);


/**
 * Define Cyphal Node Heartbeat Timeout in ms
 *
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_HB_TIMEOUT, 1250);

/**
 * Define Minimum Key Event Priority Level
 *
 * @min 0
 * @max 3
 *
 * @value 0 Debug Events and Higher
 * @value 1 Low Priority Events and Higher
 * @value 2 Medium Priority Events and Higher
 * @value 3 High Priority Events Only
 * @group MSCS
 */
PARAM_DEFINE_INT32(MSCS_KEY_EVT_LVL, 0);
