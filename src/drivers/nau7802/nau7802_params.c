/**
 * Zero offset
 *
 * This parameter sets the y-offset of the load calculation
 *
 * @unit lb
 * @min -100.0
 * @max 100.0
 * @decimal 5
 * @group NAU7802
 */
PARAM_DEFINE_FLOAT(NAU7802_ZERO_OFF, 0.0f);

/**
 * Calibration factor / slope
 *
 * This parameter sets the slope of the load calculation (m in mx + b = weight)
 *
 * @unit lb
 * @min -100.0
 * @max 100.0
 * @decimal 5
 * @group NAU7802
 */
PARAM_DEFINE_FLOAT(NAU7802_CAL_FCTR, 0.0f);
