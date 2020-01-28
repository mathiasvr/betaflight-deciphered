# Betaflight Deciphered

This is an attempt to document the variables of [Betaflight 4.1](https://github.com/betaflight/betaflight), motivated by my previous trouble to easily look up certain information.

When using the Betaflight command line interface or examining a diff, it can sometimes be difficult to understand what certain settings is supposed to do. This is intended to be used as a reference to quickly look this information up.

As I'm not an expert, any contributions or help with creating this document will be greatly appreciated!
Parts of this document has been based on information from [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator).

If you have trouble understanding any of the explanations, please open an issue so we can rewrite it and hopefully clear things up.

## Betaflight CLI Variables

| Variable name | BF Configurator Name | Description | Default Value | Allowed Values |
| ------ | ------ | ------ | ------ | ------ |
| gyro_hardware_lpf |  |  | NORMAL | NORMAL, 1KHZ_SAMPLING, EXPERIMENTAL |
| gyro_sync_denom |  |  | 1 | 1 - 32 |
| gyro_lowpass_type |  |  | PT1 | PT1, BIQUAD |
| gyro_lowpass_hz |  |  | 200 | 0 - 4000 |
| gyro_lowpass2_type |  |  | PT1 | PT1, BIQUAD |
| gyro_lowpass2_hz |  |  | 250 | 0 - 4000 |
| gyro_notch1_hz |  |  | 0 | 0 - 4000 |
| gyro_notch1_cutoff |  |  | 0 | 0 - 4000 |
| gyro_notch2_hz |  |  | 0 | 0 - 4000 |
| gyro_notch2_cutoff |  |  | 0 | 0 - 4000 |
| gyro_calib_duration |  |  | 125 | 50 - 3000 |
| gyro_calib_noise_limit |  |  | 48 | 0 - 200 |
| gyro_offset_yaw |  |  | 0 | -1000 - 1000 |
| gyro_overflow_detect |  |  | ALL | OFF, YAW, ALL |
| yaw_spin_recovery |  |  | ON | OFF, ON |
| yaw_spin_threshold |  |  | 1950 | 500 - 1950 |
| gyro_to_use |  | TODO: Selects the gyro(s) that should be used. | FIRST | FIRST, SECOND, BOTH |
| dyn_notch_range |  |  | MEDIUM | HIGH, MEDIUM, LOW, AUTO |
| dyn_notch_width_percent |  |  | 8 | 0 - 20 |
| dyn_notch_q |  |  | 120 | 1 - 1000 |
| dyn_notch_min_hz |  |  | 150 | 60 - 1000 |
| dyn_lpf_gyro_min_hz |  |  | 200 | 0 - 1000 |
| dyn_lpf_gyro_max_hz |  |  | 500 | 0 - 1000 |
| gyro_filter_debug_axis |  |  | ROLL | ROLL, PITCH, YAW |
| acc_hardware |  |  | AUTO | AUTO, NONE, ADXL345, MPU6050, MMA8452, BMA280, LSM303DLHC, MPU6000, MPU6500, MPU9250, ICM20601, ICM20602, ICM20608G, ICM20649, ICM20689, BMI160, FAKE |
| acc_lpf_hz |  |  | 10 | 0 - 400 |
| acc_trim_pitch |  |  | 0 | -300 - 300 |
| acc_trim_roll |  |  | 0 | -300 - 300 |
| acc_calibration |  | TODO: Accelerometer calibration values. Normally set automatically by pressing 'Calibrate Accelerometer' in BF Configurator. | 0,0,0 | Array length: 3 |
| align_mag |  |  | DEFAULT | DEFAULT, CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP, CUSTOM |
| mag_align_roll |  |  | 0 | -3600 - 3600 |
| mag_align_pitch |  |  | 0 | -3600 - 3600 |
| mag_align_yaw |  |  | 0 | -3600 - 3600 |
| mag_bustype |  |  | I2C | NONE, I2C, SPI, SLAVE, GYROAUTO |
| mag_i2c_device |  |  | 1 | 0 - 3 |
| mag_i2c_address |  |  | 0 | 0 - 119 |
| mag_spi_device |  |  | 0 | 0 - 3 |
| mag_hardware |  |  | NONE | AUTO, NONE, HMC5883, AK8975, AK8963, QMC5883, LIS3MDL |
| mag_declination |  |  | 0 | -18000 - 18000 |
| mag_calibration |  |  | 0,0,0 | Array length: 3 |
| baro_bustype |  |  | I2C | NONE, I2C, SPI, SLAVE, GYROAUTO |
| baro_spi_device |  |  | 0 | 0 - 5 |
| baro_i2c_device |  |  | 1 | 0 - 5 |
| baro_i2c_address |  |  | 0 | 0 - 119 |
| baro_hardware |  |  | AUTO | AUTO, NONE, BMP085, MS5611, BMP280, LPS, QMP6988, BMP388 |
| baro_tab_size |  |  | 21 | 0 - 48 |
| baro_noise_lpf |  |  | 600 | 0 - 1000 |
| baro_cf_vel |  |  | 985 | 0 - 1000 |
| mid_rc | Stick Center | The value (in us) used to determine if a stick is centered. | 1500 | 1200 - 1700 |
| min_check | 'Stick Low' Threshold | TODO: The maximum value (in us) for a stick to be recognised as low. I think this is only used for stick commands, and not a to cap channel values. | 1050 | 750 - 2250 |
| max_check | 'Stick High' Threshold | TODO: The minimum value (in us) for a stick to be recognised as high. | 1900 | 750 - 2250 |
| rssi_channel | RSSI Channel | Receiver channel (AUX channel + 4) that reports RSSI info. | 0 | 0 - 18 |
| rssi_src_frame_errors |  |  | OFF | OFF, ON |
| rssi_scale |  | GUESS: Scales the RSSI value. | 100 | 1 - 255 |
| rssi_offset |  | GUESS: Offsets the RSSI value by a given amount. | 0 | -100 - 100 |
| rssi_invert |  | GUESS: Inverts the RSSI value (high value = poor signal, low value = good signal) | OFF | OFF, ON |
| rssi_src_frame_lpf_period |  |  | 30 | 0 - 255 |
| rc_interp |  |  | AUTO | OFF, PRESET, AUTO, MANUAL |
| rc_interp_ch |  |  | RPYT | RP, RPY, RPYT, T, RPT |
| rc_interp_int |  |  | 19 | 1 - 50 |
| rc_smoothing_type |  |  | FILTER | INTERPOLATION, FILTER |
| rc_smoothing_input_hz |  |  | 0 | 0 - 255 |
| rc_smoothing_derivative_hz |  |  | 0 | 0 - 255 |
| rc_smoothing_debug_axis |  |  | ROLL | ROLL, PITCH, YAW, THROTTLE |
| rc_smoothing_input_type |  |  | BIQUAD | PT1, BIQUAD |
| rc_smoothing_derivative_type |  |  | BIQUAD | OFF, PT1, BIQUAD |
| rc_smoothing_auto_smoothness |  |  | 10 | 0 - 50 |
| fpv_mix_degrees | FPV Camera Angle [degrees] | Camera tilt angle in degrees that should be compensated for when enabling 'FPV Angle Mix' mode. It can be used to fly FPV as if the camera was tilted differently. Also known as uptilt compensation. | 0 | 0 - 90 |
| max_aux_channels |  |  | 14 | 0 - 14 |
| serialrx_provider |  |  | SBUS | SPEK1024, SPEK2048, SBUS, SUMD, SUMH, XB-B, XB-B-RJ01, IBUS, JETIEXBUS, CRSF, SRXL, CUSTOM, FPORT, SRXL2 |
| serialrx_inverted |  |  | OFF | OFF, ON |
| spektrum_sat_bind |  |  | 0 | 0 - 10 |
| spektrum_sat_bind_autoreset |  |  | ON | OFF, ON |
| srxl2_unit_id |  |  | 1 | 0 - 15 |
| srxl2_baud_fast |  |  | ON | OFF, ON |
| sbus_baud_fast |  |  | OFF | OFF, ON |
| airmode_start_throttle_percent |  |  | 25 | 0 - 100 |
| rx_min_usec | Minimum length | Minimum valid pulse length [usec]. Pulses shorter than minimum are invalid and will trigger application of individual channel fallback settings for AUX channels or entering stage 1 for flightchannels. | 885 | 750 - 2250 |
| rx_max_usec | Maximum length | Maximum valid pulse length [usec]. Pulses longer than maximum are invalid and will trigger application of individual channel fallback settings for AUX channels or entering stage 1 for flightchannels. | 2115 | 750 - 2250 |
| serialrx_halfduplex |  |  | OFF | OFF, ON |
| rx_spi_protocol |  |  | V202_250K | V202_250K, V202_1M, SYMA_X, SYMA_X5C, CX10, CX10A, H8_3D, INAV, FRSKY_D, FRSKY_X, FLYSKY, FLYSKY_2A, KN, SFHSS, SPEKTRUM, FRSKY_X_LBT |
| rx_spi_bus |  |  | 0 | 0 - 3 |
| rx_spi_led_inversion |  |  | OFF | OFF, ON |
| adc_device |  |  | 1 | 0 - 3 |
| adc_vrefint_calibration |  |  | 0 | 0 - 2000 |
| adc_tempsensor_calibration30 |  |  | 0 | 0 - 2000 |
| adc_tempsensor_calibration110 |  |  | 0 | 0 - 2000 |
| input_filtering_mode |  |  | OFF | OFF, ON |
| blackbox_p_ratio |  | Describes how many blackbox P-frames (delta) are written for every I-frame (absolute). This can also be defined as the ratio: `I-frame interval` / `P-frame interval`. It can be adjusted in BF Configurator with the 'Blackbox logging rate' option in hz units. | 32 | 0 - 32767 |
| blackbox_device | Blackbox logging device | TODO: Device used for logging blackbox stats. | SDCARD | NONE, SPIFLASH, SDCARD, SERIAL |
| blackbox_record_acc |  | GUESS: Include accelerometer data in the blackbox logs. | ON | OFF, ON |
| blackbox_mode |  | Determines when to enable blackbox logging. E.g. flipping a switch or when testing motors in BF Configurator. | NORMAL | NORMAL, MOTOR_TEST, ALWAYS |
| min_throttle |  |  | 1070 | 750 - 2250 |
| max_throttle |  |  | 2000 | 750 - 2250 |
| min_command |  |  | 1000 | 750 - 2250 |
| dshot_idle_value | Motor Idle Throttle Value [percent] | This is the 'idle' value of throttle that is sent to the ESCs when the craft is armed and the throttle stick is at minimum position. 2000 equals 20 percent. | 550 | 0 - 2000 |
| dshot_burst |  |  | ON | OFF, ON, AUTO |
| dshot_bidir | Bidirectional DShot (requires supported ESC firmware) | Bidirectional DShot. When enabled lets the DSHOT protocol receive information directly from the ESC, needed by the RPM Filter and other features. This requires custom firmware on BLHELI_S or the latest BLHELI_32 firmware. | OFF | OFF, ON |
| dshot_bitbang |  |  | AUTO | OFF, ON, AUTO |
| dshot_bitbang_timer |  |  | AUTO | AUTO, TIM1, TIM8 |
| use_unsynced_pwm |  |  | OFF | OFF, ON |
| motor_pwm_protocol |  |  | DSHOT600 | OFF, ONESHOT125, ONESHOT42, MULTISHOT, BRUSHED, DSHOT150, DSHOT300, DSHOT600, PROSHOT1000 |
| motor_pwm_rate |  |  | 480 | 200 - 32000 |
| motor_pwm_inversion |  |  | OFF | OFF, ON |
| motor_poles | Motor poles (number of magnets on the motor bell) | This setting is used for some features like the RPM Filter.<br><br>Represents the number of magnets that are on the bell of the motor. <b>Do NOT count the stators</b> where the windings are located. Typical 5" motors have 14 magnets, smaller ones like 3" or less usually have 12 magnets. | 14 | 4 - 255 |
| thr_corr_value |  |  | 0 | 0 - 150 |
| thr_corr_angle |  |  | 800 | 1 - 900 |
| failsafe_delay | Guard time for stage 2 activation after signal lost [1 = 0.1 sec.] | Time for stage 1 to wait for recovery. | 4 | 0 - 200 |
| failsafe_off_delay |  |  | 10 | 0 - 200 |
| failsafe_throttle |  |  | 1000 | 750 - 2250 |
| failsafe_switch_mode | Failsafe Switch Action | This option determines what happens when Failsafe is activated through AUX switch: ``Stage 1`` activates Stage 1 failsafe. This is useful if you want to simulate the exact signal loss failsafe behavior. ``Stage 2`` skips Stage 1 and activates the Stage 2 procedure immediately. ``Kill`` disarms instantly (your craft will crash). | STAGE1 | STAGE1, KILL, STAGE2 |
| failsafe_throttle_low_delay | Failsafe Throttle Low Delay [1 = 0.1 sec.] | Just disarm the craft instead of executing the selected failsafe procedure when the throttle was low for this amount of time. | 100 | 0 - 300 |
| failsafe_procedure | Stage 2 - Failsafe Procedure |  | DROP | AUTO-LAND, DROP, GPS-RESCUE |
| failsafe_recovery_delay |  |  | 20 | 0 - 200 |
| failsafe_stick_threshold |  |  | 30 | 0 - 50 |
| align_board_roll | Roll Degrees | Assigns how the flight controller board is aligned on the roll axis. | 0 | -180 - 360 |
| align_board_pitch | Pitch Degrees | Assigns how the flight controller board is aligned on the pitch axis. | 0 | -180 - 360 |
| align_board_yaw | Yaw Degrees | Assigns how the flight controller board is aligned on the yaw axis. | 0 | -180 - 360 |
| gimbal_mode |  |  | NORMAL | NORMAL, MIXTILT |
| bat_capacity |  | GUESS: Capacity of the battery in mAh. Can be used with current meter to detect low battery. Leave at '0' to disable. | 0 | 0 - 20000 |
| vbat_max_cell_voltage | Maximum Cell Voltage | TODO: Maximum voltage of a single cell in the battery. Not sure what this is used for? | 430 | 100 - 500 |
| vbat_full_cell_voltage |  | TODO: Voltage of a single cell in battery to be considered fully charged. I think used for warning if battery is not fully charged. | 410 | 100 - 500 |
| vbat_min_cell_voltage | Minimum Cell Voltage | Minimum voltage allowed for a single cell in the battery. Warnings can be issued with beeper and OSD if this happens. | 330 | 100 - 500 |
| vbat_warning_cell_voltage | Warning Cell Voltage | Voltage of a single cell in the battery that should issue a warning, which can be detected with beeper and OSD. | 350 | 100 - 500 |
| vbat_hysteresis |  |  | 1 | 0 - 250 |
| current_meter |  |  | ADC | NONE, ADC, VIRTUAL, ESC, MSP |
| battery_meter |  |  | ADC | NONE, ADC, ESC |
| vbat_detect_cell_voltage |  |  | 300 | 0 - 2000 |
| use_vbat_alerts |  |  | ON | OFF, ON |
| use_cbat_alerts |  |  | OFF | OFF, ON |
| cbat_alert_percent |  |  | 10 | 0 - 100 |
| vbat_cutoff_percent |  |  | 100 | 0 - 100 |
| force_battery_cell_count |  |  | 0 | 0 - 24 |
| vbat_lpf_period |  |  | 30 | 0 - 255 |
| ibat_lpf_period |  |  | 10 | 0 - 255 |
| vbat_duration_for_warning |  |  | 0 | 0 - 150 |
| vbat_duration_for_critical |  |  | 0 | 0 - 150 |
| vbat_scale |  |  | 110 | 0 - 255 |
| vbat_divider |  |  | 10 | 1 - 255 |
| vbat_multiplier |  |  | 1 | 1 - 255 |
| ibata_scale |  |  | 179 | -16000 - 16000 |
| ibata_offset |  |  | 0 | -32000 - 32000 |
| ibatv_scale |  |  | 0 | -16000 - 16000 |
| ibatv_offset |  |  | 0 | 0 - 16000 |
| beeper_inversion |  |  | ON | OFF, ON |
| beeper_od |  |  | OFF | OFF, ON |
| beeper_frequency |  |  | 0 | 0 - 16000 |
| beeper_dshot_beacon_tone |  | Adjusts the tone of the of the DShot beacon. Higher value equals a higher tone frequency. | 1 | 1 - 5 |
| yaw_motors_reversed | Motor direction is reversed | Configures the mixer to expect the motor direction to be reversed and the propellers to be on accordingly, in order to perform correct yaw movement. Warning: This does not reverse the motor direction. Use the configuration tool for your ESCs or switch the ESC motor wiring order to achieve this. Also known as 'Props out' configuration. | OFF | OFF, ON |
| crashflip_motor_percent |  |  | 0 | 0 - 100 |
| 3d_deadband_low |  |  | 1406 | 750 - 1500 |
| 3d_deadband_high |  |  | 1514 | 1500 - 2250 |
| 3d_neutral |  |  | 1460 | 750 - 2250 |
| 3d_deadband_throttle |  |  | 50 | 1 - 100 |
| 3d_limit_low |  |  | 1000 | 750 - 1500 |
| 3d_limit_high |  |  | 2000 | 1500 - 2250 |
| 3d_switched_mode |  |  | OFF | OFF, ON |
| servo_center_pulse |  |  | 1500 | 750 - 2250 |
| servo_pwm_rate |  |  | 50 | 50 - 498 |
| servo_lowpass_hz |  |  | 0 | 0 - 400 |
| tri_unarmed_servo |  |  | ON | OFF, ON |
| channel_forwarding_start |  |  | 4 | 4 - 18 |
| rateprofile_name |  | Name of the rate profile. Can be seen in OSD using `osd_rate_profile_name_pos`. | - | Unknown |
| thr_mid | Throttle MID | Throttle Mid. The `thr_expo` is centered around this point. Usually this is set around the hovering point. | 50 | 0 - 100 |
| thr_expo | Throttle EXPO | Throttle Expo. Creates an exponential throttle curve around the `thr_mid` point. Used to increase throttle resolution, usually to support more fine-grained hovering. | 0 | 0 - 100 |
| rates_type |  |  | BETAFLIGHT | BETAFLIGHT, RACEFLIGHT, KISS |
| roll_rc_rate | Roll RC Rate |  | 100 | 1 - 255 |
| pitch_rc_rate | Pitch RC Rate |  | 100 | 1 - 255 |
| yaw_rc_rate | Yaw RC Rate |  | 100 | 1 - 255 |
| roll_expo | Roll RC Expo |  | 0 | 0 - 100 |
| pitch_expo | Pitch RC Expo |  | 0 | 0 - 100 |
| yaw_expo | Yaw RC Expo |  | 0 | 0 - 100 |
| roll_srate | Roll Super Rate |  | 70 | 0 - 255 |
| pitch_srate | Pitch Super Rate |  | 70 | 0 - 255 |
| yaw_srate | Yaw Super Rate |  | 70 | 0 - 255 |
| tpa_rate | TPA | Throttle PID Attenuation rate. How much to reduce PID gains when throttle is beyond `tpa_breakpoint`. Used to eliminate fast oscillations at high throttle. | 65 | 0 - 100 |
| tpa_breakpoint | TPA Breakpoint | Throttle PID Attenuation breakpoint. The point at which TPA should take effect. This should be set around the throttle point at which fast oscillations would occur. | 1250 | 750 - 2250 |
| tpa_mode |  | Throttle PID Attenuation mode. Determines which PID gains should be reduced. Used to be both P and D, but by default only D since Betaflight 4.0. | D | PD, D |
| throttle_limit_type | Throttle Limit | Select how `throttle_limit_percent` should limit maximum throttle. ``OFF`` disables the feature. ``SCALE`` will transform the throttle range from 0 to the selected percentage using the full stick travel (linear throttle curve). ``CLIP`` will set a max throttle percentage and stick travel above that will have no additional effect. | OFF | OFF, SCALE, CLIP |
| throttle_limit_percent | Throttle Limit % | Sets the desired maximum throttle percentage, according to `throttle_limit_type`. | 100 | 25 - 100 |
| roll_rate_limit |  | Maximum velocity (deg/s) for roll. Caps a roll rate curve that would otherwise become higher. | 1998 | 200 - 1998 |
| pitch_rate_limit |  | Maximum velocity (deg/s) for pitch. Caps a pitch rate curve that would otherwise become higher. | 1998 | 200 - 1998 |
| yaw_rate_limit |  | Maximum velocity (deg/s) for yaw. Caps a yaw rate curve that would otherwise become higher. | 1998 | 200 - 1998 |
| reboot_character |  |  | 82 | 48 - 126 |
| serial_update_rate_hz |  |  | 100 | 100 - 2000 |
| imu_dcm_kp |  |  | 2500 | 0 - 32000 |
| imu_dcm_ki |  |  | 0 | 0 - 32000 |
| small_angle | Maximum ARM Angle [degrees] | Craft will not ARM if tilted more than specified number of degrees. Only applies if accelerometer is enabled. Setting to 180 will effectivly disable check. | 25 | 0 - 180 |
| auto_disarm_delay |  |  | 5 | 0 - 60 |
| gyro_cal_on_first_arm |  |  | OFF | OFF, ON |
| gps_provider |  |  | NMEA | NMEA, UBLOX, MSP |
| gps_sbas_mode |  |  | AUTO | AUTO, EGNOS, WAAS, MSAS, GAGAN |
| gps_auto_config |  |  | ON | OFF, ON |
| gps_auto_baud |  |  | OFF | OFF, ON |
| gps_ublox_use_galileo |  |  | OFF | OFF, ON |
| gps_set_home_point_once |  |  | OFF | OFF, ON |
| gps_use_3d_speed |  |  | OFF | OFF, ON |
| gps_rescue_angle |  |  | 32 | 0 - 200 |
| gps_rescue_initial_alt |  |  | 50 | 20 - 100 |
| gps_rescue_descent_dist |  |  | 200 | 30 - 500 |
| gps_rescue_landing_alt |  |  | 5 | 3 - 10 |
| gps_rescue_landing_dist |  |  | 10 | 5 - 15 |
| gps_rescue_ground_speed |  |  | 2000 | 30 - 3000 |
| gps_rescue_throttle_p |  |  | 150 | 0 - 500 |
| gps_rescue_throttle_i |  |  | 20 | 0 - 500 |
| gps_rescue_throttle_d |  |  | 50 | 0 - 500 |
| gps_rescue_velocity_p |  |  | 80 | 0 - 500 |
| gps_rescue_velocity_i |  |  | 20 | 0 - 500 |
| gps_rescue_velocity_d |  |  | 15 | 0 - 500 |
| gps_rescue_yaw_p |  |  | 40 | 0 - 500 |
| gps_rescue_throttle_min |  |  | 1100 | 1000 - 2000 |
| gps_rescue_throttle_max |  |  | 1600 | 1000 - 2000 |
| gps_rescue_ascend_rate |  |  | 500 | 100 - 2500 |
| gps_rescue_descend_rate |  |  | 150 | 100 - 500 |
| gps_rescue_throttle_hover |  |  | 1280 | 1000 - 2000 |
| gps_rescue_sanity_checks |  |  | RESCUE_SANITY_ON | RESCUE_SANITY_OFF, RESCUE_SANITY_ON, RESCUE_SANITY_FS_ONLY |
| gps_rescue_min_sats |  |  | 8 | 5 - 50 |
| gps_rescue_min_dth |  |  | 100 | 50 - 1000 |
| gps_rescue_allow_arming_without_fix |  |  | OFF | OFF, ON |
| gps_rescue_alt_mode |  |  | MAX_ALT | MAX_ALT, FIXED_ALT, CURRENT_ALT |
| gps_rescue_use_mag |  |  | ON | OFF, ON |
| deadband |  |  | 0 | 0 - 32 |
| yaw_deadband |  |  | 0 | 0 - 100 |
| yaw_control_reversed |  |  | OFF | OFF, ON |
| pid_process_denom |  |  | 4 | 1 - 16 |
| runaway_takeoff_prevention |  |  | ON | OFF, ON |
| runaway_takeoff_deactivate_delay |  |  | 500 | 100 - 1000 |
| runaway_takeoff_deactivate_throttle_percent |  |  | 20 | 0 - 100 |
| profile_name |  |  | - | Unknown |
| dyn_lpf_dterm_min_hz |  |  | 70 | 0 - 1000 |
| dyn_lpf_dterm_max_hz |  |  | 170 | 0 - 1000 |
| dterm_lowpass_type |  |  | PT1 | PT1, BIQUAD |
| dterm_lowpass_hz |  |  | 150 | 0 - 4000 |
| dterm_lowpass2_type |  |  | PT1 | PT1, BIQUAD |
| dterm_lowpass2_hz |  |  | 150 | 0 - 4000 |
| dterm_notch_hz |  |  | 0 | 0 - 4000 |
| dterm_notch_cutoff |  |  | 0 | 0 - 4000 |
| vbat_pid_gain |  |  | OFF | OFF, ON |
| pid_at_min_throttle |  |  | ON | OFF, ON |
| anti_gravity_mode |  |  | SMOOTH | SMOOTH, STEP |
| anti_gravity_threshold |  |  | 250 | 20 - 1000 |
| anti_gravity_gain |  |  | 5000 | 1000 - 30000 |
| feedforward_transition |  |  | 0 | 0 - 100 |
| acc_limit_yaw |  |  | 0 | 0 - 500 |
| acc_limit |  |  | 0 | 0 - 500 |
| crash_dthreshold |  |  | 50 | 10 - 2000 |
| crash_gthreshold |  |  | 400 | 100 - 2000 |
| crash_setpoint_threshold |  |  | 350 | 50 - 2000 |
| crash_time |  |  | 500 | 100 - 5000 |
| crash_delay |  |  | 0 | 0 - 500 |
| crash_recovery_angle |  |  | 10 | 5 - 30 |
| crash_recovery_rate |  |  | 100 | 50 - 255 |
| crash_limit_yaw |  |  | 200 | 0 - 1000 |
| crash_recovery |  |  | OFF | OFF, ON, BEEP, DISARM |
| iterm_rotation |  |  | OFF | OFF, ON |
| iterm_relax |  |  | RP | OFF, RP, RPY, RP_INC, RPY_INC |
| iterm_relax_type |  |  | SETPOINT | GYRO, SETPOINT |
| iterm_relax_cutoff |  |  | 20 | 1 - 100 |
| iterm_windup |  |  | 100 | 30 - 100 |
| iterm_limit |  |  | 400 | 0 - 500 |
| pidsum_limit |  |  | 500 | 100 - 1000 |
| pidsum_limit_yaw |  |  | 400 | 100 - 1000 |
| yaw_lowpass_hz |  |  | 0 | 0 - 500 |
| throttle_boost |  |  | 5 | 0 - 100 |
| throttle_boost_cutoff |  |  | 15 | 5 - 50 |
| acro_trainer_angle_limit | Acro Trainer Angle Limit | Adds an angle limiting mode for pilots who are learning to fly in acro mode. The range valid is 10-80 and must be activated with a switch in the modes tab. | 20 | 10 - 80 |
| acro_trainer_lookahead_ms |  |  | 50 | 10 - 200 |
| acro_trainer_debug_axis |  |  | ROLL | ROLL, PITCH |
| acro_trainer_gain |  |  | 75 | 25 - 255 |
| p_pitch |  |  | 46 | 0 - 200 |
| i_pitch |  |  | 90 | 0 - 200 |
| d_pitch |  |  | 38 | 0 - 200 |
| f_pitch |  |  | 95 | 0 - 2000 |
| p_roll |  |  | 42 | 0 - 200 |
| i_roll |  |  | 85 | 0 - 200 |
| d_roll |  |  | 35 | 0 - 200 |
| f_roll |  |  | 90 | 0 - 2000 |
| p_yaw |  |  | 30 | 0 - 200 |
| i_yaw |  |  | 90 | 0 - 200 |
| d_yaw |  |  | 0 | 0 - 200 |
| f_yaw |  |  | 90 | 0 - 2000 |
| angle_level_strength |  |  | 50 | 0 - 200 |
| horizon_level_strength |  |  | 50 | 0 - 200 |
| horizon_transition |  |  | 75 | 0 - 200 |
| level_limit |  |  | 55 | 10 - 90 |
| horizon_tilt_effect |  |  | 75 | 0 - 250 |
| horizon_tilt_expert_mode |  |  | OFF | OFF, ON |
| abs_control_gain |  |  | 0 | 0 - 20 |
| abs_control_limit |  |  | 90 | 10 - 255 |
| abs_control_error_limit |  |  | 20 | 1 - 45 |
| abs_control_cutoff |  |  | 11 | 1 - 45 |
| use_integrated_yaw | Integrated Yaw | Integrated Yaw is a feature which corrects a fundamental issue with quad control: while the pitch and roll axis are controlled by the thrust differentials the props generate yaw is different. Integrated Yaw fixes this by integrating the output of the yaw pid before applying them to the mixer. This normalizes the way the pids work. You can now tune as any other axis. It requires use of absolute control since no I is needed with Integrated Yaw. | OFF | OFF, ON |
| integrated_yaw_relax |  |  | 200 | 0 - 255 |
| d_min_roll | D Min Roll | Controls the strength of dampening (D-term) in normal forward flight. During a sharp move or during prop wash, the Active D-gain raises to the Derivative gains (`d_pitch`). | 20 | 0 - 100 |
| d_min_pitch | D Min Pitch | Controls the strength of dampening (D-term) in normal forward flight. During a sharp move or during prop wash, the Active D-gain raises to the Derivative gains (`d_pitch`). | 22 | 0 - 100 |
| d_min_yaw |  D Min Yaw | Controls the strength of dampening (D-term) in normal forward flight. During a sharp move or during prop wash, the Active D-gain raises to the Derivative gains (`d_yaw`). | 0 | 0 - 100 |
| d_min_boost_gain | D Min Gain | Adjusts how fast D gets up to its maximum value and is based on gyro to determine sharp moves and propwash events. | 27 | 0 - 100 |
| d_min_advance |  | Makes D go up earlier by using setpoint instead of gyro to determine sharp moves. aka D Min Advance | 20 | 0 - 200 |
| motor_output_limit |  |  | 100 | 1 - 100 |
| auto_profile_cell_count |  |  | 0 | -1 - 8 |
| launch_control_mode |  |  | NORMAL | NORMAL, PITCHONLY, FULL |
| launch_trigger_allow_reset |  |  | ON | OFF, ON |
| launch_trigger_throttle_percent |  |  | 20 | 0 - 90 |
| launch_angle_limit |  |  | 0 | 0 - 80 |
| launch_control_gain |  |  | 40 | 0 - 200 |
| thrust_linear |  |  | 0 | 0 - 100 |
| transient_throttle_limit |  |  | 0 | 0 - 30 |
| ff_interpolate_sp |  |  | AVERAGED | OFF, ON, AVERAGED |
| ff_spike_limit |  |  | 60 | 0 - 255 |
| ff_max_rate_limit |  |  | 100 | 0 - 150 |
| ff_boost |  |  | 15 | 0 - 50 |
| idle_min_rpm |  |  | 0 | 0 - 100 |
| idle_adjustment_speed |  |  | 50 | 25 - 200 |
| idle_p |  |  | 50 | 10 - 200 |
| idle_pid_limit |  |  | 200 | 10 - 255 |
| idle_max_increase |  |  | 150 | 0 - 255 |
| tlm_inverted |  |  | OFF | OFF, ON |
| tlm_halfduplex |  |  | ON | OFF, ON |
| frsky_default_lat |  |  | 0 | -9000 - 9000 |
| frsky_default_long |  |  | 0 | -18000 - 18000 |
| frsky_gps_format |  |  | 0 | 0 - 1 |
| frsky_unit |  |  | IMPERIAL | IMPERIAL, METRIC |
| frsky_vfas_precision |  |  | 0 | 0 - 1 |
| hott_alarm_int |  |  | 5 | 0 - 120 |
| pid_in_tlm |  |  | OFF | OFF, ON |
| report_cell_voltage |  |  | OFF | OFF, ON |
| ibus_sensor |  |  | 1,2,3,0,0,0,0,0,0,0,0,0,0,0,0 | Array length: 15 |
| mavlink_mah_as_heading_divisor |  |  | 0 | 0 - 30000 |
| telemetry_disabled_voltage |  |  | OFF | OFF, ON |
| telemetry_disabled_current |  |  | OFF | OFF, ON |
| telemetry_disabled_fuel |  |  | OFF | OFF, ON |
| telemetry_disabled_mode |  |  | OFF | OFF, ON |
| telemetry_disabled_acc_x |  |  | OFF | OFF, ON |
| telemetry_disabled_acc_y |  |  | OFF | OFF, ON |
| telemetry_disabled_acc_z |  |  | OFF | OFF, ON |
| telemetry_disabled_pitch |  |  | OFF | OFF, ON |
| telemetry_disabled_roll |  |  | OFF | OFF, ON |
| telemetry_disabled_heading |  |  | OFF | OFF, ON |
| telemetry_disabled_altitude |  |  | OFF | OFF, ON |
| telemetry_disabled_vario |  |  | OFF | OFF, ON |
| telemetry_disabled_lat_long |  |  | OFF | OFF, ON |
| telemetry_disabled_ground_speed |  |  | OFF | OFF, ON |
| telemetry_disabled_distance |  |  | OFF | OFF, ON |
| telemetry_disabled_esc_current |  |  | ON | OFF, ON |
| telemetry_disabled_esc_voltage |  |  | ON | OFF, ON |
| telemetry_disabled_esc_rpm |  |  | ON | OFF, ON |
| telemetry_disabled_esc_temperature |  |  | ON | OFF, ON |
| telemetry_disabled_temperature |  |  | OFF | OFF, ON |
| ledstrip_visual_beeper |  |  | OFF | OFF, ON |
| ledstrip_visual_beeper_color |  |  | WHITE | BLACK, WHITE, RED, ORANGE, YELLOW, LIME_GREEN, GREEN, MINT_GREEN, CYAN, LIGHT_BLUE, BLUE, DARK_VIOLET, MAGENTA, DEEP_PINK |
| ledstrip_grb_rgb |  |  | GRB | GRB, RGB |
| ledstrip_profile |  |  | STATUS | RACE, BEACON, STATUS |
| ledstrip_race_color |  |  | ORANGE | BLACK, WHITE, RED, ORANGE, YELLOW, LIME_GREEN, GREEN, MINT_GREEN, CYAN, LIGHT_BLUE, BLUE, DARK_VIOLET, MAGENTA, DEEP_PINK |
| ledstrip_beacon_color |  |  | WHITE | BLACK, WHITE, RED, ORANGE, YELLOW, LIME_GREEN, GREEN, MINT_GREEN, CYAN, LIGHT_BLUE, BLUE, DARK_VIOLET, MAGENTA, DEEP_PINK |
| ledstrip_beacon_period_ms |  |  | 500 | 50 - 10000 |
| ledstrip_beacon_percent |  |  | 50 | 0 - 100 |
| ledstrip_beacon_armed_only |  |  | OFF | OFF, ON |
| sdcard_detect_inverted |  |  | OFF | OFF, ON |
| sdcard_mode |  |  | SPI | OFF, SPI, SDIO |
| sdcard_dma |  |  | OFF | OFF, ON |
| sdcard_spi_bus |  |  | 3 | 0 - 3 |
| osd_units |  |  | METRIC | IMPERIAL, METRIC |
| osd_warn_arming_disable |  |  | ON | OFF, ON |
| osd_warn_batt_not_full |  |  | ON | OFF, ON |
| osd_warn_batt_warning |  |  | ON | OFF, ON |
| osd_warn_batt_critical |  |  | ON | OFF, ON |
| osd_warn_visual_beeper |  |  | ON | OFF, ON |
| osd_warn_crash_flip |  |  | ON | OFF, ON |
| osd_warn_esc_fail |  |  | ON | OFF, ON |
| osd_warn_core_temp |  |  | ON | OFF, ON |
| osd_warn_rc_smoothing |  |  | ON | OFF, ON |
| osd_warn_fail_safe |  |  | ON | OFF, ON |
| osd_warn_launch_control |  |  | ON | OFF, ON |
| osd_warn_no_gps_rescue |  |  | ON | OFF, ON |
| osd_warn_gps_rescue_disabled |  |  | ON | OFF, ON |
| osd_warn_rssi |  |  | OFF | OFF, ON |
| osd_warn_link_quality |  |  | OFF | OFF, ON |
| osd_rssi_alarm |  |  | 20 | 0 - 100 |
| osd_link_quality_alarm |  |  | 80 | 0 - 300 |
| osd_rssi_dbm_alarm |  |  | 60 | 0 - 130 |
| osd_cap_alarm |  |  | 2200 | 0 - 20000 |
| osd_alt_alarm |  |  | 100 | 0 - 10000 |
| osd_esc_temp_alarm |  |  | -128 | -128 - 127 |
| osd_esc_rpm_alarm |  |  | -1 | -1 - 32767 |
| osd_esc_current_alarm |  |  | -1 | -1 - 32767 |
| osd_core_temp_alarm |  |  | 70 | 0 - 255 |
| osd_ah_max_pit |  |  | 20 | 0 - 90 |
| osd_ah_max_rol |  |  | 40 | 0 - 90 |
| osd_ah_invert |  |  | OFF | OFF, ON |
| osd_tim1 |  |  | 2560 | 0 - 32767 |
| osd_tim2 |  |  | 2561 | 0 - 32767 |
| osd_vbat_pos |  |  | 234 | 0 - 15359 |
| osd_rssi_pos |  |  | 234 | 0 - 15359 |
| osd_link_quality_pos |  |  | 234 | 0 - 15359 |
| osd_rssi_dbm_pos |  |  | 234 | 0 - 15359 |
| osd_tim_1_pos |  |  | 234 | 0 - 15359 |
| osd_tim_2_pos |  |  | 234 | 0 - 15359 |
| osd_remaining_time_estimate_pos |  |  | 234 | 0 - 15359 |
| osd_flymode_pos |  |  | 234 | 0 - 15359 |
| osd_anti_gravity_pos |  |  | 234 | 0 - 15359 |
| osd_g_force_pos |  |  | 234 | 0 - 15359 |
| osd_throttle_pos |  |  | 234 | 0 - 15359 |
| osd_vtx_channel_pos |  |  | 234 | 0 - 15359 |
| osd_crosshairs_pos |  |  | 205 | 0 - 15359 |
| osd_ah_sbar_pos |  |  | 206 | 0 - 15359 |
| osd_ah_pos |  |  | 78 | 0 - 15359 |
| osd_current_pos |  |  | 234 | 0 - 15359 |
| osd_mah_drawn_pos |  |  | 234 | 0 - 15359 |
| osd_motor_diag_pos |  |  | 234 | 0 - 15359 |
| osd_craft_name_pos |  |  | 234 | 0 - 15359 |
| osd_display_name_pos |  |  | 234 | 0 - 15359 |
| osd_gps_speed_pos |  |  | 234 | 0 - 15359 |
| osd_gps_lon_pos |  |  | 234 | 0 - 15359 |
| osd_gps_lat_pos |  |  | 234 | 0 - 15359 |
| osd_gps_sats_pos |  |  | 234 | 0 - 15359 |
| osd_home_dir_pos |  |  | 234 | 0 - 15359 |
| osd_home_dist_pos |  |  | 234 | 0 - 15359 |
| osd_flight_dist_pos |  |  | 234 | 0 - 15359 |
| osd_compass_bar_pos |  |  | 234 | 0 - 15359 |
| osd_altitude_pos |  |  | 234 | 0 - 15359 |
| osd_pid_roll_pos |  |  | 234 | 0 - 15359 |
| osd_pid_pitch_pos |  |  | 234 | 0 - 15359 |
| osd_pid_yaw_pos |  |  | 234 | 0 - 15359 |
| osd_debug_pos |  |  | 234 | 0 - 15359 |
| osd_power_pos |  |  | 234 | 0 - 15359 |
| osd_pidrate_profile_pos |  |  | 234 | 0 - 15359 |
| osd_warnings_pos |  |  | 14665 | 0 - 15359 |
| osd_avg_cell_voltage_pos |  |  | 234 | 0 - 15359 |
| osd_pit_ang_pos |  |  | 234 | 0 - 15359 |
| osd_rol_ang_pos |  |  | 234 | 0 - 15359 |
| osd_battery_usage_pos |  |  | 234 | 0 - 15359 |
| osd_disarmed_pos |  |  | 234 | 0 - 15359 |
| osd_nheading_pos |  |  | 234 | 0 - 15359 |
| osd_nvario_pos |  |  | 234 | 0 - 15359 |
| osd_esc_tmp_pos |  |  | 234 | 0 - 15359 |
| osd_esc_rpm_pos |  |  | 234 | 0 - 15359 |
| osd_esc_rpm_freq_pos |  |  | 234 | 0 - 15359 |
| osd_rtc_date_time_pos |  |  | 234 | 0 - 15359 |
| osd_adjustment_range_pos |  |  | 234 | 0 - 15359 |
| osd_flip_arrow_pos |  |  | 234 | 0 - 15359 |
| osd_core_temp_pos |  |  | 234 | 0 - 15359 |
| osd_log_status_pos |  |  | 234 | 0 - 15359 |
| osd_stick_overlay_left_pos |  |  | 234 | 0 - 15359 |
| osd_stick_overlay_right_pos |  |  | 234 | 0 - 15359 |
| osd_stick_overlay_radio_mode |  |  | 2 | 1 - 4 |
| osd_rate_profile_name_pos |  |  | 234 | 0 - 15359 |
| osd_pid_profile_name_pos |  |  | 234 | 0 - 15359 |
| osd_profile_name_pos |  |  | 234 | 0 - 15359 |
| osd_stat_rtc_date_time |  |  | OFF | OFF, ON |
| osd_stat_tim_1 |  |  | OFF | OFF, ON |
| osd_stat_tim_2 |  |  | ON | OFF, ON |
| osd_stat_max_spd |  |  | ON | OFF, ON |
| osd_stat_max_dist |  |  | OFF | OFF, ON |
| osd_stat_min_batt |  |  | ON | OFF, ON |
| osd_stat_endbatt |  |  | OFF | OFF, ON |
| osd_stat_battery |  |  | OFF | OFF, ON |
| osd_stat_min_rssi |  |  | ON | OFF, ON |
| osd_stat_max_curr |  |  | ON | OFF, ON |
| osd_stat_used_mah |  |  | ON | OFF, ON |
| osd_stat_max_alt |  |  | OFF | OFF, ON |
| osd_stat_bbox |  |  | ON | OFF, ON |
| osd_stat_bb_no |  |  | ON | OFF, ON |
| osd_stat_max_g_force |  |  | OFF | OFF, ON |
| osd_stat_max_esc_temp |  |  | OFF | OFF, ON |
| osd_stat_max_esc_rpm |  |  | OFF | OFF, ON |
| osd_stat_min_link_quality |  |  | OFF | OFF, ON |
| osd_stat_flight_dist |  |  | OFF | OFF, ON |
| osd_stat_max_fft |  |  | OFF | OFF, ON |
| osd_stat_total_flights |  |  | OFF | OFF, ON |
| osd_stat_total_time |  |  | OFF | OFF, ON |
| osd_stat_total_dist |  |  | OFF | OFF, ON |
| osd_stat_min_rssi_dbm |  |  | OFF | OFF, ON |
| osd_profile |  |  | 1 | 1 - 3 |
| osd_profile_1_name |  |  | - | Unknown |
| osd_profile_2_name |  |  | - | Unknown |
| osd_profile_3_name |  |  | - | Unknown |
| osd_gps_sats_show_hdop |  |  | OFF | OFF, ON |
| system_hse_mhz |  |  | 8 | 0 - 30 |
| task_statistics |  |  | ON | OFF, ON |
| debug_mode |  |  | NONE | NONE, CYCLETIME, BATTERY, GYRO_FILTERED, ACCELEROMETER, PIDLOOP, GYRO_SCALED, RC_INTERPOLATION, ANGLERATE, ESC_SENSOR, SCHEDULER, STACK, ESC_SENSOR_RPM, ESC_SENSOR_TMP, ALTITUDE, FFT, FFT_TIME, FFT_FREQ, RX_FRSKY_SPI, RX_SFHSS_SPI, GYRO_RAW, DUAL_GYRO_RAW, DUAL_GYRO_DIFF, MAX7456_SIGNAL, MAX7456_SPICLOCK, SBUS, FPORT, RANGEFINDER, RANGEFINDER_QUALITY, LIDAR_TF, ADC_INTERNAL, RUNAWAY_TAKEOFF, SDIO, CURRENT_SENSOR, USB, SMARTAUDIO, RTH, ITERM_RELAX, ACRO_TRAINER, RC_SMOOTHING, RX_SIGNAL_LOSS, RC_SMOOTHING_RATE, ANTI_GRAVITY, DYN_LPF, RX_SPEKTRUM_SPI, DSHOT_RPM_TELEMETRY, RPM_FILTER, D_MIN, AC_CORRECTION, AC_ERROR, DUAL_GYRO_SCALED, DSHOT_RPM_ERRORS, CRSF_LINK_STATISTICS_UPLINK, CRSF_LINK_STATISTICS_PWR, CRSF_LINK_STATISTICS_DOWN, BARO, GPS_RESCUE_THROTTLE_PID, DYN_IDLE, FF_LIMIT, FF_INTERPOLATED |
| rate_6pos_switch |  | Allows selection of all 6 rate profiles using a six position switch or pot. Normally, when configuring an adjustment channel for rate profile selection the channel value is divided into 3 ranges instead of 6. | OFF | OFF, ON |
| cpu_overclock |  |  | OFF | OFF, 192MHZ, 216MHZ, 240MHZ |
| pwr_on_arm_grace |  |  | 5 | 0 - 30 |
| scheduler_optimize_rate |  |  | AUTO | OFF, ON, AUTO |
| vtx_band |  |  | 0 | 0 - 8 |
| vtx_channel |  |  | 0 | 0 - 8 |
| vtx_power |  | TODO: The power level (index) to be used by the VTX. Must have SmartAudio set up to work. | 0 | 0 - 7 |
| vtx_low_power_disarm |  |  | OFF | OFF, ON, UNTIL_FIRST_ARM |
| vtx_freq |  | TODO: The frequency to be used by the VTX. Must have SmartAudio set up to work. (todo: how does this affect band and channel) | 0 | 0 - 5999 |
| vtx_pit_mode_freq |  |  | 0 | 0 - 5999 |
| vtx_halfduplex |  |  | ON | OFF, ON |
| vtx_spi_bus |  |  | 0 | 0 - 3 |
| vcd_video_system |  | Sets the analog color system used by the connected FPV camera. | AUTO | AUTO, PAL, NTSC |
| vcd_h_offset |  |  | 0 | -32 - 31 |
| vcd_v_offset |  |  | 0 | -15 - 16 |
| max7456_clock |  |  | DEFAULT | HALF, DEFAULT, FULL |
| max7456_spi_bus |  |  | 2 | 0 - 3 |
| max7456_preinit_opu |  |  | OFF | OFF, ON |
| displayport_msp_col_adjust |  |  | 0 | -6 - 0 |
| displayport_msp_row_adjust |  |  | 0 | -3 - 0 |
| displayport_max7456_col_adjust |  |  | 0 | -6 - 0 |
| displayport_max7456_row_adjust |  |  | 0 | -3 - 0 |
| displayport_max7456_inv |  |  | OFF | OFF, ON |
| displayport_max7456_blk |  |  | 0 | 0 - 3 |
| displayport_max7456_wht |  |  | 2 | 0 - 3 |
| esc_sensor_halfduplex |  |  | OFF | OFF, ON |
| esc_sensor_current_offset |  |  | 0 | 0 - 16000 |
| frsky_spi_autobind |  |  | OFF | OFF, ON |
| frsky_spi_tx_id |  |  | 0,0 | Array length: 2 |
| frsky_spi_offset |  |  | 0 | -127 - 127 |
| frsky_spi_bind_hop_data |  | GUESS: Data used to bind specific transmitter with receiver, using Frsky SPI protocol. This can avoid the need for rebinding, etc. | 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 | Array length: 50 |
| frsky_x_rx_num |  |  | 0 | 0 - 255 |
| frsky_spi_a1_source |  |  | VBAT | VBAT, EXTADC, CONST |
| cc2500_spi_chip_detect |  |  | ON | OFF, ON |
| led_inversion |  |  | 0 | 0 - 7 |
| dashboard_i2c_bus |  |  | 1 | 0 - 3 |
| dashboard_i2c_addr |  |  | 60 | 8 - 119 |
| camera_control_mode |  |  | HARDWARE_PWM | HARDWARE_PWM, SOFTWARE_PWM, DAC |
| camera_control_ref_voltage |  |  | 330 | 200 - 400 |
| camera_control_key_delay |  |  | 180 | 100 - 500 |
| camera_control_internal_resistance |  |  | 470 | 10 - 1000 |
| camera_control_button_resistance |  |  | 450,270,150,68,0 | Array length: 5 |
| camera_control_inverted |  |  | OFF | OFF, ON |
| rangefinder_hardware |  |  | NONE | NONE, HCSR04, TFMINI, TF02 |
| pinio_config |  |  | 1,1,1,1 | Array length: 4 |
| pinio_box |  |  | 255,255,255,255 | Array length: 4 |
| usb_hid_cdc |  |  | OFF | OFF, ON |
| usb_msc_pin_pullup |  |  | ON | OFF, ON |
| flash_spi_bus |  |  | 0 | 0 - 3 |
| rcdevice_init_dev_attempts |  |  | 6 | 0 - 10 |
| rcdevice_init_dev_attempt_interval |  |  | 1000 | 0 - 5000 |
| rcdevice_protocol_version |  |  | 0 | 0 - 1 |
| rcdevice_feature |  |  | 0 | 0 - 65535 |
| gyro_1_bustype |  |  | SPI | NONE, I2C, SPI, SLAVE, GYROAUTO |
| gyro_1_spibus |  |  | 1 | 0 - 3 |
| gyro_1_i2cBus |  |  | 0 | 0 - 3 |
| gyro_1_i2c_address |  |  | 0 | 0 - 119 |
| gyro_1_sensor_align | First GYRO | GUESS: Alignment of the first gyro sensor. Represented as a rotation string in degrees, e.g. 'CW90'. This seems to be closely coupled with the `gyro_1_align_*` variables. | CW180 | DEFAULT, CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP, CUSTOM |
| gyro_1_align_roll |  | Assigns how the gyro sensor roll axis is aligned, independent of board alignment. This is updated by the 'First GYRO' (`gyro_1_sensor_align`) in BF Configurator. | 0 | -3600 - 3600 |
| gyro_1_align_pitch |  | Assigns how the gyro sensor pitch axis is aligned, independent of board alignment. This is updated by the 'First GYRO' (`gyro_1_sensor_align`) in BF Configurator. | 0 | -3600 - 3600 |
| gyro_1_align_yaw |  | Assigns how the gyro sensor yaw axis is aligned, independent of board alignment. This is updated by the 'First GYRO' (`gyro_1_sensor_align`) in BF Configurator. | 0 | -3600 - 3600 |
| gyro_2_bustype |  |  | SPI | NONE, I2C, SPI, SLAVE, GYROAUTO |
| gyro_2_spibus |  |  | 0 | 0 - 3 |
| gyro_2_i2cBus |  |  | 0 | 0 - 3 |
| gyro_2_i2c_address |  |  | 0 | 0 - 119 |
| gyro_2_sensor_align |  | Like `gyro_1_sensor_align`, but for the second gyro sensor. | CW0 | DEFAULT, CW0, CW90, CW180, CW270, CW0FLIP, CW90FLIP, CW180FLIP, CW270FLIP, CUSTOM |
| gyro_2_align_roll |  | Like `gyro_1_align_roll`, but for the second gyro sensor. | 0 | -3600 - 3600 |
| gyro_2_align_pitch |  | Like `gyro_1_align_pitch`, but for the second gyro sensor. | 0 | -3600 - 3600 |
| gyro_2_align_yaw |  | Like `gyro_1_align_yaw`, but for the second gyro sensor. | 0 | -3600 - 3600 |
| i2c1_pullup |  |  | OFF | OFF, ON |
| i2c1_overclock |  |  | ON | OFF, ON |
| i2c2_pullup |  |  | OFF | OFF, ON |
| i2c2_overclock |  |  | ON | OFF, ON |
| i2c3_pullup |  |  | OFF | OFF, ON |
| i2c3_overclock |  |  | ON | OFF, ON |
| mco2_on_pc9 |  |  | OFF | OFF, ON |
| timezone_offset_minutes |  |  | 0 | -780 - 780 |
| gyro_rpm_notch_harmonics |  |  | 3 | 0 - 3 |
| gyro_rpm_notch_q |  |  | 500 | 1 - 3000 |
| gyro_rpm_notch_min |  |  | 100 | 50 - 200 |
| dterm_rpm_notch_harmonics |  |  | 0 | 0 - 3 |
| dterm_rpm_notch_q |  |  | 500 | 1 - 3000 |
| dterm_rpm_notch_min |  |  | 100 | 50 - 200 |
| rpm_notch_lpf |  |  | 150 | 100 - 500 |
| flysky_spi_tx_id |  |  | 0 | 0 - 4294967295 |
| flysky_spi_rf_channels |  |  | 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 | Array length: 16 |
| stats |  |  | OFF | OFF, ON |
| stats_total_flights |  |  | 0 | 0 - 4294967295 |
| stats_total_time_s |  |  | 0 | 0 - 4294967295 |
| stats_total_dist_m |  |  | 0 | 0 - 4294967295 |
| name | Craft Name | Name of the craft. Used in log and config files, and can be displayed in OSD with `osd_craft_name_pos`. | - | Unknown |
| display_name | Display Name | Text that can be displayed in OSD with `osd_display_name_pos`. E.g. name of the pilot. | - | Unknown |
| position_alt_source |  |  | DEFAULT | DEFAULT, BARO_ONLY, GPS_ONLY |
