# Betaflight Deciphered

This is an attempt to document the variables of [Betaflight 4.1](https://github.com/betaflight/betaflight), motivated by my previous trouble to easily look up certain information.

When using the Betaflight command line interface or examining a diff, it can sometimes be difficult to understand what certain settings is supposed to do. This is intended to be used as a reference to quickly look this information up.

As I'm not an expert, any contributions or help with creating this document will be greatly appreciated!
Parts of this document has been based on information from [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator).

If you have trouble understanding any of the explanations, please open an issue so we can rewrite it and hopefully clear things up.

# Betaflight CLI Variables

## `gyro_hardware_lpf`
- Default: `NORMAL`
- Allowed: `NORMAL`, `1KHZ_SAMPLING`, `EXPERIMENTAL`

## `gyro_sync_denom`
- Default: `1`
- Allowed: `1 - 32`

## `gyro_lowpass_type`
- Default: `PT1`
- Allowed: `PT1`, `BIQUAD`

## `gyro_lowpass_hz`
- Default: `200`
- Allowed: `0 - 4000`

## `gyro_lowpass2_type`
- Default: `PT1`
- Allowed: `PT1`, `BIQUAD`

## `gyro_lowpass2_hz`
- Default: `250`
- Allowed: `0 - 4000`

## `gyro_notch1_hz`
- Default: `0`
- Allowed: `0 - 4000`

## `gyro_notch1_cutoff`
- Default: `0`
- Allowed: `0 - 4000`

## `gyro_notch2_hz`
- Default: `0`
- Allowed: `0 - 4000`

## `gyro_notch2_cutoff`
- Default: `0`
- Allowed: `0 - 4000`

## `gyro_calib_duration`
- Default: `125`
- Allowed: `50 - 3000`

## `gyro_calib_noise_limit`
- Default: `48`
- Allowed: `0 - 200`

## `gyro_offset_yaw`
- Default: `0`
- Allowed: `-1000 - 1000`

## `gyro_overflow_detect`
- Default: `ALL`
- Allowed: `OFF`, `YAW`, `ALL`

## `yaw_spin_recovery`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `yaw_spin_threshold`
- Default: `1950`
- Allowed: `500 - 1950`

## `gyro_to_use`
- Default: `FIRST`
- Allowed: `FIRST`, `SECOND`, `BOTH`

TODO: Selects the gyro(s) that should be used.

## `dyn_notch_range`
- Default: `MEDIUM`
- Allowed: `HIGH`, `MEDIUM`, `LOW`, `AUTO`

## `dyn_notch_width_percent`
- Default: `8`
- Allowed: `0 - 20`

## `dyn_notch_q`
- Default: `120`
- Allowed: `1 - 1000`

## `dyn_notch_min_hz`
- Default: `150`
- Allowed: `60 - 1000`

## `dyn_lpf_gyro_min_hz`
- Default: `200`
- Allowed: `0 - 1000`

## `dyn_lpf_gyro_max_hz`
- Default: `500`
- Allowed: `0 - 1000`

## `gyro_filter_debug_axis`
- Default: `ROLL`
- Allowed: `ROLL`, `PITCH`, `YAW`

## `acc_hardware`
- Default: `AUTO`
- Allowed: `AUTO`, `NONE`, `ADXL345`, `MPU6050`, `MMA8452`, `BMA280`, `LSM303DLHC`, `MPU6000`, `MPU6500`, `MPU9250`, `ICM20601`, `ICM20602`, `ICM20608G`, `ICM20649`, `ICM20689`, `BMI160`, `FAKE`

## `acc_lpf_hz`
- Default: `10`
- Allowed: `0 - 400`

## `acc_trim_pitch`
- Default: `0`
- Allowed: `-300 - 300`

## `acc_trim_roll`
- Default: `0`
- Allowed: `-300 - 300`

## `acc_calibration`
- Default: `0,0,0`
- Allowed: `Array length: 3`

TODO: Accelerometer calibration values. Normally set automatically by pressing 'Calibrate Accelerometer' in BF Configurator.

## `align_mag`
- Default: `DEFAULT`
- Allowed: `DEFAULT`, `CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`, `CW90FLIP`, `CW180FLIP`, `CW270FLIP`, `CUSTOM`

## `mag_align_roll`
- Default: `0`
- Allowed: `-3600 - 3600`

## `mag_align_pitch`
- Default: `0`
- Allowed: `-3600 - 3600`

## `mag_align_yaw`
- Default: `0`
- Allowed: `-3600 - 3600`

## `mag_bustype`
- Default: `I2C`
- Allowed: `NONE`, `I2C`, `SPI`, `SLAVE`, `GYROAUTO`

## `mag_i2c_device`
- Default: `1`
- Allowed: `0 - 3`

## `mag_i2c_address`
- Default: `0`
- Allowed: `0 - 119`

## `mag_spi_device`
- Default: `0`
- Allowed: `0 - 3`

## `mag_hardware`
- Default: `NONE`
- Allowed: `AUTO`, `NONE`, `HMC5883`, `AK8975`, `AK8963`, `QMC5883`, `LIS3MDL`

## `mag_declination`
- Default: `0`
- Allowed: `-18000 - 18000`

## `mag_calibration`
- Default: `0,0,0`
- Allowed: `Array length: 3`

## `baro_bustype`
- Default: `I2C`
- Allowed: `NONE`, `I2C`, `SPI`, `SLAVE`, `GYROAUTO`

## `baro_spi_device`
- Default: `0`
- Allowed: `0 - 5`

## `baro_i2c_device`
- Default: `1`
- Allowed: `0 - 5`

## `baro_i2c_address`
- Default: `0`
- Allowed: `0 - 119`

## `baro_hardware`
- Default: `AUTO`
- Allowed: `AUTO`, `NONE`, `BMP085`, `MS5611`, `BMP280`, `LPS`, `QMP6988`, `BMP388`

## `baro_tab_size`
- Default: `21`
- Allowed: `0 - 48`

## `baro_noise_lpf`
- Default: `600`
- Allowed: `0 - 1000`

## `baro_cf_vel`
- Default: `985`
- Allowed: `0 - 1000`

## `mid_rc`
- Default: `1500`
- Allowed: `1200 - 1700`
- BF Configurator: *Stick Center*

The value (in us) used to determine if a stick is centered.

## `min_check`
- Default: `1050`
- Allowed: `750 - 2250`
- BF Configurator: *'Stick Low' Threshold*

TODO: The maximum value (in us) for a stick to be recognised as low. I think this is only used for stick commands, and not a to cap channel values.

## `max_check`
- Default: `1900`
- Allowed: `750 - 2250`
- BF Configurator: *'Stick High' Threshold*

TODO: The minimum value (in us) for a stick to be recognised as high.

## `rssi_channel`
- Default: `0`
- Allowed: `0 - 18`
- BF Configurator: *RSSI Channel*

Receiver channel (AUX channel + 4) that reports RSSI info.

## `rssi_src_frame_errors`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `rssi_scale`
- Default: `100`
- Allowed: `1 - 255`

GUESS: Scales the RSSI value.

## `rssi_offset`
- Default: `0`
- Allowed: `-100 - 100`

GUESS: Offsets the RSSI value by a given amount.

## `rssi_invert`
- Default: `OFF`
- Allowed: `OFF`, `ON`

GUESS: Inverts the RSSI value (high value = poor signal, low value = good signal)

## `rssi_src_frame_lpf_period`
- Default: `30`
- Allowed: `0 - 255`

## `rc_interp`
- Default: `AUTO`
- Allowed: `OFF`, `PRESET`, `AUTO`, `MANUAL`

## `rc_interp_ch`
- Default: `RPYT`
- Allowed: `RP`, `RPY`, `RPYT`, `T`, `RPT`

## `rc_interp_int`
- Default: `19`
- Allowed: `1 - 50`

## `rc_smoothing_type`
- Default: `FILTER`
- Allowed: `INTERPOLATION`, `FILTER`

## `rc_smoothing_input_hz`
- Default: `0`
- Allowed: `0 - 255`

## `rc_smoothing_derivative_hz`
- Default: `0`
- Allowed: `0 - 255`

## `rc_smoothing_debug_axis`
- Default: `ROLL`
- Allowed: `ROLL`, `PITCH`, `YAW`, `THROTTLE`

## `rc_smoothing_input_type`
- Default: `BIQUAD`
- Allowed: `PT1`, `BIQUAD`

## `rc_smoothing_derivative_type`
- Default: `BIQUAD`
- Allowed: `OFF`, `PT1`, `BIQUAD`

## `rc_smoothing_auto_smoothness`
- Default: `10`
- Allowed: `0 - 50`

## `fpv_mix_degrees`
- Default: `0`
- Allowed: `0 - 90`
- BF Configurator: *FPV Camera Angle [degrees]*

Camera tilt angle in degrees that should be compensated for when enabling 'FPV Angle Mix' mode. It can be used to fly FPV as if the camera was tilted differently. Also known as uptilt compensation.

## `max_aux_channels`
- Default: `14`
- Allowed: `0 - 14`

## `serialrx_provider`
- Default: `SBUS`
- Allowed: `SPEK1024`, `SPEK2048`, `SBUS`, `SUMD`, `SUMH`, `XB-B`, `XB-B-RJ01`, `IBUS`, `JETIEXBUS`, `CRSF`, `SRXL`, `CUSTOM`, `FPORT`, `SRXL2`

## `serialrx_inverted`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `spektrum_sat_bind`
- Default: `0`
- Allowed: `0 - 10`

## `spektrum_sat_bind_autoreset`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `srxl2_unit_id`
- Default: `1`
- Allowed: `0 - 15`

## `srxl2_baud_fast`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `sbus_baud_fast`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `airmode_start_throttle_percent`
- Default: `25`
- Allowed: `0 - 100`

## `rx_min_usec`
- Default: `885`
- Allowed: `750 - 2250`
- BF Configurator: *Minimum length*

Minimum valid pulse length [usec]. Pulses shorter than minimum are invalid and will trigger application of individual channel fallback settings for AUX channels or entering stage 1 for flightchannels.

## `rx_max_usec`
- Default: `2115`
- Allowed: `750 - 2250`
- BF Configurator: *Maximum length*

Maximum valid pulse length [usec]. Pulses longer than maximum are invalid and will trigger application of individual channel fallback settings for AUX channels or entering stage 1 for flightchannels.

## `serialrx_halfduplex`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `rx_spi_protocol`
- Default: `V202_250K`
- Allowed: `V202_250K`, `V202_1M`, `SYMA_X`, `SYMA_X5C`, `CX10`, `CX10A`, `H8_3D`, `INAV`, `FRSKY_D`, `FRSKY_X`, `FLYSKY`, `FLYSKY_2A`, `KN`, `SFHSS`, `SPEKTRUM`, `FRSKY_X_LBT`

## `rx_spi_bus`
- Default: `0`
- Allowed: `0 - 3`

## `rx_spi_led_inversion`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `adc_device`
- Default: `1`
- Allowed: `0 - 3`

## `adc_vrefint_calibration`
- Default: `0`
- Allowed: `0 - 2000`

## `adc_tempsensor_calibration30`
- Default: `0`
- Allowed: `0 - 2000`

## `adc_tempsensor_calibration110`
- Default: `0`
- Allowed: `0 - 2000`

## `input_filtering_mode`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `blackbox_p_ratio`
- Default: `32`
- Allowed: `0 - 32767`

Describes how many blackbox P-frames (delta) are written for every I-frame (absolute). This can also be defined as the ratio: `I-frame interval` / `P-frame interval`. It can be adjusted in BF Configurator with the 'Blackbox logging rate' option in hz units.

## `blackbox_device`
- Default: `SDCARD`
- Allowed: `NONE`, `SPIFLASH`, `SDCARD`, `SERIAL`
- BF Configurator: *Blackbox logging device*

TODO: Device used for logging blackbox stats.

## `blackbox_record_acc`
- Default: `ON`
- Allowed: `OFF`, `ON`

GUESS: Include accelerometer data in the blackbox logs.

## `blackbox_mode`
- Default: `NORMAL`
- Allowed: `NORMAL`, `MOTOR_TEST`, `ALWAYS`

Determines when to enable blackbox logging. E.g. flipping a switch or when testing motors in BF Configurator.

## `min_throttle`
- Default: `1070`
- Allowed: `750 - 2250`

## `max_throttle`
- Default: `2000`
- Allowed: `750 - 2250`

## `min_command`
- Default: `1000`
- Allowed: `750 - 2250`

## `dshot_idle_value`
- Default: `550`
- Allowed: `0 - 2000`
- BF Configurator: *Motor Idle Throttle Value [percent]*

This is the 'idle' value of throttle that is sent to the ESCs when the craft is armed and the throttle stick is at minimum position. 2000 equals 20 percent.

## `dshot_burst`
- Default: `ON`
- Allowed: `OFF`, `ON`, `AUTO`

## `dshot_bidir`
- Default: `OFF`
- Allowed: `OFF`, `ON`
- BF Configurator: *Bidirectional DShot (requires supported ESC firmware)*

Bidirectional DShot. When enabled lets the DSHOT protocol receive information directly from the ESC, needed by the RPM Filter and other features. This requires custom firmware on BLHELI_S or the latest BLHELI_32 firmware.

## `dshot_bitbang`
- Default: `AUTO`
- Allowed: `OFF`, `ON`, `AUTO`

## `dshot_bitbang_timer`
- Default: `AUTO`
- Allowed: `AUTO`, `TIM1`, `TIM8`

## `use_unsynced_pwm`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `motor_pwm_protocol`
- Default: `DSHOT600`
- Allowed: `OFF`, `ONESHOT125`, `ONESHOT42`, `MULTISHOT`, `BRUSHED`, `DSHOT150`, `DSHOT300`, `DSHOT600`, `PROSHOT1000`

## `motor_pwm_rate`
- Default: `480`
- Allowed: `200 - 32000`

## `motor_pwm_inversion`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `motor_poles`
- Default: `14`
- Allowed: `4 - 255`
- BF Configurator: *Motor poles (number of magnets on the motor bell)*

This setting is used for some features like the RPM Filter.<br><br>Represents the number of magnets that are on the bell of the motor. <b>Do NOT count the stators</b> where the windings are located. Typical 5" motors have 14 magnets, smaller ones like 3" or less usually have 12 magnets.

## `thr_corr_value`
- Default: `0`
- Allowed: `0 - 150`

## `thr_corr_angle`
- Default: `800`
- Allowed: `1 - 900`

## `failsafe_delay`
- Default: `4`
- Allowed: `0 - 200`
- BF Configurator: *Guard time for stage 2 activation after signal lost [1 = 0.1 sec.]*

Time for stage 1 to wait for recovery.

## `failsafe_off_delay`
- Default: `10`
- Allowed: `0 - 200`

## `failsafe_throttle`
- Default: `1000`
- Allowed: `750 - 2250`

## `failsafe_switch_mode`
- Default: `STAGE1`
- Allowed: `STAGE1`, `KILL`, `STAGE2`
- BF Configurator: *Failsafe Switch Action*

This option determines what happens when Failsafe is activated through AUX switch: `Stage 1` activates Stage 1 failsafe. This is useful if you want to simulate the exact signal loss failsafe behavior. `Stage 2` skips Stage 1 and activates the Stage 2 procedure immediately. `Kill` disarms instantly (your craft will crash).

## `failsafe_throttle_low_delay`
- Default: `100`
- Allowed: `0 - 300`
- BF Configurator: *Failsafe Throttle Low Delay [1 = 0.1 sec.]*

Just disarm the craft instead of executing the selected failsafe procedure when the throttle was low for this amount of time.

## `failsafe_procedure`
- Default: `DROP`
- Allowed: `AUTO-LAND`, `DROP`, `GPS-RESCUE`
- BF Configurator: *Stage 2 - Failsafe Procedure*

## `failsafe_recovery_delay`
- Default: `20`
- Allowed: `0 - 200`

## `failsafe_stick_threshold`
- Default: `30`
- Allowed: `0 - 50`

## `align_board_roll`
- Default: `0`
- Allowed: `-180 - 360`
- BF Configurator: *Roll Degrees*

Assigns how the flight controller board is aligned on the roll axis.

## `align_board_pitch`
- Default: `0`
- Allowed: `-180 - 360`
- BF Configurator: *Pitch Degrees*

Assigns how the flight controller board is aligned on the pitch axis.

## `align_board_yaw`
- Default: `0`
- Allowed: `-180 - 360`
- BF Configurator: *Yaw Degrees*

Assigns how the flight controller board is aligned on the yaw axis.

## `gimbal_mode`
- Default: `NORMAL`
- Allowed: `NORMAL`, `MIXTILT`

## `bat_capacity`
- Default: `0`
- Allowed: `0 - 20000`

GUESS: Capacity of the battery in mAh. Can be used with current meter to detect low battery. Leave at '0' to disable.

## `vbat_max_cell_voltage`
- Default: `430`
- Allowed: `100 - 500`
- BF Configurator: *Maximum Cell Voltage*

TODO: Maximum voltage of a single cell in the battery. Not sure what this is used for?

## `vbat_full_cell_voltage`
- Default: `410`
- Allowed: `100 - 500`

TODO: Voltage of a single cell in battery to be considered fully charged. I think used for warning if battery is not fully charged.

## `vbat_min_cell_voltage`
- Default: `330`
- Allowed: `100 - 500`
- BF Configurator: *Minimum Cell Voltage*

Minimum voltage allowed for a single cell in the battery. Warnings can be issued with beeper and OSD if this happens.

## `vbat_warning_cell_voltage`
- Default: `350`
- Allowed: `100 - 500`
- BF Configurator: *Warning Cell Voltage*

Voltage of a single cell in the battery that should issue a warning, which can be detected with beeper and OSD.

## `vbat_hysteresis`
- Default: `1`
- Allowed: `0 - 250`

## `current_meter`
- Default: `ADC`
- Allowed: `NONE`, `ADC`, `VIRTUAL`, `ESC`, `MSP`

## `battery_meter`
- Default: `ADC`
- Allowed: `NONE`, `ADC`, `ESC`

## `vbat_detect_cell_voltage`
- Default: `300`
- Allowed: `0 - 2000`

## `use_vbat_alerts`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `use_cbat_alerts`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `cbat_alert_percent`
- Default: `10`
- Allowed: `0 - 100`

## `vbat_cutoff_percent`
- Default: `100`
- Allowed: `0 - 100`

## `force_battery_cell_count`
- Default: `0`
- Allowed: `0 - 24`

## `vbat_lpf_period`
- Default: `30`
- Allowed: `0 - 255`

## `ibat_lpf_period`
- Default: `10`
- Allowed: `0 - 255`

## `vbat_duration_for_warning`
- Default: `0`
- Allowed: `0 - 150`

## `vbat_duration_for_critical`
- Default: `0`
- Allowed: `0 - 150`

## `vbat_scale`
- Default: `110`
- Allowed: `0 - 255`

## `vbat_divider`
- Default: `10`
- Allowed: `1 - 255`

## `vbat_multiplier`
- Default: `1`
- Allowed: `1 - 255`

## `ibata_scale`
- Default: `179`
- Allowed: `-16000 - 16000`

## `ibata_offset`
- Default: `0`
- Allowed: `-32000 - 32000`

## `ibatv_scale`
- Default: `0`
- Allowed: `-16000 - 16000`

## `ibatv_offset`
- Default: `0`
- Allowed: `0 - 16000`

## `beeper_inversion`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `beeper_od`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `beeper_frequency`
- Default: `0`
- Allowed: `0 - 16000`

## `beeper_dshot_beacon_tone`
- Default: `1`
- Allowed: `1 - 5`

Adjusts the tone of the of the DShot beacon. Higher value equals a higher tone frequency.

## `yaw_motors_reversed`
- Default: `OFF`
- Allowed: `OFF`, `ON`
- BF Configurator: *Motor direction is reversed*

Configures the mixer to expect the motor direction to be reversed and the propellers to be on accordingly, in order to perform correct yaw movement. Warning: This does not reverse the motor direction. Use the configuration tool for your ESCs or switch the ESC motor wiring order to achieve this. Also known as 'Props out' configuration.

## `crashflip_motor_percent`
- Default: `0`
- Allowed: `0 - 100`

## `3d_deadband_low`
- Default: `1406`
- Allowed: `750 - 1500`

## `3d_deadband_high`
- Default: `1514`
- Allowed: `1500 - 2250`

## `3d_neutral`
- Default: `1460`
- Allowed: `750 - 2250`

## `3d_deadband_throttle`
- Default: `50`
- Allowed: `1 - 100`

## `3d_limit_low`
- Default: `1000`
- Allowed: `750 - 1500`

## `3d_limit_high`
- Default: `2000`
- Allowed: `1500 - 2250`

## `3d_switched_mode`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `servo_center_pulse`
- Default: `1500`
- Allowed: `750 - 2250`

## `servo_pwm_rate`
- Default: `50`
- Allowed: `50 - 498`

## `servo_lowpass_hz`
- Default: `0`
- Allowed: `0 - 400`

## `tri_unarmed_servo`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `channel_forwarding_start`
- Default: `4`
- Allowed: `4 - 18`

## `rateprofile_name`

Name of the rate profile. Can be seen in OSD using [osd_rate_profile_name_pos](#osd_rate_profile_name_pos).

## `thr_mid`
- Default: `50`
- Allowed: `0 - 100`
- BF Configurator: *Throttle MID*

Throttle Mid. The [thr_expo](#thr_expo) is centered around this point. Usually this is set around the hovering point.

## `thr_expo`
- Default: `0`
- Allowed: `0 - 100`
- BF Configurator: *Throttle EXPO*

Throttle Expo. Creates an exponential throttle curve around the [thr_mid](#thr_mid) point. Used to increase throttle resolution, usually to support more fine-grained hovering.

## `rates_type`
- Default: `BETAFLIGHT`
- Allowed: `BETAFLIGHT`, `RACEFLIGHT`, `KISS`

## `roll_rc_rate`
- Default: `100`
- Allowed: `1 - 255`
- BF Configurator: *Roll RC Rate*

## `pitch_rc_rate`
- Default: `100`
- Allowed: `1 - 255`
- BF Configurator: *Pitch RC Rate*

## `yaw_rc_rate`
- Default: `100`
- Allowed: `1 - 255`
- BF Configurator: *Yaw RC Rate*

## `roll_expo`
- Default: `0`
- Allowed: `0 - 100`
- BF Configurator: *Roll RC Expo*

## `pitch_expo`
- Default: `0`
- Allowed: `0 - 100`
- BF Configurator: *Pitch RC Expo*

## `yaw_expo`
- Default: `0`
- Allowed: `0 - 100`
- BF Configurator: *Yaw RC Expo*

## `roll_srate`
- Default: `70`
- Allowed: `0 - 255`
- BF Configurator: *Roll Super Rate*

## `pitch_srate`
- Default: `70`
- Allowed: `0 - 255`
- BF Configurator: *Pitch Super Rate*

## `yaw_srate`
- Default: `70`
- Allowed: `0 - 255`
- BF Configurator: *Yaw Super Rate*

## `tpa_rate`
- Default: `65`
- Allowed: `0 - 100`
- BF Configurator: *TPA*

Throttle PID Attenuation rate. How much to reduce PID gains when throttle is beyond [tpa_breakpoint](#tpa_breakpoint). Used to eliminate fast oscillations at high throttle.

## `tpa_breakpoint`
- Default: `1250`
- Allowed: `750 - 2250`
- BF Configurator: *TPA Breakpoint*

Throttle PID Attenuation breakpoint. The point at which TPA should take effect. This should be set around the throttle point at which fast oscillations would occur.

## `tpa_mode`
- Default: `D`
- Allowed: `PD`, `D`

Throttle PID Attenuation mode. Determines which PID gains should be reduced. Used to be both P and D, but by default only D since Betaflight 4.0.

## `throttle_limit_type`
- Default: `OFF`
- Allowed: `OFF`, `SCALE`, `CLIP`
- BF Configurator: *Throttle Limit*

Select how [throttle_limit_percent](#throttle_limit_percent) should limit maximum throttle. `OFF` disables the feature. `SCALE` will transform the throttle range from 0 to the selected percentage using the full stick travel (linear throttle curve). `CLIP` will set a max throttle percentage and stick travel above that will have no additional effect.

## `throttle_limit_percent`
- Default: `100`
- Allowed: `25 - 100`
- BF Configurator: *Throttle Limit %*

Sets the desired maximum throttle percentage, according to [throttle_limit_type](#throttle_limit_type).

## `roll_rate_limit`
- Default: `1998`
- Allowed: `200 - 1998`

Maximum velocity (deg/s) for roll. Caps a roll rate curve that would otherwise become higher.

## `pitch_rate_limit`
- Default: `1998`
- Allowed: `200 - 1998`

Maximum velocity (deg/s) for pitch. Caps a pitch rate curve that would otherwise become higher.

## `yaw_rate_limit`
- Default: `1998`
- Allowed: `200 - 1998`

Maximum velocity (deg/s) for yaw. Caps a yaw rate curve that would otherwise become higher.

## `reboot_character`
- Default: `82`
- Allowed: `48 - 126`

## `serial_update_rate_hz`
- Default: `100`
- Allowed: `100 - 2000`

## `imu_dcm_kp`
- Default: `2500`
- Allowed: `0 - 32000`

## `imu_dcm_ki`
- Default: `0`
- Allowed: `0 - 32000`

## `small_angle`
- Default: `25`
- Allowed: `0 - 180`
- BF Configurator: *Maximum ARM Angle [degrees]*

Craft will not ARM if tilted more than specified number of degrees. Only applies if accelerometer is enabled. Setting to 180 will effectivly disable check.

## `auto_disarm_delay`
- Default: `5`
- Allowed: `0 - 60`

## `gyro_cal_on_first_arm`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `gps_provider`
- Default: `NMEA`
- Allowed: `NMEA`, `UBLOX`, `MSP`

## `gps_sbas_mode`
- Default: `AUTO`
- Allowed: `AUTO`, `EGNOS`, `WAAS`, `MSAS`, `GAGAN`

## `gps_auto_config`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `gps_auto_baud`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `gps_ublox_use_galileo`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `gps_set_home_point_once`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `gps_use_3d_speed`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `gps_rescue_angle`
- Default: `32`
- Allowed: `0 - 200`

## `gps_rescue_initial_alt`
- Default: `50`
- Allowed: `20 - 100`

## `gps_rescue_descent_dist`
- Default: `200`
- Allowed: `30 - 500`

## `gps_rescue_landing_alt`
- Default: `5`
- Allowed: `3 - 10`

## `gps_rescue_landing_dist`
- Default: `10`
- Allowed: `5 - 15`

## `gps_rescue_ground_speed`
- Default: `2000`
- Allowed: `30 - 3000`

## `gps_rescue_throttle_p`
- Default: `150`
- Allowed: `0 - 500`

## `gps_rescue_throttle_i`
- Default: `20`
- Allowed: `0 - 500`

## `gps_rescue_throttle_d`
- Default: `50`
- Allowed: `0 - 500`

## `gps_rescue_velocity_p`
- Default: `80`
- Allowed: `0 - 500`

## `gps_rescue_velocity_i`
- Default: `20`
- Allowed: `0 - 500`

## `gps_rescue_velocity_d`
- Default: `15`
- Allowed: `0 - 500`

## `gps_rescue_yaw_p`
- Default: `40`
- Allowed: `0 - 500`

## `gps_rescue_throttle_min`
- Default: `1100`
- Allowed: `1000 - 2000`

## `gps_rescue_throttle_max`
- Default: `1600`
- Allowed: `1000 - 2000`

## `gps_rescue_ascend_rate`
- Default: `500`
- Allowed: `100 - 2500`

## `gps_rescue_descend_rate`
- Default: `150`
- Allowed: `100 - 500`

## `gps_rescue_throttle_hover`
- Default: `1280`
- Allowed: `1000 - 2000`

## `gps_rescue_sanity_checks`
- Default: `RESCUE_SANITY_ON`
- Allowed: `RESCUE_SANITY_OFF`, `RESCUE_SANITY_ON`, `RESCUE_SANITY_FS_ONLY`

## `gps_rescue_min_sats`
- Default: `8`
- Allowed: `5 - 50`

## `gps_rescue_min_dth`
- Default: `100`
- Allowed: `50 - 1000`

## `gps_rescue_allow_arming_without_fix`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `gps_rescue_alt_mode`
- Default: `MAX_ALT`
- Allowed: `MAX_ALT`, `FIXED_ALT`, `CURRENT_ALT`

## `gps_rescue_use_mag`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `deadband`
- Default: `0`
- Allowed: `0 - 32`

## `yaw_deadband`
- Default: `0`
- Allowed: `0 - 100`

## `yaw_control_reversed`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `pid_process_denom`
- Default: `4`
- Allowed: `1 - 16`

## `runaway_takeoff_prevention`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `runaway_takeoff_deactivate_delay`
- Default: `500`
- Allowed: `100 - 1000`

## `runaway_takeoff_deactivate_throttle_percent`
- Default: `20`
- Allowed: `0 - 100`

## `profile_name`

## `dyn_lpf_dterm_min_hz`
- Default: `70`
- Allowed: `0 - 1000`

## `dyn_lpf_dterm_max_hz`
- Default: `170`
- Allowed: `0 - 1000`

## `dterm_lowpass_type`
- Default: `PT1`
- Allowed: `PT1`, `BIQUAD`

## `dterm_lowpass_hz`
- Default: `150`
- Allowed: `0 - 4000`

## `dterm_lowpass2_type`
- Default: `PT1`
- Allowed: `PT1`, `BIQUAD`

## `dterm_lowpass2_hz`
- Default: `150`
- Allowed: `0 - 4000`

## `dterm_notch_hz`
- Default: `0`
- Allowed: `0 - 4000`

## `dterm_notch_cutoff`
- Default: `0`
- Allowed: `0 - 4000`

## `vbat_pid_gain`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `pid_at_min_throttle`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `anti_gravity_mode`
- Default: `SMOOTH`
- Allowed: `SMOOTH`, `STEP`

## `anti_gravity_threshold`
- Default: `250`
- Allowed: `20 - 1000`

## `anti_gravity_gain`
- Default: `5000`
- Allowed: `1000 - 30000`

## `feedforward_transition`
- Default: `0`
- Allowed: `0 - 100`

## `acc_limit_yaw`
- Default: `0`
- Allowed: `0 - 500`

## `acc_limit`
- Default: `0`
- Allowed: `0 - 500`

## `crash_dthreshold`
- Default: `50`
- Allowed: `10 - 2000`

## `crash_gthreshold`
- Default: `400`
- Allowed: `100 - 2000`

## `crash_setpoint_threshold`
- Default: `350`
- Allowed: `50 - 2000`

## `crash_time`
- Default: `500`
- Allowed: `100 - 5000`

## `crash_delay`
- Default: `0`
- Allowed: `0 - 500`

## `crash_recovery_angle`
- Default: `10`
- Allowed: `5 - 30`

## `crash_recovery_rate`
- Default: `100`
- Allowed: `50 - 255`

## `crash_limit_yaw`
- Default: `200`
- Allowed: `0 - 1000`

## `crash_recovery`
- Default: `OFF`
- Allowed: `OFF`, `ON`, `BEEP`, `DISARM`

## `iterm_rotation`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `iterm_relax`
- Default: `RP`
- Allowed: `OFF`, `RP`, `RPY`, `RP_INC`, `RPY_INC`

## `iterm_relax_type`
- Default: `SETPOINT`
- Allowed: `GYRO`, `SETPOINT`

## `iterm_relax_cutoff`
- Default: `20`
- Allowed: `1 - 100`

## `iterm_windup`
- Default: `100`
- Allowed: `30 - 100`

## `iterm_limit`
- Default: `400`
- Allowed: `0 - 500`

## `pidsum_limit`
- Default: `500`
- Allowed: `100 - 1000`

## `pidsum_limit_yaw`
- Default: `400`
- Allowed: `100 - 1000`

## `yaw_lowpass_hz`
- Default: `0`
- Allowed: `0 - 500`

## `throttle_boost`
- Default: `5`
- Allowed: `0 - 100`

## `throttle_boost_cutoff`
- Default: `15`
- Allowed: `5 - 50`

## `acro_trainer_angle_limit`
- Default: `20`
- Allowed: `10 - 80`
- BF Configurator: *Acro Trainer Angle Limit*

Adds an angle limiting mode for pilots who are learning to fly in acro mode. The range valid is 10-80 and must be activated with a switch in the modes tab.

## `acro_trainer_lookahead_ms`
- Default: `50`
- Allowed: `10 - 200`

## `acro_trainer_debug_axis`
- Default: `ROLL`
- Allowed: `ROLL`, `PITCH`

## `acro_trainer_gain`
- Default: `75`
- Allowed: `25 - 255`

## `p_pitch`
- Default: `46`
- Allowed: `0 - 200`

## `i_pitch`
- Default: `90`
- Allowed: `0 - 200`

## `d_pitch`
- Default: `38`
- Allowed: `0 - 200`

## `f_pitch`
- Default: `95`
- Allowed: `0 - 2000`

## `p_roll`
- Default: `42`
- Allowed: `0 - 200`

## `i_roll`
- Default: `85`
- Allowed: `0 - 200`

## `d_roll`
- Default: `35`
- Allowed: `0 - 200`

## `f_roll`
- Default: `90`
- Allowed: `0 - 2000`

## `p_yaw`
- Default: `30`
- Allowed: `0 - 200`

## `i_yaw`
- Default: `90`
- Allowed: `0 - 200`

## `d_yaw`
- Default: `0`
- Allowed: `0 - 200`

## `f_yaw`
- Default: `90`
- Allowed: `0 - 2000`

## `angle_level_strength`
- Default: `50`
- Allowed: `0 - 200`

## `horizon_level_strength`
- Default: `50`
- Allowed: `0 - 200`

## `horizon_transition`
- Default: `75`
- Allowed: `0 - 200`

## `level_limit`
- Default: `55`
- Allowed: `10 - 90`

## `horizon_tilt_effect`
- Default: `75`
- Allowed: `0 - 250`

## `horizon_tilt_expert_mode`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `abs_control_gain`
- Default: `0`
- Allowed: `0 - 20`

## `abs_control_limit`
- Default: `90`
- Allowed: `10 - 255`

## `abs_control_error_limit`
- Default: `20`
- Allowed: `1 - 45`

## `abs_control_cutoff`
- Default: `11`
- Allowed: `1 - 45`

## `use_integrated_yaw`
- Default: `OFF`
- Allowed: `OFF`, `ON`
- BF Configurator: *Integrated Yaw*

Integrated Yaw is a feature which corrects a fundamental issue with quad control: while the pitch and roll axis are controlled by the thrust differentials the props generate yaw is different. Integrated Yaw fixes this by integrating the output of the yaw pid before applying them to the mixer. This normalizes the way the pids work. You can now tune as any other axis. It requires use of absolute control since no I is needed with Integrated Yaw.

## `integrated_yaw_relax`
- Default: `200`
- Allowed: `0 - 255`

## `d_min_roll`
- Default: `20`
- Allowed: `0 - 100`
- BF Configurator: *D Min Roll*

Controls the strength of dampening (D-term) in normal forward flight. During a sharp move or during prop wash, the Active D-gain raises to the Derivative gains ([d_pitch](#d_pitch)).

## `d_min_pitch`
- Default: `22`
- Allowed: `0 - 100`
- BF Configurator: *D Min Pitch*

Controls the strength of dampening (D-term) in normal forward flight. During a sharp move or during prop wash, the Active D-gain raises to the Derivative gains ([d_pitch](#d_pitch)).

## `d_min_yaw`
- Default: `0`
- Allowed: `0 - 100`
- BF Configurator: * D Min Yaw*

Controls the strength of dampening (D-term) in normal forward flight. During a sharp move or during prop wash, the Active D-gain raises to the Derivative gains ([d_yaw](#d_yaw)).

## `d_min_boost_gain`
- Default: `27`
- Allowed: `0 - 100`
- BF Configurator: *D Min Gain*

Adjusts how fast D gets up to its maximum value and is based on gyro to determine sharp moves and propwash events.

## `d_min_advance`
- Default: `20`
- Allowed: `0 - 200`

Makes D go up earlier by using setpoint instead of gyro to determine sharp moves. aka D Min Advance

## `motor_output_limit`
- Default: `100`
- Allowed: `1 - 100`

## `auto_profile_cell_count`
- Default: `0`
- Allowed: `-1 - 8`

## `launch_control_mode`
- Default: `NORMAL`
- Allowed: `NORMAL`, `PITCHONLY`, `FULL`

## `launch_trigger_allow_reset`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `launch_trigger_throttle_percent`
- Default: `20`
- Allowed: `0 - 90`

## `launch_angle_limit`
- Default: `0`
- Allowed: `0 - 80`

## `launch_control_gain`
- Default: `40`
- Allowed: `0 - 200`

## `thrust_linear`
- Default: `0`
- Allowed: `0 - 100`

## `transient_throttle_limit`
- Default: `0`
- Allowed: `0 - 30`

## `ff_interpolate_sp`
- Default: `AVERAGED`
- Allowed: `OFF`, `ON`, `AVERAGED`

## `ff_spike_limit`
- Default: `60`
- Allowed: `0 - 255`

## `ff_max_rate_limit`
- Default: `100`
- Allowed: `0 - 150`

## `ff_boost`
- Default: `15`
- Allowed: `0 - 50`

## `idle_min_rpm`
- Default: `0`
- Allowed: `0 - 100`

## `idle_adjustment_speed`
- Default: `50`
- Allowed: `25 - 200`

## `idle_p`
- Default: `50`
- Allowed: `10 - 200`

## `idle_pid_limit`
- Default: `200`
- Allowed: `10 - 255`

## `idle_max_increase`
- Default: `150`
- Allowed: `0 - 255`

## `tlm_inverted`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `tlm_halfduplex`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `frsky_default_lat`
- Default: `0`
- Allowed: `-9000 - 9000`

## `frsky_default_long`
- Default: `0`
- Allowed: `-18000 - 18000`

## `frsky_gps_format`
- Default: `0`
- Allowed: `0 - 1`

## `frsky_unit`
- Default: `IMPERIAL`
- Allowed: `IMPERIAL`, `METRIC`

## `frsky_vfas_precision`
- Default: `0`
- Allowed: `0 - 1`

## `hott_alarm_int`
- Default: `5`
- Allowed: `0 - 120`

## `pid_in_tlm`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `report_cell_voltage`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `ibus_sensor`
- Default: `1,2,3,0,0,0,0,0,0,0,0,0,0,0,0`
- Allowed: `Array length: 15`

## `mavlink_mah_as_heading_divisor`
- Default: `0`
- Allowed: `0 - 30000`

## `telemetry_disabled_voltage`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_current`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_fuel`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_mode`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_acc_x`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_acc_y`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_acc_z`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_pitch`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_roll`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_heading`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_altitude`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_vario`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_lat_long`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_ground_speed`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_distance`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_esc_current`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_esc_voltage`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_esc_rpm`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_esc_temperature`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `telemetry_disabled_temperature`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `ledstrip_visual_beeper`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `ledstrip_visual_beeper_color`
- Default: `WHITE`
- Allowed: `BLACK`, `WHITE`, `RED`, `ORANGE`, `YELLOW`, `LIME_GREEN`, `GREEN`, `MINT_GREEN`, `CYAN`, `LIGHT_BLUE`, `BLUE`, `DARK_VIOLET`, `MAGENTA`, `DEEP_PINK`

## `ledstrip_grb_rgb`
- Default: `GRB`
- Allowed: `GRB`, `RGB`

## `ledstrip_profile`
- Default: `STATUS`
- Allowed: `RACE`, `BEACON`, `STATUS`

## `ledstrip_race_color`
- Default: `ORANGE`
- Allowed: `BLACK`, `WHITE`, `RED`, `ORANGE`, `YELLOW`, `LIME_GREEN`, `GREEN`, `MINT_GREEN`, `CYAN`, `LIGHT_BLUE`, `BLUE`, `DARK_VIOLET`, `MAGENTA`, `DEEP_PINK`

## `ledstrip_beacon_color`
- Default: `WHITE`
- Allowed: `BLACK`, `WHITE`, `RED`, `ORANGE`, `YELLOW`, `LIME_GREEN`, `GREEN`, `MINT_GREEN`, `CYAN`, `LIGHT_BLUE`, `BLUE`, `DARK_VIOLET`, `MAGENTA`, `DEEP_PINK`

## `ledstrip_beacon_period_ms`
- Default: `500`
- Allowed: `50 - 10000`

## `ledstrip_beacon_percent`
- Default: `50`
- Allowed: `0 - 100`

## `ledstrip_beacon_armed_only`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `sdcard_detect_inverted`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `sdcard_mode`
- Default: `SPI`
- Allowed: `OFF`, `SPI`, `SDIO`

## `sdcard_dma`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `sdcard_spi_bus`
- Default: `3`
- Allowed: `0 - 3`

## `osd_units`
- Default: `METRIC`
- Allowed: `IMPERIAL`, `METRIC`

## `osd_warn_arming_disable`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_batt_not_full`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_batt_warning`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_batt_critical`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_visual_beeper`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_crash_flip`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_esc_fail`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_core_temp`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_rc_smoothing`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_fail_safe`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_launch_control`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_no_gps_rescue`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_gps_rescue_disabled`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_warn_rssi`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_warn_link_quality`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_rssi_alarm`
- Default: `20`
- Allowed: `0 - 100`

## `osd_link_quality_alarm`
- Default: `80`
- Allowed: `0 - 300`

## `osd_rssi_dbm_alarm`
- Default: `60`
- Allowed: `0 - 130`

## `osd_cap_alarm`
- Default: `2200`
- Allowed: `0 - 20000`

## `osd_alt_alarm`
- Default: `100`
- Allowed: `0 - 10000`

## `osd_esc_temp_alarm`
- Default: `-128`
- Allowed: `-128 - 127`

## `osd_esc_rpm_alarm`
- Default: `-1`
- Allowed: `-1 - 32767`

## `osd_esc_current_alarm`
- Default: `-1`
- Allowed: `-1 - 32767`

## `osd_core_temp_alarm`
- Default: `70`
- Allowed: `0 - 255`

## `osd_ah_max_pit`
- Default: `20`
- Allowed: `0 - 90`

## `osd_ah_max_rol`
- Default: `40`
- Allowed: `0 - 90`

## `osd_ah_invert`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_tim1`
- Default: `2560`
- Allowed: `0 - 32767`

## `osd_tim2`
- Default: `2561`
- Allowed: `0 - 32767`

## `osd_vbat_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_rssi_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_link_quality_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_rssi_dbm_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_tim_1_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_tim_2_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_remaining_time_estimate_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_flymode_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_anti_gravity_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_g_force_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_throttle_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_vtx_channel_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_crosshairs_pos`
- Default: `205`
- Allowed: `0 - 15359`

## `osd_ah_sbar_pos`
- Default: `206`
- Allowed: `0 - 15359`

## `osd_ah_pos`
- Default: `78`
- Allowed: `0 - 15359`

## `osd_current_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_mah_drawn_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_motor_diag_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_craft_name_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_display_name_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_gps_speed_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_gps_lon_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_gps_lat_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_gps_sats_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_home_dir_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_home_dist_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_flight_dist_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_compass_bar_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_altitude_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_pid_roll_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_pid_pitch_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_pid_yaw_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_debug_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_power_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_pidrate_profile_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_warnings_pos`
- Default: `14665`
- Allowed: `0 - 15359`

## `osd_avg_cell_voltage_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_pit_ang_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_rol_ang_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_battery_usage_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_disarmed_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_nheading_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_nvario_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_esc_tmp_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_esc_rpm_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_esc_rpm_freq_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_rtc_date_time_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_adjustment_range_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_flip_arrow_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_core_temp_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_log_status_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_stick_overlay_left_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_stick_overlay_right_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_stick_overlay_radio_mode`
- Default: `2`
- Allowed: `1 - 4`

## `osd_rate_profile_name_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_pid_profile_name_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_profile_name_pos`
- Default: `234`
- Allowed: `0 - 15359`

## `osd_stat_rtc_date_time`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_tim_1`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_tim_2`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_stat_max_spd`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_stat_max_dist`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_min_batt`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_stat_endbatt`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_battery`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_min_rssi`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_stat_max_curr`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_stat_used_mah`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_stat_max_alt`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_bbox`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_stat_bb_no`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `osd_stat_max_g_force`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_max_esc_temp`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_max_esc_rpm`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_min_link_quality`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_flight_dist`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_max_fft`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_total_flights`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_total_time`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_total_dist`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_stat_min_rssi_dbm`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `osd_profile`
- Default: `1`
- Allowed: `1 - 3`

## `osd_profile_1_name`

## `osd_profile_2_name`

## `osd_profile_3_name`

## `osd_gps_sats_show_hdop`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `system_hse_mhz`
- Default: `8`
- Allowed: `0 - 30`

## `task_statistics`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `debug_mode`
- Default: `NONE`
- Allowed: `NONE`, `CYCLETIME`, `BATTERY`, `GYRO_FILTERED`, `ACCELEROMETER`, `PIDLOOP`, `GYRO_SCALED`, `RC_INTERPOLATION`, `ANGLERATE`, `ESC_SENSOR`, `SCHEDULER`, `STACK`, `ESC_SENSOR_RPM`, `ESC_SENSOR_TMP`, `ALTITUDE`, `FFT`, `FFT_TIME`, `FFT_FREQ`, `RX_FRSKY_SPI`, `RX_SFHSS_SPI`, `GYRO_RAW`, `DUAL_GYRO_RAW`, `DUAL_GYRO_DIFF`, `MAX7456_SIGNAL`, `MAX7456_SPICLOCK`, `SBUS`, `FPORT`, `RANGEFINDER`, `RANGEFINDER_QUALITY`, `LIDAR_TF`, `ADC_INTERNAL`, `RUNAWAY_TAKEOFF`, `SDIO`, `CURRENT_SENSOR`, `USB`, `SMARTAUDIO`, `RTH`, `ITERM_RELAX`, `ACRO_TRAINER`, `RC_SMOOTHING`, `RX_SIGNAL_LOSS`, `RC_SMOOTHING_RATE`, `ANTI_GRAVITY`, `DYN_LPF`, `RX_SPEKTRUM_SPI`, `DSHOT_RPM_TELEMETRY`, `RPM_FILTER`, `D_MIN`, `AC_CORRECTION`, `AC_ERROR`, `DUAL_GYRO_SCALED`, `DSHOT_RPM_ERRORS`, `CRSF_LINK_STATISTICS_UPLINK`, `CRSF_LINK_STATISTICS_PWR`, `CRSF_LINK_STATISTICS_DOWN`, `BARO`, `GPS_RESCUE_THROTTLE_PID`, `DYN_IDLE`, `FF_LIMIT`, `FF_INTERPOLATED`

## `rate_6pos_switch`
- Default: `OFF`
- Allowed: `OFF`, `ON`

Allows selection of all 6 rate profiles using a six position switch or pot. Normally, when configuring an adjustment channel for rate profile selection the channel value is divided into 3 ranges instead of 6.

## `cpu_overclock`
- Default: `OFF`
- Allowed: `OFF`, `192MHZ`, `216MHZ`, `240MHZ`

## `pwr_on_arm_grace`
- Default: `5`
- Allowed: `0 - 30`

## `scheduler_optimize_rate`
- Default: `AUTO`
- Allowed: `OFF`, `ON`, `AUTO`

## `vtx_band`
- Default: `0`
- Allowed: `0 - 8`

## `vtx_channel`
- Default: `0`
- Allowed: `0 - 8`

## `vtx_power`
- Default: `0`
- Allowed: `0 - 7`

TODO: The power level (index) to be used by the VTX. Must have SmartAudio set up to work.

## `vtx_low_power_disarm`
- Default: `OFF`
- Allowed: `OFF`, `ON`, `UNTIL_FIRST_ARM`

## `vtx_freq`
- Default: `0`
- Allowed: `0 - 5999`

TODO: The frequency to be used by the VTX. Must have SmartAudio set up to work. (todo: how does this affect band and channel)

## `vtx_pit_mode_freq`
- Default: `0`
- Allowed: `0 - 5999`

## `vtx_halfduplex`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `vtx_spi_bus`
- Default: `0`
- Allowed: `0 - 3`

## `vcd_video_system`
- Default: `AUTO`
- Allowed: `AUTO`, `PAL`, `NTSC`

Sets the analog color system used by the connected FPV camera.

## `vcd_h_offset`
- Default: `0`
- Allowed: `-32 - 31`

## `vcd_v_offset`
- Default: `0`
- Allowed: `-15 - 16`

## `max7456_clock`
- Default: `DEFAULT`
- Allowed: `HALF`, `DEFAULT`, `FULL`

## `max7456_spi_bus`
- Default: `2`
- Allowed: `0 - 3`

## `max7456_preinit_opu`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `displayport_msp_col_adjust`
- Default: `0`
- Allowed: `-6 - 0`

## `displayport_msp_row_adjust`
- Default: `0`
- Allowed: `-3 - 0`

## `displayport_max7456_col_adjust`
- Default: `0`
- Allowed: `-6 - 0`

## `displayport_max7456_row_adjust`
- Default: `0`
- Allowed: `-3 - 0`

## `displayport_max7456_inv`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `displayport_max7456_blk`
- Default: `0`
- Allowed: `0 - 3`

## `displayport_max7456_wht`
- Default: `2`
- Allowed: `0 - 3`

## `esc_sensor_halfduplex`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `esc_sensor_current_offset`
- Default: `0`
- Allowed: `0 - 16000`

## `frsky_spi_autobind`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `frsky_spi_tx_id`
- Default: `0,0`
- Allowed: `Array length: 2`

## `frsky_spi_offset`
- Default: `0`
- Allowed: `-127 - 127`

## `frsky_spi_bind_hop_data`
- Default: `0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0`
- Allowed: `Array length: 50`

GUESS: Data used to bind specific transmitter with receiver, using Frsky SPI protocol. This can avoid the need for rebinding, etc.

## `frsky_x_rx_num`
- Default: `0`
- Allowed: `0 - 255`

## `frsky_spi_a1_source`
- Default: `VBAT`
- Allowed: `VBAT`, `EXTADC`, `CONST`

## `cc2500_spi_chip_detect`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `led_inversion`
- Default: `0`
- Allowed: `0 - 7`

## `dashboard_i2c_bus`
- Default: `1`
- Allowed: `0 - 3`

## `dashboard_i2c_addr`
- Default: `60`
- Allowed: `8 - 119`

## `camera_control_mode`
- Default: `HARDWARE_PWM`
- Allowed: `HARDWARE_PWM`, `SOFTWARE_PWM`, `DAC`

## `camera_control_ref_voltage`
- Default: `330`
- Allowed: `200 - 400`

## `camera_control_key_delay`
- Default: `180`
- Allowed: `100 - 500`

## `camera_control_internal_resistance`
- Default: `470`
- Allowed: `10 - 1000`

## `camera_control_button_resistance`
- Default: `450,270,150,68,0`
- Allowed: `Array length: 5`

## `camera_control_inverted`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `rangefinder_hardware`
- Default: `NONE`
- Allowed: `NONE`, `HCSR04`, `TFMINI`, `TF02`

## `pinio_config`
- Default: `1,1,1,1`
- Allowed: `Array length: 4`

## `pinio_box`
- Default: `255,255,255,255`
- Allowed: `Array length: 4`

## `usb_hid_cdc`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `usb_msc_pin_pullup`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `flash_spi_bus`
- Default: `0`
- Allowed: `0 - 3`

## `rcdevice_init_dev_attempts`
- Default: `6`
- Allowed: `0 - 10`

## `rcdevice_init_dev_attempt_interval`
- Default: `1000`
- Allowed: `0 - 5000`

## `rcdevice_protocol_version`
- Default: `0`
- Allowed: `0 - 1`

## `rcdevice_feature`
- Default: `0`
- Allowed: `0 - 65535`

## `gyro_1_bustype`
- Default: `SPI`
- Allowed: `NONE`, `I2C`, `SPI`, `SLAVE`, `GYROAUTO`

## `gyro_1_spibus`
- Default: `1`
- Allowed: `0 - 3`

## `gyro_1_i2cBus`
- Default: `0`
- Allowed: `0 - 3`

## `gyro_1_i2c_address`
- Default: `0`
- Allowed: `0 - 119`

## `gyro_1_sensor_align`
- Default: `CW180`
- Allowed: `DEFAULT`, `CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`, `CW90FLIP`, `CW180FLIP`, `CW270FLIP`, `CUSTOM`
- BF Configurator: *First GYRO*

GUESS: Alignment of the first gyro sensor. Represented as a rotation string in degrees, e.g. 'CW90'. This seems to be closely coupled with the [gyro_1_align_*](#gyro_1_align_roll) variables.

## `gyro_1_align_roll`
- Default: `0`
- Allowed: `-3600 - 3600`

Assigns how the gyro sensor roll axis is aligned, independent of board alignment. This is updated by the 'First GYRO' ([gyro_1_sensor_align](#gyro_1_sensor_align)) in BF Configurator.

## `gyro_1_align_pitch`
- Default: `0`
- Allowed: `-3600 - 3600`

Assigns how the gyro sensor pitch axis is aligned, independent of board alignment. This is updated by the 'First GYRO' ([gyro_1_sensor_align](#gyro_1_sensor_align)) in BF Configurator.

## `gyro_1_align_yaw`
- Default: `0`
- Allowed: `-3600 - 3600`

Assigns how the gyro sensor yaw axis is aligned, independent of board alignment. This is updated by the 'First GYRO' ([gyro_1_sensor_align](#gyro_1_sensor_align)) in BF Configurator.

## `gyro_2_bustype`
- Default: `SPI`
- Allowed: `NONE`, `I2C`, `SPI`, `SLAVE`, `GYROAUTO`

## `gyro_2_spibus`
- Default: `0`
- Allowed: `0 - 3`

## `gyro_2_i2cBus`
- Default: `0`
- Allowed: `0 - 3`

## `gyro_2_i2c_address`
- Default: `0`
- Allowed: `0 - 119`

## `gyro_2_sensor_align`
- Default: `CW0`
- Allowed: `DEFAULT`, `CW0`, `CW90`, `CW180`, `CW270`, `CW0FLIP`, `CW90FLIP`, `CW180FLIP`, `CW270FLIP`, `CUSTOM`

Like [gyro_1_sensor_align](#gyro_1_sensor_align), but for the second gyro sensor.

## `gyro_2_align_roll`
- Default: `0`
- Allowed: `-3600 - 3600`

Like [gyro_1_align_roll](#gyro_1_align_roll), but for the second gyro sensor.

## `gyro_2_align_pitch`
- Default: `0`
- Allowed: `-3600 - 3600`

Like [gyro_1_align_pitch](#gyro_1_align_pitch), but for the second gyro sensor.

## `gyro_2_align_yaw`
- Default: `0`
- Allowed: `-3600 - 3600`

Like [gyro_1_align_yaw](#gyro_1_align_yaw), but for the second gyro sensor.

## `i2c1_pullup`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `i2c1_overclock`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `i2c2_pullup`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `i2c2_overclock`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `i2c3_pullup`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `i2c3_overclock`
- Default: `ON`
- Allowed: `OFF`, `ON`

## `mco2_on_pc9`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `timezone_offset_minutes`
- Default: `0`
- Allowed: `-780 - 780`

## `gyro_rpm_notch_harmonics`
- Default: `3`
- Allowed: `0 - 3`

## `gyro_rpm_notch_q`
- Default: `500`
- Allowed: `1 - 3000`

## `gyro_rpm_notch_min`
- Default: `100`
- Allowed: `50 - 200`

## `dterm_rpm_notch_harmonics`
- Default: `0`
- Allowed: `0 - 3`

## `dterm_rpm_notch_q`
- Default: `500`
- Allowed: `1 - 3000`

## `dterm_rpm_notch_min`
- Default: `100`
- Allowed: `50 - 200`

## `rpm_notch_lpf`
- Default: `150`
- Allowed: `100 - 500`

## `flysky_spi_tx_id`
- Default: `0`
- Allowed: `0 - 4294967295`

## `flysky_spi_rf_channels`
- Default: `0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0`
- Allowed: `Array length: 16`

## `stats`
- Default: `OFF`
- Allowed: `OFF`, `ON`

## `stats_total_flights`
- Default: `0`
- Allowed: `0 - 4294967295`

## `stats_total_time_s`
- Default: `0`
- Allowed: `0 - 4294967295`

## `stats_total_dist_m`
- Default: `0`
- Allowed: `0 - 4294967295`

## `name`
- BF Configurator: *Craft Name*

Name of the craft. Used in log and config files, and can be displayed in OSD with [osd_craft_name_pos](#osd_craft_name_pos).

## `display_name`
- BF Configurator: *Display Name*

Text that can be displayed in OSD with [osd_display_name_pos](#osd_display_name_pos). E.g. name of the pilot.

## `position_alt_source`
- Default: `DEFAULT`
- Allowed: `DEFAULT`, `BARO_ONLY`, `GPS_ONLY`
