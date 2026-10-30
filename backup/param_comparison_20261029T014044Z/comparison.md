Compared all 1,082 live PX4 parameters against current_qgc_params.params: 85 changed values, 12 live-only parameters, 985 matches, and no snapshot-only parameters.

Live capture: 2026-10-28 18:40:44 America/Phoenix (2026-10-29 01:40:44 UTC). Read-only capture. Floating-point values below are rounded to seven significant digits; differences.csv retains exact values.

| Parameter | Saved snapshot | Live controller |
|---|---:|---:|
| `BAT1_SOURCE` | 1 | 0 |
| `BAT1_V_CHARGED` | 4.2 | 4.05 |
| `BAT1_V_EMPTY` | 3.5 | 3.6 |
| `BAT_CRIT_THR` | 0.15 | 0.07 |
| `BAT_EMERGEN_THR` | 0.1 | 0.05 |
| `BAT_LOW_THR` | 0.35 | 0.15 |
| `CAL_ACC0_XOFF` | 0.04470817 | -0.02830553 |
| `CAL_ACC0_YOFF` | -0.148344 | -0.02126074 |
| `CAL_ACC0_ZOFF` | -0.02071214 | 0.06327343 |
| `CAL_ACC1_XOFF` | 0.0504694 | -0.01236868 |
| `CAL_ACC1_YOFF` | -0.02725935 | -0.002110958 |
| `CAL_ACC1_ZOFF` | -0.03763866 | -0.03950691 |
| `CAL_ACC2_XOFF` | 0.0851512 | 0.0009694099 |
| `CAL_ACC2_YOFF` | 0.03462982 | 0.02100563 |
| `CAL_ACC2_ZOFF` | -0.03717422 | -0.07233238 |
| `CAL_GYRO0_XOFF` | -0.003750113 | 0.004359426 |
| `CAL_GYRO0_YOFF` | -0.01460071 | -0.004889374 |
| `CAL_GYRO0_ZOFF` | 0.004547928 | -0.006022067 |
| `CAL_GYRO1_XOFF` | -0.002604536 | -0.009635263 |
| `CAL_GYRO1_YOFF` | -0.003189975 | -0.02732819 |
| `CAL_GYRO1_ZOFF` | 0.01661176 | -0.002724859 |
| `CAL_GYRO2_XOFF` | 0.0251707 | 0.01589784 |
| `CAL_GYRO2_YOFF` | -0.03839066 | -0.00035121 |
| `CAL_GYRO2_ZOFF` | 0.001679948 | -0.005518501 |
| `CAL_GYRO3_XOFF` | -5.271624e-05 | -0.0001438151 |
| `CAL_GYRO3_ZOFF` | 5.806175e-05 | 6.313728e-05 |
| `CAL_MAG0_XODIAG` | -0.006670147 | 0.009318479 |
| `CAL_MAG0_XOFF` | -0.0540239 | -0.08251144 |
| `CAL_MAG0_XSCALE` | 1.036163 | 1.02558 |
| `CAL_MAG0_YODIAG` | 0.01057517 | 0.0183459 |
| `CAL_MAG0_YOFF` | 0.08381578 | -0.01569964 |
| `CAL_MAG0_YSCALE` | 0.9793355 | 0.9467799 |
| `CAL_MAG0_ZODIAG` | -0.01253023 | -0.0006872087 |
| `CAL_MAG0_ZOFF` | 0.307139 | -0.01706763 |
| `CAL_MAG0_ZSCALE` | 1.004633 | 1.027129 |
| `CA_ROTOR0_KM` | -0.05 | 0.05 |
| `CA_ROTOR1_KM` | -0.05 | 0.05 |
| `CA_ROTOR2_KM` | 0.05 | -0.05 |
| `CA_ROTOR3_KM` | 0.05 | -0.05 |
| `COM_ARM_MAG_ANG` | 180 | 60 |
| `COM_DISARM_PRFLT` | 30 | 10 |
| `COM_FLIGHT_UUID` | 258 | 65 |
| `COM_FLTMODE2` | 11 | -1 |
| `COM_LOW_BAT_ACT` | 2 | 0 |
| `COM_RC_LOSS_T` | 5 | 0.5 |
| `COM_RC_OVERRIDE` | 0 | 1 |
| `CP_DIST` | 5 | -1 |
| `EKF2_EV_CTRL` | 11 | 0 |
| `EKF2_HGT_REF` | 3 | 0 |
| `EKF2_MAG_DECL` | -3.94683 | -3.944516 |
| `EKF2_MAG_TYPE` | 5 | 0 |
| `EKF2_OF_CTRL` | 0 | 1 |
| `EKF2_RNG_A_HMAX` | 10 | 5 |
| `EKF2_RNG_CTRL` | 0 | 2 |
| `EKF2_RNG_POS_X` | 0.06 | 0 |
| `GF_ACTION` | 5 | 2 |
| `GPS_1_CONFIG` | 0 | 201 |
| `LND_FLIGHT_T_HI` | 2 | 0 |
| `LND_FLIGHT_T_LO` | 709594857 | -1878041480 |
| `MC_PITCHRATE_D` | 0.003567038 | 0.003 |
| `MC_PITCHRATE_I` | 0.1783048 | 0.2 |
| `MC_PITCHRATE_P` | 0.1398808 | 0.15 |
| `MC_ROLLRATE_D` | 0.003479034 | 0.003 |
| `MC_ROLLRATE_I` | 0.1706056 | 0.2 |
| `MC_ROLLRATE_P` | 0.1282546 | 0.15 |
| `MC_YAWRATE_D` | 0.007903697 | 0 |
| `MC_YAWRATE_I` | 0.2044853 | 0.1 |
| `MC_YAWRATE_P` | 0.2097058 | 0.2 |
| `MC_YAW_P` | 4.553663 | 2.8 |
| `MIS_TAKEOFF_ALT` | 1 | 2.5 |
| `MPC_VEL_MANUAL` | 0.5 | 10 |
| `MPC_XY_CRUISE` | 0.5 | 5 |
| `MPC_XY_VEL_ALL` | -0.5 | -10 |
| `MPC_XY_VEL_MAX` | 7 | 12 |
| `NAV_DLL_ACT` | 3 | 0 |
| `NAV_RCL_ACT` | 3 | 2 |
| `RTL_DESCEND_ALT` | 2 | 10 |
| `RTL_RETURN_ALT` | 3 | 30 |
| `SENS_BOARD_X_OFF` | 0.08549725 | 0.4093967 |
| `SENS_BOARD_Y_OFF` | -0.2332738 | -2.31504 |
| `SENS_FLOW_MAXHGT` | 25 | 100 |
| `SENS_FLOW_MAXR` | 7.4 | 8 |
| `SYS_HAS_GPS` | 0 | 1 |
| `UAVCAN_RNG_MAX` | 30 | 999 |
| `UAVCAN_RNG_MIN` | 0.05 | 0 |

These 12 parameters exist on the live controller but are absent from the snapshot:

| Parameter | Live controller |
|---|---:|
| `GPS_1_GNSS` | 0 |
| `GPS_1_PROTOCOL` | 1 |
| `GPS_DUMP_COMM` | 0 |
| `GPS_SAT_INFO` | 0 |
| `GPS_UBX_BAUD2` | 230400 |
| `GPS_UBX_CFG_INTF` | 0 |
| `GPS_UBX_MODE` | 0 |
| `GPS_YAW_OFFSET` | 0 |
| `SENS_GPS_MASK` | 7 |
| `SENS_GPS_PRIME` | 0 |
| `SENS_GPS_TAU` | 10 |
| `SER_GPS1_BAUD` | 0 |
