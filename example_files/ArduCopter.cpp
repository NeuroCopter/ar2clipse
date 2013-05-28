#line 1 "/home/tobias/git/ardupilot/ArduCopter/ArduCopter.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V3.0.0-rc3"
/*
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Based on code and ideas from the Arducopter team: Pat Hickey, Jose Julio, Jani Hirvinen, Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni
 *  Thanks to:	Chris Anderson, Mike Smith, Jordi Munoz, Doug Weibel, James Goppert, Benjamin Pelletier, Robert Lefebvre, Marco Robustini
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  Special Thanks for Contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera		:Auto Compass Declination
 *  Amilcar Lucas		:Camera mount library
 *  Andrew Tridgell		:General development, Mavlink Support
 *  Angel Fernandez		:Alpha testing
 *  Doug Weibel			:Libraries
 *  Christof Schmid		:Alpha testing
 *  Dani Saez           :V Octo Support
 *  Gregory Fletcher	:Camera mount orientation math
 *  Guntars				:Arming safety suggestion
 *  HappyKillmore		:Mavlink GCS
 *  Hein Hollander      :Octo Support
 *  Igor van Airde      :Control Law optimization
 *  Leonard Hall 		:Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Jonathan Challinger :Inertial Navigation
 *  Jean-Louis Naudin   :Auto Landing
 *  Max Levine			:Tri Support, Graphics
 *  Jack Dunkle			:Alpha testing
 *  James Goppert		:Mavlink Support
 *  Jani Hiriven		:Testing feedback
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio			:Stabilization Control laws
 *  Marco Robustini		:Lead tester
 *  Michael Oborne		:Mission Planner GCS
 *  Mike Smith			:Libraries, Coding support
 *  Oliver				:Piezo support
 *  Olivier Adler       :PPM Encoder
 *  Robert Lefebvre		:Heli Support & LEDs
 *  Sandro Benigno      :Camera support
 *
 *  And much more so PLEASE PM me on DIYDRONES to add your contribution to the List
 *
 *  Requires modified "mrelax" version of Arduino, which can be found here:
 *  http://code.google.com/p/ardupilot-mega/downloads/list
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_SMACCM.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>     		// ArduCopter waypoint navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence.h>           // Arducopter Fence library
#include <memcheck.h>           // memory limit checker
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// Local modules
#include "Parameters.h"
#include "GCS.h"

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
#line 1 "autogenerated"
   void setup() ;
 static void compass_accumulate(void) ;
 static void barometer_accumulate(void) ;
  static void perf_update(void) ;
  void loop() ;
 static void fast_loop() ;
 static void fifty_hz_loop() ;
 static void medium_loop() ;
 static void slow_loop() ;
 static void super_slow_loop() ;
 static void update_optical_flow(void) ;
 static void update_GPS(void) ;
 bool set_yaw_mode(uint8_t new_yaw_mode) ;
 void update_yaw_mode(void) ;
 uint8_t get_wp_yaw_mode(bool rtl) ;
 bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode) ;
 void update_roll_pitch_mode(void) ;
 void update_simple_mode(void) ;
 void update_super_simple_bearing() ;
 bool set_throttle_mode( uint8_t new_throttle_mode ) ;
 void update_throttle_mode(void) ;
 static void set_target_alt_for_reporting(float alt_cm) ;
 static float get_target_alt_for_reporting() ;
  static void read_AHRS(void) ;
  static void update_trig(void);
 static void update_altitude() ;
  static void tuning();
   void set_home_is_set(bool b) ;
 void set_auto_armed(bool b) ;
 void set_simple_mode(bool b) ;
 static void set_failsafe_radio(bool mode) ;
 void set_low_battery(bool b) ;
 static void set_failsafe_gps(bool mode) ;
 static void set_failsafe_gcs(bool mode) ;
 void set_takeoff_complete(bool b) ;
 void set_land_complete(bool b) ;
  void set_compass_healthy(bool b) ;
  void set_gps_healthy(bool b) ;
  static void get_stabilize_roll(int32_t target_angle) ;
  static void get_stabilize_pitch(int32_t target_angle) ;
  static void get_stabilize_yaw(int32_t target_angle) ;
  static void get_acro_roll(int32_t target_rate) ;
  static void get_acro_pitch(int32_t target_rate) ;
  static void get_acro_yaw(int32_t target_rate) ;
 static void get_roll_rate_stabilized_ef(int32_t stick_angle) ;
 static void get_pitch_rate_stabilized_ef(int32_t stick_angle) ;
 static void get_yaw_rate_stabilized_ef(int32_t stick_angle) ;
 void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) ;
 void update_rate_contoller_targets() ;
 void run_rate_controllers() ;
 void init_rate_controllers() ;
  static int16_t get_heli_rate_roll(int32_t target_rate) ;
  static int16_t get_heli_rate_pitch(int32_t target_rate) ;
  static int16_t get_heli_rate_yaw(int32_t target_rate) ;
 static int16_t get_rate_roll(int32_t target_rate) ;
  static int16_t get_rate_pitch(int32_t target_rate) ;
  static int16_t get_rate_yaw(int32_t target_rate) ;
 static int32_t get_of_roll(int32_t input_roll) ;
  static int32_t get_of_pitch(int32_t input_pitch) ;
 static void get_circle_yaw() ;
 static void get_look_at_yaw() ;
  static void get_look_ahead_yaw(int16_t pilot_yaw) ;
 static void update_throttle_cruise(int16_t throttle) ;
 static int16_t get_angle_boost(int16_t throttle) ;
 static int16_t get_angle_boost(int16_t throttle) ;
 void set_throttle_out( int16_t throttle_out, bool apply_angle_boost ) ;
 void set_throttle_accel_target( int16_t desired_acceleration ) ;
 void throttle_accel_deactivate() ;
 static int16_t get_throttle_accel(int16_t z_target_accel) ;
 static int16_t get_pilot_desired_throttle(int16_t throttle_control) ;
 static int16_t get_pilot_desired_climb_rate(int16_t throttle_control) ;
 static int32_t get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms) ;
 static void get_throttle_rate(float z_target_speed) ;
 static void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate) ;
 static void get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate) ;
 static void get_throttle_rate_stabilized(int16_t target_rate) ;
 static void get_throttle_land() ;
 static void get_throttle_surface_tracking(int16_t target_rate) ;
 static void reset_I_all(void) ;
  static void reset_rate_I() ;
  static void reset_optflow_I(void) ;
  static void reset_throttle_I(void) ;
  static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle) ;
  static void reset_stability_I(void) ;
  static void gcs_send_heartbeat(void) ;
  static void gcs_send_deferred(void) ;
  static NOINLINE void send_heartbeat(mavlink_channel_t chan) ;
  static NOINLINE void send_attitude(mavlink_channel_t chan) ;
 static NOINLINE void send_limits_status(mavlink_channel_t chan) ;
   static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops) ;
  static void NOINLINE send_meminfo(mavlink_channel_t chan) ;
  static void NOINLINE send_location(mavlink_channel_t chan) ;
  static void NOINLINE send_nav_controller_output(mavlink_channel_t chan) ;
  static void NOINLINE send_ahrs(mavlink_channel_t chan) ;
 static void NOINLINE send_simstate(mavlink_channel_t chan) ;
  static void NOINLINE send_hwstatus(mavlink_channel_t chan) ;
  static void NOINLINE send_gps_raw(mavlink_channel_t chan) ;
  static void NOINLINE send_servo_out(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_in(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_out(mavlink_channel_t chan) ;
  static void NOINLINE send_vfr_hud(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu1(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu2(mavlink_channel_t chan) ;
  static void NOINLINE send_raw_imu3(mavlink_channel_t chan) ;
  static void NOINLINE send_current_waypoint(mavlink_channel_t chan) ;
  static void NOINLINE send_statustext(mavlink_channel_t chan) ;
 static bool telemetry_delayed(mavlink_channel_t chan) ;
 static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops) ;
 static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops) ;
  void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str) ;
 static void mavlink_delay_cb() ;
 static void gcs_send_message(enum ap_message id) ;
 static void gcs_data_stream_send(void) ;
 static void gcs_check_input(void) ;
  static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str) ;
  static bool print_log_menu(void) ;
  static void do_erase_logs(void) ;
 static void Log_Write_Current() ;
 static void Log_Write_Motors() ;
 static void Log_Write_Optflow() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_Compass() ;
 static void Log_Write_Performance() ;
 static void Log_Write_Cmd(uint8_t num, const struct Location *wp) ;
 static void Log_Write_Attitude() ;
 static void Log_Write_INAV() ;
 static void Log_Write_Mode(uint8_t mode) ;
 static void Log_Write_Startup() ;
 static void Log_Write_Event(uint8_t id) ;
 static void Log_Write_Data(uint8_t id, int16_t value) ;
 static void Log_Write_Data(uint8_t id, uint16_t value) ;
 static void Log_Write_Data(uint8_t id, int32_t value) ;
 static void Log_Write_Data(uint8_t id, uint32_t value) ;
 static void Log_Write_Data(uint8_t id, float value) ;
 static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain) ;
 static void Log_Write_DMP() ;
 static void Log_Write_Camera() ;
 static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) ;
 static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page) ;
 static void start_logging()  ;
  static void Log_Write_Startup() ;
 static void Log_Write_Cmd(uint8_t num, const struct Location *wp) ;
 static void Log_Write_Mode(uint8_t mode) ;
 static void Log_Write_IMU() ;
 static void Log_Write_GPS() ;
 static void Log_Write_Current() ;
 static void Log_Write_Compass() ;
 static void Log_Write_Attitude() ;
 static void Log_Write_INAV() ;
 static void Log_Write_Data(uint8_t id, int16_t value);
 static void Log_Write_Data(uint8_t id, uint16_t value);
 static void Log_Write_Data(uint8_t id, int32_t value);
 static void Log_Write_Data(uint8_t id, uint32_t value);
 static void Log_Write_Data(uint8_t id, float value);
 static void Log_Write_Event(uint8_t id);
 static void Log_Write_Optflow() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_Motors() ;
 static void Log_Write_Performance() ;
 static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain) ;
 static void Log_Write_DMP() ;
 static void Log_Write_Camera() ;
 static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) ;
   static void load_parameters(void) ;
 void userhook_init() ;
 void userhook_FastLoop() ;
 void userhook_50Hz() ;
 void userhook_MediumLoop() ;
 void userhook_SlowLoop() ;
 void userhook_SuperSlowLoop() ;
  static void init_commands() ;
 static struct Location get_cmd_with_index(int i) ;
 static void set_cmd_with_index(struct Location temp, int i) ;
  static int32_t get_RTL_alt() ;
 static void init_home() ;
 static void process_nav_command() ;
 static void process_cond_command() ;
 static void process_now_command() ;
 static bool verify_nav_command() ;
 static bool verify_cond_command() ;
 static void do_RTL(void) ;
 static void do_takeoff() ;
 static void do_nav_wp() ;
 static void do_land() ;
 static void do_loiter_unlimited() ;
 static void do_circle() ;
 static void do_loiter_time() ;
 static bool verify_takeoff() ;
 static bool verify_land() ;
 static bool verify_nav_wp() ;
  static bool verify_loiter_unlimited() ;
 static bool verify_loiter_time() ;
 static bool verify_circle() ;
 static bool verify_RTL() ;
  static void do_wait_delay() ;
  static void do_change_alt() ;
  static void do_within_distance() ;
  static void do_yaw() ;
  static bool verify_wait_delay() ;
  static bool verify_change_alt() ;
  static bool verify_within_distance() ;
 static bool verify_yaw() ;
 static bool verify_nav_roi() ;
  static void do_change_speed() ;
  static void do_jump() ;
  static void do_set_home() ;
  static void do_set_servo() ;
  static void do_set_relay() ;
  static void do_repeat_servo() ;
  static void do_repeat_relay() ;
 static void do_nav_roi() ;
 static void do_take_picture() ;
 static void change_command(uint8_t cmd_index) ;
 static void update_commands() ;
 static void execute_nav_command(void) ;
 static void verify_commands(void) ;
 static int16_t find_next_nav_index(int16_t search_index) ;
  static void exit_mission() ;
  void delay(uint32_t ms) ;
  void mavlink_delay(uint32_t ms) ;
  uint32_t millis() ;
  uint32_t micros() ;
  void pinMode(uint8_t pin, uint8_t output) ;
  void digitalWrite(uint8_t pin, uint8_t out) ;
  uint8_t digitalRead(uint8_t pin) ;
 static void read_control_switch() ;
  static uint8_t readSwitch(void);
  static void reset_control_switch() ;
 static void read_aux_switches() ;
 static void do_aux_switch_function(int8_t ch_function, bool ch_flag) ;
 static void save_trim() ;
 static void auto_trim() ;
 static void failsafe_radio_on_event() ;
 static void failsafe_radio_off_event() ;
  static void low_battery_event(void) ;
 static void failsafe_gps_check() ;
 static void failsafe_gps_off_event(void) ;
 static void failsafe_gcs_check() ;
 static void failsafe_gcs_off_event(void) ;
 void failsafe_enable() ;
 void failsafe_disable() ;
 void failsafe_check(uint32_t tnow) ;
 void fence_check() ;
 static void fence_send_mavlink_status(mavlink_channel_t chan) ;
  void init_flip() ;
  void roll_flip() ;
 static void read_inertia() ;
 static void read_inertial_altitude() ;
  static void update_lights() ;
  static void update_GPS_light(void) ;
  static void update_motor_light(void) ;
  static void dancing_light() ;
 static void clear_leds() ;
 static void copter_leds_init(void) ;
  static void update_copter_leds(void) ;
  static void copter_leds_reset(void) ;
  static void copter_leds_on(void) ;
  static void copter_leds_off(void) ;
  static void copter_leds_slow_blink(void) ;
  static void copter_leds_fast_blink(void) ;
  static void copter_leds_oscillate(void) ;
  static void copter_leds_GPS_on(void) ;
  static void copter_leds_GPS_off(void) ;
  static void copter_leds_GPS_slow_blink(void) ;
  static void copter_leds_GPS_fast_blink(void) ;
  static void copter_leds_aux_off(void);
  static void copter_leds_aux_on(void);
  void piezo_on();
  void piezo_off();
  void piezo_beep();
  void piezo_beep_twice();
 static void arm_motors_check() ;
 static void auto_disarm_check() ;
 static void init_arm_motors() ;
 static void pre_arm_checks(bool display_failure) ;
 static void pre_arm_rc_checks() ;
  static void init_disarm_motors() ;
 static void set_servos_4() ;
 static void update_navigation() ;
 static void run_nav_updates(void) ;
 static void calc_position();
 static void calc_distance_and_bearing() ;
 static void run_autopilot() ;
 static bool set_nav_mode(uint8_t new_nav_mode) ;
 static void update_nav_mode() ;
 static void reset_nav_params(void) ;
 static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec) ;
 static void circle_set_center(const Vector3f current_position, float heading_in_radians) ;
 static void update_circle(float dt) ;
 void perf_info_reset() ;
 void perf_info_check_loop_time(uint32_t time_in_micros) ;
 uint16_t perf_info_get_num_loops() ;
 uint32_t perf_info_get_max_time() ;
 uint16_t perf_info_get_num_long_running() ;
 const Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt) ;
 const Vector3f pv_location_to_vector(Location loc) ;
 const int32_t pv_get_lat(const Vector3f pos_vec) ;
 const int32_t pv_get_lon(const Vector3f pos_vec) ;
 const float pv_get_horizontal_distance_cm(const Vector3f origin, const Vector3f destination) ;
 const float pv_get_bearing_cd(const Vector3f origin, const Vector3f destination) ;
  static void default_dead_zones() ;
  static void init_rc_in() ;
 static void init_rc_out() ;
 void output_min() ;
 static void read_radio() ;
 static void set_throttle_and_failsafe(uint16_t throttle_pwm) ;
  static void trim_radio() ;
 static void init_sonar(void) ;
  static void init_barometer(void) ;
 static int32_t read_barometer(void) ;
 static int16_t read_sonar(void) ;
  static void init_compass() ;
  static void init_optflow() ;
 static void read_battery(void) ;
 void read_receiver_rssi(void) ;
  static void clear_offsets() ;
  static void report_batt_monitor() ;
  static void report_sonar() ;
  static void report_frame() ;
  static void report_radio() ;
  static void report_ins() ;
  static void report_compass() ;
  static void report_flight_modes() ;
  void report_optflow() ;
 static void report_heli() ;
  static void report_gyro() ;
  static void print_radio_values() ;
  static void print_switch(uint8_t p, uint8_t m, bool b) ;
  static void print_done() ;
   static void zero_eeprom(void) ;
  static void print_accel_offsets_and_scaling(void) ;
  static void print_gyro_offsets(void) ;
  static RC_Channel * heli_get_servo(int16_t servo_num);
 static int16_t read_num_from_serial() ;
  static void print_blanks(int16_t num) ;
  static void print_divider(void) ;
  static void print_enabled(bool b) ;
   static void init_esc() ;
  static void print_wp(const struct Location *cmd, uint8_t index) ;
  static void report_version() ;
   static void report_tuning() ;
  static void init_ardupilot() ;
 static void startup_ground(void) ;
 static void set_mode(uint8_t mode) ;
  static void init_simple_bearing() ;
 static void update_auto_armed() ;
 static uint32_t map_baudrate(int8_t rate, uint32_t default_baud) ;
 static void check_usb_mux(void) ;
 void flash_leds(bool on) ;
 uint16_t board_voltage(void) ;
 static void reboot_apm(void) ;
  static void print_hit_enter() ;
 void update_toy_throttle() ;
 void update_toy_altitude() ;
 void edf_toy() ;
 void roll_pitch_toy() ;
#line 123 "/home/tobias/git/ardupilot/ArduCopter/ArduCopter.pde"
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler. 

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
static AP_Scheduler scheduler;

////////////////////////////////////////////////////////////////////////////////
// prototypes
////////////////////////////////////////////////////////////////////////////////
static void update_events(void);
static void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);

////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
static DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
static DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
//static DataFlash_File DataFlash("/tmp/APMlogs");
static DataFlash_SITL DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
static DataFlash_File DataFlash("/fs/microsd/APM/logs");
#else
static DataFlash_Empty DataFlash;
#endif


////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_200HZ;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

// All GPS access should be through this pointer.
static GPS         *g_gps;

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;

#if HIL_MODE == HIL_MODE_DISABLED

 #if CONFIG_ADC == ENABLED
static AP_ADC_ADS7844 adc;
 #endif

 #if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
static AP_InertialSensor_MPU6000 ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_OILPAN
static AP_InertialSensor_Oilpan ins(&adc);
#elif CONFIG_IMU_TYPE == CONFIG_IMU_SITL
static AP_InertialSensor_Stub ins;
#elif CONFIG_IMU_TYPE == CONFIG_IMU_PX4
static AP_InertialSensor_PX4 ins;
 #endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static AP_Baro_HIL barometer;
static AP_Compass_HIL compass;
static SITL sitl;
 #else
// Otherwise, instantiate a real barometer and compass driver
  #if CONFIG_BARO == AP_BARO_BMP085
static AP_Baro_BMP085 barometer;
  #elif CONFIG_BARO == AP_BARO_PX4
static AP_Baro_PX4 barometer;
  #elif CONFIG_BARO == AP_BARO_MS5611
   #if CONFIG_MS5611_SERIAL == AP_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
   #elif CONFIG_MS5611_SERIAL == AP_BARO_MS5611_I2C
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
   #else
    #error Unrecognized CONFIG_MS5611_SERIAL setting.
   #endif
  #endif

 #if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static AP_Compass_PX4 compass;
 #else
static AP_Compass_HMC5843 compass;
 #endif
 #endif

// real GPS selection
 #if   GPS_PROTOCOL == GPS_PROTOCOL_AUTO
AP_GPS_Auto     g_gps_driver(&g_gps);

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NMEA
AP_GPS_NMEA     g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_SIRF
AP_GPS_SIRF     g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_UBLOX
AP_GPS_UBLOX    g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK
AP_GPS_MTK      g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_MTK19
AP_GPS_MTK19    g_gps_driver();

 #elif GPS_PROTOCOL == GPS_PROTOCOL_NONE
AP_GPS_None     g_gps_driver();

 #else
  #error Unrecognised GPS_PROTOCOL setting.
 #endif // GPS PROTOCOL

 #if DMP_ENABLED == ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_APM2
static AP_AHRS_MPU6000  ahrs(&ins, g_gps);               // only works with APM2
 #else
static AP_AHRS_DCM ahrs(&ins, g_gps);
 #endif

// ahrs2 object is the secondary ahrs to allow running DMP in parallel with DCM
  #if SECONDARY_DMP_ENABLED == ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_APM2
static AP_AHRS_MPU6000  ahrs2(&ins, g_gps);               // only works with APM2
  #endif

#elif HIL_MODE == HIL_MODE_SENSORS
// sensor emulators
static AP_ADC_HIL              adc;
static AP_Baro_HIL      barometer;
static AP_Compass_HIL          compass;
static AP_GPS_HIL              g_gps_driver;
static AP_InertialSensor_Stub  ins;
static AP_AHRS_DCM             ahrs(&ins, g_gps);

static int32_t gps_base_alt;

 #if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static SITL sitl;
#endif

#elif HIL_MODE == HIL_MODE_ATTITUDE
static AP_ADC_HIL              adc;
static AP_InertialSensor_Stub  ins;
static AP_AHRS_HIL             ahrs(&ins, g_gps);
static AP_GPS_HIL              g_gps_driver;
static AP_Compass_HIL          compass;                  // never used
static AP_Baro_HIL      barometer;

static int32_t gps_base_alt;

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 // When building for SITL we use the HIL barometer and compass drivers
static SITL sitl;
#endif

#else
 #error Unrecognised HIL_MODE setting.
#endif // HIL MODE

////////////////////////////////////////////////////////////////////////////////
// Optical flow sensor
////////////////////////////////////////////////////////////////////////////////
 #if OPTFLOW == ENABLED
static AP_OpticalFlow_ADNS3080 optflow;
 #else
static AP_OpticalFlow optflow;
 #endif

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static GCS_MAVLINK gcs0;
static GCS_MAVLINK gcs3;

////////////////////////////////////////////////////////////////////////////////
// SONAR selection
////////////////////////////////////////////////////////////////////////////////
//
ModeFilterInt16_Size3 sonar_mode_filter(1);
#if CONFIG_SONAR == ENABLED
static AP_HAL::AnalogSource *sonar_analog_source;
static AP_RangeFinder_MaxsonarXL *sonar;
#endif

////////////////////////////////////////////////////////////////////////////////
// User variables
////////////////////////////////////////////////////////////////////////////////
#ifdef USERHOOK_VARIABLES
 #include USERHOOK_VARIABLES
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

/* Radio values
 *               Channel assignments
 *                       1	Ailerons (rudder if no ailerons)
 *                       2	Elevator
 *                       3	Throttle
 *                       4	Rudder (if we have ailerons)
 *                       5	Mode - 3 position switch
 *                       6  User assignable
 *                       7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
 *                       8	TBD
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t home_is_set         : 1; // 0
        uint8_t simple_mode         : 1; // 1    // This is the state of simple mode
        uint8_t manual_attitude     : 1; // 2
        uint8_t manual_throttle     : 1; // 3

        uint8_t pre_arm_rc_check    : 1; // 5    // true if rc input pre-arm checks have been completed successfully
        uint8_t pre_arm_check       : 1; // 6    // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed          : 1; // 7    // stops auto missions from beginning until throttle is raised
        uint8_t logging_started     : 1; // 8    // true if dataflash logging has started

        uint8_t low_battery         : 1; // 9    // Used to track if the battery is low - LED output flashes when the batt is low
        uint8_t failsafe_radio      : 1; // 10   // A status flag for the radio failsafe
        uint8_t failsafe_batt       : 1; // 11   // A status flag for the battery failsafe
        uint8_t failsafe_gps        : 1; // 12   // A status flag for the gps failsafe
        uint8_t failsafe_gcs        : 1; // 13   // A status flag for the ground station failsafe
        uint8_t rc_override_active  : 1; // 14   // true if rc control are overwritten by ground station
        uint8_t do_flip             : 1; // 15   // Used to enable flip code
        uint8_t takeoff_complete    : 1; // 16
        uint8_t land_complete       : 1; // 17
        uint8_t compass_status      : 1; // 18
        uint8_t gps_status          : 1; // 19
    };
    uint32_t value;
} ap;


static struct AP_System{
    uint8_t GPS_light               : 1; // 0   // Solid indicates we have full 3D lock and can navigate, flash = read
    uint8_t motor_light             : 1; // 1   // Solid indicates Armed state
    uint8_t new_radio_frame         : 1; // 2   // Set true if we have new PWM data to act on from the Radio
    uint8_t CH7_flag                : 1; // 3   // true if ch7 aux switch is high
    uint8_t CH8_flag                : 1; // 4   // true if ch8 aux switch is high
    uint8_t usb_connected           : 1; // 5   // true if APM is powered from USB connection
    uint8_t yaw_stopped             : 1; // 6   // Used to manage the Yaw hold capabilities

} ap_system;

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
// Used to maintain the state of the previous control switch position
// This is set to -1 when we need to re-read the switch
static uint8_t oldSwitchPosition;

// receiver RSSI
static uint8_t receiver_rssi;


////////////////////////////////////////////////////////////////////////////////
// Motor Output
////////////////////////////////////////////////////////////////////////////////
#if FRAME_CONFIG == QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsQuad
#endif
#if FRAME_CONFIG == TRI_FRAME
 #define MOTOR_CLASS AP_MotorsTri
#endif
#if FRAME_CONFIG == HEXA_FRAME
 #define MOTOR_CLASS AP_MotorsHexa
#endif
#if FRAME_CONFIG == Y6_FRAME
 #define MOTOR_CLASS AP_MotorsY6
#endif
#if FRAME_CONFIG == OCTA_FRAME
 #define MOTOR_CLASS AP_MotorsOcta
#endif
#if FRAME_CONFIG == OCTA_QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsOctaQuad
#endif
#if FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#endif

#if FRAME_CONFIG == HELI_FRAME  // helicopter constructor requires more arguments
static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_8, &g.heli_servo_1, &g.heli_servo_2, &g.heli_servo_3, &g.heli_servo_4);
#elif FRAME_CONFIG == TRI_FRAME  // tri constructor requires additional rc_7 argument to allow tail servo reversing
static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4, &g.rc_7);
#else
static MOTOR_CLASS motors(&g.rc_1, &g.rc_2, &g.rc_3, &g.rc_4);
#endif

////////////////////////////////////////////////////////////////////////////////
// PIDs
////////////////////////////////////////////////////////////////////////////////
// This is a convienience accessor for the IMU roll rates. It's currently the raw IMU rates
// and not the adjusted omega rates, but the name is stuck
static Vector3f omega;
// This is used to hold radio tuning values for in-flight CH6 tuning
float tuning_value;
// used to limit the rate that the pid controller output is logged so that it doesn't negatively affect performance
static uint8_t pid_log_counter;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
// This is current status for the LED lights state machine
// setting this value changes the output of the LEDs
static uint8_t led_mode = NORMAL_LEDS;
// Blinking indicates GPS status
static uint8_t copter_leds_GPS_blink;
// Blinking indicates battery status
static uint8_t copter_leds_motor_blink;
// Navigation confirmation blinks
static int8_t copter_leds_nav_blink;

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// This is used to scale GPS values for EEPROM storage
// 10^7 times Decimal GPS means 1 == 1cm
// This approximation makes calculations integer and it's easy to read
static const float t7 = 10000000.0;
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the next waypoint in centi-degrees
static int32_t wp_bearing;
// The original bearing to the next waypoint.  used to check if we've passed the waypoint
static int32_t original_wp_bearing;
// The location of home in relation to the copter in centi-degrees
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next waypoint in cm.  is not static because AP_Camera uses it
uint32_t wp_distance;
// navigation mode - options include NAV_NONE, NAV_LOITER, NAV_CIRCLE, NAV_WP
static uint8_t nav_mode;
// Register containing the index of the current navigation command in the mission script
static int16_t command_nav_index;
// Register containing the index of the previous navigation command in the mission script
// Used to manage the execution of conditional commands
static uint8_t prev_nav_index;
// Register containing the index of the current conditional command in the mission script
static uint8_t command_cond_index;
// Used to track the required WP navigation information
// options include
// NAV_ALTITUDE - have we reached the desired altitude?
// NAV_LOCATION - have we reached the desired location?
// NAV_DELAY    - have we waited at the waypoint the desired time?
static float lon_error, lat_error;      // Used to report how many cm we are from the next waypoint or loiter target position
static int16_t control_roll;
static int16_t control_pitch;
static uint8_t rtl_state;               // records state of rtl (initial climb, returning home, etc)
static uint8_t land_state;              // records state of land (flying to location, descending)

////////////////////////////////////////////////////////////////////////////////
// Orientation
////////////////////////////////////////////////////////////////////////////////
// Convienience accessors for commonly used trig functions. These values are generated
// by the DCM through a few simple equations. They are used throughout the code where cos and sin
// would normally be used.
// The cos values are defaulted to 1 to get a decent initial value for a level state
static float cos_roll_x         = 1.0;
static float cos_pitch_x        = 1.0;
static float cos_yaw            = 1.0;
static float sin_yaw;
static float sin_roll;
static float sin_pitch;

////////////////////////////////////////////////////////////////////////////////
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
// or in SuperSimple mode when the copter leaves a 20m radius from home.
static int32_t initial_simple_bearing;

////////////////////////////////////////////////////////////////////////////////
// Rate contoller targets
////////////////////////////////////////////////////////////////////////////////
static uint8_t rate_targets_frame = EARTH_FRAME;    // indicates whether rate targets provided in earth or body frame
static int32_t roll_rate_target_ef;
static int32_t pitch_rate_target_ef;
static int32_t yaw_rate_target_ef;
static int32_t roll_rate_target_bf;     // body frame roll rate target
static int32_t pitch_rate_target_bf;    // body frame pitch rate target
static int32_t yaw_rate_target_bf;      // body frame yaw rate target

////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
static int16_t throttle_accel_target_ef;    // earth frame throttle acceleration target
static bool throttle_accel_controller_active;   // true when accel based throttle controller is active, false when higher level throttle controllers are providing throttle output directly
static float throttle_avg;                  // g.throttle_cruise as a float
static int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only
static float target_alt_for_reporting;      // target altitude in cm for reporting (logs and ground station)


////////////////////////////////////////////////////////////////////////////////
// ACRO Mode
////////////////////////////////////////////////////////////////////////////////
// Used to control Axis lock
static int32_t roll_axis;
static int32_t pitch_axis;

// Filters
#if FRAME_CONFIG == HELI_FRAME
static LowPassFilterFloat rate_roll_filter;    // Rate Roll filter
static LowPassFilterFloat rate_pitch_filter;   // Rate Pitch filter
// LowPassFilterFloat rate_yaw_filter;     // Rate Yaw filter
#endif // HELI_FRAME

////////////////////////////////////////////////////////////////////////////////
// Circle Mode / Loiter control
////////////////////////////////////////////////////////////////////////////////
Vector3f circle_center;     // circle position expressed in cm from home location.  x = lat, y = lon
// angle from the circle center to the copter's desired location.  Incremented at circle_rate / second
static float circle_angle;
// the total angle (in radians) travelled
static float circle_angle_total;
// deg : how many times to circle as specified by mission command
static uint8_t circle_desired_rotations;
// How long we should stay in Loiter Mode for mission scripting (time in seconds)
static uint16_t loiter_time_max;
// How long have we been loitering - The start time in millis
static uint32_t loiter_time;


////////////////////////////////////////////////////////////////////////////////
// CH7 and CH8 save waypoint control
////////////////////////////////////////////////////////////////////////////////
// This register tracks the current Mission Command index when writing
// a mission using Ch7 or Ch8 aux switches in flight
static int8_t aux_switch_wp_index;


////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
// Battery Voltage of battery, initialized above threshold for filter
static float battery_voltage1 = LOW_VOLTAGE * 1.05f;
// refers to the instant amp draw – based on an Attopilot Current sensor
static float current_amps1;
// refers to the total amps drawn – based on an Attopilot Current sensor
static float current_total1;


////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// The (throttle) controller desired altitude in cm
static float controller_desired_alt;
// The cm we are off in altitude from next_WP.alt – Positive value means we are below the WP
static int32_t altitude_error;
// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;
// The altitude as reported by Sonar in cm – Values are 20 to 700 generally.
static int16_t sonar_alt;
static uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
// The altitude as reported by Baro in cm – Values can be quite high
static int32_t baro_alt;

static int16_t saved_toy_throttle;


////////////////////////////////////////////////////////////////////////////////
// flight modes
////////////////////////////////////////////////////////////////////////////////
// Flight modes are combinations of Roll/Pitch, Yaw and Throttle control modes
// Each Flight mode is a unique combination of these modes
//
// The current desired control scheme for Yaw
static uint8_t yaw_mode;
// The current desired control scheme for roll and pitch / navigation
static uint8_t roll_pitch_mode;
// The current desired control scheme for altitude hold
static uint8_t throttle_mode;


////////////////////////////////////////////////////////////////////////////////
// flight specific
////////////////////////////////////////////////////////////////////////////////
// An additional throttle added to keep the copter at the same altitude when banking
static int16_t angle_boost;
// counter to verify landings
static uint16_t land_detector;


////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// home location is stored when we have a good GPS lock and arm the copter
// Can be reset each the copter is re-armed
static struct   Location home;
// Current location of the copter
static struct   Location current_loc;
// Holds the current loaded command from the EEPROM for navigation
static struct   Location command_nav_queue;
// Holds the current loaded command from the EEPROM for conditional scripts
static struct   Location command_cond_queue;


////////////////////////////////////////////////////////////////////////////////
// Navigation Roll/Pitch functions
////////////////////////////////////////////////////////////////////////////////
// all angles are deg * 100 : target yaw angle
// The Commanded ROll from the autopilot.
static int32_t nav_roll;
// The Commanded pitch from the autopilot. negative Pitch means go forward.
static int32_t nav_pitch;

// The Commanded ROll from the autopilot based on optical flow sensor.
static int32_t of_roll;
// The Commanded pitch from the autopilot based on optical flow sensor. negative Pitch means go forward.
static int32_t of_pitch;


////////////////////////////////////////////////////////////////////////////////
// Navigation Throttle control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Throttle from the autopilot.
static int16_t nav_throttle;    // 0-1000 for throttle control
// This is a simple counter to track the amount of throttle used during flight
// This could be useful later in determining and debuging current usage and predicting battery life
static uint32_t throttle_integrator;


////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// The Commanded Yaw from the autopilot.
static int32_t nav_yaw;
static uint8_t yaw_timer;
// Yaw will point at this location if yaw_mode is set to YAW_LOOK_AT_LOCATION
static Vector3f yaw_look_at_WP;
// bearing from current location to the yaw_look_at_WP
static int32_t yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
static int32_t yaw_look_at_heading;
// Deg/s we should turn
static int16_t yaw_look_at_heading_slew;



////////////////////////////////////////////////////////////////////////////////
// Repeat Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
// The type of repeating event - Toggle a servo channel, Toggle the APM1 relay, etc
static uint8_t event_id;
// Used to manage the timimng of repeating events
static uint32_t event_timer;
// How long to delay the next firing of event in millis
static uint16_t event_delay;
// how many times to fire : 0 = forever, 1 = do once, 2 = do twice
static int16_t event_repeat;
// per command value, such as PWM for servos
static int16_t event_value;
// the stored value used to undo commands - such as original PWM command
static int16_t event_undo_value;

////////////////////////////////////////////////////////////////////////////////
// Delay Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
static int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
static uint32_t condition_start;


////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
static AP_InertialNav inertial_nav(&ahrs, &ins, &barometer, &g_gps);

////////////////////////////////////////////////////////////////////////////////
// Waypoint navigation object
// To-Do: move inertial nav up or other navigation variables down here
////////////////////////////////////////////////////////////////////////////////
static AC_WPNav wp_nav(&inertial_nav, &ahrs, &g.pi_loiter_lat, &g.pi_loiter_lon, &g.pid_loiter_rate_lat, &g.pid_loiter_rate_lon);

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
// The number of GPS fixes we have had
static uint8_t gps_fix_count;
static int16_t pmTest1;

// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counters for branching from 10 hz control loop
static uint8_t medium_loopCounter;
// Counters for branching from 3 1/3hz control loop
static uint8_t slow_loopCounter;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// Delta Time in milliseconds for navigation computations, updated with every good GPS read
static float dTnav;
// Counters for branching from 4 minute control loop used to save Compass offsets
static int16_t superslow_loopCounter;
// Loiter timer - Records how long we have been in loiter
static uint32_t rtl_loiter_start_time;
// prevents duplicate GPS messages from entering system
static uint32_t last_gps_time;
// the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
static uint32_t last_heartbeat_ms;

// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

// Reference to the relay object (APM1 -> PORTL 2) (APM2 -> PORTB 7)
static AP_Relay relay;

//Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
  static AP_Camera camera(&relay);
#endif

// a pin for reading the receiver RSSI voltage.
static AP_HAL::AnalogSource* rssi_analog_source;


// Input sources for battery voltage, battery current, board vcc
static AP_HAL::AnalogSource* batt_volt_analog_source;
static AP_HAL::AnalogSource* batt_curr_analog_source;
static AP_HAL::AnalogSource* board_vcc_analog_source;


#if CLI_ENABLED == ENABLED
    static int8_t   setup_show (uint8_t argc, const Menu::arg *argv);
#endif

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
static AP_Mount camera_mount(&current_loc, g_gps, &ahrs, 0);
#endif

#if MOUNT2 == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
// mabe one could use current_loc for lat/lon too and eliminate g_gps alltogether?
static AP_Mount camera_mount2(&current_loc, g_gps, &ahrs, 1);
#endif

////////////////////////////////////////////////////////////////////////////////
// AC_Fence library to reduce fly-aways
////////////////////////////////////////////////////////////////////////////////
#if AC_FENCE == ENABLED
AC_Fence    fence(&inertial_nav);
#endif

////////////////////////////////////////////////////////////////////////////////
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
void get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate);
static void pre_arm_checks(bool display_failure);

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info, WP_START_BYTE);

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { update_GPS,            2,     900 },
    { update_navigation,     10,    500 },
    { medium_loop,           2,     700 },
    { update_altitude,      10,    1000 },
    { fifty_hz_loop,         2,     950 },
    { run_nav_updates,      10,     800 },
    { slow_loop,            10,     500 },
    { gcs_check_input,	     2,     700 },
    { gcs_send_heartbeat,  100,     700 },
    { gcs_data_stream_send,  2,    1500 },
    { gcs_send_deferred,     2,    1200 },
    { compass_accumulate,    2,     700 },
    { barometer_accumulate,  2,     900 },
    { super_slow_loop,     100,    1100 },
    { perf_update,        1000,     500 }
};


void setup() {
    // this needs to be the first call, as it fills memory with
    // sentinel values
    memcheck_init();
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

#if CONFIG_SONAR == ENABLED
 #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar_analog_source = new AP_ADC_AnalogSource(
            &adc, CONFIG_SONAR_SOURCE_ADC_CHANNEL, 0.25);
 #elif CONFIG_SONAR_SOURCE == SONAR_SOURCE_ANALOG_PIN
    sonar_analog_source = hal.analogin->channel(
            CONFIG_SONAR_SOURCE_ANALOG_PIN);
 #else
  #warning "Invalid CONFIG_SONAR_SOURCE"
 #endif
    sonar = new AP_RangeFinder_MaxsonarXL(sonar_analog_source,
            &sonar_mode_filter);
#endif

    rssi_analog_source      = hal.analogin->channel(g.rssi_pin);
    batt_volt_analog_source = hal.analogin->channel(g.battery_volt_pin);
    batt_curr_analog_source = hal.analogin->channel(g.battery_curr_pin);
    board_vcc_analog_source = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }    
}

/*
  try to accumulate a baro reading
 */
static void barometer_accumulate(void)
{
    barometer.accumulate();
}

// enable this to get console logging of scheduler performance
#define SCHEDULER_DEBUG 0

static void perf_update(void)
{
    if (g.log_bitmask & MASK_LOG_PM)
        Log_Write_Performance();
    if (scheduler.debug()) {
        cliSerial->printf_P(PSTR("PERF: %u/%u %lu\n"), 
                            (unsigned)perf_info_get_num_long_running(),
                            (unsigned)perf_info_get_num_loops(),
                            (unsigned long)perf_info_get_max_time());
    }
    perf_info_reset();
    gps_fix_count = 0;
    pmTest1 = 0;
}

void loop()
{
    uint32_t timer = micros();

    // We want this to execute fast
    // ----------------------------
    if (ins.num_samples_available() >= 2) {

        // check loop time
        perf_info_check_loop_time(timer - fast_loopTimer);

        G_Dt                            = (float)(timer - fast_loopTimer) / 1000000.f;                  // used by PI Loops
        fast_loopTimer          = timer;

        // for mainloop failure monitoring
        mainLoop_count++;

        // Execute the fast loop
        // ---------------------
        fast_loop();

        // tell the scheduler one tick has passed
        scheduler.tick();
    } else {
        uint16_t dt = timer - fast_loopTimer;
        if (dt < 10000) {
            uint16_t time_to_next_loop = 10000 - dt;
            scheduler.run(time_to_next_loop);
        }
    }
}


// Main loop - 100hz
static void fast_loop()
{
    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // reads all of the necessary trig functions for cameras, throttle, etc.
    // --------------------------------------------------------------------
    update_trig();

	// Acrobatic control
    if (ap.do_flip) {
        if(abs(g.rc_1.control_in) < 4000) {
            // calling roll_flip will override the desired roll rate and throttle output
            roll_flip();
        }else{
            // force an exit from the loop if we are not hands off sticks.
            ap.do_flip = false;
            Log_Write_Event(DATA_EXIT_FLIP);
        }
    }

    // run low level rate controllers that only require IMU data
    run_rate_controllers();

    // write out the servo PWM values
    // ------------------------------
    set_servos_4();

    // Inertial Nav
    // --------------------
    read_inertia();

    // optical flow
    // --------------------
#if OPTFLOW == ENABLED
    if(g.optflow_enabled) {
        update_optical_flow();
    }
#endif  // OPTFLOW == ENABLED

    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();

    // custom code/exceptions for flight modes
    // ---------------------------------------
    update_yaw_mode();
    update_roll_pitch_mode();

    // update targets to rate controllers
    update_rate_contoller_targets();

    // agmatthews - USERHOOKS
#ifdef USERHOOK_FASTLOOP
    USERHOOK_FASTLOOP
#endif
}

// stuff that happens at 50 hz
// ---------------------------
static void fifty_hz_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

#if TOY_EDF == ENABLED
    edf_toy();
#endif

    // check auto_armed status
    update_auto_armed();

#ifdef USERHOOK_50HZLOOP
    USERHOOK_50HZLOOP
#endif


#if HIL_MODE != HIL_MODE_DISABLED && FRAME_CONFIG != HELI_FRAME
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update_mount_position();
#endif

#if MOUNT2 == ENABLED
    // update camera mount's position
    camera_mount2.update_mount_position();
#endif

#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif

# if HIL_MODE == HIL_MODE_DISABLED
    if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST && motors.armed()) {
        Log_Write_Attitude();
#if SECONDARY_DMP_ENABLED == ENABLED
        Log_Write_DMP();
#endif
    }

    if (g.log_bitmask & MASK_LOG_IMU && motors.armed())
        DataFlash.Log_Write_IMU(&ins);
#endif

}

// medium_loop - runs at 10hz
static void medium_loop()
{
    // This is the start of the medium (10 Hz) loop pieces
    // -----------------------------------------
    switch(medium_loopCounter) {

    // This case reads from the battery and Compass
    //---------------------------------------------
    case 0:
        medium_loopCounter++;

        // read battery before compass because it may be used for motor interference compensation
        if (g.battery_monitoring != 0) {
            read_battery();
        }

#if HIL_MODE != HIL_MODE_ATTITUDE                                                               // don't execute in HIL mode
        if(g.compass_enabled) {
            if (compass.read()) {
                compass.null_offsets();
            }
            // log compass information
            if (motors.armed() && (g.log_bitmask & MASK_LOG_COMPASS)) {
                Log_Write_Compass();
            }
        }
#endif

        // record throttle output
        // ------------------------------
        throttle_integrator += g.rc_3.servo_out;
        break;

    // This case reads rssi information and performs auto trim
    //--------------------------------------------------------
    case 1:
        medium_loopCounter++;

        // read receiver rssi information
        read_receiver_rssi();

        // auto_trim - stores roll and pitch radio inputs to ahrs
        auto_trim();
        break;

    // This case deals with aux switches and toy mode's throttle
    //----------------------------------------------------------
    case 2:
        medium_loopCounter++;

        // check ch7 and ch8 aux switches
        read_aux_switches();

        if(control_mode == TOY_A) {
            update_toy_throttle();

            if(throttle_mode == THROTTLE_AUTO) {
                update_toy_altitude();
            }
        }
        break;

    // This case deals with logging attitude and motor information to dataflash
    // ------------------------------------------------------------------------
    case 3:
        medium_loopCounter++;

        if(motors.armed()) {
            if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) {
                Log_Write_Attitude();
#if SECONDARY_DMP_ENABLED == ENABLED
                Log_Write_DMP();
#endif
            }
            if (g.log_bitmask & MASK_LOG_MOTORS)
                Log_Write_Motors();
        }
        break;

    // This case deals with arming checks, copter LEDs and 10hz user hooks
    // -------------------------------------------------------------------------------
    case 4:
        medium_loopCounter = 0;

        // Check for motor arming or disarming
        arm_motors_check();

        // agmatthews - USERHOOKS
#ifdef USERHOOK_MEDIUMLOOP
        USERHOOK_MEDIUMLOOP
#endif

#if COPTER_LEDS == ENABLED
        update_copter_leds();
#endif
        break;

    default:
        // this is just a catch all
        // ------------------------
        medium_loopCounter = 0;
        break;
    }
}

// slow_loop - 3.3hz loop
static void slow_loop()
{
    // This is the slow (3 1/3 Hz) loop pieces
    //----------------------------------------
    switch (slow_loopCounter) {
    case 0:
        slow_loopCounter++;
        superslow_loopCounter++;

        // check if we've lost contact with the ground station
        failsafe_gcs_check();

        // record if the compass is healthy
        set_compass_healthy(compass.healthy);

        if(superslow_loopCounter > 1200) {
#if HIL_MODE != HIL_MODE_ATTITUDE
            if(g.rc_3.control_in == 0 && control_mode == STABILIZE && g.compass_enabled) {
                compass.save_offsets();
                superslow_loopCounter = 0;
            }
#endif
        }

        if(!motors.armed()) {
            // check the user hasn't updated the frame orientation
            motors.set_frame_orientation(g.frame_orientation);
        }

#if AC_FENCE == ENABLED
        // check if we have breached a fence
        fence_check();
#endif // AC_FENCE_ENABLED

        break;

    case 1:
        slow_loopCounter++;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
        update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
#elif MOUNT == ENABLED
        update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
#endif
        enable_aux_servos();

#if MOUNT == ENABLED
        camera_mount.update_mount_type();
#endif

#if MOUNT2 == ENABLED
        camera_mount2.update_mount_type();
#endif

        // agmatthews - USERHOOKS
#ifdef USERHOOK_SLOWLOOP
        USERHOOK_SLOWLOOP
#endif

        break;

    case 2:
        slow_loopCounter = 0;
        update_events();

        // blink if we are armed
        update_lights();

        if(g.radio_tuning > 0)
            tuning();

#if USB_MUX_PIN > 0
        check_usb_mux();
#endif
        break;

    default:
        slow_loopCounter = 0;
        break;
    }
}

// super_slow_loop - runs at 1Hz
static void super_slow_loop()
{
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    // log battery info to the dataflash
    if ((g.log_bitmask & MASK_LOG_CURRENT) && motors.armed())
        Log_Write_Current();

    // perform pre-arm checks
    pre_arm_checks(false);

    // auto disarm checks
    auto_disarm_check();

    // agmatthews - USERHOOKS
#ifdef USERHOOK_SUPERSLOWLOOP
    USERHOOK_SUPERSLOWLOOP
#endif
}

// called at 100hz but data from sensor only arrives at 20 Hz
#if OPTFLOW == ENABLED
static void update_optical_flow(void)
{
    static uint32_t last_of_update = 0;
    static uint8_t of_log_counter = 0;

    // if new data has arrived, process it
    if( optflow.last_update != last_of_update ) {
        last_of_update = optflow.last_update;
        optflow.update_position(ahrs.roll, ahrs.pitch, sin_yaw, cos_yaw, current_loc.alt);      // updates internal lon and lat with estimation based on optical flow

        // write to log at 5hz
        of_log_counter++;
        if( of_log_counter >= 4 ) {
            of_log_counter = 0;
            if (g.log_bitmask & MASK_LOG_OPTFLOW) {
                Log_Write_Optflow();
            }
        }
    }
}
#endif  // OPTFLOW == ENABLED

// called at 50hz
static void update_GPS(void)
{
    // A counter that is used to grab at least 10 reads before commiting the Home location
    static uint8_t ground_start_count  = 10;

    g_gps->update();
    update_GPS_light();

    set_gps_healthy(g_gps->status() >= GPS::GPS_OK_FIX_3D);

    if (g_gps->new_data && last_gps_time != g_gps->time && g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        // clear new data flag
        g_gps->new_data = false;

        // save GPS time so we don't get duplicate reads
        last_gps_time = g_gps->time;

        // log location if we have at least a 2D fix
        if (g.log_bitmask & MASK_LOG_GPS && motors.armed()) {
            DataFlash.Log_Write_GPS(g_gps, current_loc.alt);
        }

        // for performance monitoring
        gps_fix_count++;

        // check if we can initialise home yet
        if (!ap.home_is_set) {
            // if we have a 3d lock and valid location
            if(g_gps->status() >= GPS::GPS_OK_FIX_3D && g_gps->latitude != 0) {
                if( ground_start_count > 0 ) {
                    ground_start_count--;
                }else{
                    // after 10 successful reads store home location
                    // ap.home_is_set will be true so this will only happen once
                    ground_start_count = 0;
                    init_home();
                    if (g.compass_enabled) {
                        // Set compass declination automatically
                        compass.set_initial_location(g_gps->latitude, g_gps->longitude);
                    }
                }
            }else{
                // start again if we lose 3d lock
                ground_start_count = 10;
            }
        }
    }

    // check for loss of gps
    failsafe_gps_check();
}

// set_yaw_mode - update yaw mode and initialise any variables required
bool set_yaw_mode(uint8_t new_yaw_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool yaw_initialised = false;

    // return immediately if no change
    if( new_yaw_mode == yaw_mode ) {
        return true;
    }

    switch( new_yaw_mode ) {
        case YAW_HOLD:
        case YAW_ACRO:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_NEXT_WP:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_LOCATION:
            if( ap.home_is_set ) {
                // update bearing - assumes yaw_look_at_WP has been intialised before set_yaw_mode was called
                yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
                yaw_initialised = true;
            }
            break;
        case YAW_CIRCLE:
            if( ap.home_is_set ) {
                // set yaw to point to center of circle
                yaw_look_at_WP = circle_center;
                // initialise bearing to current heading
                yaw_look_at_WP_bearing = ahrs.yaw_sensor;
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AT_HEADING:
            yaw_initialised = true;
            break;
        case YAW_LOOK_AT_HOME:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_LOOK_AHEAD:
            if( ap.home_is_set ) {
                yaw_initialised = true;
            }
            break;
        case YAW_TOY:
            yaw_initialised = true;
            break;
        case YAW_RESETTOARMEDYAW:
            nav_yaw = ahrs.yaw_sensor; // store current yaw so we can start rotating back to correct one
            yaw_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( yaw_initialised ) {
        yaw_mode = new_yaw_mode;
    }

    // return success or failure
    return yaw_initialised;
}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void update_yaw_mode(void)
{
    switch(yaw_mode) {

    case YAW_HOLD:
        // heading hold at heading held in nav_yaw but allow input from pilot
        get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        break;

    case YAW_ACRO:
        // pilot controlled yaw using rate controller
        if(g.axis_enabled) {
            get_yaw_rate_stabilized_ef(g.rc_4.control_in);
        }else{
            get_acro_yaw(g.rc_4.control_in);
        }
        break;

    case YAW_LOOK_AT_NEXT_WP:
        // point towards next waypoint (no pilot input accepted)
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        nav_yaw = get_yaw_slew(nav_yaw, original_wp_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_LOCATION:
        // point towards a location held in yaw_look_at_WP
        get_look_at_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_CIRCLE:
        // points toward the center of the circle or does a panorama
        get_circle_yaw();

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HOME:
        // keep heading always pointing at home with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }
        break;

    case YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_heading, yaw_look_at_heading_slew);
        get_stabilize_yaw(nav_yaw);
        break;

	case YAW_LOOK_AHEAD:
		// Commanded Yaw to automatically look ahead.
        get_look_ahead_yaw(g.rc_4.control_in);
        break;

#if TOY_LOOKUP == TOY_EXTERNAL_MIXER
    case YAW_TOY:
        // update to allow external roll/yaw mixing
        // keep heading always pointing at home with no pilot input allowed
        nav_yaw = get_yaw_slew(nav_yaw, home_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);
        break;
#endif

    case YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        nav_yaw = get_yaw_slew(nav_yaw, initial_simple_bearing, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(nav_yaw);

        // if there is any pilot input, switch to YAW_HOLD mode for the next iteration
        if( g.rc_4.control_in != 0 ) {
            set_yaw_mode(YAW_HOLD);
        }

        break;
    }
}

// get yaw mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t get_wp_yaw_mode(bool rtl)
{
    switch (g.wp_yaw_behavior) {
        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
            return YAW_LOOK_AT_NEXT_WP;
            break;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
            if( rtl ) {
                return YAW_HOLD;
            }else{
                return YAW_LOOK_AT_NEXT_WP; 
            }
            break;

        case WP_YAW_BEHAVIOR_LOOK_AHEAD:
            return YAW_LOOK_AHEAD;
            break;

        default:
            return YAW_HOLD;
            break;
    }
}

// set_roll_pitch_mode - update roll/pitch mode and initialise any variables as required
bool set_roll_pitch_mode(uint8_t new_roll_pitch_mode)
{
    // boolean to ensure proper initialisation of throttle modes
    bool roll_pitch_initialised = false;

    // return immediately if no change
    if( new_roll_pitch_mode == roll_pitch_mode ) {
        return true;
    }

    switch( new_roll_pitch_mode ) {
        case ROLL_PITCH_STABLE:
        case ROLL_PITCH_ACRO:
        case ROLL_PITCH_AUTO:
        case ROLL_PITCH_STABLE_OF:
        case ROLL_PITCH_TOY:
            roll_pitch_initialised = true;
            break;

        case ROLL_PITCH_LOITER:
            // require gps lock
            if( ap.home_is_set ) {
                roll_pitch_initialised = true;
            }
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( roll_pitch_initialised ) {
        roll_pitch_mode = new_roll_pitch_mode;
    }

    // return success or failure
    return roll_pitch_initialised;
}

// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
void update_roll_pitch_mode(void)
{
    switch(roll_pitch_mode) {
    case ROLL_PITCH_ACRO:
        // copy user input for reporting purposes
        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

#if FRAME_CONFIG == HELI_FRAME
		if(g.axis_enabled) {
            get_roll_rate_stabilized_ef(g.rc_1.control_in);
            get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        }else{
            // ACRO does not get SIMPLE mode ability
            if (motors.flybar_mode == 1) {
                g.rc_1.servo_out = g.rc_1.control_in;
                g.rc_2.servo_out = g.rc_2.control_in;
            } else {
                get_acro_roll(g.rc_1.control_in);
                get_acro_pitch(g.rc_2.control_in);
            }
		}
#else  // !HELI_FRAME
		if(g.axis_enabled) {
            get_roll_rate_stabilized_ef(g.rc_1.control_in);
            get_pitch_rate_stabilized_ef(g.rc_2.control_in);
        }else{
            // ACRO does not get SIMPLE mode ability
            get_acro_roll(g.rc_1.control_in);
            get_acro_pitch(g.rc_2.control_in);
		}
#endif  // HELI_FRAME
        break;

    case ROLL_PITCH_STABLE:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }

        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        get_stabilize_roll(control_roll);
        get_stabilize_pitch(control_pitch);

        break;

    case ROLL_PITCH_AUTO:
        // copy latest output from nav controller to stabilize controller
        nav_roll = wp_nav.get_desired_roll();
        nav_pitch = wp_nav.get_desired_pitch();
        get_stabilize_roll(nav_roll);
        get_stabilize_pitch(nav_pitch);

        // copy control_roll and pitch for reporting purposes
        control_roll = nav_roll;
        control_pitch = nav_pitch;
        break;

    case ROLL_PITCH_STABLE_OF:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }

        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        // mix in user control with optical flow
        get_stabilize_roll(get_of_roll(control_roll));
        get_stabilize_pitch(get_of_pitch(control_pitch));
        break;

    // THOR
    // a call out to the main toy logic
    case ROLL_PITCH_TOY:
        roll_pitch_toy();
        break;

    case ROLL_PITCH_LOITER:
        // apply SIMPLE mode transform
        if(ap.simple_mode && ap_system.new_radio_frame) {
            update_simple_mode();
        }
        // copy user input for logging purposes
        control_roll            = g.rc_1.control_in;
        control_pitch           = g.rc_2.control_in;

        // update loiter target from user controls - max velocity is 5.0 m/s
        wp_nav.move_loiter_target(control_roll, control_pitch,0.01f);

        // copy latest output from nav controller to stabilize controller
        nav_roll = wp_nav.get_desired_roll();
        nav_pitch = wp_nav.get_desired_pitch();
        get_stabilize_roll(nav_roll);
        get_stabilize_pitch(nav_pitch);
        break;
    }

	#if FRAME_CONFIG != HELI_FRAME
    if(g.rc_3.control_in == 0 && control_mode <= ACRO) {
        reset_rate_I();
        reset_stability_I();
    }
	#endif //HELI_FRAME

    if(ap_system.new_radio_frame) {
        // clear new radio frame info
        ap_system.new_radio_frame = false;
    }
}

// new radio frame is used to make sure we only call this at 50hz
void update_simple_mode(void)
{
    static uint8_t simple_counter = 0;             // State machine counter for Simple Mode
    static float simple_sin_y=0, simple_cos_x=0;

    // used to manage state machine
    // which improves speed of function
    simple_counter++;

    int16_t delta = wrap_360_cd(ahrs.yaw_sensor - initial_simple_bearing)/100;

    if (simple_counter == 1) {
        // roll
        simple_cos_x = sinf(radians(90 - delta));

    }else if (simple_counter > 2) {
        // pitch
        simple_sin_y = cosf(radians(90 - delta));
        simple_counter = 0;
    }

    // Rotate input by the initial bearing
    int16_t _roll   = g.rc_1.control_in * simple_cos_x + g.rc_2.control_in * simple_sin_y;
    int16_t _pitch  = -(g.rc_1.control_in * simple_sin_y - g.rc_2.control_in * simple_cos_x);

    g.rc_1.control_in = _roll;
    g.rc_2.control_in = _pitch;
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void update_super_simple_bearing()
{
    // are we in SIMPLE mode?
    if(ap.simple_mode && g.super_simple) {
        // get distance to home
        if(home_distance > SUPER_SIMPLE_RADIUS) {        // 10m from home
            // we reset the angular offset to be a vector from home to the quad
            initial_simple_bearing = wrap_360_cd(home_bearing+18000);
        }
    }
}

// set_throttle_mode - sets the throttle mode and initialises any variables as required
bool set_throttle_mode( uint8_t new_throttle_mode )
{
    // boolean to ensure proper initialisation of throttle modes
    bool throttle_initialised = false;

    // return immediately if no change
    if( new_throttle_mode == throttle_mode ) {
        return true;
    }

    // initialise any variables required for the new throttle mode
    switch(new_throttle_mode) {
        case THROTTLE_MANUAL:
        case THROTTLE_MANUAL_TILT_COMPENSATED:
            throttle_accel_deactivate();                // this controller does not use accel based throttle controller
            altitude_error = 0;                         // clear altitude error reported to GCS
            throttle_initialised = true;
            break;

        case THROTTLE_HOLD:
        case THROTTLE_AUTO:
            controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);     // reset controller desired altitude to current altitude
            wp_nav.set_desired_alt(controller_desired_alt);                                 // same as above but for loiter controller
            if ( throttle_mode <= THROTTLE_MANUAL_TILT_COMPENSATED ) {      // reset the alt hold I terms if previous throttle mode was manual
                reset_throttle_I();
                set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
            }
            throttle_initialised = true;
            break;

        case THROTTLE_LAND:
            set_land_complete(false);   // mark landing as incomplete
            land_detector = 0;          // A counter that goes up if our climb rate stalls out.
            controller_desired_alt = get_initial_alt_hold(current_loc.alt, climb_rate);   // reset controller desired altitude to current altitude
            throttle_initialised = true;
            break;

        default:
            // To-Do: log an error message to the dataflash or tlogs instead of printing to the serial port
            cliSerial->printf_P(PSTR("Unsupported throttle mode: %d!!"),new_throttle_mode);
            break;
    }

    // update the throttle mode
    if( throttle_initialised ) {
        throttle_mode = new_throttle_mode;

        // reset some variables used for logging
        desired_climb_rate = 0;
        nav_throttle = 0;
    }

    // return success or failure
    return throttle_initialised;
}

// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
void update_throttle_mode(void)
{
    int16_t pilot_climb_rate;
    int16_t pilot_throttle_scaled;

    if(ap.do_flip)     // this is pretty bad but needed to flip in AP modes.
        return;

    // do not run throttle controllers if motors disarmed
    if( !motors.armed() ) {
        set_throttle_out(0, false);
        throttle_accel_deactivate();    // do not allow the accel based throttle to override our command
        set_target_alt_for_reporting(0);
        return;
    }

#if FRAME_CONFIG == HELI_FRAME
	if (control_mode == STABILIZE){
		motors.stab_throttle = true;
	} else {
		motors.stab_throttle = false;
	}
#endif // HELI_FRAME

    switch(throttle_mode) {

    case THROTTLE_MANUAL:
        // completely manual throttle
        if(g.rc_3.control_in <= 0){
            set_throttle_out(0, false);
        }else{
            // send pilot's output directly to motors
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, false);

            // update estimate of throttle cruise
			#if FRAME_CONFIG == HELI_FRAME
            update_throttle_cruise(motors.coll_out);
			#else
			update_throttle_cruise(pilot_throttle_scaled);
			#endif  //HELI_FRAME


            // check if we've taken off yet
            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_MANUAL_TILT_COMPENSATED:
        // manual throttle but with angle boost
        if (g.rc_3.control_in <= 0) {
            set_throttle_out(0, false); // no need for angle boost with zero throttle
        }else{
            pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);
            set_throttle_out(pilot_throttle_scaled, true);

            // update estimate of throttle cruise
            #if FRAME_CONFIG == HELI_FRAME
            update_throttle_cruise(motors.coll_out);
			#else
			update_throttle_cruise(pilot_throttle_scaled);
			#endif  //HELI_FRAME

            if (!ap.takeoff_complete && motors.armed()) {
                if (pilot_throttle_scaled > g.throttle_cruise) {
                    // we must be in the air by now
                    set_takeoff_complete(true);
                }
            }
        }
        set_target_alt_for_reporting(0);
        break;

    case THROTTLE_HOLD:
        // alt hold plus pilot input of climb rate
        pilot_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
        if( sonar_alt_health >= SONAR_ALT_HEALTH_MAX ) {
            // if sonar is ok, use surface tracking
            get_throttle_surface_tracking(pilot_climb_rate);    // this function calls set_target_alt_for_reporting for us
        }else{
            // if no sonar fall back stabilize rate controller
            get_throttle_rate_stabilized(pilot_climb_rate);     // this function calls set_target_alt_for_reporting for us
        }
        break;

    case THROTTLE_AUTO:
        // auto pilot altitude controller with target altitude held in wp_nav.get_desired_alt()
        if(ap.auto_armed) {
            get_throttle_althold_with_slew(wp_nav.get_desired_alt(), -wp_nav.get_descent_velocity(), wp_nav.get_climb_velocity());
            set_target_alt_for_reporting(wp_nav.get_desired_alt()); // To-Do: return get_destination_alt if we are flying to a waypoint
        }
        // To-Do: explicitly set what the throttle output should be (probably min throttle).  Without setting it the throttle is simply left in it's last position although that is probably zero throttle anyway
        break;

    case THROTTLE_LAND:
        // landing throttle controller
        get_throttle_land();
        set_target_alt_for_reporting(0);
        break;
    }
}

// set_target_alt_for_reporting - set target altitude in cm for reporting purposes (logs and gcs)
static void set_target_alt_for_reporting(float alt_cm)
{
    target_alt_for_reporting = alt_cm;
}

// get_target_alt_for_reporting - returns target altitude in cm for reporting purposes (logs and gcs)
static float get_target_alt_for_reporting()
{
    return target_alt_for_reporting;
}

static void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
    omega = ins.get_gyro();

#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.update();
#endif
}

static void update_trig(void){
    Vector2f yawvector;
    const Matrix3f &temp   = ahrs.get_dcm_matrix();

    yawvector.x     = temp.a.x;     // sin
    yawvector.y     = temp.b.x;         // cos
    yawvector.normalize();

    cos_pitch_x     = safe_sqrt(1 - (temp.c.x * temp.c.x));     // level = 1
    cos_roll_x      = temp.c.z / cos_pitch_x;                       // level = 1

    cos_pitch_x     = constrain_float(cos_pitch_x, 0, 1.0);
    // this relies on constrain_float() of infinity doing the right thing,
    // which it does do in avr-libc
    cos_roll_x      = constrain_float(cos_roll_x, -1.0, 1.0);

    sin_yaw         = constrain_float(yawvector.y, -1.0, 1.0);
    cos_yaw         = constrain_float(yawvector.x, -1.0, 1.0);

    // added to convert earth frame to body frame for rate controllers
    sin_pitch       = -temp.c.x;
    sin_roll        = temp.c.y / cos_pitch_x;

    // update wp_nav controller with trig values
    wp_nav.set_cos_sin_yaw(cos_yaw, sin_yaw, cos_pitch_x);

    //flat:
    // 0 ° = cos_yaw:  1.00, sin_yaw:  0.00,
    // 90° = cos_yaw:  0.00, sin_yaw:  1.00,
    // 180 = cos_yaw: -1.00, sin_yaw:  0.00,
    // 270 = cos_yaw:  0.00, sin_yaw: -1.00,
}

// read baro and sonar altitude at 20hz
static void update_altitude()
{
#if HIL_MODE == HIL_MODE_ATTITUDE
    // we are in the SIM, fake out the baro and Sonar
    baro_alt                = g_gps->altitude - gps_base_alt;

    if(g.sonar_enabled) {
        sonar_alt           = baro_alt;
    }
#else
    // read in baro altitude
    baro_alt            = read_barometer();

    // read in sonar altitude
    sonar_alt           = read_sonar();
#endif  // HIL_MODE == HIL_MODE_ATTITUDE

    // write altitude info to dataflash logs
    if ((g.log_bitmask & MASK_LOG_CTUN) && motors.armed()) {
        Log_Write_Control_Tuning();
    }
}

static void tuning(){
    tuning_value = (float)g.rc_6.control_in / 1000.0f;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);                   // 0 to 1

    switch(g.radio_tuning) {

    case CH6_RATE_KD:
        g.pid_rate_roll.kD(tuning_value);
        g.pid_rate_pitch.kD(tuning_value);
        break;

    case CH6_STABILIZE_KP:
        g.pi_stabilize_roll.kP(tuning_value);
        g.pi_stabilize_pitch.kP(tuning_value);
        break;

    case CH6_STABILIZE_KI:
        g.pi_stabilize_roll.kI(tuning_value);
        g.pi_stabilize_pitch.kI(tuning_value);
        break;

    case CH6_ACRO_KP:
        g.acro_p = tuning_value;
        break;

    case CH6_RATE_KP:
        g.pid_rate_roll.kP(tuning_value);
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_KI:
        g.pid_rate_roll.kI(tuning_value);
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_YAW_KP:
        g.pi_stabilize_yaw.kP(tuning_value);
        break;

    case CH6_YAW_KI:
        g.pi_stabilize_yaw.kI(tuning_value);
        break;

    case CH6_YAW_RATE_KP:
        g.pid_rate_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KD:
        g.pid_rate_yaw.kD(tuning_value);
        break;

    case CH6_THROTTLE_KP:
        g.pid_throttle.kP(tuning_value);
        break;

    case CH6_THROTTLE_KI:
        g.pid_throttle.kI(tuning_value);
        break;

    case CH6_THROTTLE_KD:
        g.pid_throttle.kD(tuning_value);
        break;

    case CH6_RELAY:
        if (g.rc_6.control_in > 525) relay.on();
        if (g.rc_6.control_in < 475) relay.off();
        break;

    case CH6_WP_SPEED:
        // set waypoint navigation horizontal speed to 0 ~ 1000 cm/s
        wp_nav.set_horizontal_velocity(g.rc_6.control_in);
        break;

    case CH6_LOITER_KP:
        g.pi_loiter_lat.kP(tuning_value);
        g.pi_loiter_lon.kP(tuning_value);
        break;

    case CH6_LOITER_KI:
        g.pi_loiter_lat.kI(tuning_value);
        g.pi_loiter_lon.kI(tuning_value);
        break;

    case CH6_LOITER_RATE_KP:
        g.pid_loiter_rate_lon.kP(tuning_value);
        g.pid_loiter_rate_lat.kP(tuning_value);
        break;

    case CH6_LOITER_RATE_KI:
        g.pid_loiter_rate_lon.kI(tuning_value);
        g.pid_loiter_rate_lat.kI(tuning_value);
        break;

    case CH6_LOITER_RATE_KD:
        g.pid_loiter_rate_lon.kD(tuning_value);
        g.pid_loiter_rate_lat.kD(tuning_value);
        break;

#if FRAME_CONFIG == HELI_FRAME
    case CH6_HELI_EXTERNAL_GYRO:
        motors.ext_gyro_gain = tuning_value;
        break;
#endif

    case CH6_THR_HOLD_KP:
        g.pi_alt_hold.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KP:
        g.pid_optflow_roll.kP(tuning_value);
        g.pid_optflow_pitch.kP(tuning_value);
        break;

    case CH6_OPTFLOW_KI:
        g.pid_optflow_roll.kI(tuning_value);
        g.pid_optflow_pitch.kI(tuning_value);
        break;

    case CH6_OPTFLOW_KD:
        g.pid_optflow_roll.kD(tuning_value);
        g.pid_optflow_pitch.kD(tuning_value);
        break;

#if HIL_MODE != HIL_MODE_ATTITUDE                                       // do not allow modifying _kp or _kp_yaw gains in HIL mode
    case CH6_AHRS_YAW_KP:
        ahrs._kp_yaw.set(tuning_value);
        break;

    case CH6_AHRS_KP:
        ahrs._kp.set(tuning_value);
        break;
#endif

    case CH6_INAV_TC:
        // To-Do: allowing tuning TC for xy and z separately
        inertial_nav.set_time_constant_xy(tuning_value);
        inertial_nav.set_time_constant_z(tuning_value);
        break;

    case CH6_THR_ACCEL_KP:
        g.pid_throttle_accel.kP(tuning_value);
        break;

    case CH6_THR_ACCEL_KI:
        g.pid_throttle_accel.kI(tuning_value);
        break;

    case CH6_THR_ACCEL_KD:
        g.pid_throttle_accel.kD(tuning_value);
        break;

    case CH6_DECLINATION:
        // set declination to +-20degrees
        compass.set_declination(ToRad((2.0f * g.rc_6.control_in - g.radio_tuning_high)/100.0f), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

    case CH6_CIRCLE_RATE:
        // set circle rate
        g.circle_rate.set(g.rc_6.control_in/25-20);     // allow approximately 45 degree turn rate in either direction
        //cliSerial->printf_P(PSTR("\nRate:%4.2f"),(float)g.circle_rate);
        break;
    }
}

AP_HAL_MAIN();

#line 1 "/home/tobias/git/ardupilot/ArduCopter/AP_State.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


void set_home_is_set(bool b)
{
    // if no change, exit immediately
    if( ap.home_is_set == b )
        return;

    ap.home_is_set 	= b;
    if(b) {
        Log_Write_Event(DATA_SET_HOME);
    }
}

// ---------------------------------------------
void set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        Log_Write_Event(DATA_AUTO_ARMED);
    }
}

// ---------------------------------------------
void set_simple_mode(bool b)
{
    if(ap.simple_mode != b){
        if(b){
            Log_Write_Event(DATA_SET_SIMPLE_ON);
        }else{
            Log_Write_Event(DATA_SET_SIMPLE_OFF);
        }
        ap.simple_mode = b;
    }
}

// ---------------------------------------------
static void set_failsafe_radio(bool mode)
{
    // only act on changes
    // -------------------
    if(ap.failsafe_radio != mode) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        ap.failsafe_radio = mode;

        if (ap.failsafe_radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }
    }
}


// ---------------------------------------------
void set_low_battery(bool b)
{
    ap.low_battery = b;
}


// ---------------------------------------------
static void set_failsafe_gps(bool mode)
{
    ap.failsafe_gps = mode;
}

// ---------------------------------------------
static void set_failsafe_gcs(bool mode)
{
    ap.failsafe_gcs = mode;
}

// ---------------------------------------------
void set_takeoff_complete(bool b)
{
    // if no change, exit immediately
    if( ap.takeoff_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_TAKEOFF);
    }
    ap.takeoff_complete = b;
}

// ---------------------------------------------
void set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_LAND_COMPLETE);
    }
    ap.land_complete = b;
}

// ---------------------------------------------

void set_compass_healthy(bool b)
{
    if(ap.compass_status != b) {
        if(b) {
            // compass has just recovered so log to the dataflash
            Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_ERROR_RESOLVED);
        }else{
            // compass has just failed so log an error to the dataflash
            Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_COMPASS_FAILED_TO_READ);
        }
    }
    ap.compass_status = b;
}

void set_gps_healthy(bool b)
{
    if(ap.gps_status != b){
        if(false == b){
            Log_Write_Event(DATA_LOST_GPS);
        }
    }
    ap.gps_status = b;
}

#line 1 "/home/tobias/git/ardupilot/ArduCopter/Attitude.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void
get_stabilize_roll(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180_cd(target_angle - ahrs.roll_sensor);

    // limit the error we're feeding to the PID
    target_angle            = constrain_int32(target_angle, -4500, 4500);

    // convert to desired rate
    int32_t target_rate = g.pi_stabilize_roll.get_pi(target_angle, G_Dt);

    // set targets for rate controller
    set_roll_rate_target(target_rate, EARTH_FRAME);
}

static void
get_stabilize_pitch(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180_cd(target_angle - ahrs.pitch_sensor);

    // limit the error we're feeding to the PID
    target_angle            = constrain_int32(target_angle, -4500, 4500);

    // convert to desired rate
    int32_t target_rate = g.pi_stabilize_pitch.get_pi(target_angle, G_Dt);

    // set targets for rate controller
    set_pitch_rate_target(target_rate, EARTH_FRAME);
}

static void
get_stabilize_yaw(int32_t target_angle)
{
    int32_t target_rate,i_term;
    int32_t angle_error;
    int32_t output = 0;

    // angle error
    angle_error             = wrap_180_cd(target_angle - ahrs.yaw_sensor);

    // limit the error we're feeding to the PID

    angle_error             = constrain_int32(angle_error, -4500, 4500);

    // convert angle error to desired Rate:
    target_rate = g.pi_stabilize_yaw.get_p(angle_error);
    i_term = g.pi_stabilize_yaw.get_i(angle_error, G_Dt);

    // do not use rate controllers for helicotpers with external gyros
#if FRAME_CONFIG == HELI_FRAME
    if(motors.ext_gyro_enabled) {
        g.rc_4.servo_out = constrain_int32((target_rate + i_term), -4500, 4500);
    }
#endif

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && g.radio_tuning == CH6_YAW_KP ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_KP, angle_error, target_rate, i_term, 0, output, tuning_value);
        }
    }
#endif

    // set targets for rate controller
    set_yaw_rate_target(target_rate+i_term, EARTH_FRAME);
}

static void
get_acro_roll(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_roll_rate_target(target_rate, BODY_FRAME);
}

static void
get_acro_pitch(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_pitch_rate_target(target_rate, BODY_FRAME);
}

static void
get_acro_yaw(int32_t target_rate)
{
    target_rate = target_rate * g.acro_p;

    // set targets for rate controller
    set_yaw_rate_target(target_rate, BODY_FRAME);
}

// Roll with rate input and stabilized in the earth frame
static void
get_roll_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired roll rate
    int32_t target_rate = stick_angle * g.acro_p - (roll_axis * g.acro_balance_roll)/100;

    // convert the input to the desired roll rate
    roll_axis += target_rate * G_Dt;
    roll_axis = wrap_180_cd(roll_axis);

    // ensure that we don't reach gimbal lock
    if (labs(roll_axis) > 4500 && g.acro_trainer_enabled) {
        roll_axis	= constrain_int32(roll_axis, -4500, 4500);
        angle_error = wrap_180_cd(roll_axis - ahrs.roll_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180_cd(roll_axis - ahrs.roll_sensor);
        angle_error	= constrain_int32(angle_error, -MAX_ROLL_OVERSHOOT, MAX_ROLL_OVERSHOOT);
    }

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe_radio)) {
        angle_error = 0;
    }

    // update roll_axis to be within max_angle_overshoot of our current heading
    roll_axis = wrap_180_cd(angle_error + ahrs.roll_sensor);

    // set earth frame targets for rate controller

    // set earth frame targets for rate controller
	set_roll_rate_target(g.pi_stabilize_roll.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Pitch with rate input and stabilized in the earth frame
static void
get_pitch_rate_stabilized_ef(int32_t stick_angle)
{
    int32_t angle_error = 0;

    // convert the input to the desired pitch rate
    int32_t target_rate = stick_angle * g.acro_p - (pitch_axis * g.acro_balance_pitch)/100;

    // convert the input to the desired pitch rate
    pitch_axis += target_rate * G_Dt;
    pitch_axis = wrap_180_cd(pitch_axis);

    // ensure that we don't reach gimbal lock
    if (labs(pitch_axis) > 4500) {
        pitch_axis	= constrain_int32(pitch_axis, -4500, 4500);
        angle_error = wrap_180_cd(pitch_axis - ahrs.pitch_sensor);
    } else {
        // angle error with maximum of +- max_angle_overshoot
        angle_error = wrap_180_cd(pitch_axis - ahrs.pitch_sensor);
        angle_error	= constrain_int32(angle_error, -MAX_PITCH_OVERSHOOT, MAX_PITCH_OVERSHOOT);
    }

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe_radio)) {
        angle_error = 0;
    }

    // update pitch_axis to be within max_angle_overshoot of our current heading
    pitch_axis = wrap_180_cd(angle_error + ahrs.pitch_sensor);

    // set earth frame targets for rate controller
	set_pitch_rate_target(g.pi_stabilize_pitch.get_p(angle_error) + target_rate, EARTH_FRAME);
}

// Yaw with rate input and stabilized in the earth frame
static void
get_yaw_rate_stabilized_ef(int32_t stick_angle)
{

    int32_t angle_error = 0;

    // convert the input to the desired yaw rate
    int32_t target_rate = stick_angle * g.acro_p;

    // convert the input to the desired yaw rate
    nav_yaw += target_rate * G_Dt;
    nav_yaw = wrap_360_cd(nav_yaw);

    // calculate difference between desired heading and current heading
    angle_error = wrap_180_cd(nav_yaw - ahrs.yaw_sensor);

    // limit the maximum overshoot
    angle_error	= constrain_int32(angle_error, -MAX_YAW_OVERSHOOT, MAX_YAW_OVERSHOOT);

    if (motors.armed() == false || ((g.rc_3.control_in == 0) && !ap.failsafe_radio)) {
    	angle_error = 0;
    }

    // update nav_yaw to be within max_angle_overshoot of our current heading
    nav_yaw = wrap_360_cd(angle_error + ahrs.yaw_sensor);

    // set earth frame targets for rate controller
	set_yaw_rate_target(g.pi_stabilize_yaw.get_p(angle_error)+target_rate, EARTH_FRAME);
}

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        roll_rate_target_bf = desired_rate;
    }else{
        roll_rate_target_ef = desired_rate;
    }
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        pitch_rate_target_bf = desired_rate;
    }else{
        pitch_rate_target_ef = desired_rate;
    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        yaw_rate_target_bf = desired_rate;
    }else{
        yaw_rate_target_ef = desired_rate;
    }
}

// update_rate_contoller_targets - converts earth frame rates to body frame rates for rate controllers
void
update_rate_contoller_targets()
{
    if( rate_targets_frame == EARTH_FRAME ) {
        // convert earth frame rates to body frame rates
        roll_rate_target_bf 	= roll_rate_target_ef - sin_pitch * yaw_rate_target_ef;
        pitch_rate_target_bf 	= cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef;
        yaw_rate_target_bf 		= cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef;
    }
}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
run_rate_controllers()
{
#if FRAME_CONFIG == HELI_FRAME          // helicopters only use rate controllers for yaw and only when not using an external gyro
    if(!motors.ext_gyro_enabled) {
		g.rc_1.servo_out = get_heli_rate_roll(roll_rate_target_bf);
		g.rc_2.servo_out = get_heli_rate_pitch(pitch_rate_target_bf);
        g.rc_4.servo_out = get_heli_rate_yaw(yaw_rate_target_bf);
    }
#else
    // call rate controllers
    g.rc_1.servo_out = get_rate_roll(roll_rate_target_bf);
    g.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
    g.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);
#endif

    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
    if( g.throttle_accel_enabled && throttle_accel_controller_active ) {
        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
    }
}

#if FRAME_CONFIG == HELI_FRAME
// init_rate_controllers - set-up filters for rate controller inputs
void init_rate_controllers()
{
   // initalise low pass filters on rate controller inputs
   // 1st parameter is time_step, 2nd parameter is time_constant
   rate_roll_filter.set_cutoff_frequency(0.01f, 2.0f);
   rate_pitch_filter.set_cutoff_frequency(0.01f, 2.0f);
   // rate_yaw_filter.set_cutoff_frequency(0.01f, 2.0f);
   // other option for initialisation is rate_roll_filter.set_cutoff_frequency(<time_step>,<cutoff_freq>);
}

static int16_t
get_heli_rate_roll(int32_t target_rate)
{
    int32_t p,i,d,ff;                // used to capture pid values for logging
	int32_t current_rate;			// this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

	// get current rate
	current_rate    = (omega.x * DEGX100);

    // filter input
    current_rate = rate_roll_filter.apply(current_rate);
	
    // call pid controller
    rate_error  = target_rate - current_rate;
    p   = g.pid_rate_roll.get_p(rate_error);

    if (motors.flybar_mode == 1) {												// Mechanical Flybars get regular integral for rate auto trim
		if (target_rate > -50 && target_rate < 50){								// Frozen at high rates
			i		= g.pid_rate_roll.get_i(rate_error, G_Dt);
		} else {
			i		= g.pid_rate_roll.get_integrator();
		}
	} else {
		i			= g.pid_rate_roll.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);		// Flybarless Helis get huge I-terms. I-term controls much of the rate
	}
	
	d = g.pid_rate_roll.get_d(rate_error, G_Dt);
	
	ff = g.heli_roll_ff * target_rate;

    output = p + i + d + ff;

    // constrain output
    output = constrain_int32(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_heli_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d,ff;                                                                   // used to capture pid values for logging
	int32_t current_rate;                                                     			// this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

	// get current rate
	current_rate    = (omega.y * DEGX100);
	
	// filter input
	current_rate = rate_pitch_filter.apply(current_rate);
	
	// call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);						// Helicopters get huge feed-forward
	
	if (motors.flybar_mode == 1) {												// Mechanical Flybars get regular integral for rate auto trim
		if (target_rate > -50 && target_rate < 50){								// Frozen at high rates
			i		= g.pid_rate_pitch.get_i(rate_error, G_Dt);
		} else {
			i		= g.pid_rate_pitch.get_integrator();
		}
	} else {
		i			= g.pid_rate_pitch.get_leaky_i(rate_error, G_Dt, RATE_INTEGRATOR_LEAK_RATE);	// Flybarless Helis get huge I-terms. I-term controls much of the rate
	}
	
	d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
	
	ff = g.heli_pitch_ff*target_rate;
    
    output = p + i + d + ff;

    // constrain output
    output = constrain_int32(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        if( pid_log_counter == 0 ) {               // relies on get_heli_rate_roll to update pid_log_counter
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, 0, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_heli_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d,ff;                                                                   // used to capture pid values for logging
	int32_t current_rate;                                                               // this iteration's rate
    int32_t rate_error;
    int32_t output;

	// get current rate
    current_rate    = (omega.z * DEGX100);
	
	// filter input
	// current_rate = rate_yaw_filter.apply(current_rate);
	
    // rate control
    rate_error              = target_rate - current_rate;

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    
    i = g.pid_rate_yaw.get_i(rate_error, G_Dt);

    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);
	
	ff = g.heli_yaw_ff*target_rate;

    output  = p + i + d + ff;
    output = constrain_float(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_YAW_RATE_KP || g.radio_tuning == CH6_YAW_RATE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
	
#endif

	// output control
	return output;
}
#endif // HELI_FRAME

#if FRAME_CONFIG != HELI_FRAME
static int16_t
get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
    current_rate    = (omega.x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = g.pid_rate_roll.get_p(rate_error);

    // freeze I term if we've breached roll-pitch limits
    if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
        i	= g.pid_rate_roll.get_integrator();
    }else{
        i   = g.pid_rate_roll.get_i(rate_error, G_Dt);
    }

    d = g.pid_rate_roll.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        pid_log_counter++;                          // Note: get_rate_pitch pid logging relies on this function to update pid_log_counter so if you change the line above you must change the equivalent line in get_rate_pitch
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    current_rate    = (omega.y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = g.pid_rate_pitch.get_p(rate_error);
    // freeze I term if we've breached roll-pitch limits
    if( motors.reached_limit(AP_MOTOR_ROLLPITCH_LIMIT) ) {
        i = g.pid_rate_pitch.get_integrator();
    }else{
        i = g.pid_rate_pitch.get_i(rate_error, G_Dt);
    }
    d = g.pid_rate_pitch.get_d(rate_error, G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the rate P, I or D gains
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_RATE_KP || g.radio_tuning == CH6_RATE_KI || g.radio_tuning == CH6_RATE_KD) ) {
        if( pid_log_counter == 0 ) {               // relies on get_rate_roll having updated pid_log_counter
            Log_Write_PID(CH6_RATE_KP+100, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // output control
    return output;
}

static int16_t
get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    rate_error              = target_rate - (omega.z * DEGX100);

    // separately calculate p, i, d values for logging
    p = g.pid_rate_yaw.get_p(rate_error);
    // freeze I term if we've breached yaw limits
    if( motors.reached_limit(AP_MOTOR_YAW_LIMIT) ) {
        i = g.pid_rate_yaw.get_integrator();
    }else{
        i = g.pid_rate_yaw.get_i(rate_error, G_Dt);
    }
    d = g.pid_rate_yaw.get_d(rate_error, G_Dt);

    output  = p+i+d;
    output = constrain_int32(output, -4500, 4500);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && g.radio_tuning == CH6_YAW_RATE_KP ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (100hz / 10hz) = 10
            pid_log_counter = 0;
            Log_Write_PID(CH6_YAW_RATE_KP, rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

#if FRAME_CONFIG == TRI_FRAME
    // constrain output
    return output;
#else // !TRI_FRAME
    // output control:
    int16_t yaw_limit = 2200 + abs(g.rc_4.control_in);

    // smoother Yaw control:
    return constrain_int32(output, -yaw_limit, yaw_limit);
#endif // TRI_FRAME
}
#endif // !HELI_FRAME

// calculate modified roll/pitch depending upon optical flow calculated position
static int32_t
get_of_roll(int32_t input_roll)
{
#if OPTFLOW == ENABLED
    static float tot_x_cm = 0;      // total distance from target
    static uint32_t last_of_roll_update = 0;
    int32_t new_roll = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_roll_update) {
        last_of_roll_update = optflow.last_update;

        // add new distance moved
        tot_x_cm += optflow.x_cm;

        // only stop roll if caller isn't modifying roll
        if( input_roll == 0 && current_loc.alt < 1500) {
            p = g.pid_optflow_roll.get_p(-tot_x_cm);
            i = g.pid_optflow_roll.get_i(-tot_x_cm,1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_roll.get_d(-tot_x_cm,1.0f);
            new_roll = p+i+d;
        }else{
            g.pid_optflow_roll.reset_I();
            tot_x_cm = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }
        // limit amount of change and maximum angle
        of_roll = constrain_int32(new_roll, (of_roll-20), (of_roll+20));

 #if LOGGING_ENABLED == ENABLED
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            pid_log_counter++;              // Note: get_of_pitch pid logging relies on this function updating pid_log_counter so if you change the line above you must change the equivalent line in get_of_pitch
            if( pid_log_counter >= 5 ) {    // (update rate / desired output rate) = (100hz / 10hz) = 10
                pid_log_counter = 0;
                Log_Write_PID(CH6_OPTFLOW_KP, tot_x_cm, p, i, d, of_roll, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_roll = constrain_int32(of_roll, -1000, 1000);

    return input_roll+of_roll;
#else
    return input_roll;
#endif
}

static int32_t
get_of_pitch(int32_t input_pitch)
{
#if OPTFLOW == ENABLED
    static float tot_y_cm = 0;  // total distance from target
    static uint32_t last_of_pitch_update = 0;
    int32_t new_pitch = 0;
    int32_t p,i,d;

    // check if new optflow data available
    if( optflow.last_update != last_of_pitch_update ) {
        last_of_pitch_update = optflow.last_update;

        // add new distance moved
        tot_y_cm += optflow.y_cm;

        // only stop roll if caller isn't modifying pitch
        if( input_pitch == 0 && current_loc.alt < 1500 ) {
            p = g.pid_optflow_pitch.get_p(tot_y_cm);
            i = g.pid_optflow_pitch.get_i(tot_y_cm, 1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_optflow_pitch.get_d(tot_y_cm, 1.0f);
            new_pitch = p + i + d;
        }else{
            tot_y_cm = 0;
            g.pid_optflow_pitch.reset_I();
            p = 0;              // for logging
            i = 0;
            d = 0;
        }

        // limit amount of change
        of_pitch = constrain_int32(new_pitch, (of_pitch-20), (of_pitch+20));

 #if LOGGING_ENABLED == ENABLED
        // log output if PID logging is on and we are tuning the rate P, I or D gains
        if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_OPTFLOW_KP || g.radio_tuning == CH6_OPTFLOW_KI || g.radio_tuning == CH6_OPTFLOW_KD) ) {
            if( pid_log_counter == 0 ) {        // relies on get_of_roll having updated the pid_log_counter
                Log_Write_PID(CH6_OPTFLOW_KP+100, tot_y_cm, p, i, d, of_pitch, tuning_value);
            }
        }
 #endif // LOGGING_ENABLED == ENABLED
    }

    // limit max angle
    of_pitch = constrain_int32(of_pitch, -1000, 1000);

    return input_pitch+of_pitch;
#else
    return input_pitch;
#endif
}

/*************************************************************
 * yaw controllers
 *************************************************************/

 // get_look_at_yaw - updates bearing to look at center of circle or do a panorama
// should be called at 100hz
static void get_circle_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    // if circle radius is zero do panorama
    if( g.circle_radius == 0 ) {
        // slew yaw towards circle angle
        nav_yaw = get_yaw_slew(nav_yaw, ToDeg(circle_angle)*100, AUTO_YAW_SLEW_RATE);
    }else{
        look_at_yaw_counter++;
        if( look_at_yaw_counter >= 10 ) {
            look_at_yaw_counter = 0;
            yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
        }
        // slew yaw
        nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    }

    // call stabilize yaw controller
    get_stabilize_yaw(nav_yaw);
}

// get_look_at_yaw - updates bearing to location held in look_at_yaw_WP and calls stabilize yaw controller
// should be called at 100hz
static void get_look_at_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    look_at_yaw_counter++;
    if( look_at_yaw_counter >= 10 ) {
        look_at_yaw_counter = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
    }

    // slew yaw and call stabilize controller
    nav_yaw = get_yaw_slew(nav_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    get_stabilize_yaw(nav_yaw);
}

static void get_look_ahead_yaw(int16_t pilot_yaw)
{
    // Commanded Yaw to automatically look ahead.
    if (g_gps->fix && g_gps->ground_course > YAW_LOOK_AHEAD_MIN_SPEED) {
        nav_yaw = get_yaw_slew(nav_yaw, g_gps->ground_course, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(wrap_360_cd(nav_yaw + pilot_yaw));   // Allow pilot to "skid" around corners up to 45 degrees
    }else{
        nav_yaw += pilot_yaw * g.acro_p * G_Dt;
        nav_yaw = wrap_360_cd(nav_yaw);
        get_stabilize_yaw(nav_yaw);
    }
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
static void update_throttle_cruise(int16_t throttle)
{
    // ensure throttle_avg has been initialised
    if( throttle_avg == 0 ) {
        throttle_avg = g.throttle_cruise;
    }
    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * 0.99f + (float)throttle * 0.01f;
        g.throttle_cruise = throttle_avg;
    }
}

#if FRAME_CONFIG == HELI_FRAME
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
// for traditional helicopters
static int16_t get_angle_boost(int16_t throttle)
{
    float angle_boost_factor = cos_pitch_x * cos_roll_x;
    angle_boost_factor = 1.0f - constrain_float(angle_boost_factor, .5f, 1.0f);
    int16_t throttle_above_mid = max(throttle - motors.throttle_mid,0);

    // to allow logging of angle boost
    angle_boost = throttle_above_mid*angle_boost_factor;

    return throttle + angle_boost;
}
#else   // all multicopters
// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
static int16_t get_angle_boost(int16_t throttle)
{
    float temp = cos_pitch_x * cos_roll_x;
    int16_t throttle_out;

    temp = constrain_float(temp, 0.5f, 1.0f);

    // reduce throttle if we go inverted
    temp = constrain_float(9000-max(labs(ahrs.roll_sensor),labs(ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

    // apply scale and constrain throttle
    throttle_out = constrain_float((float)(throttle-g.throttle_min) * temp + g.throttle_min, g.throttle_min, 1000);

    // to allow logging of angle boost
    angle_boost = throttle_out - throttle;

    return throttle_out;
}
#endif // FRAME_CONFIG == HELI_FRAME

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost )
{
    if( apply_angle_boost ) {
        g.rc_3.servo_out = get_angle_boost(throttle_out);
    }else{
        g.rc_3.servo_out = throttle_out;
        // clear angle_boost for logging purposes
        angle_boost = 0;
    }

    // update compass with throttle value
    compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
}

// set_throttle_accel_target - to be called by upper throttle controllers to set desired vertical acceleration in earth frame
void set_throttle_accel_target( int16_t desired_acceleration )
{
    if( g.throttle_accel_enabled ) {
        throttle_accel_target_ef = desired_acceleration;
        throttle_accel_controller_active = true;
    }else{
        // To-Do log dataflash or tlog error
        cliSerial->print_P(PSTR("Err: target sent to inactive acc thr controller!\n"));
    }
}

// disable_throttle_accel - disables the accel based throttle controller
// it will be re-enasbled on the next set_throttle_accel_target
// required when we wish to set motors to zero when pilot inputs zero throttle
void throttle_accel_deactivate()
{
    throttle_accel_controller_active = false;
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors
static int16_t
get_throttle_accel(int16_t z_target_accel)
{
    static float z_accel_error = 0;     // The acceleration error in cm.
    static uint32_t last_call_ms = 0;   // the last time this controller was called
    int32_t p,i,d;                      // used to capture pid values for logging
    int16_t output;
    float z_accel_meas;
    uint32_t now = millis();

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(ahrs.get_accel_ef().z + GRAVITY_MSS) * 100;

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_accel_error = 0;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        z_accel_error = z_accel_error + 0.11164f * (constrain_float(z_target_accel - z_accel_meas, -32000, 32000) - z_accel_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle_accel.get_p(z_accel_error);
    // freeze I term if we've breached throttle limits
    if( motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT) ) {
        i = g.pid_throttle_accel.get_integrator();
    }else{
        i = g.pid_throttle_accel.get_i(z_accel_error, .01f);
    }
    d = g.pid_throttle_accel.get_d(z_accel_error, .01f);

    //
    // limit the rate
    output =  constrain_float(p+i+d+g.throttle_cruise, g.throttle_min, g.throttle_max);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_THR_ACCEL_KP || g.radio_tuning == CH6_THR_ACCEL_KI || g.radio_tuning == CH6_THR_ACCEL_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (50hz / 10hz) = 5hz
            pid_log_counter = 0;
            Log_Write_PID(CH6_THR_ACCEL_KP, z_accel_error, p, i, d, output, tuning_value);
        }
    }
#endif

    return output;
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
static int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately in the simple cases
    if( throttle_control == 0 || g.throttle_mid == 500) {
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(500-g.throttle_min));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THROTTLE_IN_DEADBAND 100        // the throttle input channel's deadband in PWM
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+THROTTLE_IN_DEADBAND)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-THROTTLE_IN_DEADBAND)  // bottom of the deadband
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // throttle failsafe check
    if( ap.failsafe_radio ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain_int16(throttle_control,0,1000);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THROTTLE_IN_DEADBAND);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_initial_alt_hold - get new target altitude based on current altitude and climb rate
static int32_t
get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms)
{
    int32_t target_alt;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
    int32_t linear_velocity;      // the velocity we swap between linear and sqrt.

    linear_velocity = ALT_HOLD_ACCEL_MAX/g.pi_alt_hold.kP();

    if (abs(climb_rate_cms) < linear_velocity) {
        target_alt = alt_cm + climb_rate_cms/g.pi_alt_hold.kP();
    } else {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if (climb_rate_cms > 0){
            target_alt = alt_cm + linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX);
        } else {
            target_alt = alt_cm - ( linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX) );
        }
    }
    return constrain_int32(target_alt, alt_cm - ALT_HOLD_INIT_MAX_OVERSHOOT, alt_cm + ALT_HOLD_INIT_MAX_OVERSHOOT);
}

// get_throttle_rate - calculates desired accel required to achieve desired z_target_speed
// sets accel based throttle controller target
static void
get_throttle_rate(float z_target_speed)
{
    static uint32_t last_call_ms = 0;
    static float z_rate_error = 0;   // The velocity error in cm.
    static float z_target_speed_last = 0;   // The requested speed from the previous iteration
    int32_t p,i,d;      // used to capture pid values for logging
    int32_t output;     // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors
    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_rate_error    = 0;
        output = 0;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        z_rate_error    = z_rate_error + 0.20085f * ((z_target_speed - climb_rate) - z_rate_error);
        // feed forward acceleration based on change in desired speed.
        output = (z_target_speed - z_target_speed_last) * 50.0f;   // To-Do: replace 50 with dt
    }
    last_call_ms = now;

    // store target speed for next iteration
    z_target_speed_last = z_target_speed;

    // separately calculate p, i, d values for logging
    p = g.pid_throttle.get_p(z_rate_error);

    // freeze I term if we've breached throttle limits
    if(motors.reached_limit(AP_MOTOR_THROTTLE_LIMIT)) {
        i = g.pid_throttle.get_integrator();
    }else{
        i = g.pid_throttle.get_i(z_rate_error, .02);
    }
    d = g.pid_throttle.get_d(z_rate_error, .02);

    // consolidate and constrain target acceleration
    output += p+i+d;
    output = constrain_int32(output, -32000, 32000);

#if LOGGING_ENABLED == ENABLED
    // log output if PID loggins is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_THROTTLE_KP || g.radio_tuning == CH6_THROTTLE_KI || g.radio_tuning == CH6_THROTTLE_KD) ) {
        pid_log_counter++;
        if( pid_log_counter >= 10 ) {               // (update rate / desired output rate) = (50hz / 10hz) = 5hz
            pid_log_counter = 0;
            Log_Write_PID(CH6_THROTTLE_KP, z_rate_error, p, i, d, output, tuning_value);
        }
    }
#endif

    // send output to accelerometer based throttle controller if enabled otherwise send directly to motors
    if( g.throttle_accel_enabled ) {
        // set target for accel based throttle controller
        set_throttle_accel_target(output);
    }else{
        set_throttle_out(g.throttle_cruise+output, true);
    }

    // limit loiter & waypoint navigation from causing too much lean
    // To-Do: ensure that this limit is cleared when this throttle controller is not running so that loiter is not left constrained for Position mode
    wp_nav.set_angle_limit(4500 - constrain_float((z_rate_error - 100) * 10, 0, 3500));

    // update throttle cruise
    // TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
    if( z_target_speed == 0 ) {
        update_throttle_cruise(g.rc_3.servo_out);
    }
}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
static void
get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    int32_t alt_error;
    float desired_rate;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.

    // calculate altitude error
    alt_error    = target_alt - current_loc.alt;

    // check kP to avoid division by zero
    if( g.pi_alt_hold.kP() != 0 ) {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*g.pi_alt_hold.kP()*g.pi_alt_hold.kP());
        if( alt_error > 2*linear_distance ) {
            desired_rate = safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(alt_error-linear_distance));
        }else if( alt_error < -2*linear_distance ) {
            desired_rate = -safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(-alt_error-linear_distance));
        }else{
            desired_rate = g.pi_alt_hold.get_p(alt_error);
        }
    }else{
        desired_rate = 0;
    }

    desired_rate = constrain_float(desired_rate, min_climb_rate, max_climb_rate);

    // call rate based throttle controller which will update accel based throttle controller targets
    get_throttle_rate(desired_rate);

    // update altitude error reported to GCS
    altitude_error = alt_error;

    // TO-DO: enabled PID logging for this controller
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
static void
get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    // limit target altitude change
    controller_desired_alt += constrain_float(target_alt-controller_desired_alt, min_climb_rate*0.02f, max_climb_rate*0.02f);

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain_float(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    get_throttle_althold(controller_desired_alt, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_rate_stabilized - rate controller with additional 'stabilizer'
// 'stabilizer' ensure desired rate is being met
// calls normal throttle rate controller which updates accel based throttle controller targets
static void
get_throttle_rate_stabilized(int16_t target_rate)
{
    controller_desired_alt += target_rate * 0.02f;

    // do not let target altitude get too far from current altitude
    controller_desired_alt = constrain_float(controller_desired_alt,current_loc.alt-750,current_loc.alt+750);

    // update target altitude for reporting purposes
    set_target_alt_for_reporting(controller_desired_alt);

    get_throttle_althold(controller_desired_alt, -g.pilot_velocity_z_max-250, g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_land - high level landing logic
// sends the desired acceleration in the accel based throttle controller
// called at 50hz
static void
get_throttle_land()
{
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (current_loc.alt >= LAND_START_ALT && !(g.sonar_enabled && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        get_throttle_althold_with_slew(LAND_START_ALT, -wp_nav.get_descent_velocity(), -abs(g.land_speed));
    }else{
        get_throttle_rate_stabilized(-abs(g.land_speed));

        // detect whether we have landed by watching for minimum throttle and now movement
        if (abs(climb_rate) < 20 && (g.rc_3.servo_out <= get_angle_boost(g.throttle_min) || g.pid_throttle_accel.get_integrator() <= -150)) {
            if( land_detector < LAND_DETECTOR_TRIGGER ) {
                land_detector++;
            }else{
                set_land_complete(true);
                if( g.rc_3.control_in == 0 || ap.failsafe_radio ) {
                    init_disarm_motors();
                }
            }
        }else{
            // we've sensed movement up or down so decrease land_detector
            if (land_detector > 0 ) {
                land_detector--;
            }
        }
    }
}

// get_throttle_surface_tracking - hold copter at the desired distance above the ground
// updates accel based throttle controller targets
static void
get_throttle_surface_tracking(int16_t target_rate)
{
    static float target_sonar_alt = 0;   // The desired altitude in cm above the ground
    static uint32_t last_call_ms = 0;
    float distance_error;
    float sonar_induced_slew_rate;

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 200 ) {
        target_sonar_alt = sonar_alt + controller_desired_alt - current_loc.alt;
    }
    last_call_ms = now;

    target_sonar_alt += target_rate * 0.02f;

    distance_error = (target_sonar_alt-sonar_alt);
    sonar_induced_slew_rate = constrain_float(fabsf(g.sonar_gain * distance_error),0,THR_SURFACE_TRACKING_VELZ_MAX);

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain_float(target_sonar_alt,sonar_alt-750,sonar_alt+750);
    controller_desired_alt = current_loc.alt+(target_sonar_alt-sonar_alt);

    // update target altitude for reporting purposes
    set_target_alt_for_reporting(controller_desired_alt);

    get_throttle_althold_with_slew(controller_desired_alt, target_rate-sonar_induced_slew_rate, target_rate+sonar_induced_slew_rate);   // VELZ_MAX limits how quickly we react
}

/*
 *  reset all I integrators
 */
static void reset_I_all(void)
{
    reset_rate_I();
    reset_stability_I();
    reset_throttle_I();
    reset_optflow_I();

    // This is the only place we reset Yaw
    g.pi_stabilize_yaw.reset_I();
}

static void reset_rate_I()
{
    g.pid_rate_roll.reset_I();
    g.pid_rate_pitch.reset_I();
    g.pid_rate_yaw.reset_I();
}

static void reset_optflow_I(void)
{
    g.pid_optflow_roll.reset_I();
    g.pid_optflow_pitch.reset_I();
    of_roll = 0;
    of_pitch = 0;
}

static void reset_throttle_I(void)
{
    // For Altitude Hold
    g.pi_alt_hold.reset_I();
    g.pid_throttle.reset_I();
    g.pid_throttle_accel.reset_I();
}

static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_throttle_accel.set_integrator(pilot_throttle-g.throttle_cruise);
}

static void reset_stability_I(void)
{
    // Used to balance a quad
    // This only needs to be reset during Auto-leveling in flight
    g.pi_stabilize_roll.reset_I();
    g.pi_stabilize_pitch.reset_I();
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/GCS_Mavlink.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;


// this costs us 51 bytes, but means that low priority
// messages don't block the CPU
static mavlink_statustext_t pending_status;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;


// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

// prototype this for use inside the GCS class
static void gcs_send_text_fmt(const prog_char_t *fmt, ...);

static void gcs_send_heartbeat(void)
{
    gcs_send_message(MSG_HEARTBEAT);
}

static void gcs_send_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;
    
    if (ap.failsafe_radio == true)  {
        system_status = MAV_STATE_CRITICAL;
    }
    
    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

#if AC_FENCE == ENABLED
static NOINLINE void send_limits_status(mavlink_channel_t chan)
{
    fence_send_mavlink_status(chan);
}
#endif


static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops)
{
    uint32_t control_sensors_present = 0;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // first what sensors/controllers we have
    control_sensors_present |= (1<<0); // 3D gyro present
    control_sensors_present |= (1<<1); // 3D accelerometer present
    if (g.compass_enabled) {
        control_sensors_present |= (1<<2); // compass present
    }
    control_sensors_present |= (1<<3); // absolute pressure sensor present
    if (g_gps != NULL && g_gps->status() >= GPS::NO_FIX) {
        control_sensors_present |= (1<<5); // GPS present
    }
    control_sensors_present |= (1<<10); // 3D angular rate control
    control_sensors_present |= (1<<11); // attitude stabilisation
    control_sensors_present |= (1<<12); // yaw position
    control_sensors_present |= (1<<13); // altitude control
    control_sensors_present |= (1<<14); // X/Y position control
    control_sensors_present |= (1<<15); // motor control

    // now what sensors/controllers are enabled

    // first the sensors
    control_sensors_enabled = control_sensors_present & 0x1FF;

    // now the controllers
    control_sensors_enabled = control_sensors_present & 0x1FF;

    control_sensors_enabled |= (1<<10); // 3D angular rate control
    control_sensors_enabled |= (1<<11); // attitude stabilisation
    control_sensors_enabled |= (1<<13); // altitude control
    control_sensors_enabled |= (1<<15); // motor control

    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
    case POSITION:
        control_sensors_enabled |= (1<<12); // yaw position
        control_sensors_enabled |= (1<<14); // X/Y position control
        break;
    }

    // at the moment all sensors/controllers are assumed healthy
    control_sensors_health = control_sensors_present;

    if (!compass.healthy) {
        control_sensors_health &= ~(1<<2); // compass
    }
    if (!compass.use_for_yaw()) {
        control_sensors_enabled &= ~(1<<2); // compass
    }

    uint16_t battery_current = -1;
    uint8_t battery_remaining = -1;

    if (current_total1 != 0 && g.pack_capacity != 0) {
        battery_remaining = (100.0f * (g.pack_capacity - current_total1) / g.pack_capacity);
    }
    if (current_total1 != 0) {
        battery_current = current_amps1 * 100;
    }

    if (g.battery_monitoring == BATT_MONITOR_VOLTAGE_ONLY) {
        /*setting a out-of-range value.
         *  It informs to external devices that
         *  it cannot be calculated properly just by voltage*/
        battery_remaining = 150;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        0, // CPU Load not supported in AC yet
        battery_voltage1 * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

static void NOINLINE send_meminfo(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
    extern unsigned __brkval;
    mavlink_msg_meminfo_send(chan, __brkval, memcheck_available_memory());
#endif
}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.    
    if (g_gps->status() >= GPS::GPS_OK_FIX_2D) {
        fix_time = g_gps->last_fix_time;
    } else {
        fix_time = millis();
    }
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude * 10,             // millimeters above sea level
        (current_loc.alt - home.alt) * 10,           // millimeters above ground
        g_gps->velocity_north() * 100,  // X speed cm/s (+ve North)
        g_gps->velocity_east()  * 100,  // Y speed cm/s (+ve East)
        g_gps->velocity_down()  * -100, // Z speed cm/s (+ve up)
        g_gps->ground_course);          // course in 1/100 degree
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll / 1.0e2f,
        nav_pitch / 1.0e2f,
        wp_bearing / 1.0e2f,
        wp_bearing / 1.0e2f,
        wp_distance / 1.0e2f,
        altitude_error / 1.0e2f,
        0,
        0);
}

static void NOINLINE send_ahrs(mavlink_channel_t chan)
{
    Vector3f omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        1,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}

// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.simstate_send(chan);
#endif
}

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        board_voltage(),
        hal.i2c->lockup_count());
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
    mavlink_msg_gps_raw_int_send(
        chan,
        g_gps->last_fix_time*(uint64_t)1000,
        g_gps->status(),
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed,  // cm/s
        g_gps->ground_course, // 1/100 degrees,
        g_gps->num_sats);

}

static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with HIL maintainers

#if FRAME_CONFIG == HELI_FRAME

    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        0,
        0,
        0,
        0,
        receiver_rssi);
#else
 #if X_PLANE == ENABLED
    /* update by JLN for X-Plane HIL */
    if(motors.armed() && ap.auto_armed) {
        mavlink_msg_rc_channels_scaled_send(
            chan,
            millis(),
            0,         // port 0
            g.rc_1.servo_out,
            g.rc_2.servo_out,
            10000 * g.rc_3.norm_output(),
            g.rc_4.servo_out,
            10000 * g.rc_1.norm_output(),
            10000 * g.rc_2.norm_output(),
            10000 * g.rc_3.norm_output(),
            10000 * g.rc_4.norm_output(),
            receiver_rssi);
    }else{
        mavlink_msg_rc_channels_scaled_send(
            chan,
            millis(),
            0,         // port 0
            0,
            0,
            -10000,
            0,
            10000 * g.rc_1.norm_output(),
            10000 * g.rc_2.norm_output(),
            10000 * g.rc_3.norm_output(),
            10000 * g.rc_4.norm_output(),
            receiver_rssi);
    }

 #else
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,         // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        10000 * g.rc_1.norm_output(),
        10000 * g.rc_2.norm_output(),
        10000 * g.rc_3.norm_output(),
        10000 * g.rc_4.norm_output(),
        receiver_rssi);
 #endif
#endif
}

static void NOINLINE send_radio_in(mavlink_channel_t chan)
{
    mavlink_msg_rc_channels_raw_send(
        chan,
        millis(),
        0, // port
        g.rc_1.radio_in,
        g.rc_2.radio_in,
        g.rc_3.radio_in,
        g.rc_4.radio_in,
        g.rc_5.radio_in,
        g.rc_6.radio_in,
        g.rc_7.radio_in,
        g.rc_8.radio_in,
        receiver_rssi);
}

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0, // port
        motors.motor_out[AP_MOTORS_MOT_1],
        motors.motor_out[AP_MOTORS_MOT_2],
        motors.motor_out[AP_MOTORS_MOT_3],
        motors.motor_out[AP_MOTORS_MOT_4],
        motors.motor_out[AP_MOTORS_MOT_5],
        motors.motor_out[AP_MOTORS_MOT_6],
        motors.motor_out[AP_MOTORS_MOT_7],
        motors.motor_out[AP_MOTORS_MOT_8]);
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        (float)g_gps->ground_speed / 100.0f,
        (float)g_gps->ground_speed / 100.0f,
        (ahrs.yaw_sensor / 100) % 360,
        g.rc_3.servo_out/10,
        current_loc.alt / 100.0f,
        climb_rate / 100.0f);
}

static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    Vector3f accel = ins.get_accel();
    Vector3f gyro = ins.get_gyro();
    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        compass.mag_x,
        compass.mag_y,
        compass.mag_z);
}

static void NOINLINE send_raw_imu2(mavlink_channel_t chan)
{
    mavlink_msg_scaled_pressure_send(
        chan,
        millis(),
        (float)barometer.get_pressure()/100.0f,
        (float)(barometer.get_pressure() - barometer.get_ground_pressure())/100.0f,
        (int)(barometer.get_temperature()*10));
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    Vector3f mag_offsets = compass.get_offsets();
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_raw_pressure(),
                                    barometer.get_raw_temp(),
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(
        chan,
        (uint16_t)g.command_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_msg_statustext_send(
        chan,
        pending_status.severity,
        pending_status.text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > (uint8_t)g.telem_delay) {
        return false;
    }
#if USB_MUX_PIN > 0
    if (chan == MAVLINK_COMM_0 && ap_system.usb_connected) {
        // this is an APM2 with USB telemetry
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry
    return true;
#else
    if (chan == MAVLINK_COMM_0) {
        // we're on the USB port
        return false;
    }
    // don't send telemetry yet
    return true;
#endif
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (telemetry_delayed(chan)) {
        return false;
    }

    // if we don't have at least 1ms remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (scheduler.time_available_usec() < 800 && motors.armed()) {
        gcs_out_of_time = true;
        return false;
    }

    switch(id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        break;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_extended_status1(chan, packet_drops);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo(chan);
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_nav_controller_output(chan);
        break;

    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        send_gps_raw(chan);
        break;

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(chan);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu1(chan);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_raw_imu2(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_raw_imu3(chan);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_param_send();
        } else if (gcs3.initialised) {
            gcs3.queued_param_send();
        }
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_waypoint_send();
        } else {
            gcs3.queued_waypoint_send();
        }
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

#if AC_FENCE == ENABLED
    case MSG_LIMITS_STATUS:
        CHECK_PAYLOAD_SIZE(LIMITS_STATUS);
        send_limits_status(chan);
        break;
#endif

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs(chan);
        break;

    case MSG_SIMSTATE:
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
#endif
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning
    }

    return true;
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];

// send a message using mavlink
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str)
{
    if (telemetry_delayed(chan)) {
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        pending_status.severity = (uint8_t)severity;
        strncpy((char *)pending_status.text, str, sizeof(pending_status.text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
        mavlink_msg_statustext_send(
            chan,
            severity,
            str);
    }
}

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Raw sensor stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRateRawSensors,      0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Extended status stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRateExtendedStatus,  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: RC Channel stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRateRCChannels,      0),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Raw Control stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRateRawController,   0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Position stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRatePosition,        0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Extra data type 1 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRateExtra1,          0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Extra data type 2 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRateExtra2,          0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Extra data type 3 stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRateExtra3,          0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Parameter stream rate to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRateParams,          0),
    AP_GROUPEND
};


GCS_MAVLINK::GCS_MAVLINK() :
    packet_drops(0),
    waypoint_send_timeout(1000), // 1 second
    waypoint_receive_timeout(1000) // 1 second
{

}

void
GCS_MAVLINK::init(AP_HAL::UARTDriver* port)
{
    GCS_Class::init(port);
    if (port == hal.uartA) {
        mavlink_comm_0_port = port;
        chan = MAVLINK_COMM_0;
    }else{
        mavlink_comm_1_port = port;
        chan = MAVLINK_COMM_1;
    }
    _queued_parameter = NULL;
    reset_cli_timeout();
}

void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++) {
        uint8_t c = comm_receive_ch(chan);

#if CLI_ENABLED == ENABLED
        /* allow CLI to be started by hitting enter 3 times, if no
         *  heartbeat packets have been received */
        if (mavlink_active == 0 && (millis() - _cli_timeout) < 20000 && 
            !motors.armed() && comm_is_idle(chan)) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                run_cli(_port);
            }
        }
#endif

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            // we exclude radio packets to make it possible to use the
            // CLI over the radio
            if (msg.msgid != MAVLINK_MSG_ID_RADIO) {
                mavlink_active = true;
            }
            handleMessage(&msg);
        }
    }

    // Update packet drops counter
    packet_drops += status.packet_rx_drop_count;

    if (!waypoint_receiving && !waypoint_sending) {
        return;
    }

    uint32_t tnow = millis();

    if (waypoint_receiving &&
        waypoint_request_i <= (unsigned)g.command_total &&
        tnow > waypoint_timelast_request + 500 + (stream_slowdown*20)) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint sending if timeout
    if (waypoint_sending && (tnow - waypoint_timelast_send) > waypoint_send_timeout) {
        waypoint_sending = false;
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (tnow - waypoint_timelast_receive) > waypoint_receive_timeout) {
        waypoint_receiving = false;
    }
}

// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    uint8_t rate;
    switch (stream_num) {
        case STREAM_RAW_SENSORS:
            rate = streamRateRawSensors.get();
            break;
        case STREAM_EXTENDED_STATUS:
            rate = streamRateExtendedStatus.get();
            break;
        case STREAM_RC_CHANNELS:
            rate = streamRateRCChannels.get();
            break;
        case STREAM_RAW_CONTROLLER:
            rate = streamRateRawController.get();
            break;
        case STREAM_POSITION:
            rate = streamRatePosition.get();
            break;
        case STREAM_EXTRA1:
            rate = streamRateExtra1.get();
            break;
        case STREAM_EXTRA2:
            rate = streamRateExtra2.get();
            break;
        case STREAM_EXTRA3:
            rate = streamRateExtra3.get();
            break;
        case STREAM_PARAMS:
            rate = streamRateParams.get();
            break;
        default:
            rate = 0;
    }

    if (rate == 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_MAVLINK::data_stream_send(void)
{
    if (waypoint_receiving || waypoint_sending) {
        // don't interfere with mission transfer
        return;
    }

    gcs_out_of_time = false;

    if (_queued_parameter != NULL) {
        if (streamRateParams.get() <= 0) {
            streamRateParams.set(50);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
        // don't send anything else at the same time as parameters
        return;
    }

    if (gcs_out_of_time) return;

    if (in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
        //cliSerial->printf("mav1 %d\n", (int)streamRateRawSensors.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_LIMITS_STATUS);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_LOCATION);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
        //cliSerial->printf("mav4 %d\n", (int)streamRateRawController.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
        //cliSerial->printf("mav5 %d\n", (int)streamRateRCChannels.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
        //cliSerial->printf("mav6 %d\n", (int)streamRateExtra1.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
        //cliSerial->printf("mav7 %d\n", (int)streamRateExtra2.get());
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
    }
}



void
GCS_MAVLINK::send_message(enum ap_message id)
{
    mavlink_send_message(chan,id, packet_drops);
}

void
GCS_MAVLINK::send_text_P(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
        if (m.text[i] == '\0') {
            break;
        }
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    mavlink_send_text(chan, severity, (const char *)m.text);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    struct Location tell_command = {};                                  // command for telemetry
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:     //66
    {
        // decode
        mavlink_request_data_stream_t packet;
        mavlink_msg_request_data_stream_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        int16_t freq = 0;                 // packet frequency

        if (packet.start_stop == 0)
            freq = 0;                     // stop sending
        else if (packet.start_stop == 1)
            freq = packet.req_message_rate;                     // start sending
        else
            break;

        switch(packet.req_stream_id) {

        case MAV_DATA_STREAM_ALL:
            streamRateRawSensors            = freq;
            streamRateExtendedStatus        = freq;
            streamRateRCChannels            = freq;
            streamRateRawController         = freq;
            streamRatePosition                      = freq;
            streamRateExtra1                        = freq;
            streamRateExtra2                        = freq;
            //streamRateExtra3.set_and_save(freq);	// We just do set and save on the last as it takes care of the whole group.
            streamRateExtra3                        = freq;                             // Don't save!!
            break;

        case MAV_DATA_STREAM_RAW_SENSORS:
            streamRateRawSensors = freq;                                        // We do not set and save this one so that if HIL is shut down incorrectly
            // we will not continue to broadcast raw sensor data at 50Hz.
            break;
        case MAV_DATA_STREAM_EXTENDED_STATUS:
            //streamRateExtendedStatus.set_and_save(freq);
            streamRateExtendedStatus = freq;
            break;

        case MAV_DATA_STREAM_RC_CHANNELS:
            streamRateRCChannels  = freq;
            break;

        case MAV_DATA_STREAM_RAW_CONTROLLER:
            streamRateRawController = freq;
            break;

        //case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
        //	streamRateRawSensorFusion.set_and_save(freq);
        //	break;

        case MAV_DATA_STREAM_POSITION:
            streamRatePosition = freq;
            break;

        case MAV_DATA_STREAM_EXTRA1:
            streamRateExtra1 = freq;
            break;

        case MAV_DATA_STREAM_EXTRA2:
            streamRateExtra2 = freq;
            break;

        case MAV_DATA_STREAM_EXTRA3:
            streamRateExtra3 = freq;
            break;

        default:
            break;
        }
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        // decode
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        // do command
        send_text_P(SEVERITY_LOW,PSTR("command received: "));

        switch(packet.command) {

        case MAV_CMD_NAV_LOITER_UNLIM:
            set_mode(LOITER);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            set_mode(RTL);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_NAV_LAND:
            set_mode(LAND);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_MISSION_START:
            set_mode(AUTO);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            if (packet.param1 == 1 ||
                packet.param2 == 1 ||
                packet.param3 == 1) {
                ins.init_accel(flash_leds);
                ahrs.set_trim(Vector3f(0,0,0));             // clear out saved trim
            }
            if (packet.param4 == 1) {
                trim_radio();
            }
            if (packet.param5 == 1) {
                float trim_roll, trim_pitch;
                // this blocks
                AP_InertialSensor_UserInteract_MAVLink interact(chan);
                if(ins.calibrate_accel(flash_leds, &interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                }
            }
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (packet.target_component == MAV_COMP_ID_SYSTEM_CONTROL) {
                if (packet.param1 == 1.0f) {
                    // run pre-arm-checks and display failures
                    pre_arm_checks(true);
                    if(ap.pre_arm_check) {
                        init_arm_motors();
                    }
                    result = MAV_RESULT_ACCEPTED;
                } else if (packet.param1 == 0.0f)  {
                    init_disarm_motors();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_UNSUPPORTED;
                }
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1) {
                reboot_apm();
                result = MAV_RESULT_ACCEPTED;
            }
            break;


        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        mavlink_msg_command_ack_send(
            chan,
            packet.command,
            result);

        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:      //11
    {
        // decode
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(msg, &packet);

        if (!(packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
            // we ignore base_mode as there is no sane way to map
            // from that bitmap to a APM flight mode. We rely on
            // custom_mode instead.
            break;
        }
        switch (packet.custom_mode) {
        case STABILIZE:
        case ACRO:
        case ALT_HOLD:
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case POSITION:
        case LAND:
        case OF_LOITER:
            set_mode(packet.custom_mode);
            break;
        }

        break;
    }

    /*case MAVLINK_MSG_ID_SET_NAV_MODE:
     *       {
     *               // decode
     *               mavlink_set_nav_mode_t packet;
     *               mavlink_msg_set_nav_mode_decode(msg, &packet);
     *               // To set some flight modes we must first receive a "set nav mode" message and then a "set mode" message
     *               mav_nav = packet.nav_mode;
     *               break;
     *       }
     */
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:     //43
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint request list"));

        // decode
        mavlink_mission_request_list_t packet;
        mavlink_msg_mission_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // Start sending waypoints
        mavlink_msg_mission_count_send(
            chan,msg->sysid,
            msg->compid,
            g.command_total);                     // includes home

        waypoint_timelast_send          = millis();
        waypoint_sending                        = true;
        waypoint_receiving                      = false;
        waypoint_dest_sysid                     = msg->sysid;
        waypoint_dest_compid            = msg->compid;
        break;
    }

    // XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:     // 40
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint request"));

        // Check if sending waypiont
        //if (!waypoint_sending) break;
        // 5/10/11 - We are trying out relaxing the requirement that we be in waypoint sending mode to respond to a waypoint request.  DEW

        // decode
        mavlink_mission_request_t packet;
        mavlink_msg_mission_request_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // send waypoint
        tell_command = get_cmd_with_index(packet.seq);

        // set frame of waypoint
        uint8_t frame;

        if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;                     // reference frame
        } else {
            frame = MAV_FRAME_GLOBAL;                     // reference frame
        }

        float param1 = 0, param2 = 0, param3 = 0, param4 = 0;

        // time that the mav should loiter in milliseconds
        uint8_t current = 0;                 // 1 (true), 0 (false)

        if (packet.seq == (uint16_t)g.command_index)
            current = 1;

        uint8_t autocontinue = 1;                 // 1 (true), 0 (false)

        float x = 0, y = 0, z = 0;

        if (tell_command.id < MAV_CMD_NAV_LAST) {
            // command needs scaling
            x = tell_command.lat/1.0e7f;                     // local (x), global (latitude)
            y = tell_command.lng/1.0e7f;                     // local (y), global (longitude)
            // ACM is processing alt inside each command. so we save and load raw values. - this is diffrent to APM
            z = tell_command.alt/1.0e2f;                     // local (z), global/relative (altitude)
        }

        // Switch to map APM command fields into MAVLink command fields
        switch (tell_command.id) {

        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_CONDITION_CHANGE_ALT:
        case MAV_CMD_DO_SET_HOME:
            param1 = tell_command.p1;
            break;

        case MAV_CMD_NAV_ROI:
            param1 = tell_command.p1;                                   // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
            break;

        case MAV_CMD_CONDITION_YAW:
            param3 = tell_command.p1;
            param1 = tell_command.alt;
            param2 = tell_command.lat;
            param4 = tell_command.lng;
            break;

        case MAV_CMD_NAV_TAKEOFF:
            param1 = 0;
            break;

        case MAV_CMD_NAV_LOITER_TIME:
            param1 = tell_command.p1;                                   // ACM loiter time is in 1 second increments
            break;

        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            param1 = tell_command.lat;
            break;

        case MAV_CMD_DO_JUMP:
            param2 = tell_command.lat;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            param4 = tell_command.lng;
        case MAV_CMD_DO_REPEAT_RELAY:
        case MAV_CMD_DO_CHANGE_SPEED:
            param3 = tell_command.lat;
            param2 = tell_command.alt;
            param1 = tell_command.p1;
            break;

        case MAV_CMD_NAV_WAYPOINT:
            param1 = tell_command.p1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            param2 = tell_command.alt;
            param1 = tell_command.p1;
            break;
        }

        mavlink_msg_mission_item_send(chan,msg->sysid,
                                      msg->compid,
                                      packet.seq,
                                      frame,
                                      tell_command.id,
                                      current,
                                      autocontinue,
                                      param1,
                                      param2,
                                      param3,
                                      param4,
                                      x,
                                      y,
                                      z);

        // update last waypoint comm stamp
        waypoint_timelast_send = millis();
        break;
    }

    case MAVLINK_MSG_ID_MISSION_ACK:     //47
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint ack"));

        // decode
        mavlink_mission_ack_t packet;
        mavlink_msg_mission_ack_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // turn off waypoint send
        waypoint_sending = false;
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:     // 21
    {
        // gcs_send_text_P(SEVERITY_LOW,PSTR("param request list"));

        // decode
        mavlink_param_request_list_t packet;
        mavlink_msg_param_request_list_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // Start sending parameters - next call to ::update will kick the first one out

        _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
        _queued_parameter_index = 0;
        _queued_parameter_count = _count_parameters();
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        // decode
        mavlink_param_request_read_t packet;
        mavlink_msg_param_request_read_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        enum ap_var_type p_type;
        AP_Param *vp;
        char param_name[AP_MAX_NAME_SIZE+1];
        if (packet.param_index != -1) {
            AP_Param::ParamToken token;
            vp = AP_Param::find_by_index(packet.param_index, &p_type, &token);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter index %d"), packet.param_index);
                break;
            }
            vp->copy_name_token(token, param_name, AP_MAX_NAME_SIZE, true);
            param_name[AP_MAX_NAME_SIZE] = 0;
        } else {
            strncpy(param_name, packet.param_id, AP_MAX_NAME_SIZE);
            param_name[AP_MAX_NAME_SIZE] = 0;
            vp = AP_Param::find(param_name, &p_type);
            if (vp == NULL) {
                gcs_send_text_fmt(PSTR("Unknown parameter %.16s"), packet.param_id);
                break;
            }
        }

        float value = vp->cast_to_float(p_type);
        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(p_type),
            _count_parameters(),
            packet.param_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:     // 45
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint clear all"));

        // decode
        mavlink_mission_clear_all_t packet;
        mavlink_msg_mission_clear_all_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component)) break;

        // clear all waypoints
        uint8_t type = 0;                 // ok (0), error(1)
        g.command_total.set_and_save(1);

        // send acknowledgement 3 times to makes sure it is received
        for (int16_t i=0; i<3; i++)
            mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, type);

        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:     // 41
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint set current"));

        // decode
        mavlink_mission_set_current_t packet;
        mavlink_msg_mission_set_current_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // set current command
        change_command(packet.seq);

        mavlink_msg_mission_current_send(chan, g.command_index);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_COUNT:     // 44
    {
        //send_text_P(SEVERITY_LOW,PSTR("waypoint count"));

        // decode
        mavlink_mission_count_t packet;
        mavlink_msg_mission_count_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // start waypoint receiving
        if (packet.count > MAX_WAYPOINTS) {
            packet.count = MAX_WAYPOINTS;
        }
        g.command_total.set_and_save(packet.count);

        waypoint_timelast_receive = millis();
        waypoint_receiving   = true;
        waypoint_sending         = false;
        waypoint_request_i   = 0;
        waypoint_timelast_request = 0;
        break;
    }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

    // XXX receive a WP from GCS and store in EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:     //39
    {
        // decode
        mavlink_mission_item_t packet;
        mavlink_msg_mission_item_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;

        // defaults
        tell_command.id = packet.command;

        /*
         *  switch (packet.frame){
         *
         *       case MAV_FRAME_MISSION:
         *       case MAV_FRAME_GLOBAL:
         *               {
         *                       tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
         *                       tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
         *                       tell_command.alt = packet.z*1.0e2; // in as m converted to cm
         *                       tell_command.options = 0; // absolute altitude
         *                       break;
         *               }
         *
         *       case MAV_FRAME_LOCAL: // local (relative to home position)
         *               {
         *                       tell_command.lat = 1.0e7*ToDeg(packet.x/
         *                       (radius_of_earth*cosf(ToRad(home.lat/1.0e7)))) + home.lat;
         *                       tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
         *                       tell_command.alt = packet.z*1.0e2;
         *                       tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
         *                       break;
         *               }
         *       //case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
         *       default:
         *               {
         *                       tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
         *                       tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
         *                       tell_command.alt = packet.z * 1.0e2;
         *                       tell_command.options = MASK_OPTIONS_RELATIVE_ALT; // store altitude relative!! Always!!
         *                       break;
         *               }
         *  }
         */

        // we only are supporting Abs position, relative Alt
        tell_command.lat = 1.0e7f * packet.x;                 // in as DD converted to * t7
        tell_command.lng = 1.0e7f * packet.y;                 // in as DD converted to * t7
        tell_command.alt = packet.z * 1.0e2f;
        tell_command.options = 1;                 // store altitude relative to home alt!! Always!!

        switch (tell_command.id) {                                                      // Switch to map APM command fields into MAVLink command fields
        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_DO_SET_HOME:
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_NAV_ROI:
            tell_command.p1 = packet.param1;                                    // MAV_ROI (aka roi mode) is held in wp's parameter but we actually do nothing with it because we only support pointing at a specific location provided by x,y and z parameters
            break;

        case MAV_CMD_CONDITION_YAW:
            tell_command.p1 = packet.param3;
            tell_command.alt = packet.param1;
            tell_command.lat = packet.param2;
            tell_command.lng = packet.param4;
            break;

        case MAV_CMD_NAV_TAKEOFF:
            tell_command.p1 = 0;
            break;

        case MAV_CMD_CONDITION_CHANGE_ALT:
            tell_command.p1 = packet.param1 * 100;
            break;

        case MAV_CMD_NAV_LOITER_TIME:
            tell_command.p1 = packet.param1;                                    // APM loiter time is in ten second increments
            break;

        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
            tell_command.lat = packet.param1;
            break;

        case MAV_CMD_DO_JUMP:
            tell_command.lat = packet.param2;
            tell_command.p1  = packet.param1;
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            tell_command.lng = packet.param4;
        case MAV_CMD_DO_REPEAT_RELAY:
        case MAV_CMD_DO_CHANGE_SPEED:
            tell_command.lat = packet.param3;
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_NAV_WAYPOINT:
            tell_command.p1 = packet.param1;
            break;

        case MAV_CMD_DO_SET_PARAMETER:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_SET_SERVO:
            tell_command.alt = packet.param2;
            tell_command.p1 = packet.param1;
            break;
        }

        if(packet.current == 2) {                                               //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
            // switch to guided mode
            set_mode(GUIDED);

            // set wp_nav's destination
            wp_nav.set_destination(pv_location_to_vector(tell_command));

            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else if(packet.current == 3) {                                               //current = 3 is a flag to tell us this is a alt change only

            // add home alt if needed
            if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                tell_command.alt += home.alt;
            }

            // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
            // similar to how do_change_alt works
            wp_nav.set_desired_alt(tell_command.alt);

            // verify we recevied the command
            mavlink_msg_mission_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                0);

        } else {
            // Check if receiving waypoints (mission upload expected)
            if (!waypoint_receiving) break;


            //cliSerial->printf("req: %d, seq: %d, total: %d\n", waypoint_request_i,packet.seq, g.command_total.get());

            // check if this is the requested waypoint
            if (packet.seq != waypoint_request_i)
                break;

            if(packet.seq != 0)
                set_cmd_with_index(tell_command, packet.seq);

            // update waypoint receiving state machine
            waypoint_timelast_receive = millis();
            waypoint_timelast_request = 0;
            waypoint_request_i++;

            if (waypoint_request_i == (uint16_t)g.command_total) {
                uint8_t type = 0;                         // ok (0), error(1)

                mavlink_msg_mission_ack_send(
                    chan,
                    msg->sysid,
                    msg->compid,
                    type);

                send_text_P(SEVERITY_LOW,PSTR("flight plan received"));
                waypoint_receiving = false;
                // XXX ignores waypoint radius for individual waypoints, can
                // only set WP_RADIUS parameter
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:     // 23
    {
        AP_Param                  *vp;
        enum ap_var_type var_type;

        // decode
        mavlink_param_set_t packet;
        mavlink_msg_param_set_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;

        // set parameter

        char key[AP_MAX_NAME_SIZE+1];
        strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
        key[AP_MAX_NAME_SIZE] = 0;

        // find the requested parameter
        vp = AP_Param::find(key, &var_type);
        if ((NULL != vp) &&                                                                                     // exists
            !isnan(packet.param_value) &&                                                  // not nan
            !isinf(packet.param_value)) {                                                  // not inf

            // add a small amount before casting parameter values
            // from float to integer to avoid truncating to the
            // next lower integer value.
            float rounding_addition = 0.01;

            // handle variables with standard type IDs
            if (var_type == AP_PARAM_FLOAT) {
                ((AP_Float *)vp)->set_and_save(packet.param_value);
            } else if (var_type == AP_PARAM_INT32) {
#if LOGGING_ENABLED == ENABLED
                if (g.log_bitmask != 0) {
                    Log_Write_Data(DATA_MAVLINK_FLOAT, ((AP_Int32 *)vp)->get());
                }
#endif
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -2147483648.0, 2147483647.0);
                ((AP_Int32 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT16) {
#if LOGGING_ENABLED == ENABLED
                if (g.log_bitmask != 0) {
                    Log_Write_Data(DATA_MAVLINK_INT16, (int16_t)((AP_Int16 *)vp)->get());
                }
#endif
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -32768, 32767);
                ((AP_Int16 *)vp)->set_and_save(v);
            } else if (var_type == AP_PARAM_INT8) {
#if LOGGING_ENABLED == ENABLED
                if (g.log_bitmask != 0) {
                    Log_Write_Data(DATA_MAVLINK_INT8, (int8_t)((AP_Int8 *)vp)->get());
                }
#endif
                if (packet.param_value < 0) rounding_addition = -rounding_addition;
                float v = packet.param_value+rounding_addition;
                v = constrain_float(v, -128, 127);
                ((AP_Int8 *)vp)->set_and_save(v);
            } else {
                // we don't support mavlink set on this parameter
                break;
            }

            // Report back the new value if we accepted the change
            // we send the value we actually set, which could be
            // different from the value sent, in case someone sent
            // a fractional value to an integer type
            mavlink_msg_param_value_send(
                chan,
                key,
                vp->cast_to_float(var_type),
                mav_var_type(var_type),
                _count_parameters(),
                -1);                         // XXX we don't actually know what its index is...
            DataFlash.Log_Write_Parameter(key, vp->cast_to_float(var_type));
        }

        break;
    }             // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: //70
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        if (mavlink_check_target(packet.target_system,packet.target_component))
            break;

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;
        hal.rcin->set_overrides(v, 8);

        // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
        ap.rc_override_active = true;
        // a RC override message is consiered to be a 'heartbeat' from the ground station for failsafe purposes
        last_heartbeat_ms = millis();
        break;
    }


#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        float vel = pythagorous2(packet.vx, packet.vy);
        float cog = wrap_360_cd(ToDeg(atan2f(packet.vx, packet.vy)) * 100);

        // set gps hil sensor
        g_gps->setHIL(packet.time_usec/1000,
                      packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                      vel*1.0e-2, cog*1.0e-2, 0, 10);

        if (gps_base_alt == 0) {
            gps_base_alt = g_gps->altitude;
            current_loc.alt = 0;
        }

        if (!ap.home_is_set) {
            init_home();
        }


        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * (GRAVITY_MSS/1000.0);
        accels.y = packet.yacc * (GRAVITY_MSS/1000.0);
        accels.z = packet.zacc * (GRAVITY_MSS/1000.0);

        ins.set_gyro(gyros);

        ins.set_accel(accels);

        barometer.setHIL(packet.alt*0.001f);
        compass.setHIL(packet.roll, packet.pitch, packet.yaw);

 #if HIL_MODE == HIL_MODE_ATTITUDE
        // set AHRS hil sensor
        ahrs.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
                    packet.pitchspeed,packet.yawspeed);
 #endif



        break;
    }
#endif //  HIL_MODE != HIL_MODE_DISABLED


    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if(msg->sysid != g.sysid_my_gcs) break;
        last_heartbeat_ms = millis();
        pmTest1++;
        break;
    }

#if CAMERA == ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
    {
        camera.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    {
        camera.control_msg(msg);
        break;
    }
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
    {
        camera_mount.configure_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
    {
        camera_mount.control_msg(msg);
        break;
    }

    case MAVLINK_MSG_ID_MOUNT_STATUS:
    {
        camera_mount.status_msg(msg);
        break;
    }
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_RADIO:
    {
        mavlink_radio_t packet;
        mavlink_msg_radio_decode(msg, &packet);
        // use the state of the transmit buffer in the radio to
        // control the stream rate, giving us adaptive software
        // flow control
        if (packet.txbuf < 20 && stream_slowdown < 100) {
            // we are very low on space - slow down a lot
            stream_slowdown += 3;
        } else if (packet.txbuf < 50 && stream_slowdown < 100) {
            // we are a bit low on space, slow down slightly
            stream_slowdown += 1;
        } else if (packet.txbuf > 95 && stream_slowdown > 10) {
            // the buffer has plenty of space, speed up a lot
            stream_slowdown -= 2;
        } else if (packet.txbuf > 90 && stream_slowdown != 0) {
            // the buffer has enough space, speed up a bit
            stream_slowdown--;
        }
        break;
    }

/* To-Do: add back support for polygon type fence
#if AC_FENCE == ENABLED
    // receive an AP_Limits fence point from GCS and store in EEPROM
    // receive a fence point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_FENCE_POINT: {
        mavlink_fence_point_t packet;
        mavlink_msg_fence_point_decode(msg, &packet);
        if (packet.count != geofence_limit.fence_total()) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point;
            point.x = packet.lat*1.0e7f;
            point.y = packet.lng*1.0e7f;
            geofence_limit.set_fence_point_with_index(point, packet.idx);
        }
        break;
    }
    // send a fence point to GCS
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT: {
        mavlink_fence_fetch_point_t packet;
        mavlink_msg_fence_fetch_point_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system, packet.target_component))
            break;
        if (packet.idx >= geofence_limit.fence_total()) {
            send_text_P(SEVERITY_LOW,PSTR("bad fence point"));
        } else {
            Vector2l point = geofence_limit.get_fence_point_with_index(packet.idx);
            mavlink_msg_fence_point_send(chan, 0, 0, packet.idx, geofence_limit.fence_total(),
                                         point.x*1.0e-7f, point.y*1.0e-7f);
        }
        break;
    }
#endif // AC_FENCE ENABLED
*/

    }     // end switch
} // end handle mavlink

uint16_t
GCS_MAVLINK::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Param  *vp;
        AP_Param::ParamToken token;

        vp = AP_Param::first(&token, NULL);
        do {
            _parameter_count++;
        } while (NULL != (vp = AP_Param::next_scalar(&token, NULL)));
    }
    return _parameter_count;
}

/**
 * queued_param_send - Send the next pending parameter, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_param_send()
{
    // Check to see if we are sending parameters
    if (NULL == _queued_parameter) return;

    AP_Param      *vp;
    float value;

    // copy the current parameter and prepare to move to the next
    vp = _queued_parameter;

    // if the parameter can be cast to float, report it here and break out of the loop
    value = vp->cast_to_float(_queued_parameter_type);

    char param_name[AP_MAX_NAME_SIZE];
    vp->copy_name_token(_queued_parameter_token, param_name, sizeof(param_name), true);

    mavlink_msg_param_value_send(
        chan,
        param_name,
        value,
        mav_var_type(_queued_parameter_type),
        _queued_parameter_count,
        _queued_parameter_index);

    _queued_parameter = AP_Param::next_scalar(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index++;
}

/**
 * queued_waypoint_send - Send the next pending waypoint, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_waypoint_send()
{
    if (waypoint_receiving &&
        waypoint_request_i < (unsigned)g.command_total) {
        mavlink_msg_mission_request_send(
            chan,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i);
    }
}

void GCS_MAVLINK::reset_cli_timeout() {
      _cli_timeout = millis();
}

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
static void mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;

    if (!gcs0.initialised) return;

    in_mavlink_delay = true;

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_heartbeat();
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_check_input();
        gcs_data_stream_send();
        gcs_send_deferred();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text_P(SEVERITY_LOW, PSTR("Initialising APM..."));
    }
#if USB_MUX_PIN > 0
    check_usb_mux();
#endif

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    gcs0.send_message(id);
    if (gcs3.initialised) {
        gcs3.send_message(id);
    }
}

/*
 *  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(void)
{
    gcs0.data_stream_send();
    if (gcs3.initialised) {
        gcs3.data_stream_send();
    }
}

/*
 *  look for incoming commands on the GCS links
 */
static void gcs_check_input(void)
{
    gcs0.update();
    if (gcs3.initialised) {
        gcs3.update();
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    gcs0.send_text_P(severity, str);
    if (gcs3.initialised) {
        gcs3.send_text_P(severity, str);
    }
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
static void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)pending_status.text,
            sizeof(pending_status.text), fmt, arg_list);
    va_end(arg_list);
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    if (gcs3.initialised) {
        mavlink_send_message(MAVLINK_COMM_1, MSG_STATUSTEXT, 0);
    }
}

#line 1 "/home/tobias/git/ardupilot/ArduCopter/Log.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
    cliSerial->printf_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->printf_P(PSTR("none"));
    }else{
        if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST) cliSerial->printf_P(PSTR(" ATTITUDE_FAST"));
        if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) cliSerial->printf_P(PSTR(" ATTITUDE_MED"));
        if (g.log_bitmask & MASK_LOG_GPS) cliSerial->printf_P(PSTR(" GPS"));
        if (g.log_bitmask & MASK_LOG_PM) cliSerial->printf_P(PSTR(" PM"));
        if (g.log_bitmask & MASK_LOG_CTUN) cliSerial->printf_P(PSTR(" CTUN"));
        if (g.log_bitmask & MASK_LOG_NTUN) cliSerial->printf_P(PSTR(" NTUN"));
        if (g.log_bitmask & MASK_LOG_IMU) cliSerial->printf_P(PSTR(" IMU"));
        if (g.log_bitmask & MASK_LOG_CMD) cliSerial->printf_P(PSTR(" CMD"));
        if (g.log_bitmask & MASK_LOG_CURRENT) cliSerial->printf_P(PSTR(" CURRENT"));
        if (g.log_bitmask & MASK_LOG_MOTORS) cliSerial->printf_P(PSTR(" MOTORS"));
        if (g.log_bitmask & MASK_LOG_OPTFLOW) cliSerial->printf_P(PSTR(" OPTFLOW"));
        if (g.log_bitmask & MASK_LOG_PID) cliSerial->printf_P(PSTR(" PID"));
        if (g.log_bitmask & MASK_LOG_COMPASS) cliSerial->printf_P(PSTR(" COMPASS"));
        if (g.log_bitmask & MASK_LOG_INAV) cliSerial->printf_P(PSTR(" INAV"));
        if (g.log_bitmask & MASK_LOG_CAMERA) cliSerial->printf_P(PSTR(" CAMERA"));
    }

    cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);

    return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || (static_cast<uint16_t>(dump_log) <= (last_log_num - DataFlash.get_num_logs())) || (static_cast<uint16_t>(dump_log) > last_log_num)) {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return (0);
}

static void do_erase_logs(void)
{
	gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs\n"));
    DataFlash.EraseAll();
	gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete\n"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits;

    if (argc != 2) {
        cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = ~0;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(MODE);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(MOTORS);
        TARG(OPTFLOW);
        TARG(PID);
        TARG(COMPASS);
        TARG(INAV);
        TARG(CAMERA);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }

    return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    uint32_t throttle_integrator;
    int16_t battery_voltage;
    int16_t current_amps;
    uint16_t board_voltage;
    float current_total;
};

// Write an Current data packet
static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        throttle_in         : g.rc_3.control_in,
        throttle_integrator : throttle_integrator,
        battery_voltage     : (int16_t) (battery_voltage1 * 100.0f),
        current_amps        : (int16_t) (current_amps1 * 100.0f),
        board_voltage       : board_voltage(),
        current_total       : current_total1
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Motors {
    LOG_PACKET_HEADER;
#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
    int16_t motor_out[8];
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
    int16_t motor_out[6];
#elif FRAME_CONFIG == HELI_FRAME
    int16_t motor_out[4];
    int16_t ext_gyro_gain;
#else        // quads & TRI_FRAME
    int16_t motor_out[4];
#endif
};

// Write an Motors packet
static void Log_Write_Motors()
{
    struct log_Motors pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MOTORS_MSG),
#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[AP_MOTORS_MOT_5],
                         motors.motor_out[AP_MOTORS_MOT_6],
                         motors.motor_out[AP_MOTORS_MOT_7],
                         motors.motor_out[AP_MOTORS_MOT_8]}
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[AP_MOTORS_MOT_5],
                         motors.motor_out[AP_MOTORS_MOT_6]}
#elif FRAME_CONFIG == HELI_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4]},
        ext_gyro_gain   : motors.ext_gyro_gain
#elif FRAME_CONFIG == TRI_FRAME
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_4],
                         motors.motor_out[g.rc_4.radio_out]}
#else // QUAD frame
        motor_out   :   {motors.motor_out[AP_MOTORS_MOT_1],
                         motors.motor_out[AP_MOTORS_MOT_2],
                         motors.motor_out[AP_MOTORS_MOT_3],
                         motors.motor_out[AP_MOTORS_MOT_4]}
#endif
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    int16_t dx;
    int16_t dy;
    uint8_t surface_quality;
    int16_t x_cm;
    int16_t y_cm;
    float   latitude;
    float   longitude;
    int32_t roll;
    int32_t pitch;
};

// Write an optical flow packet
static void Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        dx              : optflow.dx,
        dy              : optflow.dx,
        surface_quality : optflow.surface_quality,
        x_cm            : (int16_t) optflow.x_cm,
        y_cm            : (int16_t) optflow.y_cm,
        latitude        : optflow.vlat,
        longitude       : optflow.vlon,
        roll            : of_roll,
        pitch           : of_pitch
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t wp_distance;
    int16_t  wp_bearing;
    float    pos_error_x;
    float    pos_error_y;
    float    desired_velocity_x;
    float    desired_velocity_y;
    float    velocity_x;
    float    velocity_y;
    float    desired_accel_x;
    float    desired_accel_y;
    int32_t  desired_roll;
    int32_t  desired_pitch;
};

// Write an Nav Tuning packet
static void Log_Write_Nav_Tuning()
{
    Vector3f velocity = inertial_nav.get_velocity();

    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        wp_distance         : wp_distance,
        wp_bearing          : (int16_t) (wp_bearing/100),
        pos_error_x         : wp_nav.dist_error.x,
        pos_error_y         : wp_nav.dist_error.y,
        desired_velocity_x  : wp_nav.desired_vel.x,
        desired_velocity_y  : wp_nav.desired_vel.y,
        velocity_x          : velocity.x,
        velocity_y          : velocity.y,
        desired_accel_x     : wp_nav.desired_accel.x,
        desired_accel_y     : wp_nav.desired_accel.y,
        desired_roll        : wp_nav.get_desired_roll(),
        desired_pitch       : wp_nav.get_desired_pitch()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    int16_t throttle_in;
    int16_t sonar_alt;
    int32_t baro_alt;
    float   next_wp_alt;
    int16_t nav_throttle;
    int16_t angle_boost;
    int16_t climb_rate;
    int16_t throttle_out;
    int16_t desired_climb_rate;
};

// Write a control tuning packet
static void Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        throttle_in         : g.rc_3.control_in,
        sonar_alt           : sonar_alt,
        baro_alt            : baro_alt,
        next_wp_alt         : get_target_alt_for_reporting() / 100.0f,
        nav_throttle        : nav_throttle,
        angle_boost         : angle_boost,
        climb_rate          : climb_rate,
        throttle_out        : g.rc_3.servo_out,
        desired_climb_rate  : desired_climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
    int16_t motor_offset_x;
    int16_t motor_offset_y;
    int16_t motor_offset_z;
};

// Write a Compass packet
static void Log_Write_Compass()
{
    Vector3f mag_offsets = compass.get_offsets();
    Vector3f mag_motor_offsets = compass.get_motor_offsets();
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        mag_x           : compass.mag_x,
        mag_y           : compass.mag_y,
        mag_z           : compass.mag_z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint8_t renorm_count;
    uint8_t renorm_blowup;
    uint8_t gps_fix_count;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
};

// Write a performance monitoring packet
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        renorm_count     : ahrs.renorm_range_count,
        renorm_blowup    : ahrs.renorm_blowup_count,
        gps_fix_count    : gps_fix_count,
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time(),
        pm_test          : pmTest1,
        i2c_lockup_count : hal.i2c->lockup_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint8_t command_total;
    uint8_t command_number;
    uint8_t waypoint_id;
    uint8_t waypoint_options;
    uint8_t waypoint_param1;
    int32_t waypoint_altitude;
    int32_t waypoint_latitude;
    int32_t waypoint_longitude;
};

// Write a command processing packet
static void Log_Write_Cmd(uint8_t num, const struct Location *wp)
{
    struct log_Cmd pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMD_MSG),
        command_total       : g.command_total,
        command_number      : num,
        waypoint_id         : wp->id,
        waypoint_options    : wp->options,
        waypoint_param1     : wp->p1,
        waypoint_altitude   : wp->alt,
        waypoint_latitude   : wp->lat,
        waypoint_longitude  : wp->lng
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    int16_t roll_in;
    int16_t roll;
    int16_t pitch_in;
    int16_t pitch;
    int16_t yaw_in;
    uint16_t yaw;
    uint16_t nav_yaw;
};

// Write an attitude packet
static void Log_Write_Attitude()
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        roll_in     : (int16_t)control_roll,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch_in    : (int16_t)control_pitch,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw_in      : (int16_t)g.rc_4.control_in,
        yaw         : (uint16_t)ahrs.yaw_sensor,
        nav_yaw     : (uint16_t)nav_yaw
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_INAV {
    LOG_PACKET_HEADER;
    int16_t baro_alt;
    int16_t inav_alt;
    int16_t inav_climb_rate;
    float   accel_corr_x;
    float   accel_corr_y;
    float   accel_corr_z;
    int32_t gps_lat_from_home;
    int32_t gps_lon_from_home;
    float   inav_lat_from_home;
    float   inav_lon_from_home;
};

// Write an INAV packet
static void Log_Write_INAV()
{
    Vector3f accel_corr = inertial_nav.accel_correction_ef;

    struct log_INAV pkt = {
        LOG_PACKET_HEADER_INIT(LOG_INAV_MSG),
        baro_alt            : (int16_t)baro_alt,                        // 1 barometer altitude
        inav_alt            : (int16_t)inertial_nav.get_altitude(),     // 2 accel + baro filtered altitude
        inav_climb_rate     : (int16_t)inertial_nav.get_velocity_z(),   // 3 accel + baro based climb rate
        accel_corr_x        : accel_corr.x,                             // 4 accel correction x-axis
        accel_corr_y        : accel_corr.y,                             // 5 accel correction y-axis
        accel_corr_z        : accel_corr.z,                             // 6 accel correction z-axis
        gps_lat_from_home   : g_gps->latitude-home.lat,                 // 7 lat from home
        gps_lon_from_home   : g_gps->longitude-home.lng,                // 8 lon from home
        inav_lat_from_home  : inertial_nav.get_latitude_diff(),         // 9 accel based lat from home
        inav_lon_from_home  : inertial_nav.get_longitude_diff()        // 10 accel based lon from home
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
    int16_t throttle_cruise;
};

// Write a mode packet
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode            : mode,
        throttle_cruise : g.throttle_cruise,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet
static void Log_Write_Startup()
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
static void Log_Write_Event(uint8_t id)
{
    if (g.log_bitmask != 0) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
static void Log_Write_Data(uint8_t id, int16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
static void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
static void Log_Write_Data(uint8_t id, int32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
static void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
static void Log_Write_Data(uint8_t id, float value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_PID {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t error;
    int32_t p;
    int32_t i;
    int32_t d;
    int32_t output;
    float  gain;
};

// Write an PID packet
static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain)
{
    struct log_PID pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PID_MSG),
        id      : pid_id,
        error   : error,
        p       : p,
        i       : i,
        d       : d,
        output  : output,
        gain    : gain
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_DMP {
    LOG_PACKET_HEADER;
    int16_t  dcm_roll;
    int16_t  dmp_roll;
    int16_t  dcm_pitch;
    int16_t  dmp_pitch;
    uint16_t dcm_yaw;
    uint16_t dmp_yaw;
};

#if SECONDARY_DMP_ENABLED == ENABLED
// Write a DMP attitude packet
static void Log_Write_DMP()
{
    struct log_DMP pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DMP_MSG),
        dcm_roll    : (int16_t)ahrs.roll_sensor,
        dmp_roll    : (int16_t)ahrs2.roll_sensor,
        dcm_pitch   : (int16_t)ahrs.pitch_sensor,
        dmp_pitch   : (int16_t)ahrs2.pitch_sensor,
        dcm_yaw     : (uint16_t)ahrs.yaw_sensor,
        dmp_yaw     : (uint16_t)ahrs2.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

// Write a Camera packet
static void Log_Write_Camera()
{
#if CAMERA == ENABLED
    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAMERA_MSG),
        gps_time    : g_gps->time,
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : current_loc.alt,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "hIhhhf",      "Thr,ThrInt,Volt,Curr,Vcc,CurrTot" },

#if FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME
    { LOG_MOTORS_MSG, sizeof(log_Motors),       
      "MOT",  "hhhhhhhh",    "Mot1,Mot2,Mot3,Mot4,Mot5,Mot6,Mot7,Mot8" },
#elif FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME
    { LOG_MOTORS_MSG, sizeof(log_Motors),       
      "MOT",  "hhhhhh",      "Mot1,Mot2,Mot3,Mot4,Mot5,Mot6" },
#elif FRAME_CONFIG == HELI_FRAME
    { LOG_MOTORS_MSG, sizeof(log_Motors),       
      "MOT",  "hhhhh",       "Mot1,Mot2,Mot3,Mot4,GGain" },
#else
    { LOG_MOTORS_MSG, sizeof(log_Motors),       
      "MOT",  "hhhh",        "Mot1,Mot2,Mot3,Mot4" },
#endif

    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),       
      "OF",   "hhBccffee",   "Dx,Dy,SQual,X,Y,Lat,Lng,Roll,Pitch" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),       
      "NTUN", "Ecffffffffee",    "WPDst,WPBrg,PErX,PErY,DVelX,DVelY,VelX,VelY,DAcX,DAcY,DRol,DPit" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "hcefhhhhh",   "ThrIn,SonAlt,BarAlt,WPAlt,NavThr,AngBst,CRate,ThrOut,DCRate" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "hhhhhhhhh",    "MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "BBBHHIhB",       "RenCnt,RenBlw,FixCnt,NLon,NLoop,MaxT,PMT,I2CErr" },
    { LOG_CMD_MSG, sizeof(log_Cmd),                 
      "CMD", "BBBBBeLL",     "CTot,CNum,CId,COpt,Prm1,Alt,Lat,Lng" },
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "cccccCC",      "RollIn,Roll,PitchIn,Pitch,YawIn,Yaw,NavYaw" },
    { LOG_INAV_MSG, sizeof(log_INAV),       
      "INAV", "cccfffiiff",  "BAlt,IAlt,IClb,ACorrX,ACorrY,ACorrZ,GLat,GLng,ILat,ILng" },
    { LOG_MODE_MSG, sizeof(log_Mode),
      "MODE", "Mh",          "Mode,ThrCrs" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "",            "" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "B",           "Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_PID_MSG, sizeof(log_PID),         
      "PID",   "Biiiiif",    "Id,Error,P,I,D,Out,Gain" },
    { LOG_DMP_MSG, sizeof(log_DMP),         
      "DMP",   "ccccCC",     "DCMRoll,DMPRoll,DCMPtch,DMPPtch,DCMYaw,DMPYaw" },
    { LOG_CAMERA_MSG, sizeof(log_Camera),                 
      "CAM",   "ILLeccC",    "GPSTime,Lat,Lng,Alt,Roll,Pitch,Yaw" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "BB",         "Subsys,ECode" },
};

// Read the DataFlash log memory
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
 #ifdef AIRFRAME_NAME
    cliSerial->printf_P(PSTR((AIRFRAME_NAME)));
 #endif

    cliSerial->printf_P(PSTR("\n" THISFIRMWARE
                             "\nFree RAM: %u\n"),
                        (unsigned) memcheck_available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             sizeof(log_structure)/sizeof(log_structure[0]),
                             log_structure, 
                             print_flight_mode,
                             cliSerial);
}

// start a new log
static void start_logging() 
{
    if (g.log_bitmask != 0 && !ap.logging_started) {
        ap.logging_started = true;
        DataFlash.StartNewLog(sizeof(log_structure)/sizeof(log_structure[0]), log_structure);
    }
}

#else // LOGGING_ENABLED

static void Log_Write_Startup() {}
static void Log_Write_Cmd(uint8_t num, const struct Location *wp) {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_IMU() {}
static void Log_Write_GPS() {}
static void Log_Write_Current() {}
static void Log_Write_Compass() {}
static void Log_Write_Attitude() {}
static void Log_Write_INAV() {}
static void Log_Write_Data(uint8_t id, int16_t value){}
static void Log_Write_Data(uint8_t id, uint16_t value){}
static void Log_Write_Data(uint8_t id, int32_t value){}
static void Log_Write_Data(uint8_t id, uint32_t value){}
static void Log_Write_Data(uint8_t id, float value){}
static void Log_Write_Event(uint8_t id){}
static void Log_Write_Optflow() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Motors() {}
static void Log_Write_Performance() {}
static void Log_Write_PID(uint8_t pid_id, int32_t error, int32_t p, int32_t i, int32_t d, int32_t output, float gain) {}
#if SECONDARY_DMP_ENABLED == ENABLED
static void Log_Write_DMP() {}
#endif
static void Log_Write_Camera() {}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
#line 1 "/home/tobias/git/ardupilot/ArduCopter/Parameters.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
 *  ArduCopter parameter definitions
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
    // @Param: SYSID_SW_MREV
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "SYSID_SW_MREV",   0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @User: Advanced
    GSCALAR(software_type,  "SYSID_SW_TYPE",   Parameters::k_software_type),

    // @Param: SYSID_THISMAV
    // @DisplayName: Mavlink version
    // @Description: Allows reconising the mavlink version
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: My ground station number
    // @Description: Allows restricting radio overrides to only come from my ground station
    // @User: Advanced
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial3_baud,   "SERIAL3_BAUD",     SERIAL3_BAUD/1000),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: RTL_ALT
    // @DisplayName: RTL Altitude
    // @Description: The minimum altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
    // @Units: Centimeters
    // @Range: 0 8000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_altitude,   "RTL_ALT",     RTL_ALT),

    // @Param: SONAR_ENABLE
    // @DisplayName: Enable Sonar
    // @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(sonar_enabled,  "SONAR_ENABLE",     DISABLED),

    // @Param: SONAR_TYPE
    // @DisplayName: Sonar type
    // @Description: Used to adjust scaling to match the sonar used (only Maxbotix sonars are supported at this time)
    // @Values: 0:XL-EZ0 / XL-EZ4,1:LV-EZ0,2:XLL-EZ0,3:HRLV
    // @User: Standard
    GSCALAR(sonar_type,     "SONAR_TYPE",           AP_RANGEFINDER_MAXSONARXL),

    // @Param: SONAR_GAIN
    // @DisplayName: Sonar gain
    // @Description: Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(sonar_gain,     "SONAR_GAIN",           SONAR_GAIN_DEFAULT),

    // @Param: BATT_MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
    // @User: Standard
    GSCALAR(battery_monitoring, "BATT_MONITOR", BATT_MONITOR_DISABLED),

    // @Param: FS_BATT_ENABLE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(failsafe_battery_enabled, "FS_BATT_ENABLE", FS_BATTERY),

    // @Param: FS_GPS_ENABLE
    // @DisplayName: GPS Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when gps signal is lost
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(failsafe_gps_enabled, "FS_GPS_ENABLE", FS_GPS),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: VOLT_DIVIDER
    // @DisplayName: Voltage Divider
    // @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin voltage * INPUT_VOLTS/1024 * VOLT_DIVIDER)
    // @User: Advanced
    GSCALAR(volt_div_ratio, "VOLT_DIVIDER",     VOLT_DIV_RATIO),

    // @Param: AMP_PER_VOLT
    // @DisplayName: Current Amps per volt
    // @Description: Used to convert the voltage on the current sensing pin (BATT_CURR_PIN) to the actual current being consumed in amps (curr pin voltage * INPUT_VOLTS/1024 * AMP_PER_VOLT )
    // @User: Advanced
    GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT", CURR_AMP_PER_VOLT),

    // @Param: BATT_CAPACITY
    // @DisplayName: Battery Capacity
    // @Description: Battery capacity in milliamp-hours (mAh)
    // @Units: mAh
	// @User: Standard
    GSCALAR(pack_capacity,  "BATT_CAPACITY",    HIGH_DISCHARGE),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",   MAGNETOMETER),

    // @Param: FLOW_ENABLE
    // @DisplayName: Enable Optical Flow
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(optflow_enabled,        "FLOW_ENABLE",  DISABLED),

    // @Param: LOW_VOLT
    // @DisplayName: Low Voltage
    // @Description: Set this to the voltage you want to represent low voltage
    // @Range: 0 20
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(low_voltage,    "LOW_VOLT",         LOW_VOLTAGE),

    // @Param: SUPER_SIMPLE
    // @DisplayName: Enable Super Simple Mode
    // @Description: Setting this to Enabled(1) will enable Super Simple Mode. Setting this to Disabled(0) will disable Super Simple Mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(super_simple,   "SUPER_SIMPLE",     SUPER_SIMPLE),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
    // @Units: Centimeters
    // @Range: -1 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_alt_final,  "RTL_ALT_FINAL", RTL_ALT_FINAL),

    // @Param: BATT_VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
    // @User: Standard
    GSCALAR(battery_volt_pin,    "BATT_VOLT_PIN",    BATTERY_VOLT_PIN),

    // @Param: BATT_CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
    // @Values: -1:Disabled, 1:A1, 2:A2, 12:A12
    // @User: Standard
    GSCALAR(battery_curr_pin,    "BATT_CURR_PIN",    BATTERY_CURR_PIN),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:A0, 1:A1, 2:A2, 13:A13
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: THR_ACC_ENABLE
    // @DisplayName: Enable Accel based throttle controller
    // @Description: This allows enabling and disabling the accelerometer based throttle controller.  If disabled a velocity based controller is used.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(throttle_accel_enabled,  "THR_ACC_ENABLE",   1),

    // @Param: WP_YAW_BEHAVIOR
    // @DisplayName: Yaw behaviour during missions
    // @Description: Determines how the autopilot controls the yaw during missions and RTL
    // @Values: 0:Never change yaw, 1:Face next waypoint, 2:Face next waypoint except RTL
    // @User: Advanced
    GSCALAR(wp_yaw_behavior,  "WP_YAW_BEHAVIOR",    WP_YAW_BEHAVIOR_DEFAULT),

    // @Param: WP_TOTAL
    // @DisplayName: Waypoint Total
    // @Description: Total number of commands in the mission stored in the eeprom.  Do not update this parameter directly!
    // @User: Advanced
    GSCALAR(command_total,  "WP_TOTAL",         0),

    // @Param: WP_INDEX
    // @DisplayName: Waypoint Index
    // @Description: The index number of the command that is currently being executed.  Do not update this parameter directly!
    // @User: Advanced
    GSCALAR(command_index,  "WP_INDEX",         0),

    // @Param: CIRCLE_RADIUS
    // @DisplayName: Circle radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: Meters
    // @Range: 1 127
    // @Increment: 1
    // @User: Standard
    GSCALAR(circle_radius,  "CIRCLE_RADIUS",    CIRCLE_RADIUS),

    // @Param: CIRCLE_RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in degrees / second.  Positive to turn clockwise, negative for counter clockwise
    // @Units: Degrees / second
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    GSCALAR(circle_rate,  "CIRCLE_RATE",        CIRCLE_RATE),

    // @Param: RTL_LOIT_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before begining final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME",    RTL_LOITER_TIME),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: Centimeters/Second
    // @Range: 20 200
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed,             "LAND_SPEED",   LAND_SPEED),

    // @Param: PILOT_VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_velocity_z_max,     "PILOT_VELZ_MAX",   PILOT_VELZ_MAX),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: The minimum throttle that will be sent to the motors to keep them spinning
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_min,   "THR_MIN",          MINIMUM_THROTTLE),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: The maximum throttle that will be sent to the motors
    // @Units: ms
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_max,   "THR_MAX",          MAXIMUM_THROTTLE),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_DISABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle Trim
    // @Description: The autopilot's estimate of the throttle required to maintain a level hover.  Calculated automatically from the pilot's throttle input while in stabilize mode
    // @Range: 0 1000
    // @Units: PWM
    // @User: Standard
    GSCALAR(throttle_cruise,        "TRIM_THROTTLE",    THROTTLE_CRUISE),

    // @Param: THR_MID
    // @DisplayName: Throttle Mid Position
    // @Description: The throttle output (0 ~ 1000) when throttle stick is in mid position.  Used to scale the manual throttle so that the mid throttle stick position is close to the throttle required to hover
    // @User: Standard
    // @Range: 300 700
    // @Increment: 1
    GSCALAR(throttle_mid,        "THR_MID",    THR_MID),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,8:Position,9:Land,10:OF_Loiter,11:ToyA,12:ToyM
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,8:Position,9:Land,10:OF_Loiter,11:ToyA,12:ToyM
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,8:Position,9:Land,10:OF_Loiter,11:ToyA,12:ToyM
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,8:Position,9:Land,10:OF_Loiter,11:ToyA,12:ToyM
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,8:Position,9:Land,10:OF_Loiter,11:ToyA,12:ToyM
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,8:Position,9:Land,10:OF_Loiter,11:ToyA,12:ToyM
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               FLIGHT_MODE_6),

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 2 byte bitmap of log types to enable
    // @Values: 0:Disabled,830:Default,958:Default+IMU,1854:Default+Motors,17214:Default+INav
    // @User: Advanced
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: TOY_RATE
    // @DisplayName: Toy Yaw Rate
    // @Description: Controls yaw rate in Toy mode.  Higher values will cause a slower yaw rate.  Do not set to zero!
    // @User: Advanced
    // @Range: 1 10
    GSCALAR(toy_yaw_rate, "TOY_RATE",               1),

    // @Param: ESC
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up,1:Start-up in ESC Calibration mode
    GSCALAR(esc_calibrate, "ESC",                   0),

    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:CH6_NONE,1:CH6_STABILIZE_KP,2:CH6_STABILIZE_KI,3:CH6_YAW_KP,4:CH6_RATE_KP,5:CH6_RATE_KI,6:CH6_YAW_RATE_KP,7:CH6_THROTTLE_KP,9:CH6_RELAY,10:CH6_WP_SPEED,12:CH6_LOITER_KP,13:CH6_HELI_EXTERNAL_GYRO,14:CH6_THR_HOLD_KP,17:CH6_OPTFLOW_KP,18:CH6_OPTFLOW_KI,19:CH6_OPTFLOW_KD,21:CH6_RATE_KD,22:CH6_LOITER_RATE_KP,23:CH6_LOITER_RATE_KD,24:CH6_YAW_KI,25:CH6_ACRO_KP,26:CH6_YAW_RATE_KD,27:CH6_LOITER_KI,28:CH6_LOITER_RATE_KI,29:CH6_STABILIZE_KD,30:CH6_AHRS_YAW_KP,31:CH6_AHRS_KP,32:CH6_INAV_TC,33:CH6_THROTTLE_KI,34:CH6_THR_ACCEL_KP,35:CH6_THR_ACCEL_KI,36:CH6_THR_ACCEL_KD,38:CH6_DECLINATION,39:CH6_CIRCLE_RATE
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: TUNE_LOW
    // @DisplayName: Tuning minimum
    // @Description: The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_low, "TUNE_LOW",           0),

    // @Param: TUNE_HIGH
    // @DisplayName: Tuning maximum
    // @Description: The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_high, "TUNE_HIGH",         1000),

    // @Param: FRAME
    // @DisplayName: Frame Orientation (+, X or V)
    // @Description: Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
    // @Values: 0:Plus, 1:X, 2:V, 3:H
    // @User: Standard
    GSCALAR(frame_orientation, "FRAME",             FRAME_ORIENTATION),

    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:Sonar, 11:Fence, 12:ResetToArmedYaw
    // @User: Standard
    GSCALAR(ch7_option, "CH7_OPT",                  CH7_OPTION),

    // @Param: CH8_OPT
    // @DisplayName: Channel 8 option
    // @Description: Select which function if performed when CH8 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:Sonar, 11:Fence, 12:ResetToArmedYaw
    // @User: Standard
    GSCALAR(ch8_option, "CH8_OPT",                  CH8_OPTION),

    // @Param: ARMING_CHECK
    // @DisplayName: Arming check
    // @Description: Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer and compass
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(arming_check_enabled, "ARMING_CHECK",   1),

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: HS1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_1,    "HS1_", RC_Channel),
    // @Group: HS2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_2,    "HS2_", RC_Channel),
    // @Group: HS3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_3,    "HS3_", RC_Channel),
    // @Group: HS4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_4,    "HS4_", RC_Channel),

    // @Param: RATE_PIT_FF
    // @DisplayName: Rate Pitch Feed Forward
    // @Description: Rate Pitch Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @User: Standard
	GSCALAR(heli_pitch_ff, "RATE_PIT_FF",            HELI_PITCH_FF),

    // @Param: RATE_RLL_FF
    // @DisplayName: Rate Roll Feed Forward
    // @Description: Rate Roll Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @User: Standard
	GSCALAR(heli_roll_ff, "RATE_RLL_FF",            HELI_ROLL_FF),

    // @Param: RATE_YAW_FF
    // @DisplayName: Rate Yaw Feed Forward
    // @Description: Rate Yaw Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @User: Standard
	GSCALAR(heli_yaw_ff, "RATE_YAW_FF",            HELI_YAW_FF),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_1,    "RC1_", RC_Channel),
    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_2,    "RC2_", RC_Channel),
    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_3,    "RC3_", RC_Channel),
    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_4,    "RC4_", RC_Channel),
    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,    "RC5_", RC_Channel_aux),
    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,    "RC6_", RC_Channel_aux),
    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,    "RC7_", RC_Channel_aux),
    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,    "RC8_", RC_Channel_aux),

#if MOUNT == ENABLED
    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),

    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),
    // @Group: RC12_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_12,                   "RC12_", RC_Channel_aux),
#endif

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // @Param: ACRO_P
    // @DisplayName: Acro P gain
    // @Description: Used to convert pilot roll, pitch and yaw input into a dssired rate of rotation in ACRO mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_p,                 "ACRO_P",           ACRO_P),

    // @Param: AXIS_ENABLE
    // @DisplayName: Acro Axis
    // @Description: Used to control whether acro mode actively maintains the current angle when control sticks are released (Enabled = maintains current angle)
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(axis_enabled,           "AXIS_ENABLE",      AXIS_LOCK_ENABLED),

    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro mode
    // @Range: 0 300
    // @Increment: 1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro mode
    // @Range: 0 300
    // @Increment: 1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),

    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer Enabled
    // @Description: Set to 1 (Enabled) to make roll return to within 45 degrees of level automatically
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(acro_trainer_enabled,   "ACRO_TRAINER",     ACRO_TRAINER_ENABLED),

    // @Param: LED_MODE
    // @DisplayName: Copter LED Mode
    // @Description: bitmap to control the copter led mode
    // @Values: 0:Disabled,1:Enable,2:GPS On,4:Aux,8:Buzzer,16:Oscillate,32:Nav Blink,64:GPS Nav Blink
    // @User: Standard
    GSCALAR(copter_leds_mode,       "LED_MODE",         9),

    // PID controller
    //---------------
    // @Param: RATE_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.20
    // @User: Standard

    // @Param: RATE_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @User: Standard

    // @Param: RATE_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 500
    // @Units: PWM
    // @User: Standard

    // @Param: RATE_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.008
    // @User: Standard
    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),

    // @Param: RATE_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.08 0.20
    // @User: Standard

    // @Param: RATE_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.5
    // @User: Standard

    // @Param: RATE_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 500
    // @Units: PWM
    // @User: Standard

    // @Param: RATE_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.008
    // @User: Standard
    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),

    // @Param: RATE_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.150 0.250
    // @User: Standard

    // @Param: RATE_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 0.020
    // @User: Standard

    // @Param: RATE_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 500
    // @Units: PWM
    // @User: Standard

    // @Param: RATE_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.001
    // @User: Standard
    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),

    // @Param: LOITER_LAT_P
    // @DisplayName: Loiter latitude rate controller P gain
    // @Description: Loiter latitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the latitude direction
    // @Range: 2.000 6.000
    // @User: Standard

    // @Param: LOITER_LAT_I
    // @DisplayName: Loiter latitude rate controller I gain
    // @Description: Loiter latitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the latitude direction
    // @Range: 0.020 0.060
    // @User: Standard

    // @Param: LOITER_LAT_IMAX
    // @DisplayName: Loiter rate controller I gain maximum
    // @Description: Loiter rate controller I gain maximum.  Constrains the lean angle that the I gain will output
    // @Range: 0 4500
    // @Units: Centi-Degrees
    // @User: Standard

    // @Param: LOITER_LAT_D
    // @DisplayName: Loiter latitude rate controller D gain
    // @Description: Loiter latitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.200 0.600
    // @User: Standard
    GGROUP(pid_loiter_rate_lat,      "LOITER_LAT_",  AC_PID),

    // @Param: LOITER_LON_P
    // @DisplayName: Loiter longitude rate controller P gain
    // @Description: Loiter longitude rate controller P gain.  Converts the difference between desired speed and actual speed into a lean angle in the longitude direction
    // @Range: 2.000 6.000
    // @User: Standard

    // @Param: LOITER_LON_I
    // @DisplayName: Loiter longitude rate controller I gain
    // @Description: Loiter longitude rate controller I gain.  Corrects long-term difference in desired speed and actual speed in the longitude direction
    // @Range: 0.020 0.060
    // @User: Standard

    // @Param: LOITER_LON_IMAX
    // @DisplayName: Loiter longitude rate controller I gain maximum
    // @Description: Loiter longitude rate controller I gain maximum.  Constrains the lean angle that the I gain will output
    // @Range: 0 4500
    // @Units: Centi-Degrees
    // @User: Standard

    // @Param: LOITER_LON_D
    // @DisplayName: Loiter longituderate controller D gain
    // @Description: Loiter longitude rate controller D gain.  Compensates for short-term change in desired speed vs actual speed
    // @Range: 0.200 0.600
    // @User: Standard
    GGROUP(pid_loiter_rate_lon,      "LOITER_LON_",  AC_PID),

    // @Param: THR_RATE_P
    // @DisplayName: Throttle rate controller P gain
    // @Description: Throttle rate controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard

    // @Param: THR_RATE_I
    // @DisplayName: Throttle rate controller I gain
    // @Description: Throttle rate controller I gain.  Corrects long-term difference in desired vertical speed and actual speed
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: THR_RATE_IMAX
    // @DisplayName: Throttle rate controller I gain maximum
    // @Description: Throttle rate controller I gain maximum.  Constrains the desired acceleration that the I gain will generate
    // @Range: 0 500
    // @Units: cm/s/s
    // @User: Standard

    // @Param: THR_RATE_D
    // @DisplayName: Throttle rate controller D gain
    // @Description: Throttle rate controller D gain.  Compensates for short-term change in desired vertical speed vs actual speed
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_throttle,      "THR_RATE_", AC_PID),

    // @Param: THR_ACCEL_P
    // @DisplayName: Throttle acceleration controller P gain
    // @Description: Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @User: Standard

    // @Param: THR_ACCEL_I
    // @DisplayName: Throttle acceleration controller I gain
    // @Description: Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: THR_ACCEL_IMAX
    // @DisplayName: Throttle acceleration controller I gain maximum
    // @Description: Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 500
    // @Units: PWM
    // @User: Standard

    // @Param: THR_ACCEL_D
    // @DisplayName: Throttle acceleration controller D gain
    // @Description: Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_throttle_accel,"THR_ACCEL_", AC_PID),

    // @Param: OF_RLL_P
    // @DisplayName: Optical Flow based loiter controller roll axis P gain
    // @Description: Optical Flow based loiter controller roll axis P gain.  Converts the position error from the target point to a roll angle
    // @Range: 2.000 3.000
    // @User: Standard

    // @Param: OF_RLL_I
    // @DisplayName: Optical Flow based loiter controller roll axis I gain
    // @Description: Optical Flow based loiter controller roll axis I gain.  Corrects long-term position error by more persistently rolling left or right
    // @Range: 0.250 0.750
    // @User: Standard

    // @Param: OF_RLL_IMAX
    // @DisplayName: Optical Flow based loiter controller roll axis I gain maximum
    // @Description: Optical Flow based loiter controller roll axis I gain maximum.  Constrains the maximum roll angle that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees
    // @User: Standard

    // @Param: OF_RLL_D
    // @DisplayName: Optical Flow based loiter controller roll axis D gain
    // @Description: Optical Flow based loiter controller roll axis D gain.  Compensates for short-term change in speed in the roll direction
    // @Range: 0.100 0.140
    // @User: Standard
    GGROUP(pid_optflow_roll,  "OF_RLL_",   AC_PID),

    // @Param: OF_PIT_P
    // @DisplayName: Optical Flow based loiter controller pitch axis P gain
    // @Description: Optical Flow based loiter controller pitch axis P gain.  Converts the position error from the target point to a pitch angle
    // @Range: 2.000 3.000
    // @User: Standard

    // @Param: OF_PIT_I
    // @DisplayName: Optical Flow based loiter controller pitch axis I gain
    // @Description: Optical Flow based loiter controller pitch axis I gain.  Corrects long-term position error by more persistently pitching left or right
    // @Range: 0.250 0.750
    // @User: Standard

    // @Param: OF_PIT_IMAX
    // @DisplayName: Optical Flow based loiter controller pitch axis I gain maximum
    // @Description: Optical Flow based loiter controller pitch axis I gain maximum.  Constrains the maximum pitch angle that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees
    // @User: Standard

    // @Param: OF_PIT_D
    // @DisplayName: Optical Flow based loiter controller pitch axis D gain
    // @Description: Optical Flow based loiter controller pitch axis D gain.  Compensates for short-term change in speed in the pitch direction
    // @Range: 0.100 0.140
    // @User: Standard
    GGROUP(pid_optflow_pitch, "OF_PIT_",   AC_PID),

    // PI controller
    //--------------
    // @Param: STB_RLL_P
    // @DisplayName: Roll axis stabilize controller P gain
    // @Description: Roll axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_RLL_I
    // @DisplayName: Roll axis stabilize controller I gain
    // @Description: Roll axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired roll angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_RLL_IMAX
    // @DisplayName: Roll axis stabilize controller I gain maximum
    // @Description: Roll axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum roll rate that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_roll,       "STB_RLL_", APM_PI),

    // @Param: STB_PIT_P
    // @DisplayName: Pitch axis stabilize controller P gain
    // @Description: Pitch axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_PIT_I
    // @DisplayName: Pitch axis stabilize controller I gain
    // @Description: Pitch axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired pitch angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_PIT_IMAX
    // @DisplayName: Pitch axis stabilize controller I gain maximum
    // @Description: Pitch axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum pitch rate that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_pitch,      "STB_PIT_", APM_PI),

    // @Param: STB_YAW_P
    // @DisplayName: Yaw axis stabilize controller P gain
    // @Description: Yaw axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard

    // @Param: STB_YAW_I
    // @DisplayName: Yaw axis stabilize controller I gain
    // @Description: Yaw axis stabilize (i.e. angle) controller I gain.  Corrects for longer-term difference in desired yaw angle and actual angle
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: STB_YAW_IMAX
    // @DisplayName: Yaw axis stabilize controller I gain maximum
    // @Description: Yaw axis stabilize (i.e. angle) controller I gain maximum.  Constrains the maximum yaw rate that the I term will generate
    // @Range: 0 4500
    // @Units: Centi-Degrees/Sec
    // @User: Standard
    GGROUP(pi_stabilize_yaw,        "STB_YAW_", APM_PI),

    // @Param: THR_ALT_P
    // @DisplayName: Altitude controller P gain
    // @Description: Altitude controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard

    // @Param: THR_ALT_I
    // @DisplayName: Altitude controller I gain
    // @Description: Altitude controller I gain.  Corrects for longer-term difference in desired altitude and actual altitude
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: THR_ALT_IMAX
    // @DisplayName: Altitude controller I gain maximum
    // @Description: Altitude controller I gain maximum.  Constrains the maximum climb rate rate that the I term will generate
    // @Range: 0 500
    // @Units: cm/s
    // @User: Standard
    GGROUP(pi_alt_hold,     "THR_ALT_", APM_PI),

    // @Param: HLD_LAT_P
    // @DisplayName: Loiter latitude position controller P gain
    // @Description: Loiter latitude position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.100 0.300
    // @User: Standard

    // @Param: HLD_LAT_I
    // @DisplayName: Loiter latitude position controller I gain
    // @Description: Loiter latitude position controller I gain.  Corrects for longer-term distance (in latitude) to the target location
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: HLD_LAT_IMAX
    // @DisplayName: Loiter latitude position controller I gain maximum
    // @Description: Loiter latitude position controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
    // @Range: 0 3000
    // @Units: cm/s
    // @User: Standard
    GGROUP(pi_loiter_lat,   "HLD_LAT_", APM_PI),

    // @Param: HLD_LON_P
    // @DisplayName: Loiter longitude position controller P gain
    // @Description: Loiter longitude position controller P gain.  Converts the distance (in the longitude direction) to the target location into a desired speed which is then passed to the loiter longitude rate controller
    // @Range: 0.100 0.300
    // @User: Standard

    // @Param: HLD_LON_I
    // @DisplayName: Loiter longitude position controller I gain
    // @Description: Loiter longitude position controller I gain.  Corrects for longer-term distance (in longitude direction) to the target location
    // @Range: 0.000 0.100
    // @User: Standard

    // @Param: HLD_LON_IMAX
    // @DisplayName: Loiter longitudeposition controller I gain maximum
    // @Description: Loiter  longitudeposition controller I gain maximum.  Constrains the maximum desired speed that the I term will generate
    // @Range: 0 3000
    // @Units: cm/s
    // @User: Standard
    GGROUP(pi_loiter_lon,   "HLD_LON_", APM_PI),

    // variables not in the g class which contain EEPROM saved variables

    // variables not in the g class which contain EEPROM saved variables
#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,           "CAM_", AP_Camera),
#endif

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
#if HIL_MODE == HIL_MODE_DISABLED
    GOBJECT(ins,            "INS_", AP_InertialSensor),
#endif

    // @Group: INAV_
    // @Path: ../libraries/AP_InertialNav/AP_InertialNav.cpp
    GOBJECT(inertial_nav,           "INAV_",    AP_InertialNav),

    //@Group: WPNAV_
    //@Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECT(wp_nav, "WPNAV_",       AC_WPNav),

    // @Group: SR0_
    // @Path: ./GCS_Mavlink.pde
    GOBJECT(gcs0,                   "SR0_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: ./GCS_Mavlink.pde
    GOBJECT(gcs3,                   "SR3_",     GCS_MAVLINK),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT_", AP_Mount),
#endif

#if MOUNT2 == ENABLED
    // @Group: MNT2_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    GOBJECT(sitl, "SIM_", SITL),
#endif

    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

#if AC_FENCE == ENABLED
    //@Group: FENCE_
    //@Path: ../libraries/AC_Fence/AC_Fence.cpp
    GOBJECT(fence,      "FENCE_",   AC_Fence),
#endif

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECT(motors, "H_",           AP_MotorsHeli),
#else
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_Motors_Class.cpp
    GOBJECT(motors, "MOT_",         AP_Motors),
#endif

    AP_VAREND
};


static void load_parameters(void)
{
    // change the default for the AHRS_GPS_GAIN for ArduCopter
    // if it hasn't been set by the user
    if (!ahrs.gps_gain.load()) {
        ahrs.gps_gain.set_and_save(1.0);
    }
    // disable centrifugal force correction, it will be enabled as part of the arming process
    ahrs.set_correct_centrifugal(false);

    // setup different AHRS gains for ArduCopter than the default
    // but allow users to override in their config
    if (!ahrs._kp.load()) {
        ahrs._kp.set_and_save(0.1);
    }
    if (!ahrs._kp_yaw.load()) {
        ahrs._kp_yaw.set_and_save(0.1);
    }

#if SECONDARY_DMP_ENABLED == ENABLED
    if (!ahrs2._kp.load()) {
        ahrs2._kp.set(0.1);
    }
    if (!ahrs2._kp_yaw.load()) {
        ahrs2._kp_yaw.set(0.1);
    }
#endif

    // setup different Compass learn setting for ArduCopter than the default
    // but allow users to override in their config
    if (!compass._learn.load()) {
        compass._learn.set_and_save(0);
    }

    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        default_dead_zones();
        cliSerial->println_P(PSTR("done."));
    } else {
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();

        cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/UserCode.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
#line 1 "/home/tobias/git/ardupilot/ArduCopter/commands.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_commands()
{
    g.command_index         = NO_COMMAND;
    command_nav_index       = NO_COMMAND;
    command_cond_index      = NO_COMMAND;
    prev_nav_index          = NO_COMMAND;
    command_cond_queue.id   = NO_COMMAND;
    command_nav_queue.id    = NO_COMMAND;
}

// Getters
// -------
static struct Location get_cmd_with_index(int i)
{
    struct Location temp;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (i >= g.command_total) {
        // we do not have a valid command to load
        // return a WP with a "Blank" id
        temp.id = CMD_BLANK;

        // no reason to carry on
        return temp;

    }else{
        // we can load a command, we don't process it yet
        // read WP position
        uint16_t mem = (WP_START_BYTE) + (i * WP_SIZE);

        temp.id = hal.storage->read_byte(mem);

        mem++;
        temp.options = hal.storage->read_byte(mem);

        mem++;
        temp.p1 = hal.storage->read_byte(mem);

        mem++;
        temp.alt = hal.storage->read_dword(mem);           // alt is stored in CM! Alt is stored relative!

        mem += 4;
        temp.lat = hal.storage->read_dword(mem);         // lat is stored in decimal * 10,000,000

        mem += 4;
        temp.lng = hal.storage->read_dword(mem);         // lon is stored in decimal * 10,000,000
    }

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    //if((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) && (temp.options & MASK_OPTIONS_RELATIVE_ALT)){
    //temp.alt += home.alt;
    //}

    if(temp.options & WP_OPTION_RELATIVE) {
        // If were relative, just offset from home
        temp.lat        +=      home.lat;
        temp.lng        +=      home.lng;
    }

    return temp;
}

// Setters
// -------
static void set_cmd_with_index(struct Location temp, int i)
{

    i = constrain_int16(i, 0, g.command_total.get());
    //cliSerial->printf("set_command: %d with id: %d\n", i, temp.id);

    // store home as 0 altitude!!!
    // Home is always a MAV_CMD_NAV_WAYPOINT (16)
    if (i == 0) {
        temp.alt = 0;
        temp.id = MAV_CMD_NAV_WAYPOINT;
    }

    uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

    hal.storage->write_byte(mem, temp.id);

    mem++;
    hal.storage->write_byte(mem, temp.options);

    mem++;
    hal.storage->write_byte(mem, temp.p1);

    mem++;
    hal.storage->write_dword(mem, temp.alt);     // Alt is stored in CM!

    mem += 4;
    hal.storage->write_dword(mem, temp.lat);     // Lat is stored in decimal degrees * 10^7

    mem += 4;
    hal.storage->write_dword(mem, temp.lng);     // Long is stored in decimal degrees * 10^7

    // Make sure our WP_total
    if(g.command_total < (i+1))
        g.command_total.set_and_save(i+1);
}

static int32_t get_RTL_alt()
{
    if(g.rtl_altitude <= 0) {
		return min(current_loc.alt, RTL_ALT_MAX);
    }else if (g.rtl_altitude < current_loc.alt) {
		return min(current_loc.alt, RTL_ALT_MAX);
    }else{
        return g.rtl_altitude;
    }
}

// run this at setup on the ground
// -------------------------------
static void init_home()
{
    set_home_is_set(true);
    home.id         = MAV_CMD_NAV_WAYPOINT;
    home.lng        = g_gps->longitude;                                 // Lon * 10**7
    home.lat        = g_gps->latitude;                                  // Lat * 10**7
    home.alt        = 0;                                                        // Home is always 0

    // Save Home to EEPROM
    // -------------------
    // no need to save this to EPROM
    set_cmd_with_index(home, 0);

    // set inertial nav's home position
    inertial_nav.set_current_position(g_gps->longitude, g_gps->latitude);

    if (g.log_bitmask & MASK_LOG_CMD)
        Log_Write_Cmd(0, &home);

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    scaleLongDown = longitude_scale(&home);
    scaleLongUp   = 1.0f/scaleLongDown;
}



#line 1 "/home/tobias/git/ardupilot/ArduCopter/commands_logic.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
// process_nav_command - main switch statement to initiate the next nav command in the command_nav_queue
static void process_nav_command()
{
    switch(command_nav_queue.id) {

    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle();
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    // point the copter and camera at a region of interest (ROI)
    case MAV_CMD_NAV_ROI:             // 80
        do_nav_roi();
        break;

    default:
        break;
    }

}

// process_cond_command - main switch statement to initiate the next conditional command in the command_cond_queue
static void process_cond_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:             // 113
        do_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw();
        break;

    default:
        break;
    }
}

// process_now_command - main switch statement to initiate the next now command in the command_cond_queue
// now commands are conditional commands that are executed immediately so they do not require a corresponding verify to be run later
static void process_now_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_DO_JUMP:              // 177
        do_jump();
        break;

    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed();
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home();
        break;

    case MAV_CMD_DO_SET_SERVO:             // 183
        do_set_servo();
        break;

    case MAV_CMD_DO_SET_RELAY:             // 181
        do_set_relay();
        break;

    case MAV_CMD_DO_REPEAT_SERVO:             // 184
        do_repeat_servo();
        break;

    case MAV_CMD_DO_REPEAT_RELAY:             // 182
        do_repeat_relay();
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        do_take_picture();
        break;
#endif

#if MOUNT == ENABLED
    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
        break;
#endif

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// verify_nav_command - switch statement to ensure the active navigation command is progressing
// returns true once the active navigation command completes successfully
static bool verify_nav_command()
{
    switch(command_nav_queue.id) {

    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:
        return verify_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle();
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();
        break;

    case MAV_CMD_NAV_ROI:             // 80
        return verify_nav_roi();
        break;

    default:
        //gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current Must commands"));
        return false;
        break;
    }
}

// verify_cond_command - switch statement to ensure the active conditional command is progressing
// returns true once the active conditional command completes successfully
static bool verify_cond_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();
        break;

    default:
        //gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current May commands"));
        return false;
        break;
    }
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
static void do_RTL(void)
{
    // set rtl state
    rtl_state = RTL_STATE_START;

    // verify_RTL will do the initialisation for us
    verify_RTL();
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
static void do_takeoff()
{
    // set roll-pitch mode
    set_roll_pitch_mode(AUTO_RP);

    // set yaw mode
    set_yaw_mode(YAW_HOLD);

    // set throttle mode to AUTO although we should already be in this mode
    set_throttle_mode(THROTTLE_AUTO);

    // set our nav mode to loiter
    set_nav_mode(NAV_WP);

    // Set wp navigation target to safe altitude above current position
    Vector3f pos = inertial_nav.get_position();
    pos.z = max(pos.z, command_nav_queue.alt);
    wp_nav.set_destination(pos);

    // prevent flips
    // To-Do: check if this is still necessary
    reset_I_all();    
}

// do_nav_wp - initiate move to next waypoint
static void do_nav_wp()
{
    // set roll-pitch mode
    set_roll_pitch_mode(AUTO_RP);

    // set throttle mode
    set_throttle_mode(THROTTLE_AUTO);

    // set nav mode
    set_nav_mode(NAV_WP);

    // Set wp navigation target
    wp_nav.set_destination(pv_location_to_vector(command_nav_queue));

    // initialise original_wp_bearing which is used to check if we have missed the waypoint
    wp_bearing = wp_nav.get_bearing_to_destination();
    original_wp_bearing = wp_bearing;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time     = 0;
    // this is the delay, stored in seconds and expanded to millis
    loiter_time_max = command_nav_queue.p1;
    // if no delay set the waypoint as "fast"
    if (loiter_time_max == 0 ) {
        wp_nav.set_fast_waypoint(true);
    }

    // set yaw_mode depending upon contents of WP_YAW_BEHAVIOR parameter
    set_yaw_mode(get_wp_yaw_mode(false));
}

// do_land - initiate landing procedure
// caller should set roll_pitch_mode to ROLL_PITCH_AUTO (for no pilot input) or ROLL_PITCH_LOITER (for pilot input)
// caller should set yaw_mode
static void do_land()
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (command_nav_queue.lat != 0 || command_nav_queue.lng != 0) {
        // set state to fly to location
        land_state = LAND_STATE_FLY_TO_LOCATION;

        // set roll-pitch mode
        set_roll_pitch_mode(AUTO_RP);

        // set yaw_mode depending upon contents of WP_YAW_BEHAVIOR parameter
        set_yaw_mode(get_wp_yaw_mode(false));

        // set throttle mode
        set_throttle_mode(THROTTLE_AUTO);

        // set nav mode
        set_nav_mode(NAV_WP);

        // calculate and set desired location above landing target
        Vector3f pos = pv_location_to_vector(command_nav_queue);
        pos.z = min(current_loc.alt, RTL_ALT_MAX);
        wp_nav.set_destination(pos);
    }else{
        // set landing state
        land_state = LAND_STATE_DESCENDING;

        // switch to loiter which restores horizontal control to pilot
        // To-Do: check that we are not in failsafe to ensure we don't process bad roll-pitch commands
        set_roll_pitch_mode(ROLL_PITCH_LOITER);

        // hold yaw while landing
        set_yaw_mode(YAW_HOLD);

        // set throttle mode to land
        set_throttle_mode(THROTTLE_LAND);

        // switch into loiter nav mode
        set_nav_mode(NAV_LOITER);
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
static void do_loiter_unlimited()
{
    // set roll-pitch mode (no pilot input)
    set_roll_pitch_mode(AUTO_RP);

    // set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
    set_throttle_mode(THROTTLE_AUTO);

    // hold yaw
    set_yaw_mode(YAW_HOLD);

    // get current position
    // To-Do: change this to projection based on current location and velocity
    Vector3f curr = inertial_nav.get_position();

    // default to use position provided
    Vector3f pos = pv_location_to_vector(command_nav_queue);

    // use current altitude if not provided
    if( command_nav_queue.alt == 0 ) {
        pos.z = curr.z;
    }

    // use current location if not provided
    if(command_nav_queue.lat == 0 && command_nav_queue.lng == 0) {
        pos.x = curr.x;
        pos.y = curr.y;
    }

    // start way point navigator and provide it the desired location
    set_nav_mode(NAV_WP);
    wp_nav.set_destination(pos);
}

// do_circle - initiate moving in a circle
static void do_circle()
{
    // set roll-pitch mode (no pilot input)
    set_roll_pitch_mode(AUTO_RP);

    // set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
    set_throttle_mode(THROTTLE_AUTO);

    // set nav mode to CIRCLE
    set_nav_mode(NAV_CIRCLE);

    // set target altitude if provided
    if( command_nav_queue.alt != 0 ) {
        wp_nav.set_desired_alt(command_nav_queue.alt);
    }

    // override default horizontal location target
    if( command_nav_queue.lat != 0 || command_nav_queue.lng != 0) {
        circle_set_center(pv_location_to_vector(command_nav_queue), ahrs.yaw);
    }

    // set yaw to point to center of circle
    set_yaw_mode(CIRCLE_YAW);

    // set angle travelled so far to zero
    circle_angle_total = 0;

    // record number of desired rotations from mission command
    circle_desired_rotations = command_nav_queue.p1;
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
static void do_loiter_time()
{
    // set roll-pitch mode (no pilot input)
    set_roll_pitch_mode(AUTO_RP);

    // set throttle mode to AUTO which, if not already active, will default to hold at our current altitude
    set_throttle_mode(THROTTLE_AUTO);

    // hold yaw
    set_yaw_mode(YAW_HOLD);

    // get current position
    // To-Do: change this to projection based on current location and velocity
    Vector3f curr = inertial_nav.get_position();

    // default to use position provided
    Vector3f pos = pv_location_to_vector(command_nav_queue);

    // use current altitude if not provided
    if( command_nav_queue.alt == 0 ) {
        pos.z = curr.z;
    }

    // use current location if not provided
    if(command_nav_queue.lat == 0 && command_nav_queue.lng == 0) {
        pos.x = curr.x;
        pos.y = curr.y;
    }

    // start way point navigator and provide it the desired location
    set_nav_mode(NAV_WP);
    wp_nav.set_destination(pos);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = command_nav_queue.p1;     // units are (seconds)
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
static bool verify_takeoff()
{
    // wait until we are ready!
    if(g.rc_3.control_in == 0) {
        // To-Do: reset loiter target if we have not yet taken-off
        // do not allow I term to build up if we have not yet taken-off
        return false;
    }
    // have we reached our target altitude?
    return wp_nav.reached_destination();
}

// verify_land - returns true if landing has been completed
static bool verify_land()
{
    bool retval = false;

    switch( land_state ) {
        case LAND_STATE_FLY_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_destination()) {
                // get destination so we can use it for loiter target
                Vector3f dest = wp_nav.get_destination();

                // switch into loiter nav mode
                set_nav_mode(NAV_LOITER);

                // override loiter target
                wp_nav.set_loiter_target(dest);

                // switch to loiter which restores horizontal control to pilot
                // To-Do: check that we are not in failsafe to ensure we don't process bad roll-pitch commands
                set_roll_pitch_mode(ROLL_PITCH_LOITER);

                // give pilot control of yaw
                set_yaw_mode(YAW_HOLD);

                // set throttle mode to land
                set_throttle_mode(THROTTLE_LAND);

                // advance to next state
                land_state = LAND_STATE_DESCENDING;
            }
            break;

        case LAND_STATE_DESCENDING:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

// verify_nav_wp - check if we have reached the next way point
static bool verify_nav_wp()
{
    // check if we have reached the waypoint
    if( !wp_nav.reached_destination() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),command_nav_index);
        copter_leds_nav_blink = 15;             // Cause the CopterLEDs to blink three times to indicate waypoint reached
        return true;
    }else{
        return false;
    }
}

static bool verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
static bool verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav.reached_destination()) {
        return false;
    }

    // start our loiter timer
    if( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
static bool verify_circle()
{
    // have we rotated around the center enough times?
    return fabsf(circle_angle_total/(2*M_PI)) >= circle_desired_rotations;
}

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
static bool verify_RTL()
{
    bool retval = false;

    switch( rtl_state ) {
        case RTL_STATE_START:
            // set roll, pitch and yaw modes
            set_roll_pitch_mode(RTL_RP);
            set_throttle_mode(RTL_THR);

            // set navigation mode
            set_nav_mode(NAV_WP);

            // if we are below rtl alt do initial climb
            if( current_loc.alt < get_RTL_alt() ) {
                // first stage of RTL is the initial climb so just hold current yaw
                set_yaw_mode(YAW_HOLD);

                // use projection of safe stopping point based on current location and velocity
                Vector3f origin, dest;
                wp_nav.get_stopping_point(inertial_nav.get_position(),inertial_nav.get_velocity(),origin);
                dest.x = origin.x;
                dest.y = origin.y;
                dest.z = get_RTL_alt();
                wp_nav.set_origin_and_destination(origin,dest);

                // advance to next rtl state
                rtl_state = RTL_STATE_INITIAL_CLIMB;
            }else{
                // point nose towards home (maybe)
                set_yaw_mode(get_wp_yaw_mode(true));

                // Set wp navigation target to above home
                wp_nav.set_destination(Vector3f(0,0,get_RTL_alt()));

                // initialise original_wp_bearing which is used to point the nose home
                wp_bearing = wp_nav.get_bearing_to_destination();
                original_wp_bearing = wp_bearing;
                
                // advance to next rtl state
                rtl_state = RTL_STATE_RETURNING_HOME;
            }
            break;
        case RTL_STATE_INITIAL_CLIMB:
            // check if we've reached the safe altitude
            if (wp_nav.reached_destination()) {
                // set nav mode
                set_nav_mode(NAV_WP);

                // Set wp navigation target to above home
                wp_nav.set_destination(Vector3f(0,0,get_RTL_alt()));

                // initialise original_wp_bearing which is used to point the nose home
                wp_bearing = wp_nav.get_bearing_to_destination();
                original_wp_bearing = wp_bearing;

                // point nose towards home (maybe)
                set_yaw_mode(get_wp_yaw_mode(true));

                // advance to next rtl state
                rtl_state = RTL_STATE_RETURNING_HOME;
            }
            break;

        case RTL_STATE_RETURNING_HOME:
            // check if we've reached home
            if (wp_nav.reached_destination()) {
                // Note: we remain in NAV_WP nav mode which should hold us above home

                // start timer
                rtl_loiter_start_time = millis();

                // give pilot back control of yaw
                set_yaw_mode(YAW_HOLD);

                // advance to next rtl state
                rtl_state = RTL_STATE_LOITERING_AT_HOME;
            }
            break;

        case RTL_STATE_LOITERING_AT_HOME:
            // check if we've loitered long enough
            if( millis() - rtl_loiter_start_time > (uint32_t)g.rtl_loiter_time.get() ) {
                // initiate landing or descent
                if(g.rtl_alt_final == 0 || ap.failsafe_radio) {
                    // switch to loiter which restores horizontal control to pilot
                    // To-Do: check that we are not in failsafe to ensure we don't process bad roll-pitch commands
                    set_roll_pitch_mode(ROLL_PITCH_LOITER);
                    // switch into loiter nav mode
                    set_nav_mode(NAV_LOITER);
                    // override landing location (loiter defaults to a projection from current location)
                    wp_nav.set_loiter_target(Vector3f(0,0,0));

                    // hold yaw while landing
                    set_yaw_mode(YAW_HOLD);

                    // set throttle mode to land
                    set_throttle_mode(THROTTLE_LAND);

                    // update RTL state
                    rtl_state = RTL_STATE_LAND;
                }else{
                    // descend using waypoint controller
                    if(current_loc.alt > g.rtl_alt_final) {
                        // set navigation mode
                        set_nav_mode(NAV_WP);
                        // Set wp navigation alt target to rtl_alt_final
                        wp_nav.set_destination(Vector3f(0,0,g.rtl_alt_final));
                    }
                    // update RTL state
                    rtl_state = RTL_STATE_FINAL_DESCENT;
                }
            }
            break;

        case RTL_STATE_FINAL_DESCENT:
            // check we have reached final altitude
            if(current_loc.alt <= g.rtl_alt_final || wp_nav.reached_destination()) {
                // indicate that we've completed RTL
                retval = true;
            }
            break;

        case RTL_STATE_LAND:
            // rely on land_complete flag to indicate if we have landed
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully completed RTL
    return retval;
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
    //cliSerial->print("dwd ");
    condition_start = millis();
    condition_value = command_cond_queue.lat * 1000;     // convert to milliseconds
    //cliSerial->println(condition_value,DEC);
}

static void do_change_alt()
{
    // adjust target appropriately for each nav mode
    switch (nav_mode) {
        case NAV_CIRCLE:
        case NAV_LOITER:
            // update loiter target altitude
            wp_nav.set_desired_alt(command_cond_queue.alt);
            break;

        case NAV_WP:
            // To-Do: update waypoint nav's destination altitude
            break;
    }

    // To-Do: store desired altitude in a variable so that it can be verified later
}

static void do_within_distance()
{
    condition_value  = command_cond_queue.lat * 100;
}

static void do_yaw()
{
    // get final angle, 1 = Relative, 0 = Absolute
    if( command_cond_queue.lng == 0 ) {
        // absolute angle
        yaw_look_at_heading = wrap_360_cd(command_cond_queue.alt * 100);
    }else{
        // relative angle
        yaw_look_at_heading = wrap_360_cd(nav_yaw + command_cond_queue.alt * 100);
    }

    // get turn speed
    if( command_cond_queue.lat == 0 ) {
        // default to regular auto slew rate
        yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    }else{
        int32_t turn_rate = (wrap_180_cd(yaw_look_at_heading - nav_yaw) / 100) / command_cond_queue.lat;
        yaw_look_at_heading_slew = constrain_int32(turn_rate, 1, 360);    // deg / sec
    }

    // set yaw mode
    set_yaw_mode(YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise / counter clockwise rotation held in command_cond_queue.p1
    // command_cond_queue.p1; // 0 = undefined, 1 = clockwise, -1 = counterclockwise
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
    //cliSerial->print("vwd");
    if (millis() - condition_start > (uint32_t)max(condition_value,0)) {
        //cliSerial->println("y");
        condition_value = 0;
        return true;
    }
    //cliSerial->println("n");
    return false;
}

static bool verify_change_alt()
{
    // To-Do: use recorded target altitude to verify we have reached the target
    return true;
}

static bool verify_within_distance()
{
    //cliSerial->printf("cond dist :%d\n", (int)condition_value);
    if (wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
static bool verify_yaw()
{
    if( labs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200 ) {
        return true;
    }else{
        return false;
    }
}

// verify_nav_roi - verifies that actions required by MAV_CMD_NAV_ROI have completed
//              we assume the camera command has been successfully implemented by the do_nav_roi command
//              so all we need to check is whether we needed to yaw the copter (due to the mount type) and
//              whether that yaw has completed
//	TO-DO: add support for other features of MAV_NAV_ROI including pointing at a given waypoint
static bool verify_nav_roi()
{
#if MOUNT == ENABLED
    // check if mount type requires us to rotate the quad
    if( camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll ) {
        // ensure yaw has gotten to within 2 degrees of the target
        if( labs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_WP_bearing)) <= 200 ) {
            return true;
        }else{
            return false;
        }
    }else{
        // if no rotation required, assume the camera instruction was implemented immediately
        return true;
    }
#else
    // if we have no camera mount simply check we've reached the desired yaw
    // ensure yaw has gotten to within 2 degrees of the target
    if( labs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_WP_bearing)) <= 200 ) {
        return true;
    }else{
        return false;
    }
#endif
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

static void do_change_speed()
{
    wp_nav.set_horizontal_velocity(command_cond_queue.p1 * 100);
}

static void do_jump()
{
    // Used to track the state of the jump command in Mission scripting
    // -10 is a value that means the register is unused
    // when in use, it contains the current remaining jumps
    static int8_t jump = -10;                                                                   // used to track loops in jump command

    //cliSerial->printf("do Jump: %d\n", jump);

    if(jump == -10) {
        //cliSerial->printf("Fresh Jump\n");
        // we use a locally stored index for jump
        jump = command_cond_queue.lat;
    }
    //cliSerial->printf("Jumps left: %d\n",jump);

    if(jump > 0) {
        //cliSerial->printf("Do Jump to %d\n",command_cond_queue.p1);
        jump--;
        change_command(command_cond_queue.p1);

    } else if (jump == 0) {
        //cliSerial->printf("Did last jump\n");
        // we're done, move along
        jump = -11;

    } else if (jump == -1) {
        //cliSerial->printf("jumpForever\n");
        // repeat forever
        change_command(command_cond_queue.p1);
    }
}

static void do_set_home()
{
    if(command_cond_queue.p1 == 1) {
        init_home();
    } else {
        home.id         = MAV_CMD_NAV_WAYPOINT;
        home.lng        = command_cond_queue.lng;                                       // Lon * 10**7
        home.lat        = command_cond_queue.lat;                                       // Lat * 10**7
        home.alt        = 0;
        //home_is_set 	= true;
        set_home_is_set(true);
    }
}

static void do_set_servo()
{
    uint8_t channel_num = 0xff;

    switch( command_cond_queue.p1 ) {
        case 1:
            channel_num = CH_1;
            break;
        case 2:
            channel_num = CH_2;
            break;
        case 3:
            channel_num = CH_3;
            break;
        case 4:
            channel_num = CH_4;
            break;
        case 5:
            channel_num = CH_5;
            break;
        case 6:
            channel_num = CH_6;
            break;
        case 7:
            channel_num = CH_7;
            break;
        case 8:
            channel_num = CH_8;
            break;
        case 9:
            // not used
            break;
        case 10:
            channel_num = CH_10;
            break;
        case 11:
            channel_num = CH_11;
            break;
    }

    // send output to channel
    if (channel_num != 0xff) {
        hal.rcout->enable_ch(channel_num);
        hal.rcout->write(channel_num, command_cond_queue.alt);
    }
}

static void do_set_relay()
{
    if (command_cond_queue.p1 == 1) {
        relay.on();
    } else if (command_cond_queue.p1 == 0) {
        relay.off();
    }else{
        relay.toggle();
    }
}

static void do_repeat_servo()
{
    event_id = command_cond_queue.p1 - 1;

    if(command_cond_queue.p1 >= CH_5 + 1 && command_cond_queue.p1 <= CH_8 + 1) {

        event_timer             = 0;
        event_value             = command_cond_queue.alt;
        event_repeat    = command_cond_queue.lat * 2;
        event_delay             = command_cond_queue.lng * 500.0f;         // /2 (half cycle time) * 1000 (convert to milliseconds)

        switch(command_cond_queue.p1) {
        case CH_5:
            event_undo_value = g.rc_5.radio_trim;
            break;
        case CH_6:
            event_undo_value = g.rc_6.radio_trim;
            break;
        case CH_7:
            event_undo_value = g.rc_7.radio_trim;
            break;
        case CH_8:
            event_undo_value = g.rc_8.radio_trim;
            break;
        }
        update_events();
    }
}

static void do_repeat_relay()
{
    event_id                = RELAY_TOGGLE;
    event_timer             = 0;
    event_delay             = command_cond_queue.lat * 500.0f;     // /2 (half cycle time) * 1000 (convert to milliseconds)
    event_repeat    = command_cond_queue.alt * 2;
    update_events();
}

// do_nav_roi - starts actions required by MAV_CMD_NAV_ROI
//              this involves either moving the camera to point at the ROI (region of interest)
//              and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//				Note: the ROI should already be in the command_nav_queue global variable
//	TO-DO: add support for other features of MAV_NAV_ROI including pointing at a given waypoint
static void do_nav_roi()
{
#if MOUNT == ENABLED

    // check if mount type requires us to rotate the quad
    if( camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll ) {
        yaw_look_at_WP = pv_location_to_vector(command_nav_queue);
        set_yaw_mode(YAW_LOOK_AT_LOCATION);
    }
    // send the command to the camera mount
    camera_mount.set_roi_cmd(&command_nav_queue);

    // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
    //		0: do nothing
    //		1: point at next waypoint
    //		2: point at a waypoint taken from WP# parameter (2nd parameter?)
    //		3: point at a location given by alt, lon, lat parameters
    //		4: point at a target given a target id (can't be implmented)
#else
    // if we have no camera mount aim the quad at the location
    yaw_look_at_WP = pv_location_to_vector(command_nav_queue);
    set_yaw_mode(YAW_LOOK_AT_LOCATION);
#endif
}

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
    if (g.log_bitmask & MASK_LOG_CAMERA) {
        Log_Write_Camera();
    }
#endif
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/commands_process.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
static void change_command(uint8_t cmd_index)
{
    //cliSerial->printf("change_command: %d\n",cmd_index );
    // limit range
    cmd_index = min(g.command_total - 1, cmd_index);

    // load command
    struct Location temp = get_cmd_with_index(cmd_index);

    //cliSerial->printf("loading cmd: %d with id:%d\n", cmd_index, temp.id);

    // verify it's a nav command
    if(temp.id > MAV_CMD_NAV_LAST) {
        //gcs_send_text_P(SEVERITY_LOW,PSTR("error: non-Nav cmd"));

    }else{
        // clear out command queue
        init_commands();

        // copy command to the queue
        command_nav_queue               = temp;
        command_nav_index               = cmd_index;
        execute_nav_command();
    }
}

// update_commands - initiates new navigation commands if we have completed the previous command
// called by 10 Hz loop
static void update_commands()
{
    //cliSerial->printf("update_commands: %d\n",increment );
    // A: if we do not have any commands there is nothing to do
    // B: We have completed the mission, don't redo the mission
    // XXX debug
    //uint8_t tmp = g.command_index.get();
    //cliSerial->printf("command_index %u \n", tmp);

    if(g.command_total <= 1)
        return;

    if(command_nav_queue.id == NO_COMMAND) {
        // Our queue is empty
        // fill command queue with a new command if available, or exit mission
        // -------------------------------------------------------------------

        // find next nav command
        int16_t tmp_index;

        if(command_nav_index < g.command_total) {

            // what is the next index for a nav command?
            tmp_index = find_next_nav_index(command_nav_index + 1);

            if(tmp_index == -1) {
                exit_mission();
                return;
            }else{
                command_nav_index = tmp_index;
                command_nav_queue = get_cmd_with_index(command_nav_index);
                execute_nav_command();
            }
        }else{
            // we are out of commands
            exit_mission();
            return;
        }
    }

    if(command_cond_queue.id == NO_COMMAND) {
        // Our queue is empty
        // fill command queue with a new command if available, or do nothing
        // -------------------------------------------------------------------

        // no nav commands completed yet
        if(prev_nav_index == NO_COMMAND)
            return;

        if(command_cond_index >= command_nav_index) {
            // don't process the fututre
            return;

        }else if(command_cond_index == NO_COMMAND) {
            // start from scratch
            // look at command after the most recent completed nav
            command_cond_index = prev_nav_index + 1;

        }else{
            // we've completed 1 cond, look at next command for another
            command_cond_index++;
        }

        if(command_cond_index < (g.command_total -2)) {
            // we're OK to load a new command (last command must be a nav command)
            command_cond_queue = get_cmd_with_index(command_cond_index);

            if(command_cond_queue.id > MAV_CMD_CONDITION_LAST) {
                // this is a do now command
                process_now_command();

                // clear command queue
                command_cond_queue.id = NO_COMMAND;

            }else if(command_cond_queue.id > MAV_CMD_NAV_LAST) {
                // this is a conditional command
                process_cond_command();

            }else{
                // this is a nav command, don't process
                // clear the command conditional queue and index
                prev_nav_index                  = NO_COMMAND;
                command_cond_index              = NO_COMMAND;
                command_cond_queue.id   = NO_COMMAND;
            }

        }
    }
}

// execute_nav_command - performs minor initialisation and logging before next navigation command in the queue is executed
static void execute_nav_command(void)
{
    // This is what we report to MAVLINK
    g.command_index = command_nav_index;

    // Save CMD to Log
    if(g.log_bitmask & MASK_LOG_CMD)
        Log_Write_Cmd(g.command_index, &command_nav_queue);

    // clear navigation prameters
    reset_nav_params();

    // Act on the new command
    process_nav_command();

    // clear May indexes to force loading of more commands
    // existing May commands are tossed.
    command_cond_index      = NO_COMMAND;
}

// verify_commands - high level function to check if navigation and conditional commands have completed
static void verify_commands(void)
{
    // check if navigation command completed
    if(verify_nav_command()) {
        // clear navigation command queue so next command can be loaded
        command_nav_queue.id    = NO_COMMAND;

        // store our most recent executed nav command
        prev_nav_index          = command_nav_index;

        // Wipe existing conditionals
        command_cond_index      = NO_COMMAND;
        command_cond_queue.id   = NO_COMMAND;
    }

    // check if conditional command completed
    if(verify_cond_command()) {
        // clear conditional command queue so next command can be loaded
        command_cond_queue.id = NO_COMMAND;
    }
}

// Finds the next navgation command in EEPROM
static int16_t find_next_nav_index(int16_t search_index)
{
    Location tmp;
    while(search_index < g.command_total) {
        tmp = get_cmd_with_index(search_index);
        if(tmp.id <= MAV_CMD_NAV_LAST) {
            return search_index;
        }else{
            search_index++;
        }
    }
    return -1;
}

static void exit_mission()
{
    // we are out of commands
    g.command_index = 255;

    // if we are on the ground, enter stabilize, else Land
    if(ap.land_complete) {
        // we will disarm the motors after landing.
    }else{
        // If the approach altitude is valid (above 1m), do approach, else land
        if(g.rtl_alt_final == 0) {
            set_mode(LAND);
        }else{
            set_mode(LOITER);
        }
    }

}

#line 1 "/home/tobias/git/ardupilot/ArduCopter/compat.pde"


void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

void mavlink_delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

uint32_t millis()
{
    return hal.scheduler->millis();
}

uint32_t micros()
{
    return hal.scheduler->micros();
}

void pinMode(uint8_t pin, uint8_t output)
{
    hal.gpio->pinMode(pin, output);
}

void digitalWrite(uint8_t pin, uint8_t out)
{
    hal.gpio->write(pin,out);
}

uint8_t digitalRead(uint8_t pin)
{
    return hal.gpio->read(pin);
}

#line 1 "/home/tobias/git/ardupilot/ArduCopter/control_modes.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define CONTROL_SWITCH_COUNTER  10  // 10 iterations at 100hz (i.e. 1/10th of a second) at a new switch position will cause flight mode change
static void read_control_switch()
{
    static uint8_t switch_counter = 0;

    uint8_t switchPosition = readSwitch();

    if (oldSwitchPosition != switchPosition) {
        switch_counter++;
        if(switch_counter >= CONTROL_SWITCH_COUNTER) {
            oldSwitchPosition       = switchPosition;
            switch_counter          = 0;

            // ignore flight mode changes if in failsafe
            if( !ap.failsafe_radio ) {
                set_mode(flight_modes[switchPosition]);

                if(g.ch7_option != AUX_SWITCH_SIMPLE_MODE && g.ch8_option != AUX_SWITCH_SIMPLE_MODE) {
                    // set Simple mode using stored paramters from Mission planner
                    // rather than by the control switch
                    set_simple_mode(BIT_IS_SET(g.simple_modes, switchPosition));
                }
            }
        }
    }else{
        // reset switch_counter if there's been no change
        // we don't want 10 intermittant blips causing a flight mode change
        switch_counter = 0;
    }
}

static uint8_t readSwitch(void){
    int16_t pulsewidth = g.rc_5.radio_in;   // default for Arducopter

    if (pulsewidth < 1231) return 0;
    if (pulsewidth < 1361) return 1;
    if (pulsewidth < 1491) return 2;
    if (pulsewidth < 1621) return 3;
    if (pulsewidth < 1750) return 4;        // Software Manual
    return 5;                               // Hardware Manual
}

static void reset_control_switch()
{
    oldSwitchPosition = -1;
    read_control_switch();
}

// read_aux_switches - checks aux switch positions and invokes configured actions
static void read_aux_switches()
{
    // check if ch7 switch has changed position
    if (ap_system.CH7_flag != (g.rc_7.radio_in >= AUX_SWITCH_PWM_TRIGGER)) {
        // set the ch7 flag
        ap_system.CH7_flag = (g.rc_7.radio_in >= AUX_SWITCH_PWM_TRIGGER);

        // invoke the appropriate function
        do_aux_switch_function(g.ch7_option, ap_system.CH7_flag);
    }

    // safety check to ensure we ch7 and ch8 have different functions
    if (g.ch7_option == g.ch8_option) {
        return;
    }

    // check if ch8 switch has changed position
    if (ap_system.CH8_flag != (g.rc_8.radio_in >= AUX_SWITCH_PWM_TRIGGER)) {
        // set the ch8 flag
        ap_system.CH8_flag = (g.rc_8.radio_in >= AUX_SWITCH_PWM_TRIGGER);
        // invoke the appropriate function
        do_aux_switch_function(g.ch8_option, ap_system.CH8_flag);
    }
}

// do_aux_switch_function - implement the function invoked by the ch7 or ch8 switch
static void do_aux_switch_function(int8_t ch_function, bool ch_flag)
{
    int8_t tmp_function = ch_function;

    // multi mode check
    if(ch_function == AUX_SWITCH_MULTI_MODE) {
        if (g.rc_6.radio_in < CH6_PWM_TRIGGER_LOW) {
            tmp_function = AUX_SWITCH_FLIP;
        }else if (g.rc_6.radio_in > CH6_PWM_TRIGGER_HIGH) {
            tmp_function = AUX_SWITCH_SAVE_WP;
        }else{
            tmp_function = AUX_SWITCH_RTL;
        }
    }

    switch(tmp_function) {
        case AUX_SWITCH_FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if(ch_flag && g.rc_3.control_in >= 0 && ap.takeoff_complete) {
                init_flip();
            }
            break;

        case AUX_SWITCH_SIMPLE_MODE:
            set_simple_mode(ch_flag);
            break;

        case AUX_SWITCH_RTL:
            if (ch_flag) {
                // engage RTL
                set_mode(RTL);
            }else{
                // disengage RTL to previous flight mode if we are currently in RTL or loiter
                if (control_mode == RTL || control_mode == LOITER) {
                    reset_control_switch();
                }
            }
            break;

        case AUX_SWITCH_SAVE_TRIM:
            if(ch_flag && control_mode <= ACRO && g.rc_3.control_in == 0) {
                save_trim();
            }
            break;

        case AUX_SWITCH_SAVE_WP:
            // save waypoint when switch is switched off
            if (ch_flag == false) {

                // if in auto mode, reset the mission
                if(control_mode == AUTO) {
                    aux_switch_wp_index = 0;
                    g.command_total.set_and_save(1);
                    set_mode(RTL);
                    return;
                }

                if(aux_switch_wp_index == 0) {
                    // this is our first WP, let's save WP 1 as a takeoff
                    // increment index to WP index of 1 (home is stored at 0)
                    aux_switch_wp_index = 1;

                    Location temp   = home;
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    temp.id = MAV_CMD_NAV_TAKEOFF;
                    temp.alt = current_loc.alt;

                    // save command:
                    // we use the current altitude to be the target for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    // If we are above the altitude, we will skip the command.
                    set_cmd_with_index(temp, aux_switch_wp_index);
                }

                // increment index
                aux_switch_wp_index++;

                // set the next_WP (home is stored at 0)
                // max out at 100 since I think we need to stay under the EEPROM limit
                aux_switch_wp_index = constrain_int16(aux_switch_wp_index, 1, 100);

                if(g.rc_3.control_in > 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    current_loc.id = MAV_CMD_NAV_WAYPOINT;
                }else{
                    // set our location ID to 21, MAV_CMD_NAV_LAND
                    current_loc.id = MAV_CMD_NAV_LAND;
                }

                // save command
                set_cmd_with_index(current_loc, aux_switch_wp_index);

                // Cause the CopterLEDs to blink twice to indicate saved waypoint
                copter_leds_nav_blink = 10;
            }
            break;

#if CAMERA == ENABLED
        case AUX_SWITCH_CAMERA_TRIGGER:
            if(ch_flag) {
                do_take_picture();
            }
            break;
#endif

        case AUX_SWITCH_SONAR:
            // enable or disable the sonar
            g.sonar_enabled = ch_flag;
            break;

#if AC_FENCE == ENABLED
        case AUX_SWITCH_FENCE:
            // enable or disable the fence
            fence.enable(ch_flag);
            break;
#endif
        case AUX_SWITCH_RESETTOARMEDYAW:
            if (ch_flag) {
                set_yaw_mode(YAW_RESETTOARMEDYAW);
            }else{
                set_yaw_mode(YAW_HOLD);
            }
            break; 
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
static void save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)g.rc_1.control_in/100.0f);
    float pitch_trim = ToRad((float)g.rc_2.control_in/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
static void auto_trim()
{
    if(auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        led_mode = SAVE_TRIM_LEDS;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)g.rc_1.control_in / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)g.rc_2.control_in / 4000.0f);

        // make sure accelerometer values impact attitude quickly
        ahrs.set_fast_gains(true);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if(auto_trim_counter == 0) {
            ahrs.set_fast_gains(false);
            led_mode = NORMAL_LEDS;
        }
    }
}

#line 1 "/home/tobias/git/ardupilot/ArduCopter/events.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
static void failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    // This is how to handle a failsafe.
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero disarm motors
            if (g.rc_3.control_in == 0) {
                init_disarm_motors();
            }else if(ap.home_is_set == true && home_distance > wp_nav.get_waypoint_radius()) {
                set_mode(RTL);
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
        case AUTO:
            // failsafe_throttle is 1 do RTL, 2 means continue with the mission
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                if(home_distance > wp_nav.get_waypoint_radius()) {
                    set_mode(RTL);
                }else{
                    // We are very close to home so we will land
                    set_mode(LAND);
                }
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;
        default:
            if(ap.home_is_set == true && home_distance > wp_nav.get_waypoint_radius()) {
                set_mode(RTL);
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
static void failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

static void low_battery_event(void)
{
    // failsafe check
    if (g.failsafe_battery_enabled && !ap.low_battery && motors.armed()) {
        switch(control_mode) {
            case STABILIZE:
            case ACRO:
                // if throttle is zero disarm motors
                if (g.rc_3.control_in == 0) {
                    init_disarm_motors();
                }else{
                    set_mode(LAND);
                }
                break;
            case AUTO:
                if(ap.home_is_set == true && home_distance > wp_nav.get_waypoint_radius()) {
                    set_mode(RTL);
                }else{
                    // We have no GPS or are very close to home so we will land
                    set_mode(LAND);
                }
                break;
            default:
                set_mode(LAND);
                break;
        }
    }

    // set the low battery flag
    set_low_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

#if COPTER_LEDS == ENABLED
    // Only Activate if a battery is connected to avoid alarm on USB only
    piezo_on();
#endif // COPTER_LEDS
}

// failsafe_gps_check - check for gps failsafe
static void failsafe_gps_check()
{
    uint32_t last_gps_update_ms;

    // return immediately if gps failsafe is disabled
    if( !g.failsafe_gps_enabled ) {
        return;
    }

    // calc time since last gps update
    last_gps_update_ms = millis() - g_gps->last_fix_time;

    // check if all is well
    if( last_gps_update_ms < FAILSAFE_GPS_TIMEOUT_MS) {
        // check for recovery from gps failsafe
        if( ap.failsafe_gps ) {
            failsafe_gps_off_event();
            set_failsafe_gps(false);
        }
        return;
    }

    // do nothing if gps failsafe already triggered or motors disarmed
    if( ap.failsafe_gps || !motors.armed()) {
        return;
    }

    // GPS failsafe event has occured
    // update state, warn the ground station and log to dataflash
    set_failsafe_gps(true);
    gcs_send_text_P(SEVERITY_LOW,PSTR("Lost GPS!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on flight mode
    switch(control_mode) {
        // for modes that do not require gps, do nothing
        case STABILIZE:
        case ACRO:
        case ALT_HOLD:
        case OF_LOITER:
            // do nothing
            break;

        // modes requiring GPS force a land
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case POSITION:
            // We have no GPS or are very close to home so we will land
            set_mode(LAND);
            break;

        case LAND:
            // if we're already landing do nothing
            break;
    }
}

// failsafe_gps_off_event - actions to take when GPS contact is restored
static void failsafe_gps_off_event(void)
{
    // log recovery of GPS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_RESOLVED);
}

// failsafe_gcs_check - check for ground station failsafe
static void failsafe_gcs_check()
{
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs
    if( g.failsafe_gcs == FS_GCS_DISABLED || last_heartbeat_ms == 0 || !ap.rc_override_active) {
        return;
    }

    // calc time since last gps update
    last_gcs_update_ms = millis() - last_heartbeat_ms;

    // check if all is well
    if( last_gcs_update_ms < FS_GCS_TIMEOUT_MS) {
        // check for recovery from gps failsafe
        if( ap.failsafe_gcs ) {
            failsafe_gcs_off_event();
            set_failsafe_gcs(false);
        }
        return;
    }

    // do nothing if gps failsafe already triggered or motors disarmed
    if( ap.failsafe_gcs || !motors.armed()) {
        return;
    }

    // GCS failsafe event has occured
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // This is how to handle a failsafe.
    // use the throttle failsafe setting to decide what to do
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero disarm motors
            if (g.rc_3.control_in == 0) {
                init_disarm_motors();
            }else if(ap.home_is_set == true && home_distance > wp_nav.get_waypoint_radius()) {
                set_mode(RTL);
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
        case AUTO:
            // if g.failsafe_gcs is 1 do RTL, 2 means continue with the mission
            if (g.failsafe_gcs == FS_GCS_ENABLED_ALWAYS_RTL) {
                if(home_distance > wp_nav.get_waypoint_radius()) {
                    set_mode(RTL);
                }else{
                    // We are very close to home so we will land
                    set_mode(LAND);
                }
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;
        default:
            if(ap.home_is_set == true && home_distance > wp_nav.get_waypoint_radius()) {
                set_mode(RTL);
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode(LAND);
            }
            break;
    }
}

// failsafe_gps_off_event - actions to take when GPS contact is restored
static void failsafe_gcs_off_event(void)
{
    // log recovery of GPS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

static void update_events()     // Used for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
{
    if(event_repeat == 0 || (millis() - event_timer) < event_delay)
        return;

    if(event_repeat != 0) {             // event_repeat = -1 means repeat forever
        event_timer = millis();

        if (event_id >= CH_5 && event_id <= CH_8) {
            if(event_repeat%2) {
                hal.rcout->write(event_id, event_value);                 // send to Servos
            } else {
                hal.rcout->write(event_id, event_undo_value);
            }
        }

        if  (event_id == RELAY_TOGGLE) {
            relay.toggle();
        }
        if (event_repeat > 0) {
            event_repeat--;
        }
    }
}

#line 1 "/home/tobias/git/ardupilot/ArduCopter/failsafe.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled = true;
static uint16_t failsafe_last_mainLoop_count;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void failsafe_check(uint32_t tnow)
{
    if (mainLoop_count != failsafe_last_mainLoop_count) {
        // the main loop is running, all is OK
        failsafe_last_mainLoop_count = mainLoop_count;
        failsafe_last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(motors.armed()) {
            motors.armed(false);
            motors.output();
        }
    }
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/fence.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

uint8_t lim_state = 0, lim_old_state = 0;

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void fence_check()
{
    uint8_t new_breaches; // the type of fence that has been breached
    uint8_t orig_breaches = fence.get_breaches();

    // return immediately if motors are not armed
    if(!motors.armed()) {
        return;
    }

    // give fence library our current distance from home in meters
    fence.set_home_distance(home_distance*0.01f);

    // check for a breach
    new_breaches = fence.check_fence();

    // if there is a new breach take action
    if( new_breaches != AC_FENCE_TYPE_NONE ) {

        // if the user wants some kind of response and motors are armed
        if(fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if(control_mode <= ACRO && g.rc_3.control_in == 0 && !ap.failsafe_radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0)){
                init_disarm_motors();
            }else{
                // if we have a GPS
                if( ap.home_is_set ) {
                    // if we are within 100m of the fence, RTL
                    if( fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                        if(control_mode != RTL) {
                            set_mode(RTL);
                        }
                    }else{
                        // if more than 100m outside the fence just force a land
                        if(control_mode != RTL) {
                            set_mode(LAND);
                        }
                    }
                }else{
                    // we have no GPS so LAND
                    if(control_mode != LAND) {
                        set_mode(LAND);
                    }
                }
            }
        }

        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);
    }

    // record clearing of breach
    if(orig_breaches != AC_FENCE_TYPE_NONE && fence.get_breaches() == AC_FENCE_TYPE_NONE) {
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
    }
}

// fence_send_mavlink_status - send fence status to ground station
static void fence_send_mavlink_status(mavlink_channel_t chan)
{   
    if (fence.enabled()) {
        // traslate fence library breach types to mavlink breach types
        uint8_t mavlink_breach_type = FENCE_BREACH_NONE;
        uint8_t breaches = fence.get_breaches();
        if ((breaches & AC_FENCE_TYPE_ALT_MAX) != 0) {
            mavlink_breach_type = FENCE_BREACH_MAXALT;
        }
        if ((breaches & AC_FENCE_TYPE_CIRCLE) != 0) {
            mavlink_breach_type = FENCE_BREACH_BOUNDARY;
        }

        // send status
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)(fence.get_breaches()!=0),
                                      fence.get_breach_count(),
                                      mavlink_breach_type,
                                      fence.get_breach_time());
    }
}

#endif
#line 1 "/home/tobias/git/ardupilot/ArduCopter/flip.pde"
// 2010 Jose Julio
// 2011 Adapted and updated for AC2 by Jason Short
//
// Automatic Acrobatic Procedure (AAP) v1 : Roll flip
// State machine aproach:
//    Some states are fixed commands (for a fixed time)
//    Some states are fixed commands (until some IMU condition)
//    Some states include controls inside
uint8_t flip_timer;
uint8_t flip_state;

#define AAP_THR_INC 170
#define AAP_THR_DEC 120
#define AAP_ROLL_OUT 2000

static int8_t flip_dir;

void init_flip()
{
    if(false == ap.do_flip) {
        ap.do_flip = true;
        flip_state = 0;
        flip_dir = (ahrs.roll_sensor >= 0) ? 1 : -1;
		Log_Write_Event(DATA_BEGIN_FLIP);
    }
}

void roll_flip()
{
    int32_t roll = ahrs.roll_sensor * flip_dir;

    // Roll State machine
    switch (flip_state) {
    case 0:
        if (roll < 4500) {
            // Roll control
			roll_rate_target_bf     = 40000 * flip_dir;
		    if(ap.manual_throttle){
    		    // increase throttle right before flip
                set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
            }
        }else{
            flip_state++;
        }
        break;

    case 1:
        if((roll >= 4500) || (roll < -9000)) {
		    #if FRAME_CONFIG == HELI_FRAME
				roll_rate_target_bf = 40000 * flip_dir;
		    #else
			    roll_rate_target_bf = 40000 * flip_dir;
		    #endif
		    // decrease throttle while inverted
		    if(ap.manual_throttle){
                set_throttle_out(g.rc_3.control_in - AAP_THR_DEC, false);
            }
        }else{
            flip_state++;
            flip_timer = 0;
        }
        break;

    case 2:
        // 100 = 1 second with 100hz
        if (flip_timer < 100) {
            // we no longer need to adjust the roll_rate. 
            // It will be handled by normal flight control loops

            // increase throttle to gain any lost alitude
            if(ap.manual_throttle){
                set_throttle_out(g.rc_3.control_in + AAP_THR_INC, false);
            }
            flip_timer++;
        }else{
        	Log_Write_Event(DATA_END_FLIP);
            ap.do_flip = false;
            flip_state = 0;
        }
        break;
    }
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/inertia.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// read_inertia - read inertia in from accelerometers
static void read_inertia()
{
    static uint8_t log_counter_inav = 0;

    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    if( motors.armed() && (g.log_bitmask & MASK_LOG_INAV) ) {
        log_counter_inav++;
        if( log_counter_inav >= 10 ) {
            log_counter_inav = 0;
            Log_Write_INAV();
        }
    }
}

// read_inertial_altitude - pull altitude and climb rate from inertial nav library
static void read_inertial_altitude()
{
    // with inertial nav we can update the altitude and climb rate at 50hz
    current_loc.alt = inertial_nav.get_altitude();
    climb_rate = inertial_nav.get_velocity_z();
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/leds.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void update_lights()
{
    switch(led_mode) {
    case NORMAL_LEDS:
        update_motor_light();
        update_GPS_light();
        break;

    case SAVE_TRIM_LEDS:
        dancing_light();
        break;
    }
}

static void update_GPS_light(void)
{
    // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
    // ---------------------------------------------------------------------
    switch (g_gps->status()) {
        case GPS::NO_FIX:
        case GPS::GPS_OK_FIX_2D:
            // check if we've blinked since the last gps update
            if (g_gps->valid_read) {
                g_gps->valid_read = false;
                ap_system.GPS_light = !ap_system.GPS_light;                     // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
                if (ap_system.GPS_light) {
                    digitalWrite(C_LED_PIN, LED_OFF);
                }else{
                    digitalWrite(C_LED_PIN, LED_ON);
                }
            }
            break;

        case GPS::GPS_OK_FIX_3D:
            if(ap.home_is_set) {
                digitalWrite(C_LED_PIN, LED_ON);                  //Turn LED C on when gps has valid fix AND home is set.
            } else {
                digitalWrite(C_LED_PIN, LED_OFF);
            }
            break;

        default:
            digitalWrite(C_LED_PIN, LED_OFF);
            break;
    }
}

static void update_motor_light(void)
{
    if(motors.armed() == false) {
        ap_system.motor_light = !ap_system.motor_light;

        // blink
        if(ap_system.motor_light) {
            digitalWrite(A_LED_PIN, LED_ON);
        }else{
            digitalWrite(A_LED_PIN, LED_OFF);
        }
    }else{
        if(!ap_system.motor_light) {
            ap_system.motor_light = true;
            digitalWrite(A_LED_PIN, LED_ON);
        }
    }
}

static void dancing_light()
{
    static uint8_t step;

    if (step++ == 3)
        step = 0;

    switch(step)
    {
    case 0:
        digitalWrite(C_LED_PIN, LED_OFF);
        digitalWrite(A_LED_PIN, LED_ON);
        break;

    case 1:
        digitalWrite(A_LED_PIN, LED_OFF);
        digitalWrite(B_LED_PIN, LED_ON);
        break;

    case 2:
        digitalWrite(B_LED_PIN, LED_OFF);
        digitalWrite(C_LED_PIN, LED_ON);
        break;
    }
}
static void clear_leds()
{
    digitalWrite(A_LED_PIN, LED_OFF);
    digitalWrite(B_LED_PIN, LED_OFF);
    digitalWrite(C_LED_PIN, LED_OFF);
    ap_system.motor_light = false;
    led_mode = NORMAL_LEDS;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//	Copter LEDS by Robert Lefebvre
//	Based on the work of U4eake, Bill Sanford, Max Levine, and Oliver
//	g.copter_leds_mode controls the copter leds function via bitmath
//	Zeroeth bit turns motor leds on and off:                                00000001
//	First bit turns GPS function on and off:                                00000010
//	Second bit turns Aux function on and off:                               00000100
//	Third bit turns on Beeper (legacy Piezo) function:                      00001000
//	Fourth bit toggles between Fast Flash or Oscillate on Low Battery:      00010000		(0) does Fast Flash, (1) does Oscillate
//	Fifth bit causes motor LEDs to Nav Blink:                               00100000
//	Sixth bit causes GPS LEDs to Nav Blink:                                 01000000
//	This code is written in order to be backwards compatible with the old Motor_LEDS code
//	I hope to include at least some of the Show_LEDS code in the future
//	copter_leds_GPS_blink controls the blinking of the GPS LEDS
//	copter_leds_motor_blink controls the blinking of the motor LEDS
//	Piezo Code and beeps once on Startup to verify operation
//	Piezo Enables Tone on reaching low battery or current alert
/////////////////////////////////////////////////////////////////////////////////////////////

#define COPTER_LEDS_BITMASK_ENABLED         0x01        // bit #0
#define COPTER_LEDS_BITMASK_GPS             0x02        // bit #1
#define COPTER_LEDS_BITMASK_AUX             0x04        // bit #2
#define COPTER_LEDS_BITMASK_BEEPER          0x08        // bit #3
#define COPTER_LEDS_BITMASK_BATT_OSCILLATE  0x10        // bit #4
#define COPTER_LEDS_BITMASK_MOTOR_NAV_BLINK 0x20        // bit #5
#define COPTER_LEDS_BITMASK_GPS_NAV_BLINK   0x40        // bit #6

#if COPTER_LEDS == ENABLED
static void copter_leds_init(void)
{
    pinMode(COPTER_LED_1, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_2, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_3, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_4, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_5, OUTPUT);              //Motor or Aux LED
    pinMode(COPTER_LED_6, OUTPUT);              //Motor or Aux LED
    pinMode(COPTER_LED_7, OUTPUT);              //Motor or GPS LED
    pinMode(COPTER_LED_8, OUTPUT);              //Motor or GPS LED

    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
        piezo_beep();
    }
}

static void update_copter_leds(void)
{
    if (g.copter_leds_mode == 0) {
        copter_leds_reset();                                        //method of reintializing LED state
    }

    // motor leds control
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_ENABLED) {
        if (motors.armed()) {
            if (ap.low_battery) {
                if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BATT_OSCILLATE) {
                    copter_leds_oscillate();                        //if motors are armed, but battery level is low, motor leds fast blink
                } else {
                    copter_leds_fast_blink();                       //if motors are armed, but battery level is low, motor leds oscillate
                }
            } else {
                if (g.copter_leds_mode & COPTER_LEDS_BITMASK_MOTOR_NAV_BLINK) {
                    if (copter_leds_nav_blink > 0) {
                        copter_leds_slow_blink();                   //if nav command was seen, blink LEDs.
                    } else {
                        copter_leds_on();
                    }
                } else {
                    copter_leds_on();                               //if motors are armed, battery level OK, all motor leds ON
                }
            }
        } else {
            copter_leds_slow_blink();                               //if motors are not armed, blink motor leds
        }
    }

    // GPS led control
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS) {

        // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
        // ---------------------------------------------------------------------
        switch (g_gps->status()) {

        case GPS::NO_GPS:
            copter_leds_GPS_off();                                  //if no valid GPS signal, turn GPS LED off
            break;

        case GPS::NO_FIX:
            copter_leds_GPS_slow_blink();                           //if GPS has valid reads, but no fix, blink GPS LED slow
            break;

        case GPS::GPS_OK_FIX_2D:
        case GPS::GPS_OK_FIX_3D:
            if(ap.home_is_set) {
                if (g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS_NAV_BLINK) {
                    if (copter_leds_nav_blink >0) {
                        copter_leds_GPS_slow_blink();               //if nav command was seen, blink LEDs.
                    } else {
                        copter_leds_GPS_on();
                    }
                } else {
                    copter_leds_GPS_on();							//Turn GPS LEDs on when gps has valid fix AND home is set
                }
            } else {
                copter_leds_GPS_fast_blink();                       //if GPS has fix, but home is not set, blink GPS LED fast
            }
            break;
        }
    }

    // AUX led control
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX) {
        if (ap_system.CH7_flag) {
            copter_leds_aux_on();                                   //if sub-control of Ch7 is high, turn Aux LED on
        } else {
            copter_leds_aux_off();                                  //if sub-control of Ch7 is low, turn Aux LED off
        }
    }
}

static void copter_leds_reset(void) {
    digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_4, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_5, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_6, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_7, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_8, COPTER_LED_OFF);
}

static void copter_leds_on(void) {
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX)) {
        digitalWrite(COPTER_LED_1, COPTER_LED_ON);
    }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
        digitalWrite(COPTER_LED_2, COPTER_LED_ON);
    }
 #else
    digitalWrite(COPTER_LED_2, COPTER_LED_ON);
 #endif
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS) ) {
        digitalWrite(COPTER_LED_3, COPTER_LED_ON);
    }
    digitalWrite(COPTER_LED_4, COPTER_LED_ON);
    digitalWrite(COPTER_LED_5, COPTER_LED_ON);
    digitalWrite(COPTER_LED_6, COPTER_LED_ON);
    digitalWrite(COPTER_LED_7, COPTER_LED_ON);
    digitalWrite(COPTER_LED_8, COPTER_LED_ON);
}

static void copter_leds_off(void) {
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX)) {
        digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
    }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
        digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
    }
 #else
    digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
 #endif
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS)) {
        digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
    }
    digitalWrite(COPTER_LED_4, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_5, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_6, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_7, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_8, COPTER_LED_OFF);
}

static void copter_leds_slow_blink(void) {
    copter_leds_motor_blink++;                                                  // this increments once every 1/10 second because it is in the 10hz loop

    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 6 ) {         // when the counter reaches 5 (1/2 sec), then toggle the leds
        copter_leds_off();

        // if blinking is called by the Nav Blinker decrement the Nav Blink counter
        if ((g.copter_leds_mode & COPTER_LEDS_BITMASK_MOTOR_NAV_BLINK) && !(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS_NAV_BLINK) && copter_leds_nav_blink >0 ) {
            copter_leds_nav_blink--;
        }
    }else if (5 < copter_leds_motor_blink && copter_leds_motor_blink < 11) {
        copter_leds_on();
    }else{
        copter_leds_motor_blink = 0;                                            // start blink cycle again
    }
}

static void copter_leds_fast_blink(void) {    
    copter_leds_motor_blink++;                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 3 ) {         // when the counter reaches 3 (1/5 sec), then toggle the leds
        copter_leds_on();
    }else if (2 < copter_leds_motor_blink && copter_leds_motor_blink < 5) {
        copter_leds_off();
    }else{
        copter_leds_motor_blink = 0;                                            // start blink cycle again
    }
}

static void copter_leds_oscillate(void) {
    copter_leds_motor_blink++;                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 3 ) {         // when the counter reaches 3 (1/5 sec), then toggle the leds
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX)) {
            digitalWrite(COPTER_LED_1, COPTER_LED_ON);
        }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
            digitalWrite(COPTER_LED_2, COPTER_LED_ON);
        }
 #else
        digitalWrite(COPTER_LED_2, COPTER_LED_ON);
 #endif
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS)) {
            digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
        }
        digitalWrite(COPTER_LED_4, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_5, COPTER_LED_ON);
        digitalWrite(COPTER_LED_6, COPTER_LED_ON);
        digitalWrite(COPTER_LED_7, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_8, COPTER_LED_OFF);
    }else if (2 < copter_leds_motor_blink && copter_leds_motor_blink < 5) {
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX)) {
            digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
        }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
            digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
        }
 #else
        digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
 #endif
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS) ) {
            digitalWrite(COPTER_LED_3, COPTER_LED_ON);
        }
        digitalWrite(COPTER_LED_4, COPTER_LED_ON);
        digitalWrite(COPTER_LED_5, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_6, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_7, COPTER_LED_ON);
        digitalWrite(COPTER_LED_8, COPTER_LED_ON);
    }else{
        copter_leds_motor_blink = 0;                                            // start blink cycle again
    }
}

static void copter_leds_GPS_on(void) {
    digitalWrite(COPTER_LED_3, COPTER_LED_ON);
}

static void copter_leds_GPS_off(void) {
    digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
}

static void copter_leds_GPS_slow_blink(void) {
    copter_leds_GPS_blink++;                                                    // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_GPS_blink && copter_leds_GPS_blink < 6 ) {             // when the counter reaches 5 (1/2 sec), then toggle the leds
        copter_leds_GPS_off();
        if ((g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS_NAV_BLINK) && copter_leds_nav_blink >0 ) {   // if blinking is called by the Nav Blinker...
            copter_leds_nav_blink--;                                                                                                                    // decrement the Nav Blink counter
        }
    }else if (5 < copter_leds_GPS_blink && copter_leds_GPS_blink < 11) {
        copter_leds_GPS_on();
    }
    else copter_leds_GPS_blink = 0;                                             // start blink cycle again
}

static void copter_leds_GPS_fast_blink(void) {
    copter_leds_GPS_blink++;                                                    // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_GPS_blink && copter_leds_GPS_blink < 3 ) {             // when the counter reaches 3 (1/5 sec), then toggle the leds
        copter_leds_GPS_off();
    }else if (2 < copter_leds_GPS_blink && copter_leds_GPS_blink < 5) {
        copter_leds_GPS_on();
    }
    else copter_leds_GPS_blink = 0;                                             // start blink cycle again
}

static void copter_leds_aux_off(void){
    digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
}

static void copter_leds_aux_on(void){
    digitalWrite(COPTER_LED_1, COPTER_LED_ON);
}

void piezo_on(){
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        digitalWrite(PIEZO_PIN,HIGH);
    }
}

void piezo_off(){
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        digitalWrite(PIEZO_PIN,LOW);
    }
}

void piezo_beep(){                                                              // Note! This command should not be used in time sensitive loops
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        piezo_on();
        delay(100);
        piezo_off();
    }
}

void piezo_beep_twice(){                                                        // Note! This command should not be used in time sensitive loops
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        piezo_beep();
        delay(50);
        piezo_beep();
    }
}

#endif                  //COPTER_LEDS
#line 1 "/home/tobias/git/ardupilot/ArduCopter/motors.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    25  // called at 1hz so 25 seconds

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
static void arm_motors_check()
{
    static int16_t arming_counter;

    // ensure throttle is down
    if (g.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    // ensure we are in Stabilize, Acro or TOY mode
    if ((control_mode > ACRO) && ((control_mode != TOY_A) && (control_mode != TOY_M))) {
        arming_counter = 0;
        return;
    }
	
	#if FRAME_CONFIG == HELI_FRAME
	if ((motors.rsc_mode > 0) && (g.rc_8.control_in >= 10)){
		arming_counter = 0;
		return;
	}
	#endif  // HELI_FRAME

#if TOY_EDF == ENABLED
    int16_t tmp = g.rc_1.control_in;
#else
    int16_t tmp = g.rc_4.control_in;
#endif

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
            // run pre-arm-checks and display failures
            pre_arm_checks(true);
            if(ap.pre_arm_check) {
                init_arm_motors();
            }else{
                // reset arming counter if pre-arm checks fail
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed()) {
            auto_trim_counter = 250;
        }

    // full left
    }else if (tmp < -4000) {

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors.armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 25 seconds
// called at 1hz
static void auto_disarm_check()
{
    static uint8_t auto_disarming_counter;

    if((control_mode <= ACRO) && (g.rc_3.control_in == 0) && motors.armed()) {
        auto_disarming_counter++;

        if(auto_disarming_counter == AUTO_DISARMING_DELAY) {
            init_disarm_motors();
        }else if (auto_disarming_counter > AUTO_DISARMING_DELAY) {
            auto_disarming_counter = AUTO_DISARMING_DELAY + 1;
        }
    }else{
        auto_disarming_counter = 0;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
static void init_arm_motors()
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

#if LOGGING_ENABLED == ENABLED
    // start dataflash
    start_logging();
#endif

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
    hal.uartA->set_blocking_writes(false);
    if (gcs3.initialised) {
        hal.uartC->set_blocking_writes(false);
    }

#if COPTER_LEDS == ENABLED
    piezo_beep_twice();
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    // Reset home position
    // -------------------
    if(ap.home_is_set)
        init_home();

    // all I terms are invalid
    // -----------------------
    reset_I_all();

    if(did_ground_start == false) {
        did_ground_start = true;
        startup_ground();
    }

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground -
    // this resets Baro for more accuracy
    //-----------------------------------
    init_barometer();
#endif

    // temp hack
    ap_system.motor_light = true;
    digitalWrite(A_LED_PIN, LED_ON);

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_fast_gains(false);
#endif

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);

    // enable output to motors
    output_min();

    // finally actually arm the motors
    motors.armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // reenable failsafe
    failsafe_enable();
}

// perform pre-arm checks and set ap.pre_arm_check flag
static void pre_arm_checks(bool display_failure)
{
    // exit immediately if we've already successfully performed the pre-arm check
    if( ap.pre_arm_check ) {
        return;
    }

    // succeed if pre arm checks are disabled
    if(!g.arming_check_enabled) {
        ap.pre_arm_check = true;
        return;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: RC not calibrated"));
        }
        return;
    }

    // check accelerometers have been calibrated
    if(!ins.calibrated()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not calibrated"));
        }
        return;
    }

    // check the compass is healthy
    if(!compass.healthy) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy"));
        }
        return;
    }

    // check compass learning is on or offsets have been set
    Vector3f offsets = compass.get_offsets();
    if(!compass._learn && offsets.length() == 0) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
        }
        return;
    }

    // check for unreasonable compass offsets
    if(offsets.length() > 500) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass offsets too high"));
        }
        return;
    }

    // barometer health check
    if(!barometer.healthy) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Baro not healthy"));
        }
        return;
    }

#if AC_FENCE == ENABLED
    // check fence is initialised
    if(!fence.pre_arm_check()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: No GPS Lock"));
        }
        return;
    }
#endif

    // if we've gotten this far then pre arm checks have completed
    ap.pre_arm_check = true;
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
static void pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.pre_arm_rc_check ) {
        return;
    }

    // check if radio has been calibrated
    if(!g.rc_3.radio_min.load()) {
        return;
    }

    // check if throttle min is reasonable
    if(g.rc_3.radio_min > 1300) {
        return;
    }

    // if we've gotten this far rc is ok
    ap.pre_arm_rc_check = true;
}

static void init_disarm_motors()
{
#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    motors.armed(false);

    compass.save_offsets();

    g.throttle_cruise.save();

    // we are not in the air
    set_takeoff_complete(false);

#if COPTER_LEDS == ENABLED
    piezo_beep();
#endif

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_fast_gains(true);
#endif

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void
set_servos_4()
{
#if FRAME_CONFIG == TRI_FRAME
    // To-Do: implement improved stability patch for tri so that we do not need to limit throttle input to motors
    g.rc_3.servo_out = min(g.rc_3.servo_out, 800);
#endif
    motors.output();
}

#line 1 "/home/tobias/git/ardupilot/ArduCopter/navigation.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// update_navigation - invokes navigation routines
// called at 10hz
static void update_navigation()
{
    static uint32_t nav_last_update = 0;        // the system time of the last time nav was run update

    // exit immediately if not auto_armed
    if (!ap.auto_armed) {
        return;
    }

    // check for inertial nav updates
    if( inertial_nav.position_ok() ) {

        // calculate time since nav controllers last ran
        dTnav = (float)(millis() - nav_last_update)/ 1000.0f;
        nav_last_update = millis();

        // prevent runnup in dTnav value
        dTnav = min(dTnav, 1.0f);

        // run the navigation controllers
        update_nav_mode();
    }
}

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
static void run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_position - get lat and lon positions from inertial nav library
static void calc_position(){
    if( inertial_nav.position_ok() ) {
        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }
}

// calc_distance_and_bearing - calculate distance and direction to waypoints for reporting and autopilot decisions
static void calc_distance_and_bearing()
{
    Vector3f curr = inertial_nav.get_position();
    
    // get target from loiter or wpinav controller
    if( nav_mode == NAV_LOITER || nav_mode == NAV_CIRCLE ) {
        wp_distance = wp_nav.get_distance_to_target();
        wp_bearing = wp_nav.get_bearing_to_target();
    }else if( nav_mode == NAV_WP ) {
        wp_distance = wp_nav.get_distance_to_destination();
        wp_bearing = wp_nav.get_bearing_to_destination();
    }else{
        wp_distance = 0;
        wp_bearing = 0;
    }

    // calculate home distance and bearing
    if( ap.home_is_set ) {
        home_distance = pythagorous2(curr.x, curr.y);
        home_bearing = pv_get_bearing_cd(curr,Vector3f(0,0,0));

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing();
    }else{
        home_distance = 0;
        home_bearing = 0;
    }
}

// run_autopilot - highest level call to process mission commands
static void run_autopilot()
{
    switch( control_mode ) {
        case AUTO:
            // load the next command if the command queues are empty
            update_commands();

            // process the active navigation and conditional commands
            verify_commands();
            break;
        case GUIDED:
            // no need to do anything - wp_nav should take care of getting us to the desired location
            break;
        case RTL:
            verify_RTL();
            break;
    }
}

// set_nav_mode - update nav mode and initialise any variables as required
static bool set_nav_mode(uint8_t new_nav_mode)
{
    // boolean to ensure proper initialisation of nav modes
    bool nav_initialised = false;

    // return immediately if no change
    if( new_nav_mode == nav_mode ) {
        return true;
    }

    switch( new_nav_mode ) {

        case NAV_NONE:
            nav_initialised = true;
            break;

        case NAV_CIRCLE:
            // set center of circle to current position
            circle_set_center(inertial_nav.get_position(), ahrs.yaw);
            nav_initialised = true;
            break;

        case NAV_LOITER:
            // set target to current position
            wp_nav.set_loiter_target(inertial_nav.get_position(), inertial_nav.get_velocity());
            nav_initialised = true;
            break;

        case NAV_WP:
            nav_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( nav_initialised ) {
        nav_mode = new_nav_mode;
    }

    // return success or failure
    return nav_initialised;
}

// update_nav_mode - run navigation controller based on nav_mode
static void update_nav_mode()
{
    switch( nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

        case NAV_CIRCLE:
            // call circle controller which in turn calls loiter controller
            update_circle(dTnav);
            break;

        case NAV_LOITER:
            // call loiter controller
            wp_nav.update_loiter();
            break;

        case NAV_WP:
            // call waypoint controller
            wp_nav.update_wpnav();
            break;
    }

    // log to dataflash
    if ((g.log_bitmask & MASK_LOG_NTUN) && nav_mode != NAV_NONE) {
        Log_Write_Nav_Tuning();
    }

    /*
    // To-Do: check that we haven't broken toy mode
    case TOY_A:
    case TOY_M:
        set_nav_mode(NAV_NONE);
        update_nav_wp();
        break;
    }
    */
}

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by nav or loiter controllers
    lon_error                       = 0;
    lat_error                       = 0;
    nav_roll 						= 0;
    nav_pitch 						= 0;
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
    return wrap_360_cd(current_yaw + constrain_int16(wrap_180_cd(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}


//////////////////////////////////////////////////////////
// circle navigation controller
//////////////////////////////////////////////////////////

// circle_set_center -- set circle controller's center position and starting angle
static void
circle_set_center(const Vector3f current_position, float heading_in_radians)
{
    // set circle center to circle_radius ahead of current position
    circle_center.x = current_position.x + (float)g.circle_radius * 100 * sin_yaw;
    circle_center.y = current_position.y + (float)g.circle_radius * 100 * cos_yaw;

    // if we are doing a panorama set the circle_angle to the current heading
    if( g.circle_radius == 0 ) {
        circle_angle = heading_in_radians;
    }else{
        // set starting angle to current heading - 180 degrees
        circle_angle = heading_in_radians-ToRad(180);
        if( circle_angle > 180 ) {
            circle_angle -= 180;
        }
        if( circle_angle < -180 ) {
            circle_angle -= 180;
        }
    }

    // initialise other variables
    circle_angle_total = 0;
}

// update_circle - circle position controller's main call which in turn calls loiter controller with updated target position
static void
update_circle(float dt)
{
    float angle_delta = ToRad(g.circle_rate) * dt;
    float cir_radius = g.circle_radius * 100;
    Vector3f circle_target;

    // update the target angle
    circle_angle += angle_delta;
    if( circle_angle > 180 ) {
        circle_angle -= 360;
    }
    if( circle_angle <= -180 ) {
        circle_angle += 360;
    }

    // update the total angle travelled
    circle_angle_total += angle_delta;

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    if( g.circle_radius != 0.0 ) {
        // calculate target position
        circle_target.x = circle_center.x + cir_radius * sinf(1.57f - circle_angle);
        circle_target.y = circle_center.y + cir_radius * cosf(1.57f - circle_angle);

        // re-use loiter position controller
        wp_nav.set_loiter_target(circle_target);
    }

    // call loiter controller
    wp_nav.update_loiter();
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/perf_info.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  high level performance monitoring
//
//  we measure the main loop time
//

#define PERF_INFO_OVERTIME_THRESHOLD_MICROS 10500

uint16_t perf_info_loop_count;
uint32_t perf_info_max_time;
uint16_t perf_info_long_running;

// perf_info_reset - reset all records of loop time to zero
void perf_info_reset()
{
    perf_info_loop_count = 0;
    perf_info_max_time = 0;
    perf_info_long_running = 0;
}

// perf_info_check_loop_time - check latest loop time vs min, max and overtime threshold
void perf_info_check_loop_time(uint32_t time_in_micros)
{
    perf_info_loop_count++;
    if( time_in_micros > perf_info_max_time) {
        perf_info_max_time = time_in_micros;
    }
    if( time_in_micros > PERF_INFO_OVERTIME_THRESHOLD_MICROS ) {
        perf_info_long_running++;
    }
}

// perf_info_get_long_running_percentage - get number of long running loops as a percentage of the total number of loops
uint16_t perf_info_get_num_loops()
{
    return perf_info_loop_count;
}

// perf_info_get_max_time - return maximum loop time (in microseconds)
uint32_t perf_info_get_max_time()
{
    return perf_info_max_time;
}

// perf_info_get_num_long_running - get number of long running loops
uint16_t perf_info_get_num_long_running()
{
    return perf_info_long_running;
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/position_vector.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector2f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
const Vector3f pv_latlon_to_vector(int32_t lat, int32_t lon, int32_t alt)
{
    Vector3f tmp((lat-home.lat) * LATLON_TO_CM, (lon-home.lng) * LATLON_TO_CM * scaleLongDown, alt);
    return tmp;
}

// pv_latlon_to_vector - convert lat/lon coordinates to a position vector
const Vector3f pv_location_to_vector(Location loc)
{
    Vector3f tmp((loc.lat-home.lat) * LATLON_TO_CM, (loc.lng-home.lng) * LATLON_TO_CM * scaleLongDown, loc.alt);
    return tmp;
}

// pv_get_lon - extract latitude from position vector
const int32_t pv_get_lat(const Vector3f pos_vec)
{
    return home.lat + (int32_t)(pos_vec.x / LATLON_TO_CM);
}

// pv_get_lon - extract longitude from position vector
const int32_t pv_get_lon(const Vector3f pos_vec)
{
    return home.lng + (int32_t)(pos_vec.y / LATLON_TO_CM * scaleLongUp);
}

// pv_get_horizontal_distance_cm - return distance between two positions in cm
const float pv_get_horizontal_distance_cm(const Vector3f origin, const Vector3f destination)
{
    return pythagorous2(destination.x-origin.x,destination.y-origin.y);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
const float pv_get_bearing_cd(const Vector3f origin, const Vector3f destination)
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/radio.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

extern RC_Channel* rc_ch[8];

static void default_dead_zones()
{
    g.rc_1.set_dead_zone(60);
    g.rc_2.set_dead_zone(60);
#if FRAME_CONFIG == HELI_FRAME
    g.rc_3.set_dead_zone(20);
    g.rc_4.set_dead_zone(30);
#else
    g.rc_3.set_dead_zone(60);
    g.rc_4.set_dead_zone(80);
#endif
    g.rc_6.set_dead_zone(0);
}

static void init_rc_in()
{
    // set rc channel ranges
    g.rc_1.set_angle(MAX_INPUT_ROLL_ANGLE);
    g.rc_2.set_angle(MAX_INPUT_PITCH_ANGLE);
#if FRAME_CONFIG == HELI_FRAME
    // we do not want to limit the movment of the heli's swash plate
    g.rc_3.set_range(0, 1000);
#else
    g.rc_3.set_range(g.throttle_min, g.throttle_max);
#endif
    g.rc_4.set_angle(4500);

    // reverse: CW = left
    // normal:  CW = left???

    g.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    rc_ch[CH_1] = &g.rc_1;
    rc_ch[CH_2] = &g.rc_2;
    rc_ch[CH_3] = &g.rc_3;
    rc_ch[CH_4] = &g.rc_4;
    rc_ch[CH_5] = &g.rc_5;
    rc_ch[CH_6] = &g.rc_6;
    rc_ch[CH_7] = &g.rc_7;
    rc_ch[CH_8] = &g.rc_8;

    //set auxiliary ranges
    g.rc_5.set_range(0,1000);
    g.rc_6.set_range(0,1000);
    g.rc_7.set_range(0,1000);
    g.rc_8.set_range(0,1000);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_9, &g.rc_10, &g.rc_11, &g.rc_12);
#elif MOUNT == ENABLED
    update_aux_servo_function(&g.rc_5, &g.rc_6, &g.rc_7, &g.rc_8, &g.rc_10, &g.rc_11);
#endif
}

 // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
static void init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.Init();                                              // motor initialisation
    motors.set_min_throttle(g.throttle_min);
    motors.set_max_throttle(g.throttle_max);

    for(uint8_t i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }

    // we want the input to be scaled correctly
    g.rc_3.set_range_out(0,1000);

    // full throttle means to enter ESC calibration
    if(g.rc_3.control_in >= (MAXIMUM_THROTTLE - 50)) {
        if(g.esc_calibrate == 0) {
            // we will enter esc_calibrate mode on next reboot
            g.esc_calibrate.set_and_save(1);
            // display message on console
            cliSerial->printf_P(PSTR("Entering ESC Calibration: please restart APM.\n"));
            // block until we restart
            while(1) {
                delay(200);
                dancing_light();
            }
        }else{
            cliSerial->printf_P(PSTR("ESC Calibration active: passing throttle through to ESCs.\n"));
            // clear esc flag
            g.esc_calibrate.set_and_save(0);
            // pass through user throttle to escs
            init_esc();
        }
    }else{
        // did we abort the calibration?
        if(g.esc_calibrate == 1)
            g.esc_calibrate.set_and_save(0);
    }

    // enable output to motors
    pre_arm_rc_checks();
    if (ap.pre_arm_rc_check) {
        output_min();
    }

#if TOY_EDF == ENABLED
    // add access to CH8 and CH6
    APM_RC.enable_out(CH_8);
    APM_RC.enable_out(CH_6);
#endif
}

// output_min - enable and output lowest possible value to motors
void output_min()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

#define FAILSAFE_RADIO_TIMEOUT_MS 2000       // 2 seconds
static void read_radio()
{
    static uint32_t last_update = 0;
    if (hal.rcin->valid_channels() > 0) {
        last_update = millis();
        ap_system.new_radio_frame = true;
        uint16_t periods[8];
        hal.rcin->read(periods,8);
        g.rc_1.set_pwm(periods[0]);
        g.rc_2.set_pwm(periods[1]);

        set_throttle_and_failsafe(periods[2]);

        g.rc_4.set_pwm(periods[3]);
        g.rc_5.set_pwm(periods[4]);
        g.rc_6.set_pwm(periods[5]);
        g.rc_7.set_pwm(periods[6]);
        g.rc_8.set_pwm(periods[7]);

#if FRAME_CONFIG != HELI_FRAME
        // limit our input to 800 so we can still pitch and roll
        g.rc_3.control_in = min(g.rc_3.control_in, MAXIMUM_THROTTLE);
#endif
    }else{
        uint32_t elapsed = millis() - last_update;
        // turn on throttle failsafe if no update from ppm encoder for 2 seconds
        if ((elapsed >= FAILSAFE_RADIO_TIMEOUT_MS)
                && g.failsafe_throttle && motors.armed() && !ap.failsafe_radio) {
            Log_Write_Error(ERROR_SUBSYSTEM_RADIO, ERROR_CODE_RADIO_LATE_FRAME);
            set_failsafe_radio(true);
        }
    }
}

#define FS_COUNTER 3
static void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    static int8_t failsafe_counter = 0;

    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        g.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (ap.failsafe_radio || !motors.armed()) {
            g.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        failsafe_counter++;
        if( failsafe_counter >= FS_COUNTER ) {
            failsafe_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            g.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe_counter--;
        if( failsafe_counter <= 0 ) {
            failsafe_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (ap.failsafe_radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        g.rc_3.set_pwm(throttle_pwm);
    }
}

static void trim_radio()
{
    for (uint8_t i = 0; i < 30; i++) {
        read_radio();
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();
}

#line 1 "/home/tobias/git/ardupilot/ArduCopter/sensors.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Sensors are not available in HIL_MODE_ATTITUDE
#if HIL_MODE != HIL_MODE_ATTITUDE

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
  #if CONFIG_SONAR_SOURCE == SONAR_SOURCE_ADC
    sonar->calculate_scaler(g.sonar_type, 3.3f);
  #else
    sonar->calculate_scaler(g.sonar_type, 5.0f);
  #endif
}
 #endif

static void init_barometer(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    barometer.calibrate();
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static int32_t read_barometer(void)
{
    barometer.read();
    return barometer.get_altitude() * 100.0f;
}

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    // exit immediately if sonar is disabled
    if( !g.sonar_enabled ) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar->read();

    if (temp_alt >= sonar->min_distance && temp_alt <= sonar->max_distance * 0.70f) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = cos_pitch_x * cos_roll_x;
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}


#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_compass(&compass);
#endif
}

static void init_optflow()
{
#if OPTFLOW == ENABLED
    if( optflow.init() == false ) {
        g.optflow_enabled = false;
        cliSerial->print_P(PSTR("\nFailed to Init OptFlow "));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }else{
        // suspend timer while we set-up SPI communication
        hal.scheduler->suspend_timer_procs();

        optflow.set_orientation(OPTFLOW_ORIENTATION);   // set optical flow sensor's orientation on aircraft
        optflow.set_frame_rate(2000);                   // set minimum update rate (which should lead to maximum low light performance
        optflow.set_resolution(OPTFLOW_RESOLUTION);     // set optical flow sensor's resolution
        optflow.set_field_of_view(OPTFLOW_FOV);         // set optical flow sensor's field of view

        // resume timer
        hal.scheduler->resume_timer_procs();
    }
#endif      // OPTFLOW == ENABLED
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
#define BATTERY_FS_COUNTER  100     // 100 iterations at 10hz is 10 seconds
static void read_battery(void)
{
    static uint8_t low_battery_counter = 0;

    if(g.battery_monitoring == BATT_MONITOR_DISABLED) {
        battery_voltage1 = 0;
        return;
    }

    if(g.battery_monitoring == BATT_MONITOR_VOLTAGE_ONLY || g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        batt_volt_analog_source->set_pin(g.battery_volt_pin);
        battery_voltage1 = BATTERY_VOLTAGE(batt_volt_analog_source);
    }
    if(g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        batt_curr_analog_source->set_pin(g.battery_curr_pin);
        current_amps1    = CURRENT_AMPS(batt_curr_analog_source);
        current_total1   += current_amps1 * 0.02778f;            // called at 100ms on average, .0002778 is 1/3600 (conversion to hours)

        // update compass with current value
        compass.set_current(current_amps1);
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    if (!ap.low_battery && ( battery_voltage1 < g.low_voltage || (g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT && current_total1 > g.pack_capacity))) {
        low_battery_counter++;
        if( low_battery_counter >= BATTERY_FS_COUNTER ) {
            low_battery_counter = BATTERY_FS_COUNTER;   // ensure counter does not overflow
            low_battery_event();
        }
    }else{
        // reset low_battery_counter in case it was a temporary voltage dip
        low_battery_counter = 0;
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    rssi_analog_source->set_pin(g.rssi_pin);
    float ret = rssi_analog_source->voltage_average() * 50;
    receiver_rssi = constrain_int16(ret, 0, 255);
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/setup.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// Functions called from the setup menu
static int8_t   setup_radio             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_motors            (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_accel_scale       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_frame             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_erase             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_flightmodes       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_batt_monitor      (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_sonar             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compass           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_compassmot        (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_tune              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_range             (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_declination       (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_optflow           (uint8_t argc, const Menu::arg *argv);

 #if FRAME_CONFIG == HELI_FRAME
static int8_t   setup_heli              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_gyro              (uint8_t argc, const Menu::arg *argv);
 #endif

static int8_t   setup_set               (uint8_t argc, const Menu::arg *argv);

// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"erase",                       setup_erase},
    {"reset",                       setup_factory},
    {"radio",                       setup_radio},
    {"frame",                       setup_frame},
    {"motors",                      setup_motors},
    {"level",                       setup_accel},
    {"accel",                       setup_accel_scale},
    {"modes",                       setup_flightmodes},
    {"battery",                     setup_batt_monitor},
    {"sonar",                       setup_sonar},
    {"compass",                     setup_compass},
    {"compassmot",                  setup_compassmot},
    {"tune",                        setup_tune},
    {"range",                       setup_range},
    {"declination",         setup_declination},
    {"optflow",                     setup_optflow},
 #if FRAME_CONFIG == HELI_FRAME
    {"heli",                        setup_heli},
    {"gyro",                        setup_gyro},
 #endif
    {"show",                        setup_show},
    {"set",                         setup_set}
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf_P(PSTR("Setup Mode\n\n\n"));

    if(g.rc_1.radio_min >= 1300) {
        delay(1000);
        cliSerial->printf_P(PSTR("\n!Warning, radio not configured!"));
        delay(1000);
        cliSerial->printf_P(PSTR("\n Type 'radio' now.\n\n"));
    }

    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
    return 0;
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Param *param;
    ap_var_type type;

    //If a parameter name is given as an argument to show, print only that parameter
    if(argc>=2)
    {

        param=AP_Param::find(argv[1].str, &type);

        if(!param)
        {
            cliSerial->printf_P(PSTR("Parameter not found: '%s'\n"), argv[1]);
            return 0;
        }
        AP_Param::show(param, argv[1].str, type, cliSerial);
        return 0;
    }

    // clear the area
    print_blanks(8);

    report_version();
    report_radio();
    report_frame();
    report_batt_monitor();
    report_sonar();
    report_flight_modes();
    report_ins();
    report_compass();
    report_optflow();

 #if FRAME_CONFIG == HELI_FRAME
    report_heli();
    report_gyro();
 #endif

    AP_Param::show_all(cliSerial);

    return(0);
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    cliSerial->printf_P(PSTR("\n'Y' = factory reset, any other key to abort:\n"));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    cliSerial->printf_P(PSTR("\nReboot APM"));

    delay(1000);

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
}

// Perform radio setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_radio(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("\n\nRadio Setup:"));
    uint8_t i;

    for(i = 0; i < 100; i++) {
        delay(20);
        read_radio();
    }

    if(g.rc_1.radio_in < 500) {
        while(1) {
            //cliSerial->printf_P(PSTR("\nNo radio; Check connectors."));
            delay(1000);
            // stop here
        }
    }

    g.rc_1.radio_min = g.rc_1.radio_in;
    g.rc_2.radio_min = g.rc_2.radio_in;
    g.rc_3.radio_min = g.rc_3.radio_in;
    g.rc_4.radio_min = g.rc_4.radio_in;
    g.rc_5.radio_min = g.rc_5.radio_in;
    g.rc_6.radio_min = g.rc_6.radio_in;
    g.rc_7.radio_min = g.rc_7.radio_in;
    g.rc_8.radio_min = g.rc_8.radio_in;

    g.rc_1.radio_max = g.rc_1.radio_in;
    g.rc_2.radio_max = g.rc_2.radio_in;
    g.rc_3.radio_max = g.rc_3.radio_in;
    g.rc_4.radio_max = g.rc_4.radio_in;
    g.rc_5.radio_max = g.rc_5.radio_in;
    g.rc_6.radio_max = g.rc_6.radio_in;
    g.rc_7.radio_max = g.rc_7.radio_in;
    g.rc_8.radio_max = g.rc_8.radio_in;

    g.rc_1.radio_trim = g.rc_1.radio_in;
    g.rc_2.radio_trim = g.rc_2.radio_in;
    g.rc_4.radio_trim = g.rc_4.radio_in;
    // 3 is not trimed
    g.rc_5.radio_trim = 1500;
    g.rc_6.radio_trim = 1500;
    g.rc_7.radio_trim = 1500;
    g.rc_8.radio_trim = 1500;


    cliSerial->printf_P(PSTR("\nMove all controls to extremes. Enter to save: "));
    while(1) {

        delay(20);
        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        g.rc_1.update_min_max();
        g.rc_2.update_min_max();
        g.rc_3.update_min_max();
        g.rc_4.update_min_max();
        g.rc_5.update_min_max();
        g.rc_6.update_min_max();
        g.rc_7.update_min_max();
        g.rc_8.update_min_max();

        if(cliSerial->available() > 0) {
            delay(20);
            while (cliSerial->read() != -1); /* flush */

            g.rc_1.save_eeprom();
            g.rc_2.save_eeprom();
            g.rc_3.save_eeprom();
            g.rc_4.save_eeprom();
            g.rc_5.save_eeprom();
            g.rc_6.save_eeprom();
            g.rc_7.save_eeprom();
            g.rc_8.save_eeprom();

            print_done();
            break;
        }
    }
    report_radio();
    return(0);
}

static int8_t
setup_motors(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR(
                        "Connect battery for this test.\n"
                        "Motors will not spin in channel order (1,2,3,4) but by frame position order.\n"
                        "Front (& right of centerline) motor first, then in clockwise order around frame.\n"
                        "http://code.google.com/p/arducopter/wiki/AC2_Props_2 for demo video.\n"
                        "Remember to disconnect battery after this test.\n"
                        "Any key to exit.\n"));
    while(1) {
        delay(20);
        read_radio();
        motors.output_test();
        if(cliSerial->available() > 0) {
            g.esc_calibrate.set_and_save(0);
            return(0);
        }
    }
}

static int8_t
setup_accel(uint8_t argc, const Menu::arg *argv)
{
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    ins.init_accel(flash_leds);
    ahrs.set_trim(Vector3f(0,0,0));     // clear out saved trim
    report_ins();
    return(0);
}

static int8_t
setup_accel_scale(uint8_t argc, const Menu::arg *argv)
{
    float trim_roll, trim_pitch;

    cliSerial->println_P(PSTR("Initialising gyros"));
    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    AP_InertialSensor_UserInteractStream interact(hal.console);
    if(ins.calibrate_accel(flash_leds, &interact, trim_roll, trim_pitch)) {
        // reset ahrs's trim to suggested values from calibration routine
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
    report_ins();
    return(0);
}

static int8_t
setup_frame(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("x"))) {
        g.frame_orientation.set_and_save(X_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("p"))) {
        g.frame_orientation.set_and_save(PLUS_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("+"))) {
        g.frame_orientation.set_and_save(PLUS_FRAME);
    } else if (!strcmp_P(argv[1].str, PSTR("v"))) {
        g.frame_orientation.set_and_save(V_FRAME);
    }else{
        cliSerial->printf_P(PSTR("\nOp:[x,+,v]\n"));
        report_frame();
        return 0;
    }

    report_frame();
    return 0;
}

static int8_t
setup_flightmodes(uint8_t argc, const Menu::arg *argv)
{
    uint8_t _switchPosition = 0;
    uint8_t _oldSwitchPosition = 0;
    int8_t mode = 0;

    cliSerial->printf_P(PSTR("\nMode switch to edit, aileron: select modes, rudder: Simple on/off\n"));
    print_hit_enter();

    while(1) {
        delay(20);
        read_radio();
        _switchPosition = readSwitch();


        // look for control switch change
        if (_oldSwitchPosition != _switchPosition) {

            mode = flight_modes[_switchPosition];
            mode = constrain_int16(mode, 0, NUM_MODES-1);

            // update the user
            print_switch(_switchPosition, mode, BIT_IS_SET(g.simple_modes, _switchPosition));

            // Remember switch position
            _oldSwitchPosition = _switchPosition;
        }

        // look for stick input
        if (abs(g.rc_1.control_in) > 3000) {
            mode++;
            if(mode >= NUM_MODES)
                mode = 0;

            // save new mode
            flight_modes[_switchPosition] = mode;

            // print new mode
            print_switch(_switchPosition, mode, BIT_IS_SET(g.simple_modes, _switchPosition));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in > 3000) {
            g.simple_modes |= (1<<_switchPosition);
            // print new mode
            print_switch(_switchPosition, mode, BIT_IS_SET(g.simple_modes, _switchPosition));
            delay(500);
        }

        // look for stick input
        if (g.rc_4.control_in < -3000) {
            g.simple_modes &= ~(1<<_switchPosition);
            // print new mode
            print_switch(_switchPosition, mode, BIT_IS_SET(g.simple_modes, _switchPosition));
            delay(500);
        }

        // escape hatch
        if(cliSerial->available() > 0) {
            for (mode = 0; mode < 6; mode++)
                flight_modes[mode].save();

            g.simple_modes.save();
            print_done();
            report_flight_modes();
            return (0);
        }
    }
}

static int8_t
setup_declination(uint8_t argc, const Menu::arg *argv)
{
    compass.set_declination(radians(argv[1].f));
    report_compass();
    return 0;
}

static int8_t
setup_tune(uint8_t argc, const Menu::arg *argv)
{
    g.radio_tuning.set_and_save(argv[1].i);
    report_tuning();
    return 0;
}

static int8_t
setup_range(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("\nCH 6 Ranges are divided by 1000: [low, high]\n"));

    g.radio_tuning_low.set_and_save(argv[1].i);
    g.radio_tuning_high.set_and_save(argv[2].i);
    report_tuning();
    return 0;
}



static int8_t
setup_erase(uint8_t argc, const Menu::arg *argv)
{
    zero_eeprom();
    return 0;
}

static int8_t
setup_compass(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.compass_enabled.set_and_save(true);
        init_compass();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        clear_offsets();
        g.compass_enabled.set_and_save(false);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on,off]\n"));
        report_compass();
        return 0;
    }

    g.compass_enabled.save();
    report_compass();
    return 0;
}

// setup_compassmot - sets compass's motor interference parameters
static int8_t
setup_compassmot(uint8_t argc, const Menu::arg *argv)
{
    int8_t   comp_type;                 // throttle or current based compensation
    Vector3f compass_base;              // compass vector when throttle is zero
    Vector3f motor_impact;              // impact of motors on compass vector
    Vector3f motor_impact_scaled;       // impact of motors on compass vector scaled with throttle
    Vector3f motor_compensation;        // final compensation to be stored to eeprom
    float    throttle_pct;              // throttle as a percentage 0.0 ~ 1.0
    float    throttle_pct_max = 0.0f;   // maximum throttle reached (as a percentage 0~1.0)
    float    current_amps_max = 0.0f;   // maximum current reached
    float    interference_pct;          // interference as a percentage of total mag field (for reporting purposes only)
    uint32_t last_run_time;
    uint8_t  print_counter = 49;
    bool     updated = false;           // have we updated the compensation vector at least once

    // default compensation type to use current if possible
    if( g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT ) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }else{
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // check if user wants throttle compensation
    if( !strcmp_P(argv[1].str, PSTR("t")) || !strcmp_P(argv[1].str, PSTR("T")) ) {
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // check if user wants current compensation
    if( !strcmp_P(argv[1].str, PSTR("c")) || !strcmp_P(argv[1].str, PSTR("C")) ) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }

    // check if radio is calibration
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        cliSerial->print_P(PSTR("radio not calibrated, exiting"));
        return 0;
    }

    // check compass is enabled
    if( !g.compass_enabled ) {
        cliSerial->print_P(PSTR("compass disabled, exiting"));
        return 0;
    }

    // check if we have a current monitor
    if( comp_type == AP_COMPASS_MOT_COMP_CURRENT && g.battery_monitoring != BATT_MONITOR_VOLTAGE_AND_CURRENT ) {
        cliSerial->print_P(PSTR("current monitor disabled, exiting"));
        return 0;
    }

    // initialise compass
    init_compass();

    // disable motor compensation
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    compass.set_motor_compensation(Vector3f(0,0,0));

    // print warning that motors will spin
    // ask user to raise throttle
    // inform how to stop test
    cliSerial->print_P(PSTR("This records the impact on the compass of running the motors.  The motors will spin!\nHold throttle low, then raise to mid for 5 sec, then quickly back to low.\nAt any time you may press any key to exit.\n\nmeasuring compass vs "));

    // inform what type of compensation we are attempting
    if( comp_type == AP_COMPASS_MOT_COMP_CURRENT ) {
        cliSerial->print_P(PSTR("CURRENT\n"));
    }else{
        cliSerial->print_P(PSTR("THROTTLE\n"));
    }

    // clear out user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // disable throttle and battery failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_battery_enabled = false;

    // read radio
    read_radio();

    // exit immediately if throttle is not zero
    if( g.rc_3.control_in != 0 ) {
        cliSerial->print_P(PSTR("throttle not zero, exiting\n"));
        return 0;
    }

    // get some initial compass readings
    last_run_time = millis();
    while( millis() - last_run_time < 2000 ) {
        compass.accumulate();
    }
    compass.read();

    // exit immediately if the compass is not healthy
    if( !compass.healthy ) {
        cliSerial->print_P(PSTR("compass not healthy, exiting\n"));
        return 0;
    }

    // store initial x,y,z compass values
    compass_base.x = compass.mag_x;
    compass_base.y = compass.mag_y;
    compass_base.z = compass.mag_z;

    // initialise motor compensation
    motor_compensation = Vector3f(0,0,0);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // enable motors and pass through throttle
    init_rc_out();
    output_min();
    motors.armed(true);

    // initialise run time
    last_run_time = millis();

    // main run while there is no user input and the compass is healthy
    while(!cliSerial->available() && compass.healthy) {

        // 50hz loop
        if( millis() - last_run_time > 20 ) {
            last_run_time = millis();

            // read radio input
            read_radio();

            // pass through throttle to motors
            motors.throttle_pass_through();

            // read some compass values
            compass.read();

            // read current
            read_battery();

            // calculate scaling for throttle
            throttle_pct = (float)g.rc_3.control_in / 1000.0f;
            throttle_pct = constrain_float(throttle_pct,0.0f,1.0f);

            // if throttle is zero, update base x,y,z values
            if( throttle_pct == 0.0f ) {
                compass_base.x = compass_base.x * 0.99f + (float)compass.mag_x * 0.01f;
                compass_base.y = compass_base.y * 0.99f + (float)compass.mag_y * 0.01f;
                compass_base.z = compass_base.z * 0.99f + (float)compass.mag_z * 0.01f;

                // causing printing to happen as soon as throttle is lifted
                print_counter = 49;
            }else{

                // calculate diff from compass base and scale with throttle
                motor_impact.x = compass.mag_x - compass_base.x;
                motor_impact.y = compass.mag_y - compass_base.y;
                motor_impact.z = compass.mag_z - compass_base.z;

                // throttle based compensation
                if( comp_type == AP_COMPASS_MOT_COMP_THROTTLE ) {
                    // scale by throttle
                    motor_impact_scaled = motor_impact / throttle_pct;

                    // adjust the motor compensation to negate the impact
                    motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                    updated = true;
                }else{
                    // current based compensation if more than 3amps being drawn
                    motor_impact_scaled = motor_impact / current_amps1;

                    // adjust the motor compensation to negate the impact if drawing over 3amps
                    if( current_amps1 >= 3.0f ) {
                        motor_compensation = motor_compensation * 0.99f - motor_impact_scaled * 0.01f;
                        updated = true;
                    }
                }

                // record maximum throttle and current
                throttle_pct_max = max(throttle_pct_max, throttle_pct);
                current_amps_max = max(current_amps_max, current_amps1);

                // display output at 1hz if throttle is above zero
                print_counter++;
                if(print_counter >= 50) {
                    print_counter = 0;
                    cliSerial->printf_P(PSTR("thr:%d cur:%4.2f mot x:%4.1f y:%4.1f z:%4.1f  comp x:%4.2f y:%4.2f z:%4.2f\n"),(int)g.rc_3.control_in, (float)current_amps1, (float)motor_impact.x, (float)motor_impact.y, (float)motor_impact.z, (float)motor_compensation.x, (float)motor_compensation.y, (float)motor_compensation.z);
                }
            }
        }else{
            // grab some compass values
            compass.accumulate();
        }
    }

    // stop motors
    motors.output_min();
    motors.armed(false);

    // clear out any user input
    while( cliSerial->available() ) {
        cliSerial->read();
    }

    // print one more time so the last thing printed matches what appears in the report_compass
    cliSerial->printf_P(PSTR("thr:%d cur:%4.2f mot x:%4.1f y:%4.1f z:%4.1f  comp x:%4.2f y:%4.2f z:%4.2f\n"),(int)g.rc_3.control_in, (float)current_amps1, (float)motor_impact.x, (float)motor_impact.y, (float)motor_impact.z, (float)motor_compensation.x, (float)motor_compensation.y, (float)motor_compensation.z);

    // set and save motor compensation
    if( updated ) {
        compass.motor_compensation_type(comp_type);
        compass.set_motor_compensation(motor_compensation);
        compass.save_motor_compensation();

        // calculate and display interference compensation at full throttle as % of total mag field
        if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
            // interference is impact@fullthrottle / mag field * 100
            interference_pct = motor_compensation.length() / 330.0f * 100.0f;
        }else{
            // interference is impact/amp * (max current seen / max throttle seen) / mag field * 100
            interference_pct = motor_compensation.length() * (current_amps_max/throttle_pct_max) / 330.0f * 100.0f;
        }
        cliSerial->printf_P(PSTR("\nInterference at full throttle is %d%% of mag field\n\n"),(int)interference_pct);
    }else{
        // compensation vector never updated, report failure
        cliSerial->printf_P(PSTR("Failed! Compensation disabled.  Did you forget to raise the throttle high enough?"));
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // display new motor offsets and save
    report_compass();

    return 0;
}

static int8_t
setup_batt_monitor(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.battery_monitoring.set_and_save(0);

    } else if(argv[1].i > 0 && argv[1].i <= 4) {
        g.battery_monitoring.set_and_save(argv[1].i);

    } else {
        cliSerial->printf_P(PSTR("\nOp: off, 3-4"));
    }

    report_batt_monitor();
    return 0;
}

static int8_t
setup_sonar(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.sonar_enabled.set_and_save(true);

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.sonar_enabled.set_and_save(false);

    } else if (argc > 1 && (argv[1].i >= 0 && argv[1].i <= 3)) {
        g.sonar_enabled.set_and_save(true);      // if you set the sonar type, surely you want it on
        g.sonar_type.set_and_save(argv[1].i);

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off, 0-3]\n"));
        report_sonar();
        return 0;
    }

    report_sonar();
    return 0;
}

 #if FRAME_CONFIG == HELI_FRAME

// Perform heli setup.
// Called by the setup menu 'radio' command.
static int8_t
setup_heli(uint8_t argc, const Menu::arg *argv)
{

    uint8_t active_servo = 0;
    int16_t value = 0;
    int16_t temp;
    int16_t state = 0;       // 0 = set rev+pos, 1 = capture min/max
    int16_t max_roll=0, max_pitch=0, min_collective=0, max_collective=0, min_tail=0, max_tail=0;

    // initialise swash plate
    motors.init_swash();

    // source swash plate movements directly from radio
    motors.servo_manual = true;

    // display initial settings
    report_heli();

    // display help
    cliSerial->printf_P(PSTR("Instructions:"));
    print_divider();
    cliSerial->printf_P(PSTR("\td\t\tdisplay settings\n"));
    cliSerial->printf_P(PSTR("\t1~4\t\tselect servo\n"));
    cliSerial->printf_P(PSTR("\ta or z\t\tmove mid up/down\n"));
    cliSerial->printf_P(PSTR("\tc\t\tset coll when blade pitch zero\n"));
    cliSerial->printf_P(PSTR("\tm\t\tset roll, pitch, coll min/max\n"));
    cliSerial->printf_P(PSTR("\tp<angle>\tset pos (i.e. p0 = front, p90 = right)\n"));
    cliSerial->printf_P(PSTR("\tr\t\treverse servo\n"));
    cliSerial->printf_P(PSTR("\tu a|d\t\tupdate rate (a=analog servo, d=digital)\n"));
    cliSerial->printf_P(PSTR("\tt<angle>\tset trim (-500 ~ 500)\n"));
    cliSerial->printf_P(PSTR("\tx\t\texit & save\n"));

    // start capturing
    while( value != 'x' ) {

        // read radio although we don't use it yet
        read_radio();

        // allow swash plate to move
        motors.output_armed();

        // record min/max
        if( state == 1 ) {
            if( abs(g.rc_1.control_in) > max_roll )
                max_roll = abs(g.rc_1.control_in);
            if( abs(g.rc_2.control_in) > max_pitch )
                max_pitch = abs(g.rc_2.control_in);
            if( g.rc_3.radio_out < min_collective )
                min_collective = g.rc_3.radio_out;
            if( g.rc_3.radio_out > max_collective )
                max_collective = g.rc_3.radio_out;
            min_tail = min(g.rc_4.radio_out, min_tail);
            max_tail = max(g.rc_4.radio_out, max_tail);
        }

        if( cliSerial->available() ) {
            value = cliSerial->read();

            // process the user's input
            switch( value ) {
            case '1':
                active_servo = CH_1;
                break;
            case '2':
                active_servo = CH_2;
                break;
            case '3':
                active_servo = CH_3;
                break;
            case '4':
                active_servo = CH_4;
                break;
            case 'a':
            case 'A':
                heli_get_servo(active_servo)->radio_trim += 10;
                break;
            case 'c':
            case 'C':
                if( g.rc_3.radio_out >= 900 && g.rc_3.radio_out <= 2100 ) {
                    motors.collective_mid = g.rc_3.radio_out;
                    cliSerial->printf_P(PSTR("Collective when blade pitch at zero: %d\n"),(int)motors.collective_mid);
                }
                break;
            case 'd':
            case 'D':
                // display settings
                report_heli();
                break;
            case 'm':
            case 'M':
                if( state == 0 ) {
                    state = 1;                          // switch to capture min/max mode
                    cliSerial->printf_P(PSTR("Move coll, roll, pitch and tail to extremes, press 'm' when done\n"));

                    // reset servo ranges
                    motors.roll_max = motors.pitch_max = 4500;
                    motors.collective_min = 1000;
                    motors.collective_max = 2000;
                    motors._servo_4->radio_min = 1000;
                    motors._servo_4->radio_max = 2000;

                    // set sensible values in temp variables
                    max_roll = abs(g.rc_1.control_in);
                    max_pitch = abs(g.rc_2.control_in);
                    min_collective = 2000;
                    max_collective = 1000;
                    min_tail = max_tail = abs(g.rc_4.radio_out);
                }else{
                    state = 0;                          // switch back to normal mode
                    // double check values aren't totally terrible
                    if( max_roll <= 1000 || max_pitch <= 1000 || (max_collective - min_collective < 200) || (max_tail - min_tail < 200) || min_tail < 1000 || max_tail > 2000 )
                        cliSerial->printf_P(PSTR("Invalid min/max captured roll:%d,  pitch:%d,  collective min: %d max: %d,  tail min:%d max:%d\n"),max_roll,max_pitch,min_collective,max_collective,min_tail,max_tail);
                    else{
                        motors.roll_max = max_roll;
                        motors.pitch_max = max_pitch;
                        motors.collective_min = min_collective;
                        motors.collective_max = max_collective;
                        motors._servo_4->radio_min = min_tail;
                        motors._servo_4->radio_max = max_tail;

                        // reinitialise swash
                        motors.init_swash();

                        // display settings
                        report_heli();
                    }
                }
                break;
            case 'p':
            case 'P':
                temp = read_num_from_serial();
                if( temp >= -360 && temp <= 360 ) {
                    if( active_servo == CH_1 )
                        motors.servo1_pos = temp;
                    if( active_servo == CH_2 )
                        motors.servo2_pos = temp;
                    if( active_servo == CH_3 )
                        motors.servo3_pos = temp;
                    motors.init_swash();
                    cliSerial->printf_P(PSTR("Servo %d\t\tpos:%d\n"),active_servo+1, temp);
                }
                break;
            case 'r':
            case 'R':
                heli_get_servo(active_servo)->set_reverse(!heli_get_servo(active_servo)->get_reverse());
                break;
            case 't':
            case 'T':
                temp = read_num_from_serial();
                if( temp > 1000 )
                    temp -= 1500;
                if( temp > -500 && temp < 500 ) {
                    heli_get_servo(active_servo)->radio_trim = 1500 + temp;
                    motors.init_swash();
                    cliSerial->printf_P(PSTR("Servo %d\t\ttrim:%d\n"),active_servo+1, 1500 + temp);
                }
                break;
            case 'u':
            case 'U':
                temp = 0;
                // delay up to 2 seconds for servo type from user
                while( !cliSerial->available() && temp < 20 ) {
                    temp++;
                    delay(100);
                }
                if( cliSerial->available() ) {
                    value = cliSerial->read();
                    if( value == 'a' || value == 'A' ) {
                        g.rc_speed.set_and_save(AP_MOTORS_HELI_SPEED_ANALOG_SERVOS);
                        //motors._speed_hz = AP_MOTORS_HELI_SPEED_ANALOG_SERVOS;  // need to force this update to take effect immediately
                        cliSerial->printf_P(PSTR("Analog Servo %dhz\n"),(int)g.rc_speed);
                    }
                    if( value == 'd' || value == 'D' ) {
                        g.rc_speed.set_and_save(AP_MOTORS_HELI_SPEED_ANALOG_SERVOS);
                        //motors._speed_hz = AP_MOTORS_HELI_SPEED_ANALOG_SERVOS;  // need to force this update to take effect immediately
                        cliSerial->printf_P(PSTR("Digital Servo %dhz\n"),(int)g.rc_speed);
                    }
                }
                break;
            case 'z':
            case 'Z':
                heli_get_servo(active_servo)->radio_trim -= 10;
                break;
            }
        }

        delay(20);
    }

    // display final settings
    report_heli();

    // save to eeprom
    motors._servo_1->save_eeprom();
    motors._servo_2->save_eeprom();
    motors._servo_3->save_eeprom();
    motors._servo_4->save_eeprom();
    motors.servo1_pos.save();
    motors.servo2_pos.save();
    motors.servo3_pos.save();
    motors.roll_max.save();
    motors.pitch_max.save();
    motors.collective_min.save();
    motors.collective_max.save();
    motors.collective_mid.save();

    // return swash plate movements to attitude controller
    motors.servo_manual = false;

    return(0);
}

// setup for external tail gyro (for heli only)
static int8_t
setup_gyro(uint8_t argc, const Menu::arg *argv)
{
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        motors.ext_gyro_enabled.set_and_save(true);

        // optionally capture the gain
        if( argc >= 2 && argv[2].i >= 1000 && argv[2].i <= 2000 ) {
            motors.ext_gyro_gain = argv[2].i;
            motors.ext_gyro_gain.save();
        }

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        motors.ext_gyro_enabled.set_and_save(false);

        // capture gain if user simply provides a number
    } else if( argv[1].i >= 1000 && argv[1].i <= 2000 ) {
        motors.ext_gyro_enabled.set_and_save(true);
        motors.ext_gyro_gain = argv[1].i;
        motors.ext_gyro_gain.save();

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off] gain\n"));
    }

    report_gyro();
    return 0;
}

 #endif // FRAME_CONFIG == HELI

static void clear_offsets()
{
    Vector3f _offsets(0.0,0.0,0.0);
    compass.set_offsets(_offsets);
    compass.save_offsets();
}

static int8_t
setup_optflow(uint8_t argc, const Menu::arg *argv)
{
 #if OPTFLOW == ENABLED
    if (!strcmp_P(argv[1].str, PSTR("on"))) {
        g.optflow_enabled = true;
        init_optflow();

    } else if (!strcmp_P(argv[1].str, PSTR("off"))) {
        g.optflow_enabled = false;

    }else{
        cliSerial->printf_P(PSTR("\nOp:[on, off]\n"));
        report_optflow();
        return 0;
    }

    g.optflow_enabled.save();
    report_optflow();
 #endif     // OPTFLOW == ENABLED
    return 0;
}

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
static int8_t setup_set(uint8_t argc, const Menu::arg *argv)
{
    int8_t value_int8;
    int16_t value_int16;

    AP_Param *param;
    enum ap_var_type p_type;

    if(argc!=3)
    {
        cliSerial->printf_P(PSTR("Invalid command. Usage: set <name> <value>\n"));
        return 0;
    }

    param = AP_Param::find(argv[1].str, &p_type);
    if(!param)
    {
        cliSerial->printf_P(PSTR("Param not found: %s\n"), argv[1].str);
        return 0;
    }

    switch(p_type)
    {
        case AP_PARAM_INT8:
            value_int8 = (int8_t)(argv[2].i);
            if(argv[2].i!=value_int8)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT8\n"));
                return 0;
            }
            ((AP_Int8*)param)->set_and_save(value_int8);
            break;
        case AP_PARAM_INT16:
            value_int16 = (int16_t)(argv[2].i);
            if(argv[2].i!=value_int16)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT16\n"));
                return 0;
            }
            ((AP_Int16*)param)->set_and_save(value_int16);
            break;

        //int32 and float don't need bounds checking, just use the value provoded by Menu::arg
        case AP_PARAM_INT32:
            ((AP_Int32*)param)->set_and_save(argv[2].i);
            break;
        case AP_PARAM_FLOAT:
            ((AP_Float*)param)->set_and_save(argv[2].f);
            break;
        default:
            cliSerial->printf_P(PSTR("Cannot set parameter of type %d.\n"), p_type);
            break;
    }

    return 0;
}

/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if(g.battery_monitoring == BATT_MONITOR_DISABLED) print_enabled(false);
    if(g.battery_monitoring == BATT_MONITOR_VOLTAGE_ONLY) cliSerial->printf_P(PSTR("volts"));
    if(g.battery_monitoring == BATT_MONITOR_VOLTAGE_AND_CURRENT) cliSerial->printf_P(PSTR("volts and cur"));
    print_blanks(2);
}

static void report_wp(uint8_t index = 255)
{
    if(index == 255) {
        for(uint8_t i = 0; i < g.command_total; i++) {
            struct Location temp = get_cmd_with_index(i);
            print_wp(&temp, i);
        }
    }else{
        struct Location temp = get_cmd_with_index(index);
        print_wp(&temp, index);
    }
}

static void report_sonar()
{
    cliSerial->printf_P(PSTR("Sonar\n"));
    print_divider();
    print_enabled(g.sonar_enabled.get());
    cliSerial->printf_P(PSTR("Type: %d (0=XL, 1=LV, 2=XLL, 3=HRLV)"), (int)g.sonar_type);
    print_blanks(2);
}

static void report_frame()
{
    cliSerial->printf_P(PSTR("Frame\n"));
    print_divider();

 #if FRAME_CONFIG == QUAD_FRAME
    cliSerial->printf_P(PSTR("Quad frame\n"));
 #elif FRAME_CONFIG == TRI_FRAME
    cliSerial->printf_P(PSTR("TRI frame\n"));
 #elif FRAME_CONFIG == HEXA_FRAME
    cliSerial->printf_P(PSTR("Hexa frame\n"));
 #elif FRAME_CONFIG == Y6_FRAME
    cliSerial->printf_P(PSTR("Y6 frame\n"));
 #elif FRAME_CONFIG == OCTA_FRAME
    cliSerial->printf_P(PSTR("Octa frame\n"));
 #elif FRAME_CONFIG == HELI_FRAME
    cliSerial->printf_P(PSTR("Heli frame\n"));
 #endif

 #if FRAME_CONFIG != HELI_FRAME
    if(g.frame_orientation == X_FRAME)
        cliSerial->printf_P(PSTR("X mode\n"));
    else if(g.frame_orientation == PLUS_FRAME)
        cliSerial->printf_P(PSTR("+ mode\n"));
    else if(g.frame_orientation == V_FRAME)
        cliSerial->printf_P(PSTR("V mode\n"));
 #endif

    print_blanks(2);
}

static void report_radio()
{
    cliSerial->printf_P(PSTR("Radio\n"));
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

static void report_ins()
{
    cliSerial->printf_P(PSTR("INS\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
    print_blanks(2);
}

static void report_compass()
{
    cliSerial->printf_P(PSTR("Compass\n"));
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    cliSerial->printf_P(PSTR("Mag Dec: %4.4f\n"),
                    degrees(compass.get_declination()));

    Vector3f offsets = compass.get_offsets();

    // mag offsets
    cliSerial->printf_P(PSTR("Mag off: %4.4f, %4.4f, %4.4f\n"),
                    offsets.x,
                    offsets.y,
                    offsets.z);

    // motor compensation
    cliSerial->print_P(PSTR("Motor Comp: "));
    if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_DISABLED ) {
        cliSerial->print_P(PSTR("Off\n"));
    }else{
        if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_THROTTLE ) {
            cliSerial->print_P(PSTR("Throttle"));
        }
        if( compass.motor_compensation_type() == AP_COMPASS_MOT_COMP_CURRENT ) {
            cliSerial->print_P(PSTR("Current"));
        }
        Vector3f motor_compensation = compass.get_motor_compensation();
        cliSerial->printf_P(PSTR("\nComp Vec: %4.2f, %4.2f, %4.2f\n"),
                        motor_compensation.x,
                        motor_compensation.y,
                        motor_compensation.z);
    }
    print_blanks(1);
}

static void report_flight_modes()
{
    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i], BIT_IS_SET(g.simple_modes, i));
    }
    print_blanks(2);
}

void report_optflow()
{
 #if OPTFLOW == ENABLED
    cliSerial->printf_P(PSTR("OptFlow\n"));
    print_divider();

    print_enabled(g.optflow_enabled);

    print_blanks(2);
 #endif     // OPTFLOW == ENABLED
}

 #if FRAME_CONFIG == HELI_FRAME
static void report_heli()
{
    cliSerial->printf_P(PSTR("Heli\n"));
    print_divider();

    // main servo settings
    cliSerial->printf_P(PSTR("Servo \tpos \tmin \tmax \trev\n"));
    cliSerial->printf_P(PSTR("1:\t%d \t%d \t%d \t%d\n"),(int)motors.servo1_pos, (int)motors._servo_1->radio_min, (int)motors._servo_1->radio_max, (int)motors._servo_1->get_reverse());
    cliSerial->printf_P(PSTR("2:\t%d \t%d \t%d \t%d\n"),(int)motors.servo2_pos, (int)motors._servo_2->radio_min, (int)motors._servo_2->radio_max, (int)motors._servo_2->get_reverse());
    cliSerial->printf_P(PSTR("3:\t%d \t%d \t%d \t%d\n"),(int)motors.servo3_pos, (int)motors._servo_3->radio_min, (int)motors._servo_3->radio_max, (int)motors._servo_3->get_reverse());
    cliSerial->printf_P(PSTR("tail:\t\t%d \t%d \t%d\n"), (int)motors._servo_4->radio_min, (int)motors._servo_4->radio_max, (int)motors._servo_4->get_reverse());

    cliSerial->printf_P(PSTR("roll max: \t%d\n"), (int)motors.roll_max);
    cliSerial->printf_P(PSTR("pitch max: \t%d\n"), (int)motors.pitch_max);
    cliSerial->printf_P(PSTR("coll min:\t%d\t mid:%d\t max:%d\n"),(int)motors.collective_min, (int)motors.collective_mid, (int)motors.collective_max);

    // calculate and print servo rate
    cliSerial->printf_P(PSTR("servo rate:\t%d hz\n"),(int)g.rc_speed);

    print_blanks(2);
}

static void report_gyro()
{

    cliSerial->printf_P(PSTR("Gyro:\n"));
    print_divider();

    print_enabled( motors.ext_gyro_enabled );
    if( motors.ext_gyro_enabled )
        cliSerial->printf_P(PSTR("gain: %d"),(int)motors.ext_gyro_gain);

    print_blanks(2);
}

 #endif // FRAME_CONFIG == HELI_FRAME

/***************************************************************************/
// CLI utilities
/***************************************************************************/

/*static void
 *  print_PID(PI * pid)
 *  {
 *       cliSerial->printf_P(PSTR("P: %4.2f, I:%4.2f, IMAX:%ld\n"),
 *                                               pid->kP(),
 *                                               pid->kI(),
 *                                               (long)pid->imax());
 *  }
 */

static void
print_radio_values()
{
    cliSerial->printf_P(PSTR("CH1: %d | %d\n"), (int)g.rc_1.radio_min, (int)g.rc_1.radio_max);
    cliSerial->printf_P(PSTR("CH2: %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_max);
    cliSerial->printf_P(PSTR("CH3: %d | %d\n"), (int)g.rc_3.radio_min, (int)g.rc_3.radio_max);
    cliSerial->printf_P(PSTR("CH4: %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_max);
    cliSerial->printf_P(PSTR("CH5: %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_max);
    cliSerial->printf_P(PSTR("CH6: %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_max);
    cliSerial->printf_P(PSTR("CH7: %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_max);
    //cliSerial->printf_P(PSTR("CH8: %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

static void
print_switch(uint8_t p, uint8_t m, bool b)
{
    cliSerial->printf_P(PSTR("Pos %d:\t"),p);
    print_flight_mode(cliSerial, m);
    cliSerial->printf_P(PSTR(",\t\tSimple: "));
    if(b)
        cliSerial->printf_P(PSTR("ON\n"));
    else
        cliSerial->printf_P(PSTR("OFF\n"));
}

static void
print_done()
{
    cliSerial->printf_P(PSTR("\nSaved\n"));
}


static void zero_eeprom(void)
{
    cliSerial->printf_P(PSTR("\nErasing EEPROM\n"));

    for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i++) {
        hal.storage->write_byte(i, 0);
    }

    cliSerial->printf_P(PSTR("done\n"));
}

static void
print_accel_offsets_and_scaling(void)
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    cliSerial->printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\nA_scale: %4.2f, %4.2f, %4.2f\n"),
                    (float)accel_offsets.x,                           // Pitch
                    (float)accel_offsets.y,                           // Roll
                    (float)accel_offsets.z,                           // YAW
                    (float)accel_scale.x,                             // Pitch
                    (float)accel_scale.y,                             // Roll
                    (float)accel_scale.z);                            // YAW
}

static void
print_gyro_offsets(void)
{
    Vector3f gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
}

 #if FRAME_CONFIG == HELI_FRAME

static RC_Channel *
heli_get_servo(int16_t servo_num){
    if( servo_num == CH_1 )
        return motors._servo_1;
    if( servo_num == CH_2 )
        return motors._servo_2;
    if( servo_num == CH_3 )
        return motors._servo_3;
    if( servo_num == CH_4 )
        return motors._servo_4;
    return NULL;
}

// Used to read integer values from the serial port
static int16_t read_num_from_serial() {
    uint8_t index = 0;
    uint8_t timeout = 0;
    char data[5] = "";

    do {
        if (cliSerial->available() == 0) {
            delay(10);
            timeout++;
        }else{
            data[index] = cliSerial->read();
            timeout = 0;
            index++;
        }
    } while (timeout < 5 && index < 5);

    return atoi(data);
}
 #endif

#endif // CLI_ENABLED

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}

static void
print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->print_P(PSTR("-"));
    }
    cliSerial->println();
}

static void print_enabled(bool b)
{
    if(b)
        cliSerial->print_P(PSTR("en"));
    else
        cliSerial->print_P(PSTR("dis"));
    cliSerial->print_P(PSTR("abled\n"));
}


static void
init_esc()
{
    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // we enable the motors directly here instead of calling output_min because output_min would send a low signal to the ESC and disrupt the calibration process
    motors.enable();
    motors.armed(true);
    while(1) {
        read_radio();
        delay(100);
        dancing_light();
        motors.throttle_pass_through();
    }
}

static void print_wp(const struct Location *cmd, uint8_t index)
{
    cliSerial->printf_P(PSTR("cmd#: %d | %d, %d, %d, %ld, %ld, %ld\n"),
                    index,
                    cmd->id,
                    cmd->options,
                    cmd->p1,
                    cmd->alt,
                    cmd->lat,
                    cmd->lng);
}

static void report_version()
{
    cliSerial->printf_P(PSTR("FW Ver: %d\n"),(int)g.k_format_version);
    print_divider();
    print_blanks(2);
}


static void report_tuning()
{
    cliSerial->printf_P(PSTR("\nTUNE:\n"));
    print_divider();
    if (g.radio_tuning == 0) {
        print_enabled(g.radio_tuning.get());
    }else{
        float low  = (float)g.radio_tuning_low.get() / 1000;
        float high = (float)g.radio_tuning_high.get() / 1000;
        cliSerial->printf_P(PSTR(" %d, Low:%1.4f, High:%1.4f\n"),(int)g.radio_tuning.get(), low, high);
    }
    print_blanks(2);
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/system.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED
// Functions called from the top-level menu
static int8_t   process_logs(uint8_t argc, const Menu::arg *argv);      // in Log.pde
static int8_t   setup_mode(uint8_t argc, const Menu::arg *argv);        // in setup.pde
static int8_t   test_mode(uint8_t argc, const Menu::arg *argv);         // in test.cpp
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of print_f that reads from flash memory
static int8_t   main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Commands:\n"
                         "  logs\n"
                         "  setup\n"
                         "  test\n"
                         "  reboot\n"
                         "\n"));
    return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
    {"logs",                process_logs},
    {"setup",               setup_mode},
    {"test",                test_mode},
    {"reboot",              reboot_board},
    {"help",                main_menu_help},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    reboot_apm();
    return 0;
}

// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    // disable main_loop failsafe
    failsafe_disable();

    // cut the engines
    if(motors.armed()) {
        motors.armed(false);
        motors.output();
    }
    
    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED

static void init_ardupilot()
{
#if USB_MUX_PIN > 0
    // on the APM2 board we have a mux thet switches UART0 between
    // USB and the board header. If the right ArduPPM firmware is
    // installed we can detect if USB is connected using the
    // USB_MUX_PIN
    pinMode(USB_MUX_PIN, INPUT);

    ap_system.usb_connected = !digitalRead(USB_MUX_PIN);
    if (!ap_system.usb_connected) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }
#endif

    // Console serial port
    //
    // The console port buffers are defined to be sufficiently large to support
    // the MAVLink protocol efficiently
    //
#if HIL_MODE != HIL_MODE_DISABLED
    // we need more memory for HIL, as we get a much higher packet rate
    hal.uartA->begin(SERIAL0_BAUD, 256, 256);
#else
    // use a bit less for non-HIL operation
    hal.uartA->begin(SERIAL0_BAUD, 128, 128);
#endif

    // GPS serial port.
    //
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    hal.uartB->begin(38400, 256, 16);
#endif

    cliSerial->printf_P(PSTR("\n\nInit " THISFIRMWARE
                         "\n\nFree RAM: %u\n"),
                    memcheck_available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // setup IO pins
    pinMode(A_LED_PIN, OUTPUT);                                 // GPS status LED
    digitalWrite(A_LED_PIN, LED_OFF);

    pinMode(B_LED_PIN, OUTPUT);                         // GPS status LED
    digitalWrite(B_LED_PIN, LED_OFF);

    pinMode(C_LED_PIN, OUTPUT);                         // GPS status LED
    digitalWrite(C_LED_PIN, LED_OFF);

    relay.init(); 

#if COPTER_LEDS == ENABLED
    copter_leds_init();
#endif

    // load parameters from EEPROM
    load_parameters();

#if HIL_MODE != HIL_MODE_ATTITUDE
    barometer.init();
#endif

    // init the GCS
    gcs0.init(hal.uartA);

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

#if USB_MUX_PIN > 0
    if (!ap_system.usb_connected) {
        // we are not connected via USB, re-init UART0 with right
        // baud rate
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
#else
    // we have a 2nd serial port for telemetry
    hal.uartC->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD), 128, 128);
    gcs3.init(hal.uartC);
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;
    mavlink_system.type = 2; //MAV_QUADROTOR;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init();
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("No dataflash inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("ERASING LOGS"));
        do_erase_logs();
        gcs0.reset_cli_timeout();
    }
#endif

#if FRAME_CONFIG == HELI_FRAME
    motors.servo_manual = false;
    motors.init_swash();              // heli initialisation
#endif

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

#if HIL_MODE != HIL_MODE_ATTITUDE
 #if CONFIG_ADC == ENABLED
    // begin filtering the ADC Gyros
    adc.Init();           // APM ADC library initialization
 #endif // CONFIG_ADC
#endif // HIL_MODE

    // Do GPS init
    g_gps = &g_gps_driver;
    // GPS Initialization
    g_gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_1G);

    if(g.compass_enabled)
        init_compass();

    // init the optical flow sensor
    if(g.optflow_enabled) {
        init_optflow();
    }

    // initialise inertial nav
    inertial_nav.init();

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED
    const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
    cliSerial->println_P(msg);
#if USB_MUX_PIN == 0
    hal.uartC->println_P(msg);
#endif
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_DISABLED
    while (!barometer.healthy) {
        // the barometer becomes healthy when we get the first
        // HIL_STATE message
        gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }
#endif

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground
    //-----------------------------
    init_barometer();
#endif

    // initialise sonar
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif

#if FRAME_CONFIG == HELI_FRAME
    // initialise controller filters
    init_rate_controllers();
#endif // HELI_FRAME

    // initialize commands
    // -------------------
    init_commands();

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();


    startup_ground();

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif

    cliSerial->print_P(PSTR("\nReady to FLY "));
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
static void startup_ground(void)
{
    gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();

    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(AP_InertialSensor::COLD_START,
             ins_sample_rate,
             flash_leds);
 #if CLI_ENABLED == ENABLED
    report_ins();
 #endif

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.init(&timer_scheduler);
    ahrs2.set_as_secondary(true);
    ahrs2.set_fast_gains(true);
#endif

    // reset the leds
    // ---------------------------
    clear_leds();

    // when we re-calibrate the gyros,
    // all previous I values are invalid
    reset_I_all();
}

// set_mode - change flight mode and perform any necessary initialisation
static void set_mode(uint8_t mode)
{
    // Switch to stabilize mode if requested mode requires a GPS lock
    if(!ap.home_is_set) {
        if (mode > ALT_HOLD && mode != TOY_A && mode != TOY_M && mode != OF_LOITER && mode != LAND) {
            mode = STABILIZE;
        }
    }

    // Switch to stabilize if OF_LOITER requested but no optical flow sensor
    if (mode == OF_LOITER && !g.optflow_enabled ) {
        mode = STABILIZE;
    }

    control_mode 	= mode;
    control_mode    = constrain_int16(control_mode, 0, NUM_MODES - 1);

    // if we change modes, we must clear landed flag
    set_land_complete(false);

    // report the GPS and Motor arming status
    led_mode = NORMAL_LEDS;

    switch(control_mode)
    {
    case ACRO:
    	ap.manual_throttle = true;
    	ap.manual_attitude = true;
        set_yaw_mode(ACRO_YAW);
        set_roll_pitch_mode(ACRO_RP);
        set_throttle_mode(ACRO_THR);
        set_nav_mode(NAV_NONE);
        // reset acro axis targets to current attitude
		if(g.axis_enabled){
            roll_axis 	= ahrs.roll_sensor;
            pitch_axis 	= ahrs.pitch_sensor;
            nav_yaw 	= ahrs.yaw_sensor;
        }
        break;

    case STABILIZE:
    	ap.manual_throttle = true;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_HOLD);
        set_roll_pitch_mode(ROLL_PITCH_STABLE);
        set_throttle_mode(THROTTLE_MANUAL_TILT_COMPENSATED);
        set_nav_mode(NAV_NONE);
        break;

    case ALT_HOLD:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(ALT_HOLD_YAW);
        set_roll_pitch_mode(ALT_HOLD_RP);
        set_throttle_mode(ALT_HOLD_THR);
        set_nav_mode(NAV_NONE);
        break;

    case AUTO:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        // roll-pitch, throttle and yaw modes will all be set by the first nav command
        init_commands();            // clear the command queues. will be reloaded when "run_autopilot" calls "update_commands" function
        break;

    case CIRCLE:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_roll_pitch_mode(CIRCLE_RP);
        set_throttle_mode(CIRCLE_THR);
        set_nav_mode(CIRCLE_NAV);
        set_yaw_mode(CIRCLE_YAW);
        break;

    case LOITER:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(LOITER_YAW);
        set_roll_pitch_mode(LOITER_RP);
        set_throttle_mode(LOITER_THR);
        set_nav_mode(LOITER_NAV);
        break;

    case POSITION:
    	ap.manual_throttle = true;
    	ap.manual_attitude = false;
        set_yaw_mode(POSITION_YAW);
        set_roll_pitch_mode(POSITION_RP);
        set_throttle_mode(POSITION_THR);
        set_nav_mode(POSITION_NAV);
        wp_nav.clear_angle_limit();     // ensure there are no left over angle limits from throttle controller.  To-Do: move this to the exit routine of throttle controller
        break;

    case GUIDED:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(get_wp_yaw_mode(false));
        set_roll_pitch_mode(GUIDED_RP);
        set_throttle_mode(GUIDED_THR);
        set_nav_mode(GUIDED_NAV);
        break;

    case LAND:
        // To-Do: it is messy to set manual_attitude here because the do_land function is reponsible for setting the roll_pitch_mode
        if( ap.home_is_set ) {
            // switch to loiter if we have gps
            ap.manual_attitude = false;
        }else{
            // otherwise remain with stabilize roll and pitch
            ap.manual_attitude = true;
        }
    	ap.manual_throttle = false;
        do_land();
        break;

    case RTL:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        do_RTL();
        break;

    case OF_LOITER:
    	ap.manual_throttle = false;
    	ap.manual_attitude = false;
        set_yaw_mode(OF_LOITER_YAW);
        set_roll_pitch_mode(OF_LOITER_RP);
        set_throttle_mode(OF_LOITER_THR);
        set_nav_mode(OF_LOITER_NAV);
        break;

    // THOR
    // These are the flight modes for Toy mode
    // See the defines for the enumerated values
    case TOY_A:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_TOY);
        set_roll_pitch_mode(ROLL_PITCH_TOY);
        set_throttle_mode(THROTTLE_AUTO);
        set_nav_mode(NAV_NONE);

        // save throttle for fast exit of Alt hold
        saved_toy_throttle = g.rc_3.control_in;
        break;

    case TOY_M:
    	ap.manual_throttle = false;
    	ap.manual_attitude = true;
        set_yaw_mode(YAW_TOY);
        set_roll_pitch_mode(ROLL_PITCH_TOY);
        set_nav_mode(NAV_NONE);
        set_throttle_mode(THROTTLE_HOLD);
        break;

    default:
        break;
    }

    if(ap.manual_attitude) {
        // We are under manual attitude control
        // remove the navigation from roll and pitch command
        reset_nav_params();
    }

    Log_Write_Mode(control_mode);
}

static void
init_simple_bearing()
{
    initial_simple_bearing = ahrs.yaw_sensor;
    if (g.log_bitmask != 0) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, initial_simple_bearing);
    }
}

// update_auto_armed - update status of auto_armed flag
static void update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors.armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(control_mode <= ACRO && g.rc_3.control_in == 0 && !ap.failsafe_radio) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        // if motors are armed and throttle is above zero auto_armed should be true
        if(motors.armed() && g.rc_3.control_in != 0) {
            set_auto_armed(true);
        }
    }
}

/*
 *  map from a 8 bit EEPROM baud rate to a real baud rate
 */
static uint32_t map_baudrate(int8_t rate, uint32_t default_baud)
{
    switch (rate) {
    case 1:    return 1200;
    case 2:    return 2400;
    case 4:    return 4800;
    case 9:    return 9600;
    case 19:   return 19200;
    case 38:   return 38400;
    case 57:   return 57600;
    case 111:  return 111100;
    case 115:  return 115200;
    }
    //cliSerial->println_P(PSTR("Invalid SERIAL3_BAUD"));
    return default_baud;
}

#if USB_MUX_PIN > 0
static void check_usb_mux(void)
{
    bool usb_check = !digitalRead(USB_MUX_PIN);
    if (usb_check == ap_system.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap_system.usb_connected = usb_check;
    if (ap_system.usb_connected) {
        hal.uartA->begin(SERIAL0_BAUD);
    } else {
        hal.uartA->begin(map_baudrate(g.serial3_baud, SERIAL3_BAUD));
    }
}
#endif

/*
 *  called by gyro/accel init to flash LEDs so user
 *  has some mesmerising lights to watch while waiting
 */
void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on ? LED_OFF : LED_ON);
    digitalWrite(C_LED_PIN, on ? LED_ON : LED_OFF);
}

/*
 * Read Vcc vs 1.1v internal reference
 */
uint16_t board_voltage(void)
{
    return board_vcc_analog_source->read_latest();
}

/*
  force a software reset of the APM
 */
static void reboot_apm(void) {
    hal.scheduler->reboot();
}

//
// print_flight_mode - prints flight mode to serial port.
//
static void
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        port->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        port->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        port->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        port->print_P(PSTR("LOITER"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        port->print_P(PSTR("CIRCLE"));
        break;
    case POSITION:
        port->print_P(PSTR("POSITION"));
        break;
    case LAND:
        port->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        port->print_P(PSTR("OF_LOITER"));
        break;
    case TOY_M:
        port->print_P(PSTR("TOY_M"));
        break;
    case TOY_A:
        port->print_P(PSTR("TOY_A"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}
#line 1 "/home/tobias/git/ardupilot/ArduCopter/test.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   test_radio_pwm(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
//static int8_t	test_failsafe(uint8_t argc,     const Menu::arg *argv);
//static int8_t	test_stabilize(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
//static int8_t	test_tri(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_adc(uint8_t argc,          const Menu::arg *argv);
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
//static int8_t	test_imu(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_dcm_eulers(uint8_t argc,   const Menu::arg *argv);
//static int8_t	test_dcm(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_omega(uint8_t argc,        const Menu::arg *argv);
//static int8_t	test_stab_d(uint8_t argc,       const Menu::arg *argv);
static int8_t   test_battery(uint8_t argc,              const Menu::arg *argv);
//static int8_t	test_toy(uint8_t argc,      const Menu::arg *argv);
static int8_t   test_wp_nav(uint8_t argc,               const Menu::arg *argv);
//static int8_t	test_reverse(uint8_t argc,      const Menu::arg *argv);
static int8_t   test_tuning(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_wp(uint8_t argc,                   const Menu::arg *argv);
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
static int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
static int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
#endif
static int8_t   test_mag(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_logging(uint8_t argc,              const Menu::arg *argv);
//static int8_t	test_xbee(uint8_t argc,         const Menu::arg *argv);
static int8_t   test_eedump(uint8_t argc,               const Menu::arg *argv);
//static int8_t   test_rawgps(uint8_t argc,               const Menu::arg *argv);
//static int8_t	test_mission(uint8_t argc,      const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static int8_t   test_shell(uint8_t argc,              const Menu::arg *argv);
#endif

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of printf that reads from flash memory
/*static int8_t	help_test(uint8_t argc,             const Menu::arg *argv)
 *  {
 *       cliSerial->printf_P(PSTR("\n"
 *                                                "Commands:\n"
 *                                                "  radio\n"
 *                                                "  servos\n"
 *                                                "  g_gps\n"
 *                                                "  imu\n"
 *                                                "  battery\n"
 *                                                "\n"));
 *  }*/

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
    {"pwm",                 test_radio_pwm},
    {"radio",               test_radio},
    {"gps",                 test_gps},
    {"ins",                 test_ins},
    {"battery",             test_battery},
    {"tune",                test_tuning},
    {"relay",               test_relay},
    {"wp",                  test_wp},
//	{"toy",			        test_toy},
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
    {"baro",                test_baro},
    {"sonar",               test_sonar},
#endif
    {"compass",             test_mag},
    {"optflow",             test_optflow},
    //{"xbee",		test_xbee},
    {"eedump",              test_eedump},
    {"logging",             test_logging},
    {"nav",                 test_wp_nav},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    {"shell", 				test_shell},
#endif
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    //cliSerial->printf_P(PSTR("Test Mode\n\n"));
    test_menu.run();
    return 0;
}

static int8_t
test_eedump(uint8_t argc, const Menu::arg *argv)
{

    // hexdump the EEPROM
    for (uint16_t i = 0; i < EEPROM_MAX_ADDR; i += 16) {
        cliSerial->printf_P(PSTR("%04x:"), i);
        for (uint16_t j = 0; j < 16; j++)  {
            int b = hal.storage->read_byte(i+j);
            cliSerial->printf_P(PSTR(" %02x"), b);
        }
        cliSerial->println();
    }
    return(0);
}


static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        // servo Yaw
        //APM_RC.OutputCh(CH_7, g.rc_4.radio_out);

        cliSerial->printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        g.rc_1.radio_in,
                        g.rc_2.radio_in,
                        g.rc_3.radio_in,
                        g.rc_4.radio_in,
                        g.rc_5.radio_in,
                        g.rc_6.radio_in,
                        g.rc_7.radio_in,
                        g.rc_8.radio_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

/*
//static int8_t
//test_toy(uint8_t argc, const Menu::arg *argv)
{

 	for(altitude_error = 2000; altitude_error > -100; altitude_error--){
 		int16_t temp = get_desired_climb_rate();
		cliSerial->printf("%ld, %d\n", altitude_error, temp);
	}
 	return 0;
}
{	wp_distance = 0;
	int16_t max_speed = 0;

 	for(int16_t i = 0; i < 200; i++){
	 	int32_t temp = 2 * 100 * (wp_distance - wp_nav.get_waypoint_radius());
		max_speed = sqrtf((float)temp);
		max_speed = min(max_speed, wp_nav.get_horizontal_speed());
		cliSerial->printf("Zspeed: %ld, %d, %ld\n", temp, max_speed, wp_distance);
	 	wp_distance += 100;
	}
 	return 0;
 }
//*/

/*static int8_t
 *  //test_toy(uint8_t argc, const Menu::arg *argv)
 *  {
 *       int16_t yaw_rate;
 *       int16_t roll_rate;
 *       g.rc_1.control_in = -2500;
 *       g.rc_2.control_in = 2500;
 *
 *       g.toy_yaw_rate = 3;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *
 *       g.toy_yaw_rate = 2;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *
 *       g.toy_yaw_rate = 1;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *  }*/

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);
        read_radio();


        cliSerial->printf_P(PSTR("IN  1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n"),
                        g.rc_1.control_in,
                        g.rc_2.control_in,
                        g.rc_3.control_in,
                        g.rc_4.control_in,
                        g.rc_5.control_in,
                        g.rc_6.control_in,
                        g.rc_7.control_in);

        //cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (g.rc_1.servo_out / 100), (g.rc_2.servo_out / 100), g.rc_3.servo_out, (g.rc_4.servo_out / 100));

        /*cliSerial->printf_P(PSTR(	"min: %d"
         *                                               "\t in: %d"
         *                                               "\t pwm_in: %d"
         *                                               "\t sout: %d"
         *                                               "\t pwm_out %d\n"),
         *                                               g.rc_3.radio_min,
         *                                               g.rc_3.control_in,
         *                                               g.rc_3.radio_in,
         *                                               g.rc_3.servo_out,
         *                                               g.rc_3.pwm_out
         *                                               );
         */
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
    Vector3f gyro, accel;
    print_hit_enter();
    cliSerial->printf_P(PSTR("INS\n"));
    delay(1000);

    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();

        float test = accel.length() / GRAVITY_MSS;

        cliSerial->printf_P(PSTR("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %74f | %7.4f\n"),
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            test);

        delay(40);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(100);

        // Blink GPS LED if we don't have a fix
        // ------------------------------------
        update_GPS_light();

        g_gps->update();

        if (g_gps->new_data) {
            cliSerial->printf_P(PSTR("Lat: "));
            print_latlon(cliSerial, g_gps->latitude);
            cliSerial->printf_P(PSTR(", Lon "));
            print_latlon(cliSerial, g_gps->longitude);
            cliSerial->printf_P(PSTR(", Alt: %ldm, #sats: %d\n"),
                            g_gps->altitude/100,
                            g_gps->num_sats);
            g_gps->new_data = false;
        }else{
            cliSerial->print_P(PSTR("."));
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}

static int8_t
test_tuning(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();

    while(1) {
        delay(200);
        read_radio();
        tuning();
        cliSerial->printf_P(PSTR("tune: %1.3f\n"), tuning_value);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
    // check if radio is calibration
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        cliSerial->print_P(PSTR("radio not calibrated, exiting"));
        return(0);
    }

    cliSerial->printf_P(PSTR("\nCareful! Motors will spin! Press Enter to start.\n"));
    while (cliSerial->read() != -1); /* flush */
    while(!cliSerial->available()) { /* wait for input */
        delay(100);
    }
    while (cliSerial->read() != -1); /* flush */
    print_hit_enter();

    // allow motors to spin
    output_min();
    motors.armed(true);

    while(1) {
        delay(100);
        read_radio();
        read_battery();
        if (g.battery_monitoring == BATT_MONITOR_VOLTAGE_ONLY) {
            cliSerial->printf_P(PSTR("V: %4.4f\n"),
                            battery_voltage1,
                            current_amps1,
                            current_total1);
        } else {
            cliSerial->printf_P(PSTR("V: %4.4f, A: %4.4f, Ah: %4.4f\n"),
                            battery_voltage1,
                            current_amps1,
                            current_total1);
        }
        motors.throttle_pass_through();

        if(cliSerial->available() > 0) {
            motors.armed(false);
            return (0);
        }
    }
    motors.armed(false);
    return (0);
}

static int8_t test_relay(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        cliSerial->printf_P(PSTR("Relay on\n"));
        relay.on();
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf_P(PSTR("Relay off\n"));
        relay.off();
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
    delay(1000);

    // save the alitude above home option
    cliSerial->printf_P(PSTR("Hold alt "));
    if(g.rtl_altitude < 0) {
        cliSerial->printf_P(PSTR("\n"));
    }else{
        cliSerial->printf_P(PSTR("of %dm\n"), (int)g.rtl_altitude / 100);
    }

    cliSerial->printf_P(PSTR("%d wp\n"), (int)g.command_total);
    cliSerial->printf_P(PSTR("Hit rad: %dm\n"), (int)wp_nav.get_waypoint_radius());

    report_wp();

    return (0);
}

#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
static int8_t
test_baro(uint8_t argc, const Menu::arg *argv)
{
    int32_t alt;
    print_hit_enter();
    init_barometer();

    while(1) {
        delay(100);
        alt = read_barometer();

        if (!barometer.healthy) {
            cliSerial->println_P(PSTR("not healthy"));
        } else {
            cliSerial->printf_P(PSTR("Alt: %0.2fm, Raw: %f Temperature: %.1f\n"),
                            alt / 100.0,
                            barometer.get_pressure(), 0.1*barometer.get_temperature());
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}
#endif


static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
    uint8_t delta_ms_fast_loop;

    if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
        print_enabled(false);
        return (0);
    }

    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    ahrs.reset();
    int16_t counter = 0;
    float heading = 0;

    print_hit_enter();

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer;
            G_Dt                    = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer          = millis();

            // INS
            // ---
            ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5) {
                if (compass.read()) {
                    // Calculate heading
                    const Matrix3f &m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.null_offsets();
                }
                medium_loopCounter = 0;
            }

            counter++;
            if (counter>20) {
                if (compass.healthy) {
                    Vector3f maggy = compass.get_offsets();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %d, %d, %d,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                    (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                    (int)compass.mag_x,
                                    (int)compass.mag_y,
                                    (int)compass.mag_z,
                                    maggy.x,
                                    maggy.y,
                                    maggy.z);
                } else {
                    cliSerial->println_P(PSTR("compass not healthy"));
                }
                counter=0;
            }
        }
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.
    cliSerial->println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
/*
 *  test the sonar
 */
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
#if CONFIG_SONAR == ENABLED
    if(g.sonar_enabled == false) {
        cliSerial->printf_P(PSTR("Sonar disabled\n"));
        return (0);
    }

    // make sure sonar is initialised
    init_sonar();

    print_hit_enter();
    while(1) {
        delay(100);

        cliSerial->printf_P(PSTR("Sonar: %d cm\n"), sonar->read());

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
    return (0);
}
#endif


static int8_t
test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(g.optflow_enabled) {
        cliSerial->printf_P(PSTR("man id: %d\t"),optflow.read_register(ADNS3080_PRODUCT_ID));
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update(millis());
            Log_Write_Optflow();
            cliSerial->printf_P(PSTR("x/dx: %d/%d\t y/dy %d/%d\t squal:%d\n"),
                            optflow.x,
                            optflow.dx,
                            optflow.y,
                            optflow.dy,
                            optflow.surface_quality);

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("OptFlow: "));
        print_enabled(false);
    }
    return (0);
#else
    return (0);
#endif      // OPTFLOW == ENABLED
}


static int8_t
test_wp_nav(uint8_t argc, const Menu::arg *argv)
{
    current_loc.lat = 389539260;
    current_loc.lng = -1199540200;

    wp_nav.set_destination(pv_latlon_to_vector(389538528,-1199541248,0));

    // got 23506;, should be 22800
    update_navigation();
    cliSerial->printf_P(PSTR("bear: %ld\n"), wp_bearing);
    return 0;
}

/*
 *  test the dataflash is working
 */

static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->println_P(PSTR("Testing dataflash logging"));
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
/*
 *  run a debug shell
 */
static int8_t
test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

#endif // CLI_ENABLED
#line 1 "/home/tobias/git/ardupilot/ArduCopter/toy.pde"
////////////////////////////////////////////////////////////////////////////////
// Toy Mode - THOR
////////////////////////////////////////////////////////////////////////////////
static bool CH7_toy_flag;

#if TOY_MIXER == TOY_LOOKUP_TABLE
static const int16_t toy_lookup[] = {
    186,    373,    558,    745,
    372,    745,    1117,   1490,
    558,    1118,   1675,   2235,
    743,    1490,   2233,   2980,
    929,    1863,   2792,   3725,
    1115,   2235,   3350,   4470,
    1301,   2608,   3908,   4500,
    1487,   2980,   4467,   4500,
    1673,   3353,   4500,   4500
};
#endif

//called at 10hz
void update_toy_throttle()
{
    // look for a change in throttle position to exit throttle hold
    if(abs(g.rc_3.control_in - saved_toy_throttle) > 40) {
        throttle_mode   = THROTTLE_MANUAL;
    }
}

#define TOY_ALT_SMALL 25
#define TOY_ALT_LARGE 100

//called at 10hz
void update_toy_altitude()
{
    int16_t input = g.rc_3.radio_in;     // throttle
    float current_target_alt = wp_nav.get_desired_alt();
    //int16_t input = g.rc_7.radio_in;

    // Trigger upward alt change
    if(false == CH7_toy_flag && input > 1666) {
        CH7_toy_flag = true;
        // go up
        if(current_target_alt >= 400) {
            wp_nav.set_desired_alt(current_target_alt + TOY_ALT_LARGE);
        }else{
            wp_nav.set_desired_alt(current_target_alt + TOY_ALT_SMALL);
        }

        // Trigger downward alt change
    }else if(false == CH7_toy_flag && input < 1333) {
        CH7_toy_flag = true;
        // go down
        if(current_target_alt >= (400 + TOY_ALT_LARGE)) {
            wp_nav.set_desired_alt(current_target_alt - TOY_ALT_LARGE);
        }else if(current_target_alt >= TOY_ALT_SMALL) {
            wp_nav.set_desired_alt(current_target_alt - TOY_ALT_SMALL);
        }else if(current_target_alt < TOY_ALT_SMALL) {
            wp_nav.set_desired_alt(0); 
        }

        // clear flag
    }else if (CH7_toy_flag && ((input < 1666) && (input > 1333))) {
        CH7_toy_flag = false;
    }
}

// called at 50 hz from all flight modes
#if TOY_EDF == ENABLED
void edf_toy()
{
    // EDF control:
    g.rc_8.radio_out = 1000 + ((abs(g.rc_2.control_in) << 1) / 9);
    if(g.rc_8.radio_out < 1050)
        g.rc_8.radio_out = 1000;

    // output throttle to EDF
    if(motors.armed()) {
        APM_RC.OutputCh(CH_8, g.rc_8.radio_out);
    }else{
        APM_RC.OutputCh(CH_8, 1000);
    }

    // output Servo direction
    if(g.rc_2.control_in > 0) {
        APM_RC.OutputCh(CH_6, 1000);         // 1000 : 2000
    }else{
        APM_RC.OutputCh(CH_6, 2000);         // less than 1450
    }
}
#endif

// The function call for managing the flight mode Toy
void roll_pitch_toy()
{
#if TOY_MIXER == TOY_LOOKUP_TABLE || TOY_MIXER == TOY_LINEAR_MIXER
    int16_t yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;

    if(g.rc_1.control_in != 0) {    // roll
        get_acro_yaw(yaw_rate/2);
        ap_system.yaw_stopped = false;
        yaw_timer = 150;

    }else if (!ap_system.yaw_stopped) {
        get_acro_yaw(0);
        yaw_timer--;

        if((yaw_timer == 0) || (fabsf(omega.z) < 0.17f)) {
            ap_system.yaw_stopped = true;
            nav_yaw = ahrs.yaw_sensor;
        }
    }else{
        if(motors.armed() == false || g.rc_3.control_in == 0)
            nav_yaw = ahrs.yaw_sensor;

        get_stabilize_yaw(nav_yaw);
    }
#endif

    // roll_rate is the outcome of the linear equation or lookup table
    // based on speed and Yaw rate
    int16_t roll_rate = 0;

#if TOY_MIXER == TOY_LOOKUP_TABLE
    uint8_t xx, yy;
    // Lookup value
    //xx	= g_gps->ground_speed / 200;
    xx      = abs(g.rc_2.control_in / 1000);
    yy      = abs(yaw_rate / 500);

    // constrain to lookup Array range
    xx = constrain_int16(xx, 0, 3);
    yy = constrain_int16(yy, 0, 8);

    roll_rate = toy_lookup[yy * 4 + xx];

    if(yaw_rate == 0) {
        roll_rate = 0;
    }else if(yaw_rate < 0) {
        roll_rate = -roll_rate;
    }

    int16_t roll_limit = 4500 / g.toy_yaw_rate;
    roll_rate = constrain_int16(roll_rate, -roll_limit, roll_limit);

#elif TOY_MIXER == TOY_LINEAR_MIXER
    roll_rate = -((int32_t)g.rc_2.control_in * (yaw_rate/100)) /30;
    //cliSerial->printf("roll_rate: %d\n",roll_rate);
    roll_rate = constrain_int32(roll_rate, -2000, 2000);

#elif TOY_MIXER == TOY_EXTERNAL_MIXER
    // JKR update to allow external roll/yaw mixing
    roll_rate = g.rc_1.control_in;
#endif

#if TOY_EDF == ENABLED
    // Output the attitude
    //g.rc_1.servo_out = get_stabilize_roll(roll_rate);
    //g.rc_2.servo_out = get_stabilize_pitch(g.rc_6.control_in);             // use CH6 to trim pitch
    get_stabilize_roll(roll_rate);
    get_stabilize_pitch(g.rc_6.control_in);             // use CH6 to trim pitch

#else
    // Output the attitude
    //g.rc_1.servo_out = get_stabilize_roll(roll_rate);
    //g.rc_2.servo_out = get_stabilize_pitch(g.rc_2.control_in);
    get_stabilize_roll(roll_rate);
    get_stabilize_pitch(g.rc_2.control_in);
#endif

}
