/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "FixedwingAttitudeControl.hpp"


#include <stdlib.h>

#include <vtol_att_control/vtol_type.h>

using namespace time_literals;
using math::constrain;
using math::gradual;
using math::radians;

typedef std::vector<RowVector*> data;

FixedwingAttitudeControl::FixedwingAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::attitude_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_fw) : ORB_ID(actuator_controls_0)),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// check if VTOL first
	if (vtol) {
		int32_t vt_type = -1;

		if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
			_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
		}
	}

	/* fetch initial parameter values */
	parameters_update();

	// set initial maximum body rate setpoints
	_roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
	_pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
	_pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
	_yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	return true;
}

int
FixedwingAttitudeControl::parameters_update()
{
	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_param_fw_p_tc.get());
	_pitch_ctrl.set_k_p(_param_fw_pr_p.get());
	_pitch_ctrl.set_k_i(_param_fw_pr_i.get());
	_pitch_ctrl.set_k_ff(_param_fw_pr_ff.get());
	_pitch_ctrl.set_integrator_max(_param_fw_pr_imax.get());

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_param_fw_r_tc.get());
	_roll_ctrl.set_k_p(_param_fw_rr_p.get());
	_roll_ctrl.set_k_i(_param_fw_rr_i.get());
	_roll_ctrl.set_k_ff(_param_fw_rr_ff.get());
	_roll_ctrl.set_integrator_max(_param_fw_rr_imax.get());

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_param_fw_yr_p.get());
	_yaw_ctrl.set_k_i(_param_fw_yr_i.get());
	_yaw_ctrl.set_k_ff(_param_fw_yr_ff.get());
	_yaw_ctrl.set_integrator_max(_param_fw_yr_imax.get());

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_param_fw_wr_p.get());
	_wheel_ctrl.set_k_i(_param_fw_wr_i.get());
	_wheel_ctrl.set_k_ff(_param_fw_wr_ff.get());
	_wheel_ctrl.set_integrator_max(_param_fw_wr_imax.get());
	_wheel_ctrl.set_max_rate(radians(_param_fw_w_rmax.get()));

	return PX4_OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	if (_vehicle_status.is_vtol) {
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;
		const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

		if (is_hovering || is_tailsitter_transition) {
			_vcontrol_mode.flag_control_attitude_enabled = false;
			_vcontrol_mode.flag_control_manual_enabled = false;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	const bool is_tailsitter_transition = _is_tailsitter && _vehicle_status.in_transition_mode;
	const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	if (_vcontrol_mode.flag_control_manual_enabled && (!is_tailsitter_transition || is_fixed_wing)) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			// Check if we are in rattitude mode and the pilot is above the threshold on pitch
			if (_vcontrol_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_setpoint.y) > _param_fw_ratt_th.get()
				    || fabsf(_manual_control_setpoint.x) > _param_fw_ratt_th.get()) {
					_vcontrol_mode.flag_control_attitude_enabled = false;
				}
			}

			if (!_vcontrol_mode.flag_control_climb_rate_enabled &&
			    !_vcontrol_mode.flag_control_offboard_enabled) {

				if (_vcontrol_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs

					_att_sp.roll_body = _manual_control_setpoint.y * radians(_param_fw_man_r_max.get()) + radians(_param_fw_rsp_off.get());
					_att_sp.roll_body = constrain(_att_sp.roll_body,
								      -radians(_param_fw_man_r_max.get()), radians(_param_fw_man_r_max.get()));

					_att_sp.pitch_body = -_manual_control_setpoint.x * radians(_param_fw_man_p_max.get())
							     + radians(_param_fw_psp_off.get());
					_att_sp.pitch_body = constrain(_att_sp.pitch_body,
								       -radians(_param_fw_man_p_max.get()), radians(_param_fw_man_p_max.get()));

					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust_body[0] = _manual_control_setpoint.z;

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);

					_att_sp.timestamp = hrt_absolute_time();

					_attitude_sp_pub.publish(_att_sp);

				} else if (_vcontrol_mode.flag_control_rates_enabled &&
					   !_vcontrol_mode.flag_control_attitude_enabled) {

					// RATE mode we need to generate the rate setpoint from manual user inputs
					_rates_sp.timestamp = hrt_absolute_time();
					_rates_sp.roll = _manual_control_setpoint.y * radians(_param_fw_acro_x_max.get());
					_rates_sp.pitch = -_manual_control_setpoint.x * radians(_param_fw_acro_y_max.get());
					_rates_sp.yaw = _manual_control_setpoint.r * radians(_param_fw_acro_z_max.get());
					_rates_sp.thrust_body[0] = _manual_control_setpoint.z;

					_rate_sp_pub.publish(_rates_sp);

				} else {
					/* manual/direct control */
					_actuators.control[actuator_controls_s::INDEX_ROLL] =
						_manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
					_actuators.control[actuator_controls_s::INDEX_PITCH] =
						-_manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();
					_actuators.control[actuator_controls_s::INDEX_YAW] =
						_manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual_control_setpoint.z;
				}
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
FixedwingAttitudeControl::vehicle_rates_setpoint_poll()
{
	if (_rates_sp_sub.update(&_rates_sp)) {
		if (_is_tailsitter) {
			float tmp = _rates_sp.roll;
			_rates_sp.roll = -_rates_sp.yaw;
			_rates_sp.yaw = tmp;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingAttitudeControl::get_airspeed_and_update_scaling()
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().equivalent_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if ((_param_fw_arsp_mode.get() == 0) && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().equivalent_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the minimum airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_min.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	const float airspeed_constrained = constrain(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_max.get());

	_airspeed_scaling = (_param_fw_arsp_scale_en.get()) ? (_param_fw_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed;
}

void FixedwingAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

    if (!_creada){
        while(NeuralNetwork({  2,10,20,50,10,1 },0.0000001f)){};
        _creada = true;
        weights[0]->coeffRef(0, 0) = 0.680375;
        weights[0]->coeffRef(1, 0) = -0.211234;
        weights[0]->coeffRef(2, 0) = 0.566634;
        weights[0]->coeffRef(0, 1) = 0.59688;
        weights[0]->coeffRef(1, 1) = 0.823295;
        weights[0]->coeffRef(2, 1) = -0.606359;
        weights[0]->coeffRef(0, 2) = -0.329554;
        weights[0]->coeffRef(1, 2) = 0.536459;
        weights[0]->coeffRef(2, 2) = -0.518802;
        weights[0]->coeffRef(0, 3) = 0.10794;
        weights[0]->coeffRef(1, 3) = -0.0452059;
        weights[0]->coeffRef(2, 3) = 0.279063;
        weights[0]->coeffRef(0, 4) = -0.270431;
        weights[0]->coeffRef(1, 4) = 0.0268018;
        weights[0]->coeffRef(2, 4) = 0.902427;
        weights[0]->coeffRef(0, 5) = 0.83239;
        weights[0]->coeffRef(1, 5) = 0.271423;
        weights[0]->coeffRef(2, 5) = 0.426643;
        weights[0]->coeffRef(0, 6) = -0.716795;
        weights[0]->coeffRef(1, 6) = 0.213938;
        weights[0]->coeffRef(2, 6) = -1.01522;
        weights[0]->coeffRef(0, 7) = -0.514226;
        weights[0]->coeffRef(1, 7) = -0.725537;
        weights[0]->coeffRef(2, 7) = 0.53194;
        weights[0]->coeffRef(0, 8) = -0.686642;
        weights[0]->coeffRef(1, 8) = -0.198111;
        weights[0]->coeffRef(2, 8) = -0.673496;
        weights[0]->coeffRef(0, 9) = -0.782382;
        weights[0]->coeffRef(1, 9) = 0.997849;
        weights[0]->coeffRef(2, 9) = -0.524439;
        weights[1]->coeffRef(0, 0) = -0.411595;
        weights[1]->coeffRef(1, 0) = 0.279425;
        weights[1]->coeffRef(2, 0) = 0.0491589;
        weights[1]->coeffRef(3, 0) = -0.0162801;
        weights[1]->coeffRef(4, 0) = 0.928797;
        weights[1]->coeffRef(5, 0) = -0.420787;
        weights[1]->coeffRef(6, 0) = 0.553283;
        weights[1]->coeffRef(7, 0) = 0.0491052;
        weights[1]->coeffRef(8, 0) = 0.550246;
        weights[1]->coeffRef(9, 0) = -0.205219;
        weights[1]->coeffRef(10, 0) = 0.766311;
        weights[1]->coeffRef(0, 1) = -0.449921;
        weights[1]->coeffRef(1, 1) = -0.287758;
        weights[1]->coeffRef(2, 1) = 0.620296;
        weights[1]->coeffRef(3, 1) = 0.827261;
        weights[1]->coeffRef(4, 1) = -0.905744;
        weights[1]->coeffRef(5, 1) = 0.874803;
        weights[1]->coeffRef(6, 1) = 0.0885463;
        weights[1]->coeffRef(7, 1) = -0.836061;
        weights[1]->coeffRef(8, 1) = -0.579935;
        weights[1]->coeffRef(9, 1) = 0.31765;
        weights[1]->coeffRef(10, 1) = 0.732807;
        weights[1]->coeffRef(0, 2) = -0.291686;
        weights[1]->coeffRef(1, 2) = -0.871205;
        weights[1]->coeffRef(2, 2) = -0.963684;
        weights[1]->coeffRef(3, 2) = -0.0796021;
        weights[1]->coeffRef(4, 2) = -0.856858;
        weights[1]->coeffRef(5, 2) = -0.509018;
        weights[1]->coeffRef(6, 2) = 0.922391;
        weights[1]->coeffRef(7, 2) = 0.804811;
        weights[1]->coeffRef(8, 2) = 0.683417;
        weights[1]->coeffRef(9, 2) = -0.467342;
        weights[1]->coeffRef(10, 2) = 0.0989607;
        weights[1]->coeffRef(0, 3) = -0.245753;
        weights[1]->coeffRef(1, 3) = 0.525232;
        weights[1]->coeffRef(2, 3) = 0.0231465;
        weights[1]->coeffRef(3, 3) = 0.335595;
        weights[1]->coeffRef(4, 3) = 0.0588376;
        weights[1]->coeffRef(5, 3) = -0.916853;
        weights[1]->coeffRef(6, 3) = -0.127695;
        weights[1]->coeffRef(7, 3) = 0.859521;
        weights[1]->coeffRef(8, 3) = 0.858691;
        weights[1]->coeffRef(9, 3) = 0.436082;
        weights[1]->coeffRef(10, 3) = -0.434027;
        weights[1]->coeffRef(0, 4) = 0.457911;
        weights[1]->coeffRef(1, 4) = 0.290855;
        weights[1]->coeffRef(2, 4) = -0.286485;
        weights[1]->coeffRef(3, 4) = 0.362395;
        weights[1]->coeffRef(4, 4) = -0.725658;
        weights[1]->coeffRef(5, 4) = -0.147644;
        weights[1]->coeffRef(6, 4) = 0.804403;
        weights[1]->coeffRef(7, 4) = 0.646703;
        weights[1]->coeffRef(8, 4) = -0.296286;
        weights[1]->coeffRef(9, 4) = -0.554901;
        weights[1]->coeffRef(10, 4) = 0.726786;
        weights[1]->coeffRef(0, 5) = -0.302402;
        weights[1]->coeffRef(1, 5) = 0.375035;
        weights[1]->coeffRef(2, 5) = 0.913616;
        weights[1]->coeffRef(3, 5) = 0.175037;
        weights[1]->coeffRef(4, 5) = 0.304691;
        weights[1]->coeffRef(5, 5) = 0.712609;
        weights[1]->coeffRef(6, 5) = -0.113487;
        weights[1]->coeffRef(7, 5) = 0.846051;
        weights[1]->coeffRef(8, 5) = -0.195755;
        weights[1]->coeffRef(9, 5) = 0.626937;
        weights[1]->coeffRef(10, 5) = 0.358164;
        weights[1]->coeffRef(0, 6) = 0.832005;
        weights[1]->coeffRef(1, 6) = -0.037627;
        weights[1]->coeffRef(2, 6) = -0.571733;
        weights[1]->coeffRef(3, 6) = 0.906294;
        weights[1]->coeffRef(4, 6) = 0.862815;
        weights[1]->coeffRef(5, 6) = -0.690658;
        weights[1]->coeffRef(6, 6) = 0.74172;
        weights[1]->coeffRef(7, 6) = 0.285339;
        weights[1]->coeffRef(8, 6) = -0.155821;
        weights[1]->coeffRef(9, 6) = 0.241703;
        weights[1]->coeffRef(10, 6) = -0.413477;
        weights[1]->coeffRef(0, 7) = 0.569293;
        weights[1]->coeffRef(1, 7) = -0.383351;
        weights[1]->coeffRef(2, 7) = -0.105156;
        weights[1]->coeffRef(3, 7) = -0.549726;
        weights[1]->coeffRef(4, 7) = -0.633414;
        weights[1]->coeffRef(5, 7) = -0.451465;
        weights[1]->coeffRef(6, 7) = 0.119273;
        weights[1]->coeffRef(7, 7) = -0.168823;
        weights[1]->coeffRef(8, 7) = -0.654604;
        weights[1]->coeffRef(9, 7) = 0.811654;
        weights[1]->coeffRef(10, 7) = -0.802438;
        weights[1]->coeffRef(0, 8) = -0.756292;
        weights[1]->coeffRef(1, 8) = -0.00242442;
        weights[1]->coeffRef(2, 8) = 0.522943;
        weights[1]->coeffRef(3, 8) = 0.962862;
        weights[1]->coeffRef(4, 8) = 0.839728;
        weights[1]->coeffRef(5, 8) = 0.356176;
        weights[1]->coeffRef(6, 8) = -0.21232;
        weights[1]->coeffRef(7, 8) = 0.492571;
        weights[1]->coeffRef(8, 8) = -0.241824;
        weights[1]->coeffRef(9, 8) = -0.420097;
        weights[1]->coeffRef(10, 8) = -0.566409;
        weights[1]->coeffRef(0, 9) = 0.176195;
        weights[1]->coeffRef(1, 9) = -0.517979;
        weights[1]->coeffRef(2, 9) = -0.696787;
        weights[1]->coeffRef(3, 9) = 0.470371;
        weights[1]->coeffRef(4, 9) = -0.720647;
        weights[1]->coeffRef(5, 9) = 0.597969;
        weights[1]->coeffRef(6, 9) = -0.690939;
        weights[1]->coeffRef(7, 9) = 0.497122;
        weights[1]->coeffRef(8, 9) = -0.86967;
        weights[1]->coeffRef(9, 9) = 0.908737;
        weights[1]->coeffRef(10, 9) = -0.866186;
        weights[1]->coeffRef(0, 10) = 0.0411453;
        weights[1]->coeffRef(1, 10) = -0.644286;
        weights[1]->coeffRef(2, 10) = -0.519495;
        weights[1]->coeffRef(3, 10) = 0.59347;
        weights[1]->coeffRef(4, 10) = 0.454655;
        weights[1]->coeffRef(5, 10) = 0.310023;
        weights[1]->coeffRef(6, 10) = 0.94113;
        weights[1]->coeffRef(7, 10) = 0.275698;
        weights[1]->coeffRef(8, 10) = 0.525535;
        weights[1]->coeffRef(9, 10) = -0.816754;
        weights[1]->coeffRef(10, 10) = -0.740702;
        weights[1]->coeffRef(0, 11) = 0.0331766;
        weights[1]->coeffRef(1, 11) = -0.842033;
        weights[1]->coeffRef(2, 11) = -0.857904;
        weights[1]->coeffRef(3, 11) = -0.594816;
        weights[1]->coeffRef(4, 11) = -0.0932079;
        weights[1]->coeffRef(5, 11) = 0.629112;
        weights[1]->coeffRef(6, 11) = 0.161243;
        weights[1]->coeffRef(7, 11) = 0.509171;
        weights[1]->coeffRef(8, 11) = -0.881831;
        weights[1]->coeffRef(9, 11) = -0.686306;
        weights[1]->coeffRef(10, 11) = 0.982577;
        weights[1]->coeffRef(0, 12) = -0.588142;
        weights[1]->coeffRef(1, 12) = 0.776104;
        weights[1]->coeffRef(2, 12) = -0.749349;
        weights[1]->coeffRef(3, 12) = 0.998726;
        weights[1]->coeffRef(4, 12) = -0.876435;
        weights[1]->coeffRef(5, 12) = 0.746395;
        weights[1]->coeffRef(6, 12) = -0.864863;
        weights[1]->coeffRef(7, 12) = -0.987754;
        weights[1]->coeffRef(8, 12) = 0.836566;
        weights[1]->coeffRef(9, 12) = 0.193406;
        weights[1]->coeffRef(10, 12) = -0.623859;
        weights[1]->coeffRef(0, 13) = -0.666765;
        weights[1]->coeffRef(1, 13) = -0.220093;
        weights[1]->coeffRef(2, 13) = 0.824115;
        weights[1]->coeffRef(3, 13) = 0.644106;
        weights[1]->coeffRef(4, 13) = -0.261656;
        weights[1]->coeffRef(5, 13) = 0.115135;
        weights[1]->coeffRef(6, 13) = 0.143059;
        weights[1]->coeffRef(7, 13) = -0.0910267;
        weights[1]->coeffRef(8, 13) = 0.359325;
        weights[1]->coeffRef(9, 13) = -0.796376;
        weights[1]->coeffRef(10, 13) = 0.0826979;
        weights[1]->coeffRef(0, 14) = 0.521388;
        weights[1]->coeffRef(1, 14) = -0.395928;
        weights[1]->coeffRef(2, 14) = 0.982397;
        weights[1]->coeffRef(3, 14) = 0.158805;
        weights[1]->coeffRef(4, 14) = 0.776441;
        weights[1]->coeffRef(5, 14) = 0.505376;
        weights[1]->coeffRef(6, 14) = 0.241816;
        weights[1]->coeffRef(7, 14) = -0.924456;
        weights[1]->coeffRef(8, 14) = 0.48023;
        weights[1]->coeffRef(9, 14) = 0.671209;
        weights[1]->coeffRef(10, 14) = 0.872722;
        weights[1]->coeffRef(0, 15) = 0.741844;
        weights[1]->coeffRef(1, 15) = 0.661113;
        weights[1]->coeffRef(2, 15) = 0.960417;
        weights[1]->coeffRef(3, 15) = 0.485572;
        weights[1]->coeffRef(4, 15) = 0.800224;
        weights[1]->coeffRef(5, 15) = 0.960657;
        weights[1]->coeffRef(6, 15) = 0.341777;
        weights[1]->coeffRef(7, 15) = -0.00500109;
        weights[1]->coeffRef(8, 15) = -0.664058;
        weights[1]->coeffRef(9, 15) = 0.66066;
        weights[1]->coeffRef(10, 15) = 0.770177;
        weights[1]->coeffRef(0, 16) = -0.844006;
        weights[1]->coeffRef(1, 16) = 0.294623;
        weights[1]->coeffRef(2, 16) = -0.503792;
        weights[1]->coeffRef(3, 16) = 0.2618;
        weights[1]->coeffRef(4, 16) = -0.526477;
        weights[1]->coeffRef(5, 16) = 0.404925;
        weights[1]->coeffRef(6, 16) = -0.374291;
        weights[1]->coeffRef(7, 16) = -0.337729;
        weights[1]->coeffRef(8, 16) = -0.545137;
        weights[1]->coeffRef(9, 16) = -0.845211;
        weights[1]->coeffRef(10, 16) = 0.280906;
        weights[1]->coeffRef(0, 17) = -0.546073;
        weights[1]->coeffRef(1, 17) = 0.29824;
        weights[1]->coeffRef(2, 17) = 0.019574;
        weights[1]->coeffRef(3, 17) = 0.947645;
        weights[1]->coeffRef(4, 17) = -0.419289;
        weights[1]->coeffRef(5, 17) = 0.101909;
        weights[1]->coeffRef(6, 17) = 0.422987;
        weights[1]->coeffRef(7, 17) = -0.769143;
        weights[1]->coeffRef(8, 17) = -0.0721897;
        weights[1]->coeffRef(9, 17) = 0.189946;
        weights[1]->coeffRef(10, 17) = 0.910019;
        weights[1]->coeffRef(0, 18) = -0.0984855;
        weights[1]->coeffRef(1, 18) = -0.324169;
        weights[1]->coeffRef(2, 18) = 0.695041;
        weights[1]->coeffRef(3, 18) = -0.132299;
        weights[1]->coeffRef(4, 18) = -1.00148;
        weights[1]->coeffRef(5, 18) = -0.311063;
        weights[1]->coeffRef(6, 18) = 0.200282;
        weights[1]->coeffRef(7, 18) = 0.663508;
        weights[1]->coeffRef(8, 18) = -0.528951;
        weights[1]->coeffRef(9, 18) = 0.346987;
        weights[1]->coeffRef(10, 18) = -0.0415227;
        weights[1]->coeffRef(0, 19) = -0.0414836;
        weights[1]->coeffRef(1, 19) = -0.381197;
        weights[1]->coeffRef(2, 19) = 0.424718;
        weights[1]->coeffRef(3, 19) = -0.640868;
        weights[1]->coeffRef(4, 19) = 0.213217;
        weights[1]->coeffRef(5, 19) = -0.927157;
        weights[1]->coeffRef(6, 19) = -0.154379;
        weights[1]->coeffRef(7, 19) = 0.383138;
        weights[1]->coeffRef(8, 19) = 0.365233;
        weights[1]->coeffRef(9, 19) = 0.263853;
        weights[1]->coeffRef(10, 19) = -0.33571;
        weights[2]->coeffRef(0, 0) = 0.0669815;
        weights[2]->coeffRef(1, 0) = -0.82151;
        weights[2]->coeffRef(2, 0) = -0.48725;
        weights[2]->coeffRef(3, 0) = 0.750155;
        weights[2]->coeffRef(4, 0) = 0.372495;
        weights[2]->coeffRef(5, 0) = -0.807045;
        weights[2]->coeffRef(6, 0) = -0.776751;
        weights[2]->coeffRef(7, 0) = -0.280466;
        weights[2]->coeffRef(8, 0) = 0.15665;
        weights[2]->coeffRef(9, 0) = 0.187717;
        weights[2]->coeffRef(10, 0) = 0.329056;
        weights[2]->coeffRef(11, 0) = -0.415119;
        weights[2]->coeffRef(12, 0) = 0.546117;
        weights[2]->coeffRef(13, 0) = -0.426751;
        weights[2]->coeffRef(14, 0) = -0.335112;
        weights[2]->coeffRef(15, 0) = -0.610268;
        weights[2]->coeffRef(16, 0) = 0.969062;
        weights[2]->coeffRef(17, 0) = -0.992542;
        weights[2]->coeffRef(18, 0) = 0.651819;
        weights[2]->coeffRef(19, 0) = -0.339289;
        weights[2]->coeffRef(20, 0) = -0.619568;
        weights[2]->coeffRef(0, 1) = -0.128668;
        weights[2]->coeffRef(1, 1) = 0.915098;
        weights[2]->coeffRef(2, 1) = 0.843574;
        weights[2]->coeffRef(3, 1) = 0.532888;
        weights[2]->coeffRef(4, 1) = 0.397997;
        weights[2]->coeffRef(5, 1) = -0.761687;
        weights[2]->coeffRef(6, 1) = 0.370967;
        weights[2]->coeffRef(7, 1) = -0.229683;
        weights[2]->coeffRef(8, 1) = 0.546161;
        weights[2]->coeffRef(9, 1) = 0.885249;
        weights[2]->coeffRef(10, 1) = 0.835377;
        weights[2]->coeffRef(11, 1) = 0.718572;
        weights[2]->coeffRef(12, 1) = -0.58878;
        weights[2]->coeffRef(13, 1) = 0.589818;
        weights[2]->coeffRef(14, 1) = 0.0920131;
        weights[2]->coeffRef(15, 1) = -0.412673;
        weights[2]->coeffRef(16, 1) = 0.809786;
        weights[2]->coeffRef(17, 1) = 0.81911;
        weights[2]->coeffRef(18, 1) = 0.750023;
        weights[2]->coeffRef(19, 1) = -0.00228189;
        weights[2]->coeffRef(20, 1) = 0.149476;
        weights[2]->coeffRef(0, 2) = -0.673755;
        weights[2]->coeffRef(1, 2) = -0.450719;
        weights[2]->coeffRef(2, 2) = 0.725443;
        weights[2]->coeffRef(3, 2) = -0.0174003;
        weights[2]->coeffRef(4, 2) = -0.072516;
        weights[2]->coeffRef(5, 2) = 0.70016;
        weights[2]->coeffRef(6, 2) = -0.00782872;
        weights[2]->coeffRef(7, 2) = -0.419498;
        weights[2]->coeffRef(8, 2) = -0.637776;
        weights[2]->coeffRef(9, 2) = 0.369025;
        weights[2]->coeffRef(10, 2) = 0.453319;
        weights[2]->coeffRef(11, 2) = -0.718608;
        weights[2]->coeffRef(12, 2) = 0.204098;
        weights[2]->coeffRef(13, 2) = -0.016689;
        weights[2]->coeffRef(14, 2) = 0.678587;
        weights[2]->coeffRef(15, 2) = 0.452966;
        weights[2]->coeffRef(16, 2) = -0.643227;
        weights[2]->coeffRef(17, 2) = -0.55594;
        weights[2]->coeffRef(18, 2) = -0.00432541;
        weights[2]->coeffRef(19, 2) = -0.758664;
        weights[2]->coeffRef(20, 2) = -0.7218;
        weights[2]->coeffRef(0, 3) = -0.278205;
        weights[2]->coeffRef(1, 3) = -0.347876;
        weights[2]->coeffRef(2, 3) = 0.857501;
        weights[2]->coeffRef(3, 3) = 0.813098;
        weights[2]->coeffRef(4, 3) = 0.244521;
        weights[2]->coeffRef(5, 3) = 0.67721;
        weights[2]->coeffRef(6, 3) = 0.63646;
        weights[2]->coeffRef(7, 3) = -0.0104625;
        weights[2]->coeffRef(8, 3) = -0.327871;
        weights[2]->coeffRef(9, 3) = -0.210108;
        weights[2]->coeffRef(10, 3) = 0.314706;
        weights[2]->coeffRef(11, 3) = 0.223193;
        weights[2]->coeffRef(12, 3) = -0.485259;
        weights[2]->coeffRef(13, 3) = -0.70005;
        weights[2]->coeffRef(14, 3) = -0.851294;
        weights[2]->coeffRef(15, 3) = -0.776983;
        weights[2]->coeffRef(16, 3) = 0.295316;
        weights[2]->coeffRef(17, 3) = -0.272573;
        weights[2]->coeffRef(18, 3) = -0.425827;
        weights[2]->coeffRef(19, 3) = -0.339466;
        weights[2]->coeffRef(20, 3) = -0.81495;
        weights[2]->coeffRef(0, 4) = -0.148056;
        weights[2]->coeffRef(1, 4) = 0.864909;
        weights[2]->coeffRef(2, 4) = 0.177695;
        weights[2]->coeffRef(3, 4) = -0.46314;
        weights[2]->coeffRef(4, 4) = 0.317163;
        weights[2]->coeffRef(5, 4) = 0.516528;
        weights[2]->coeffRef(6, 4) = -0.0260695;
        weights[2]->coeffRef(7, 4) = -0.680711;
        weights[2]->coeffRef(8, 4) = 0.761856;
        weights[2]->coeffRef(9, 4) = 0.249656;
        weights[2]->coeffRef(10, 4) = 0.0406117;
        weights[2]->coeffRef(11, 4) = -0.593809;
        weights[2]->coeffRef(12, 4) = 0.122131;
        weights[2]->coeffRef(13, 4) = -0.143086;
        weights[2]->coeffRef(14, 4) = 0.65269;
        weights[2]->coeffRef(15, 4) = -0.22435;
        weights[2]->coeffRef(16, 4) = -0.511783;
        weights[2]->coeffRef(17, 4) = -0.348332;
        weights[2]->coeffRef(18, 4) = 0.462547;
        weights[2]->coeffRef(19, 4) = 0.280192;
        weights[2]->coeffRef(20, 4) = 0.964492;
        weights[2]->coeffRef(0, 5) = -0.320584;
        weights[2]->coeffRef(1, 5) = 0.798998;
        weights[2]->coeffRef(2, 5) = -0.737987;
        weights[2]->coeffRef(3, 5) = -0.183982;
        weights[2]->coeffRef(4, 5) = -0.988921;
        weights[2]->coeffRef(5, 5) = 0.573605;
        weights[2]->coeffRef(6, 5) = 0.549823;
        weights[2]->coeffRef(7, 5) = -0.417314;
        weights[2]->coeffRef(8, 5) = -0.766455;
        weights[2]->coeffRef(9, 5) = 0.732576;
        weights[2]->coeffRef(10, 5) = 0.436977;
        weights[2]->coeffRef(11, 5) = -0.892421;
        weights[2]->coeffRef(12, 5) = -0.109044;
        weights[2]->coeffRef(13, 5) = 0.968508;
        weights[2]->coeffRef(14, 5) = 0.42303;
        weights[2]->coeffRef(15, 5) = -0.565391;
        weights[2]->coeffRef(16, 5) = -0.0520812;
        weights[2]->coeffRef(17, 5) = 0.730692;
        weights[2]->coeffRef(18, 5) = -0.815807;
        weights[2]->coeffRef(19, 5) = -0.803417;
        weights[2]->coeffRef(20, 5) = -0.229051;
        weights[2]->coeffRef(0, 6) = -0.392926;
        weights[2]->coeffRef(1, 6) = 0.319226;
        weights[2]->coeffRef(2, 6) = 0.605218;
        weights[2]->coeffRef(3, 6) = -0.743802;
        weights[2]->coeffRef(4, 6) = -0.896611;
        weights[2]->coeffRef(5, 6) = -0.884326;
        weights[2]->coeffRef(6, 6) = -0.0833229;
        weights[2]->coeffRef(7, 6) = 0.555825;
        weights[2]->coeffRef(8, 6) = 0.389444;
        weights[2]->coeffRef(9, 6) = -0.112888;
        weights[2]->coeffRef(10, 6) = -0.768186;
        weights[2]->coeffRef(11, 6) = 0.191044;
        weights[2]->coeffRef(12, 6) = 0.148302;
        weights[2]->coeffRef(13, 6) = 0.0541877;
        weights[2]->coeffRef(14, 6) = 0.199128;
        weights[2]->coeffRef(15, 6) = -0.259875;
        weights[2]->coeffRef(16, 6) = -0.391082;
        weights[2]->coeffRef(17, 6) = 0.777873;
        weights[2]->coeffRef(18, 6) = -0.0515171;
        weights[2]->coeffRef(19, 6) = -0.66375;
        weights[2]->coeffRef(20, 6) = 0.225959;
        weights[2]->coeffRef(0, 7) = 0.0484466;
        weights[2]->coeffRef(1, 7) = 0.231916;
        weights[2]->coeffRef(2, 7) = 0.207548;
        weights[2]->coeffRef(3, 7) = -0.523716;
        weights[2]->coeffRef(4, 7) = 0.65896;
        weights[2]->coeffRef(5, 7) = -0.869143;
        weights[2]->coeffRef(6, 7) = -0.803254;
        weights[2]->coeffRef(7, 7) = 0.85406;
        weights[2]->coeffRef(8, 7) = -0.666391;
        weights[2]->coeffRef(9, 7) = -0.0392534;
        weights[2]->coeffRef(10, 7) = -0.54175;
        weights[2]->coeffRef(11, 7) = 0.640061;
        weights[2]->coeffRef(12, 7) = -0.409559;
        weights[2]->coeffRef(13, 7) = -0.279332;
        weights[2]->coeffRef(14, 7) = 0.747042;
        weights[2]->coeffRef(15, 7) = -0.329759;
        weights[2]->coeffRef(16, 7) = 0.628387;
        weights[2]->coeffRef(17, 7) = 0.317786;
        weights[2]->coeffRef(18, 7) = -0.921708;
        weights[2]->coeffRef(19, 7) = -0.480273;
        weights[2]->coeffRef(20, 7) = 0.54943;
        weights[2]->coeffRef(0, 8) = 0.251268;
        weights[2]->coeffRef(1, 8) = 0.670089;
        weights[2]->coeffRef(2, 8) = -0.378392;
        weights[2]->coeffRef(3, 8) = -0.554652;
        weights[2]->coeffRef(4, 8) = -0.604265;
        weights[2]->coeffRef(5, 8) = 0.22199;
        weights[2]->coeffRef(6, 8) = -0.780662;
        weights[2]->coeffRef(7, 8) = 0.351396;
        weights[2]->coeffRef(8, 8) = 0.562724;
        weights[2]->coeffRef(9, 8) = 0.437832;
        weights[2]->coeffRef(10, 8) = -0.596832;
        weights[2]->coeffRef(11, 8) = -0.202211;
        weights[2]->coeffRef(12, 8) = -0.366247;
        weights[2]->coeffRef(13, 8) = -0.129891;
        weights[2]->coeffRef(14, 8) = -0.540947;
        weights[2]->coeffRef(15, 8) = -0.234597;
        weights[2]->coeffRef(16, 8) = 0.0648209;
        weights[2]->coeffRef(17, 8) = -0.690736;
        weights[2]->coeffRef(18, 8) = 0.112817;
        weights[2]->coeffRef(19, 8) = -0.968868;
        weights[2]->coeffRef(20, 8) = -0.241851;
        weights[2]->coeffRef(0, 9) = -0.232986;
        weights[2]->coeffRef(1, 9) = -0.385229;
        weights[2]->coeffRef(2, 9) = 0.464512;
        weights[2]->coeffRef(3, 9) = -0.484947;
        weights[2]->coeffRef(4, 9) = 0.299663;
        weights[2]->coeffRef(5, 9) = 0.111515;
        weights[2]->coeffRef(6, 9) = 0.840123;
        weights[2]->coeffRef(7, 9) = 0.367269;
        weights[2]->coeffRef(8, 9) = 0.623734;
        weights[2]->coeffRef(9, 9) = 0.397355;
        weights[2]->coeffRef(10, 9) = -0.381141;
        weights[2]->coeffRef(11, 9) = 0.301206;
        weights[2]->coeffRef(12, 9) = -0.994942;
        weights[2]->coeffRef(13, 9) = 0.0614811;
        weights[2]->coeffRef(14, 9) = 0.694857;
        weights[2]->coeffRef(15, 9) = 0.249747;
        weights[2]->coeffRef(16, 9) = 0.285822;
        weights[2]->coeffRef(17, 9) = 0.0373399;
        weights[2]->coeffRef(18, 9) = -0.202362;
        weights[2]->coeffRef(19, 9) = -0.278513;
        weights[2]->coeffRef(20, 9) = 0.442862;
        weights[2]->coeffRef(0, 10) = 0.606002;
        weights[2]->coeffRef(1, 10) = 0.358864;
        weights[2]->coeffRef(2, 10) = -0.702709;
        weights[2]->coeffRef(3, 10) = -0.939023;
        weights[2]->coeffRef(4, 10) = -0.872587;
        weights[2]->coeffRef(5, 10) = 0.377099;
        weights[2]->coeffRef(6, 10) = -0.623984;
        weights[2]->coeffRef(7, 10) = 0.234033;
        weights[2]->coeffRef(8, 10) = 0.404035;
        weights[2]->coeffRef(9, 10) = 0.137034;
        weights[2]->coeffRef(10, 10) = -1.00188;
        weights[2]->coeffRef(11, 10) = -0.980787;
        weights[2]->coeffRef(12, 10) = -0.395261;
        weights[2]->coeffRef(13, 10) = -0.480513;
        weights[2]->coeffRef(14, 10) = 0.316519;
        weights[2]->coeffRef(15, 10) = 0.725677;
        weights[2]->coeffRef(16, 10) = -0.637312;
        weights[2]->coeffRef(17, 10) = -0.317049;
        weights[2]->coeffRef(18, 10) = 0.331562;
        weights[2]->coeffRef(19, 10) = 0.755697;
        weights[2]->coeffRef(20, 10) = 0.310838;
        weights[2]->coeffRef(0, 11) = -0.371078;
        weights[2]->coeffRef(1, 11) = 0.772898;
        weights[2]->coeffRef(2, 11) = -0.635051;
        weights[2]->coeffRef(3, 11) = -0.689752;
        weights[2]->coeffRef(4, 11) = 0.00706864;
        weights[2]->coeffRef(5, 11) = 0.663422;
        weights[2]->coeffRef(6, 11) = 0.35223;
        weights[2]->coeffRef(7, 11) = 0.804759;
        weights[2]->coeffRef(8, 11) = -0.6145;
        weights[2]->coeffRef(9, 11) = -0.209922;
        weights[2]->coeffRef(10, 11) = 0.40832;
        weights[2]->coeffRef(11, 11) = 0.74487;
        weights[2]->coeffRef(12, 11) = 0.088938;
        weights[2]->coeffRef(13, 11) = 0.474537;
        weights[2]->coeffRef(14, 11) = 0.870618;
        weights[2]->coeffRef(15, 11) = -0.523962;
        weights[2]->coeffRef(16, 11) = 0.853003;
        weights[2]->coeffRef(17, 11) = 0.103121;
        weights[2]->coeffRef(18, 11) = 0.864145;
        weights[2]->coeffRef(19, 11) = -0.0128841;
        weights[2]->coeffRef(20, 11) = 0.109128;
        weights[2]->coeffRef(0, 12) = 0.88022;
        weights[2]->coeffRef(1, 12) = 0.603099;
        weights[2]->coeffRef(2, 12) = 0.618531;
        weights[2]->coeffRef(3, 12) = 0.183204;
        weights[2]->coeffRef(4, 12) = 0.314835;
        weights[2]->coeffRef(5, 12) = 0.996595;
        weights[2]->coeffRef(6, 12) = 0.872309;
        weights[2]->coeffRef(7, 12) = -0.355185;
        weights[2]->coeffRef(8, 12) = 0.752289;
        weights[2]->coeffRef(9, 12) = 0.180076;
        weights[2]->coeffRef(10, 12) = 0.270877;
        weights[2]->coeffRef(11, 12) = 0.527369;
        weights[2]->coeffRef(12, 12) = 0.54517;
        weights[2]->coeffRef(13, 12) = 0.585774;
        weights[2]->coeffRef(14, 12) = -0.468317;
        weights[2]->coeffRef(15, 12) = 0.220508;
        weights[2]->coeffRef(16, 12) = -0.0579372;
        weights[2]->coeffRef(17, 12) = -0.66578;
        weights[2]->coeffRef(18, 12) = 0.587336;
        weights[2]->coeffRef(19, 12) = 0.727079;
        weights[2]->coeffRef(20, 12) = 0.750614;
        weights[2]->coeffRef(0, 13) = 0.326688;
        weights[2]->coeffRef(1, 13) = -0.178048;
        weights[2]->coeffRef(2, 13) = 0.231802;
        weights[2]->coeffRef(3, 13) = 0.198157;
        weights[2]->coeffRef(4, 13) = 0.290976;
        weights[2]->coeffRef(5, 13) = 0.0717806;
        weights[2]->coeffRef(6, 13) = -0.704069;
        weights[2]->coeffRef(7, 13) = 0.161617;
        weights[2]->coeffRef(8, 13) = -0.937269;
        weights[2]->coeffRef(9, 13) = 0.400614;
        weights[2]->coeffRef(10, 13) = 0.0401714;
        weights[2]->coeffRef(11, 13) = 0.658098;
        weights[2]->coeffRef(12, 13) = 0.0355128;
        weights[2]->coeffRef(13, 13) = -0.771314;
        weights[2]->coeffRef(14, 13) = -0.025838;
        weights[2]->coeffRef(15, 13) = 0.0108536;
        weights[2]->coeffRef(16, 13) = -0.903217;
        weights[2]->coeffRef(17, 13) = 0.628446;
        weights[2]->coeffRef(18, 13) = -0.227849;
        weights[2]->coeffRef(19, 13) = 0.277367;
        weights[2]->coeffRef(20, 13) = -0.0996833;
        weights[2]->coeffRef(0, 14) = -0.712223;
        weights[2]->coeffRef(1, 14) = -0.174004;
        weights[2]->coeffRef(2, 14) = -0.505549;
        weights[2]->coeffRef(3, 14) = -0.18634;
        weights[2]->coeffRef(4, 14) = -0.965058;
        weights[2]->coeffRef(5, 14) = 0.434852;
        weights[2]->coeffRef(6, 14) = 0.147384;
        weights[2]->coeffRef(7, 14) = 0.626044;
        weights[2]->coeffRef(8, 14) = 0.165196;
        weights[2]->coeffRef(9, 14) = -0.106517;
        weights[2]->coeffRef(10, 14) = -0.0450533;
        weights[2]->coeffRef(11, 14) = 0.99008;
        weights[2]->coeffRef(12, 14) = -0.882207;
        weights[2]->coeffRef(13, 14) = -0.851311;
        weights[2]->coeffRef(14, 14) = 0.281178;
        weights[2]->coeffRef(15, 14) = 0.194031;
        weights[2]->coeffRef(16, 14) = -0.554697;
        weights[2]->coeffRef(17, 14) = -0.560467;
        weights[2]->coeffRef(18, 14) = 0.260574;
        weights[2]->coeffRef(19, 14) = 0.847037;
        weights[2]->coeffRef(20, 14) = 0.475678;
        weights[2]->coeffRef(0, 15) = -0.0770922;
        weights[2]->coeffRef(1, 15) = -0.12547;
        weights[2]->coeffRef(2, 15) = 0.708161;
        weights[2]->coeffRef(3, 15) = 0.908767;
        weights[2]->coeffRef(4, 15) = 0.897796;
        weights[2]->coeffRef(5, 15) = 0.792616;
        weights[2]->coeffRef(6, 15) = 0.532927;
        weights[2]->coeffRef(7, 15) = -0.32942;
        weights[2]->coeffRef(8, 15) = 0.0702321;
        weights[2]->coeffRef(9, 15) = -0.562487;
        weights[2]->coeffRef(10, 15) = -0.0412761;
        weights[2]->coeffRef(11, 15) = 0.893051;
        weights[2]->coeffRef(12, 15) = -0.0614585;
        weights[2]->coeffRef(13, 15) = 0.771881;
        weights[2]->coeffRef(14, 15) = 0.928842;
        weights[2]->coeffRef(15, 15) = -0.641873;
        weights[2]->coeffRef(16, 15) = -0.0832912;
        weights[2]->coeffRef(17, 15) = 0.560232;
        weights[2]->coeffRef(18, 15) = 0.535299;
        weights[2]->coeffRef(19, 15) = 0.810728;
        weights[2]->coeffRef(20, 15) = -0.48875;
        weights[2]->coeffRef(0, 16) = 0.522672;
        weights[2]->coeffRef(1, 16) = 0.92638;
        weights[2]->coeffRef(2, 16) = -0.334679;
        weights[2]->coeffRef(3, 16) = -0.19442;
        weights[2]->coeffRef(4, 16) = 0.121559;
        weights[2]->coeffRef(5, 16) = 0.107687;
        weights[2]->coeffRef(6, 16) = 0.244137;
        weights[2]->coeffRef(7, 16) = -0.617201;
        weights[2]->coeffRef(8, 16) = -0.0447752;
        weights[2]->coeffRef(9, 16) = -0.279985;
        weights[2]->coeffRef(10, 16) = 0.308596;
        weights[2]->coeffRef(11, 16) = 0.831615;
        weights[2]->coeffRef(12, 16) = -0.577353;
        weights[2]->coeffRef(13, 16) = 0.213805;
        weights[2]->coeffRef(14, 16) = 0.729625;
        weights[2]->coeffRef(15, 16) = -0.782557;
        weights[2]->coeffRef(16, 16) = -0.252811;
        weights[2]->coeffRef(17, 16) = -0.602067;
        weights[2]->coeffRef(18, 16) = 0.293589;
        weights[2]->coeffRef(19, 16) = 0.185713;
        weights[2]->coeffRef(20, 16) = 0.352262;
        weights[2]->coeffRef(0, 17) = 0.191896;
        weights[2]->coeffRef(1, 17) = -0.88414;
        weights[2]->coeffRef(2, 17) = 0.126475;
        weights[2]->coeffRef(3, 17) = 0.130139;
        weights[2]->coeffRef(4, 17) = -0.514995;
        weights[2]->coeffRef(5, 17) = -0.964936;
        weights[2]->coeffRef(6, 17) = -0.312536;
        weights[2]->coeffRef(7, 17) = -0.979816;
        weights[2]->coeffRef(8, 17) = 0.845671;
        weights[2]->coeffRef(9, 17) = 0.201933;
        weights[2]->coeffRef(10, 17) = 0.5436;
        weights[2]->coeffRef(11, 17) = 0.770177;
        weights[2]->coeffRef(12, 17) = 0.869052;
        weights[2]->coeffRef(13, 17) = -0.651943;
        weights[2]->coeffRef(14, 17) = -0.106845;
        weights[2]->coeffRef(15, 17) = -0.0301481;
        weights[2]->coeffRef(16, 17) = 0.589848;
        weights[2]->coeffRef(17, 17) = 0.277873;
        weights[2]->coeffRef(18, 17) = 0.933166;
        weights[2]->coeffRef(19, 17) = -0.687695;
        weights[2]->coeffRef(20, 17) = -0.416369;
        weights[2]->coeffRef(0, 18) = 0.763692;
        weights[2]->coeffRef(1, 18) = -0.268493;
        weights[2]->coeffRef(2, 18) = 0.800365;
        weights[2]->coeffRef(3, 18) = 0.495949;
        weights[2]->coeffRef(4, 18) = -0.0483663;
        weights[2]->coeffRef(5, 18) = -0.455324;
        weights[2]->coeffRef(6, 18) = 0.892991;
        weights[2]->coeffRef(7, 18) = -0.754585;
        weights[2]->coeffRef(8, 18) = 0.730608;
        weights[2]->coeffRef(9, 18) = 0.246271;
        weights[2]->coeffRef(10, 18) = 0.438136;
        weights[2]->coeffRef(11, 18) = 0.847656;
        weights[2]->coeffRef(12, 18) = -0.630363;
        weights[2]->coeffRef(13, 18) = -0.434709;
        weights[2]->coeffRef(14, 18) = -0.667008;
        weights[2]->coeffRef(15, 18) = -0.596143;
        weights[2]->coeffRef(16, 18) = 0.252499;
        weights[2]->coeffRef(17, 18) = -0.647576;
        weights[2]->coeffRef(18, 18) = -0.74617;
        weights[2]->coeffRef(19, 18) = -0.544737;
        weights[2]->coeffRef(20, 18) = 0.892958;
        weights[2]->coeffRef(0, 19) = -0.971812;
        weights[2]->coeffRef(1, 19) = -0.677659;
        weights[2]->coeffRef(2, 19) = -0.76184;
        weights[2]->coeffRef(3, 19) = -0.0773628;
        weights[2]->coeffRef(4, 19) = 0.297165;
        weights[2]->coeffRef(5, 19) = 0.831637;
        weights[2]->coeffRef(6, 19) = -0.79812;
        weights[2]->coeffRef(7, 19) = 0.227607;
        weights[2]->coeffRef(8, 19) = -0.858145;
        weights[2]->coeffRef(9, 19) = -0.212196;
        weights[2]->coeffRef(10, 19) = -0.00801665;
        weights[2]->coeffRef(11, 19) = -0.125119;
        weights[2]->coeffRef(12, 19) = -0.414864;
        weights[2]->coeffRef(13, 19) = -0.512648;
        weights[2]->coeffRef(14, 19) = 0.826004;
        weights[2]->coeffRef(15, 19) = 0.13459;
        weights[2]->coeffRef(16, 19) = -0.618474;
        weights[2]->coeffRef(17, 19) = -0.930526;
        weights[2]->coeffRef(18, 19) = -0.137;
        weights[2]->coeffRef(19, 19) = 0.627286;
        weights[2]->coeffRef(20, 19) = 0.507677;
        weights[2]->coeffRef(0, 20) = -0.288313;
        weights[2]->coeffRef(1, 20) = 0.992995;
        weights[2]->coeffRef(2, 20) = -0.921275;
        weights[2]->coeffRef(3, 20) = 0.051663;
        weights[2]->coeffRef(4, 20) = -0.598501;
        weights[2]->coeffRef(5, 20) = 0.3194;
        weights[2]->coeffRef(6, 20) = 0.399324;
        weights[2]->coeffRef(7, 20) = -0.341677;
        weights[2]->coeffRef(8, 20) = 0.776103;
        weights[2]->coeffRef(9, 20) = 0.291961;
        weights[2]->coeffRef(10, 20) = -0.313564;
        weights[2]->coeffRef(11, 20) = -0.906094;
        weights[2]->coeffRef(12, 20) = 0.53704;
        weights[2]->coeffRef(13, 20) = 0.609614;
        weights[2]->coeffRef(14, 20) = 0.39317;
        weights[2]->coeffRef(15, 20) = 0.355227;
        weights[2]->coeffRef(16, 20) = 0.807307;
        weights[2]->coeffRef(17, 20) = -0.374382;
        weights[2]->coeffRef(18, 20) = 0.507756;
        weights[2]->coeffRef(19, 20) = -0.4015;
        weights[2]->coeffRef(20, 20) = 0.615487;
        weights[2]->coeffRef(0, 21) = -0.618201;
        weights[2]->coeffRef(1, 21) = 0.1871;
        weights[2]->coeffRef(2, 21) = -0.905849;
        weights[2]->coeffRef(3, 21) = -0.804053;
        weights[2]->coeffRef(4, 21) = -0.685133;
        weights[2]->coeffRef(5, 21) = -0.502864;
        weights[2]->coeffRef(6, 21) = -0.726351;
        weights[2]->coeffRef(7, 21) = 0.172398;
        weights[2]->coeffRef(8, 21) = -0.878626;
        weights[2]->coeffRef(9, 21) = 0.780986;
        weights[2]->coeffRef(10, 21) = 0.884674;
        weights[2]->coeffRef(11, 21) = -0.876397;
        weights[2]->coeffRef(12, 21) = 0.841363;
        weights[2]->coeffRef(13, 21) = -0.0674464;
        weights[2]->coeffRef(14, 21) = -0.477012;
        weights[2]->coeffRef(15, 21) = 0.190136;
        weights[2]->coeffRef(16, 21) = -0.662173;
        weights[2]->coeffRef(17, 21) = 0.169597;
        weights[2]->coeffRef(18, 21) = -0.051848;
        weights[2]->coeffRef(19, 21) = 0.627923;
        weights[2]->coeffRef(20, 21) = 0.858589;
        weights[2]->coeffRef(0, 22) = 0.0494179;
        weights[2]->coeffRef(1, 22) = 0.1587;
        weights[2]->coeffRef(2, 22) = 0.473748;
        weights[2]->coeffRef(3, 22) = -0.541006;
        weights[2]->coeffRef(4, 22) = -0.472169;
        weights[2]->coeffRef(5, 22) = 0.257408;
        weights[2]->coeffRef(6, 22) = 0.0751434;
        weights[2]->coeffRef(7, 22) = -0.96003;
        weights[2]->coeffRef(8, 22) = 0.857154;
        weights[2]->coeffRef(9, 22) = -0.307359;
        weights[2]->coeffRef(10, 22) = -0.581273;
        weights[2]->coeffRef(11, 22) = 0.0318513;
        weights[2]->coeffRef(12, 22) = -0.188424;
        weights[2]->coeffRef(13, 22) = -0.379317;
        weights[2]->coeffRef(14, 22) = 0.349827;
        weights[2]->coeffRef(15, 22) = 0.27182;
        weights[2]->coeffRef(16, 22) = -0.114143;
        weights[2]->coeffRef(17, 22) = -0.462462;
        weights[2]->coeffRef(18, 22) = 0.411828;
        weights[2]->coeffRef(19, 22) = -0.329968;
        weights[2]->coeffRef(20, 22) = -0.578217;
        weights[2]->coeffRef(0, 23) = 0.516465;
        weights[2]->coeffRef(1, 23) = -0.486307;
        weights[2]->coeffRef(2, 23) = 0.373744;
        weights[2]->coeffRef(3, 23) = -0.964084;
        weights[2]->coeffRef(4, 23) = 0.6901;
        weights[2]->coeffRef(5, 23) = 0.700197;
        weights[2]->coeffRef(6, 23) = 0.200831;
        weights[2]->coeffRef(7, 23) = -0.354025;
        weights[2]->coeffRef(8, 23) = 0.333178;
        weights[2]->coeffRef(9, 23) = 0.0527164;
        weights[2]->coeffRef(10, 23) = 0.699297;
        weights[2]->coeffRef(11, 23) = -0.505534;
        weights[2]->coeffRef(12, 23) = -0.482783;
        weights[2]->coeffRef(13, 23) = -0.850649;
        weights[2]->coeffRef(14, 23) = 0.0240239;
        weights[2]->coeffRef(15, 23) = 0.771244;
        weights[2]->coeffRef(16, 23) = 0.222801;
        weights[2]->coeffRef(17, 23) = 0.0618324;
        weights[2]->coeffRef(18, 23) = 0.645;
        weights[2]->coeffRef(19, 23) = 0.919513;
        weights[2]->coeffRef(20, 23) = 0.470137;
        weights[2]->coeffRef(0, 24) = -0.309739;
        weights[2]->coeffRef(1, 24) = -0.275961;
        weights[2]->coeffRef(2, 24) = -0.922836;
        weights[2]->coeffRef(3, 24) = -0.958495;
        weights[2]->coeffRef(4, 24) = -0.989422;
        weights[2]->coeffRef(5, 24) = -0.0187519;
        weights[2]->coeffRef(6, 24) = -0.413478;
        weights[2]->coeffRef(7, 24) = 0.411816;
        weights[2]->coeffRef(8, 24) = 0.644387;
        weights[2]->coeffRef(9, 24) = 0.0166672;
        weights[2]->coeffRef(10, 24) = -0.0702084;
        weights[2]->coeffRef(11, 24) = -0.833931;
        weights[2]->coeffRef(12, 24) = -0.624527;
        weights[2]->coeffRef(13, 24) = -0.0371763;
        weights[2]->coeffRef(14, 24) = 0.853645;
        weights[2]->coeffRef(15, 24) = -0.900218;
        weights[2]->coeffRef(16, 24) = -0.830351;
        weights[2]->coeffRef(17, 24) = -0.509934;
        weights[2]->coeffRef(18, 24) = 0.418766;
        weights[2]->coeffRef(19, 24) = 0.219257;
        weights[2]->coeffRef(20, 24) = -0.809202;
        weights[2]->coeffRef(0, 25) = 0.922059;
        weights[2]->coeffRef(1, 25) = 0.733452;
        weights[2]->coeffRef(2, 25) = -0.663911;
        weights[2]->coeffRef(3, 25) = -0.045917;
        weights[2]->coeffRef(4, 25) = 0.51444;
        weights[2]->coeffRef(5, 25) = 0.552359;
        weights[2]->coeffRef(6, 25) = -0.986428;
        weights[2]->coeffRef(7, 25) = 0.15903;
        weights[2]->coeffRef(8, 25) = 0.471318;
        weights[2]->coeffRef(9, 25) = 0.486841;
        weights[2]->coeffRef(10, 25) = 0.84706;
        weights[2]->coeffRef(11, 25) = -0.810796;
        weights[2]->coeffRef(12, 25) = 0.578012;
        weights[2]->coeffRef(13, 25) = 0.894568;
        weights[2]->coeffRef(14, 25) = -0.799756;
        weights[2]->coeffRef(15, 25) = -0.455114;
        weights[2]->coeffRef(16, 25) = -0.521478;
        weights[2]->coeffRef(17, 25) = 0.619375;
        weights[2]->coeffRef(18, 25) = -0.808484;
        weights[2]->coeffRef(19, 25) = 0.494491;
        weights[2]->coeffRef(20, 25) = -0.447545;
        weights[2]->coeffRef(0, 26) = -0.653881;
        weights[2]->coeffRef(1, 26) = 0.874619;
        weights[2]->coeffRef(2, 26) = 0.523821;
        weights[2]->coeffRef(3, 26) = -0.805411;
        weights[2]->coeffRef(4, 26) = 0.96213;
        weights[2]->coeffRef(5, 26) = 0.689204;
        weights[2]->coeffRef(6, 26) = -0.317085;
        weights[2]->coeffRef(7, 26) = 0.385872;
        weights[2]->coeffRef(8, 26) = -0.0877944;
        weights[2]->coeffRef(9, 26) = -0.131568;
        weights[2]->coeffRef(10, 26) = 0.30907;
        weights[2]->coeffRef(11, 26) = -0.353953;
        weights[2]->coeffRef(12, 26) = 0.202307;
        weights[2]->coeffRef(13, 26) = -0.739158;
        weights[2]->coeffRef(14, 26) = -0.838839;
        weights[2]->coeffRef(15, 26) = -0.246581;
        weights[2]->coeffRef(16, 26) = -0.726239;
        weights[2]->coeffRef(17, 26) = 0.319695;
        weights[2]->coeffRef(18, 26) = -0.770297;
        weights[2]->coeffRef(19, 26) = 0.761992;
        weights[2]->coeffRef(20, 26) = 0.16388;
        weights[2]->coeffRef(0, 27) = -0.577498;
        weights[2]->coeffRef(1, 27) = 0.337031;
        weights[2]->coeffRef(2, 27) = 0.0566481;
        weights[2]->coeffRef(3, 27) = -0.375709;
        weights[2]->coeffRef(4, 27) = 0.886386;
        weights[2]->coeffRef(5, 27) = 0.537594;
        weights[2]->coeffRef(6, 27) = -0.75549;
        weights[2]->coeffRef(7, 27) = -0.924121;
        weights[2]->coeffRef(8, 27) = 0.030546;
        weights[2]->coeffRef(9, 27) = -0.20139;
        weights[2]->coeffRef(10, 27) = -0.577509;
        weights[2]->coeffRef(11, 27) = -0.0935379;
        weights[2]->coeffRef(12, 27) = -0.681186;
        weights[2]->coeffRef(13, 27) = -0.384111;
        weights[2]->coeffRef(14, 27) = -0.131256;
        weights[2]->coeffRef(15, 27) = -0.98739;
        weights[2]->coeffRef(16, 27) = 0.299172;
        weights[2]->coeffRef(17, 27) = -0.747528;
        weights[2]->coeffRef(18, 27) = -0.0764464;
        weights[2]->coeffRef(19, 27) = -0.831574;
        weights[2]->coeffRef(20, 27) = 0.561289;
        weights[2]->coeffRef(0, 28) = 0.574635;
        weights[2]->coeffRef(1, 28) = 0.374286;
        weights[2]->coeffRef(2, 28) = 0.807811;
        weights[2]->coeffRef(3, 28) = 0.727031;
        weights[2]->coeffRef(4, 28) = -0.874155;
        weights[2]->coeffRef(5, 28) = -0.897648;
        weights[2]->coeffRef(6, 28) = 0.0550312;
        weights[2]->coeffRef(7, 28) = -0.651299;
        weights[2]->coeffRef(8, 28) = 0.860578;
        weights[2]->coeffRef(9, 28) = -0.778766;
        weights[2]->coeffRef(10, 28) = -0.230118;
        weights[2]->coeffRef(11, 28) = 0.203666;
        weights[2]->coeffRef(12, 28) = 0.269146;
        weights[2]->coeffRef(13, 28) = 0.395383;
        weights[2]->coeffRef(14, 28) = 0.0869811;
        weights[2]->coeffRef(15, 28) = -0.171376;
        weights[2]->coeffRef(16, 28) = 0.645828;
        weights[2]->coeffRef(17, 28) = 0.155781;
        weights[2]->coeffRef(18, 28) = 0.838437;
        weights[2]->coeffRef(19, 28) = -0.56037;
        weights[2]->coeffRef(20, 28) = 0.584499;
        weights[2]->coeffRef(0, 29) = -0.254301;
        weights[2]->coeffRef(1, 29) = -0.240402;
        weights[2]->coeffRef(2, 29) = -0.794222;
        weights[2]->coeffRef(3, 29) = 0.622037;
        weights[2]->coeffRef(4, 29) = -0.225712;
        weights[2]->coeffRef(5, 29) = 0.487431;
        weights[2]->coeffRef(6, 29) = 0.867448;
        weights[2]->coeffRef(7, 29) = 0.703391;
        weights[2]->coeffRef(8, 29) = 0.658636;
        weights[2]->coeffRef(9, 29) = 0.427118;
        weights[2]->coeffRef(10, 29) = 0.275691;
        weights[2]->coeffRef(11, 29) = 0.0225567;
        weights[2]->coeffRef(12, 29) = 0.256388;
        weights[2]->coeffRef(13, 29) = 0.00940801;
        weights[2]->coeffRef(14, 29) = 0.150341;
        weights[2]->coeffRef(15, 29) = 0.3303;
        weights[2]->coeffRef(16, 29) = -0.941567;
        weights[2]->coeffRef(17, 29) = 0.511531;
        weights[2]->coeffRef(18, 29) = 0.203346;
        weights[2]->coeffRef(19, 29) = -0.718991;
        weights[2]->coeffRef(20, 29) = -0.717405;
        weights[2]->coeffRef(0, 30) = -0.609183;
        weights[2]->coeffRef(1, 30) = 0.554574;
        weights[2]->coeffRef(2, 30) = 0.689379;
        weights[2]->coeffRef(3, 30) = 0.470678;
        weights[2]->coeffRef(4, 30) = -0.63181;
        weights[2]->coeffRef(5, 30) = 0.332176;
        weights[2]->coeffRef(6, 30) = -0.374454;
        weights[2]->coeffRef(7, 30) = -0.788283;
        weights[2]->coeffRef(8, 30) = 0.776199;
        weights[2]->coeffRef(9, 30) = -0.795378;
        weights[2]->coeffRef(10, 30) = -0.0398869;
        weights[2]->coeffRef(11, 30) = -0.460246;
        weights[2]->coeffRef(12, 30) = -0.598841;
        weights[2]->coeffRef(13, 30) = -0.423993;
        weights[2]->coeffRef(14, 30) = 0.313994;
        weights[2]->coeffRef(15, 30) = 0.892449;
        weights[2]->coeffRef(16, 30) = -0.555469;
        weights[2]->coeffRef(17, 30) = 0.0137965;
        weights[2]->coeffRef(18, 30) = 0.557093;
        weights[2]->coeffRef(19, 30) = 0.872333;
        weights[2]->coeffRef(20, 30) = -0.716514;
        weights[2]->coeffRef(0, 31) = -0.413243;
        weights[2]->coeffRef(1, 31) = 0.118517;
        weights[2]->coeffRef(2, 31) = 0.298138;
        weights[2]->coeffRef(3, 31) = 0.751911;
        weights[2]->coeffRef(4, 31) = -0.534576;
        weights[2]->coeffRef(5, 31) = 0.34184;
        weights[2]->coeffRef(6, 31) = 0.257864;
        weights[2]->coeffRef(7, 31) = 0.66925;
        weights[2]->coeffRef(8, 31) = 0.622299;
        weights[2]->coeffRef(9, 31) = 0.545187;
        weights[2]->coeffRef(10, 31) = -0.938613;
        weights[2]->coeffRef(11, 31) = 0.172551;
        weights[2]->coeffRef(12, 31) = 0.241393;
        weights[2]->coeffRef(13, 31) = 0.531454;
        weights[2]->coeffRef(14, 31) = 0.542572;
        weights[2]->coeffRef(15, 31) = -0.442818;
        weights[2]->coeffRef(16, 31) = -0.846784;
        weights[2]->coeffRef(17, 31) = 0.759724;
        weights[2]->coeffRef(18, 31) = -0.651255;
        weights[2]->coeffRef(19, 31) = -0.639612;
        weights[2]->coeffRef(20, 31) = -0.28497;
        weights[2]->coeffRef(0, 32) = -0.116673;
        weights[2]->coeffRef(1, 32) = -0.246731;
        weights[2]->coeffRef(2, 32) = 0.305807;
        weights[2]->coeffRef(3, 32) = -0.792552;
        weights[2]->coeffRef(4, 32) = -0.348925;
        weights[2]->coeffRef(5, 32) = 0.731723;
        weights[2]->coeffRef(6, 32) = 0.214254;
        weights[2]->coeffRef(7, 32) = -0.786792;
        weights[2]->coeffRef(8, 32) = 0.60727;
        weights[2]->coeffRef(9, 32) = 0.497714;
        weights[2]->coeffRef(10, 32) = -0.197174;
        weights[2]->coeffRef(11, 32) = -0.276143;
        weights[2]->coeffRef(12, 32) = -0.204364;
        weights[2]->coeffRef(13, 32) = -0.451011;
        weights[2]->coeffRef(14, 32) = 0.19197;
        weights[2]->coeffRef(15, 32) = -0.876916;
        weights[2]->coeffRef(16, 32) = 0.802631;
        weights[2]->coeffRef(17, 32) = -0.135952;
        weights[2]->coeffRef(18, 32) = 0.766382;
        weights[2]->coeffRef(19, 32) = 0.352655;
        weights[2]->coeffRef(20, 32) = -0.0840073;
        weights[2]->coeffRef(0, 33) = -0.0587928;
        weights[2]->coeffRef(1, 33) = -0.418783;
        weights[2]->coeffRef(2, 33) = -0.541794;
        weights[2]->coeffRef(3, 33) = -0.502391;
        weights[2]->coeffRef(4, 33) = 0.153067;
        weights[2]->coeffRef(5, 33) = -0.403596;
        weights[2]->coeffRef(6, 33) = -0.748507;
        weights[2]->coeffRef(7, 33) = 0.503034;
        weights[2]->coeffRef(8, 33) = -0.0432977;
        weights[2]->coeffRef(9, 33) = -0.0298922;
        weights[2]->coeffRef(10, 33) = -0.610503;
        weights[2]->coeffRef(11, 33) = 0.709319;
        weights[2]->coeffRef(12, 33) = -0.727493;
        weights[2]->coeffRef(13, 33) = -0.40972;
        weights[2]->coeffRef(14, 33) = -0.636903;
        weights[2]->coeffRef(15, 33) = -1.00578;
        weights[2]->coeffRef(16, 33) = 0.800822;
        weights[2]->coeffRef(17, 33) = -0.422801;
        weights[2]->coeffRef(18, 33) = 0.620703;
        weights[2]->coeffRef(19, 33) = 0.303811;
        weights[2]->coeffRef(20, 33) = 0.370579;
        weights[2]->coeffRef(0, 34) = -0.648406;
        weights[2]->coeffRef(1, 34) = -0.90901;
        weights[2]->coeffRef(2, 34) = 0.915575;
        weights[2]->coeffRef(3, 34) = 0.547869;
        weights[2]->coeffRef(4, 34) = -0.773922;
        weights[2]->coeffRef(5, 34) = 0.724904;
        weights[2]->coeffRef(6, 34) = -0.585274;
        weights[2]->coeffRef(7, 34) = 0.986762;
        weights[2]->coeffRef(8, 34) = 0.073649;
        weights[2]->coeffRef(9, 34) = 0.336485;
        weights[2]->coeffRef(10, 34) = -0.0701972;
        weights[2]->coeffRef(11, 34) = 0.660404;
        weights[2]->coeffRef(12, 34) = 0.782468;
        weights[2]->coeffRef(13, 34) = 0.422232;
        weights[2]->coeffRef(14, 34) = -0.187042;
        weights[2]->coeffRef(15, 34) = -0.608384;
        weights[2]->coeffRef(16, 34) = 0.676312;
        weights[2]->coeffRef(17, 34) = -0.690421;
        weights[2]->coeffRef(18, 34) = 0.345898;
        weights[2]->coeffRef(19, 34) = -0.353497;
        weights[2]->coeffRef(20, 34) = -0.303839;
        weights[2]->coeffRef(0, 35) = 0.066;
        weights[2]->coeffRef(1, 35) = -0.0845218;
        weights[2]->coeffRef(2, 35) = 0.278079;
        weights[2]->coeffRef(3, 35) = 0.432838;
        weights[2]->coeffRef(4, 35) = -0.0798413;
        weights[2]->coeffRef(5, 35) = 0.0843087;
        weights[2]->coeffRef(6, 35) = -0.987942;
        weights[2]->coeffRef(7, 35) = -0.463905;
        weights[2]->coeffRef(8, 35) = -0.615549;
        weights[2]->coeffRef(9, 35) = 0.387056;
        weights[2]->coeffRef(10, 35) = -0.113167;
        weights[2]->coeffRef(11, 35) = -0.524823;
        weights[2]->coeffRef(12, 35) = 0.303965;
        weights[2]->coeffRef(13, 35) = -0.562898;
        weights[2]->coeffRef(14, 35) = -0.299268;
        weights[2]->coeffRef(15, 35) = 0.0322095;
        weights[2]->coeffRef(16, 35) = -0.147338;
        weights[2]->coeffRef(17, 35) = -0.312869;
        weights[2]->coeffRef(18, 35) = -0.899985;
        weights[2]->coeffRef(19, 35) = -0.811866;
        weights[2]->coeffRef(20, 35) = 0.620151;
        weights[2]->coeffRef(0, 36) = 0.756025;
        weights[2]->coeffRef(1, 36) = 0.970207;
        weights[2]->coeffRef(2, 36) = 0.0504631;
        weights[2]->coeffRef(3, 36) = -0.426966;
        weights[2]->coeffRef(4, 36) = -0.639979;
        weights[2]->coeffRef(5, 36) = -0.286761;
        weights[2]->coeffRef(6, 36) = -0.122691;
        weights[2]->coeffRef(7, 36) = 0.71112;
        weights[2]->coeffRef(8, 36) = 0.363048;
        weights[2]->coeffRef(9, 36) = 0.571108;
        weights[2]->coeffRef(10, 36) = -0.223507;
        weights[2]->coeffRef(11, 36) = -0.72643;
        weights[2]->coeffRef(12, 36) = -0.141687;
        weights[2]->coeffRef(13, 36) = -0.789837;
        weights[2]->coeffRef(14, 36) = 0.195437;
        weights[2]->coeffRef(15, 36) = 0.925541;
        weights[2]->coeffRef(16, 36) = -0.78189;
        weights[2]->coeffRef(17, 36) = 0.7379;
        weights[2]->coeffRef(18, 36) = -0.678478;
        weights[2]->coeffRef(19, 36) = 0.6074;
        weights[2]->coeffRef(20, 36) = -0.37751;
        weights[2]->coeffRef(0, 37) = -0.209753;
        weights[2]->coeffRef(1, 37) = -0.0905446;
        weights[2]->coeffRef(2, 37) = 0.0696419;
        weights[2]->coeffRef(3, 37) = 0.492898;
        weights[2]->coeffRef(4, 37) = 0.93989;
        weights[2]->coeffRef(5, 37) = 0.914344;
        weights[2]->coeffRef(6, 37) = -0.82331;
        weights[2]->coeffRef(7, 37) = -0.956779;
        weights[2]->coeffRef(8, 37) = -0.895776;
        weights[2]->coeffRef(9, 37) = 0.794917;
        weights[2]->coeffRef(10, 37) = 0.801441;
        weights[2]->coeffRef(11, 37) = -0.925028;
        weights[2]->coeffRef(12, 37) = -0.15864;
        weights[2]->coeffRef(13, 37) = -0.630308;
        weights[2]->coeffRef(14, 37) = -0.563523;
        weights[2]->coeffRef(15, 37) = 0.550718;
        weights[2]->coeffRef(16, 37) = 0.245224;
        weights[2]->coeffRef(17, 37) = -0.852881;
        weights[2]->coeffRef(18, 37) = -0.0751893;
        weights[2]->coeffRef(19, 37) = -0.180566;
        weights[2]->coeffRef(20, 37) = -0.0825076;
        weights[2]->coeffRef(0, 38) = 0.198988;
        weights[2]->coeffRef(1, 38) = 0.662477;
        weights[2]->coeffRef(2, 38) = 0.148625;
        weights[2]->coeffRef(3, 38) = -0.582675;
        weights[2]->coeffRef(4, 38) = 0.605563;
        weights[2]->coeffRef(5, 38) = 0.331374;
        weights[2]->coeffRef(6, 38) = -0.858813;
        weights[2]->coeffRef(7, 38) = 0.934721;
        weights[2]->coeffRef(8, 38) = -0.0580196;
        weights[2]->coeffRef(9, 38) = -0.234824;
        weights[2]->coeffRef(10, 38) = -0.272936;
        weights[2]->coeffRef(11, 38) = 0.842164;
        weights[2]->coeffRef(12, 38) = 0.846796;
        weights[2]->coeffRef(13, 38) = -0.784352;
        weights[2]->coeffRef(14, 38) = 0.787762;
        weights[2]->coeffRef(15, 38) = 0.724564;
        weights[2]->coeffRef(16, 38) = -0.618284;
        weights[2]->coeffRef(17, 38) = 0.842062;
        weights[2]->coeffRef(18, 38) = 0.865475;
        weights[2]->coeffRef(19, 38) = -0.813944;
        weights[2]->coeffRef(20, 38) = 0.631483;
        weights[2]->coeffRef(0, 39) = 0.93727;
        weights[2]->coeffRef(1, 39) = 0.0163839;
        weights[2]->coeffRef(2, 39) = -0.987784;
        weights[2]->coeffRef(3, 39) = -0.621411;
        weights[2]->coeffRef(4, 39) = -0.425918;
        weights[2]->coeffRef(5, 39) = 0.254261;
        weights[2]->coeffRef(6, 39) = -0.475875;
        weights[2]->coeffRef(7, 39) = 0.498205;
        weights[2]->coeffRef(8, 39) = -0.927551;
        weights[2]->coeffRef(9, 39) = 0.442819;
        weights[2]->coeffRef(10, 39) = -0.297844;
        weights[2]->coeffRef(11, 39) = 0.742028;
        weights[2]->coeffRef(12, 39) = -0.429761;
        weights[2]->coeffRef(13, 39) = 0.106338;
        weights[2]->coeffRef(14, 39) = 0.34975;
        weights[2]->coeffRef(15, 39) = 0.912835;
        weights[2]->coeffRef(16, 39) = 0.246938;
        weights[2]->coeffRef(17, 39) = 0.275469;
        weights[2]->coeffRef(18, 39) = -0.133129;
        weights[2]->coeffRef(19, 39) = -0.981244;
        weights[2]->coeffRef(20, 39) = 0.991318;
        weights[2]->coeffRef(0, 40) = -0.275343;
        weights[2]->coeffRef(1, 40) = 0.846476;
        weights[2]->coeffRef(2, 40) = -0.790157;
        weights[2]->coeffRef(3, 40) = -0.46435;
        weights[2]->coeffRef(4, 40) = 0.601669;
        weights[2]->coeffRef(5, 40) = -0.425303;
        weights[2]->coeffRef(6, 40) = -0.628887;
        weights[2]->coeffRef(7, 40) = 0.464459;
        weights[2]->coeffRef(8, 40) = -0.243044;
        weights[2]->coeffRef(9, 40) = -0.987922;
        weights[2]->coeffRef(10, 40) = 0.401704;
        weights[2]->coeffRef(11, 40) = 0.768873;
        weights[2]->coeffRef(12, 40) = -0.969289;
        weights[2]->coeffRef(13, 40) = 0.777496;
        weights[2]->coeffRef(14, 40) = -0.654184;
        weights[2]->coeffRef(15, 40) = 0.26444;
        weights[2]->coeffRef(16, 40) = -0.704106;
        weights[2]->coeffRef(17, 40) = 0.850381;
        weights[2]->coeffRef(18, 40) = 0.355501;
        weights[2]->coeffRef(19, 40) = 0.743264;
        weights[2]->coeffRef(20, 40) = -0.453746;
        weights[2]->coeffRef(0, 41) = 0.10117;
        weights[2]->coeffRef(1, 41) = -0.680406;
        weights[2]->coeffRef(2, 41) = 0.63355;
        weights[2]->coeffRef(3, 41) = -0.56756;
        weights[2]->coeffRef(4, 41) = -0.773358;
        weights[2]->coeffRef(5, 41) = -0.0791916;
        weights[2]->coeffRef(6, 41) = 0.723471;
        weights[2]->coeffRef(7, 41) = 0.0810048;
        weights[2]->coeffRef(8, 41) = -0.0681889;
        weights[2]->coeffRef(9, 41) = 0.717546;
        weights[2]->coeffRef(10, 41) = 0.807466;
        weights[2]->coeffRef(11, 41) = -0.205413;
        weights[2]->coeffRef(12, 41) = 0.897032;
        weights[2]->coeffRef(13, 41) = -0.661781;
        weights[2]->coeffRef(14, 41) = -0.608825;
        weights[2]->coeffRef(15, 41) = -0.476449;
        weights[2]->coeffRef(16, 41) = -0.278347;
        weights[2]->coeffRef(17, 41) = 0.835589;
        weights[2]->coeffRef(18, 41) = 0.247109;
        weights[2]->coeffRef(19, 41) = -0.272513;
        weights[2]->coeffRef(20, 41) = 0.242486;
        weights[2]->coeffRef(0, 42) = 0.038572;
        weights[2]->coeffRef(1, 42) = -0.23645;
        weights[2]->coeffRef(2, 42) = -0.0117121;
        weights[2]->coeffRef(3, 42) = 0.379424;
        weights[2]->coeffRef(4, 42) = -0.963477;
        weights[2]->coeffRef(5, 42) = 0.310223;
        weights[2]->coeffRef(6, 42) = 0.240217;
        weights[2]->coeffRef(7, 42) = 0.380484;
        weights[2]->coeffRef(8, 42) = 0.0463263;
        weights[2]->coeffRef(9, 42) = 0.793219;
        weights[2]->coeffRef(10, 42) = -0.524682;
        weights[2]->coeffRef(11, 42) = 0.364507;
        weights[2]->coeffRef(12, 42) = 0.437955;
        weights[2]->coeffRef(13, 42) = -0.0777658;
        weights[2]->coeffRef(14, 42) = 0.586779;
        weights[2]->coeffRef(15, 42) = -0.627661;
        weights[2]->coeffRef(16, 42) = -0.348841;
        weights[2]->coeffRef(17, 42) = -0.331464;
        weights[2]->coeffRef(18, 42) = 0.27019;
        weights[2]->coeffRef(19, 42) = -0.640336;
        weights[2]->coeffRef(20, 42) = -0.505426;
        weights[2]->coeffRef(0, 43) = -0.947318;
        weights[2]->coeffRef(1, 43) = -0.719489;
        weights[2]->coeffRef(2, 43) = -0.17597;
        weights[2]->coeffRef(3, 43) = -0.581552;
        weights[2]->coeffRef(4, 43) = -0.229183;
        weights[2]->coeffRef(5, 43) = 0.563737;
        weights[2]->coeffRef(6, 43) = -0.739359;
        weights[2]->coeffRef(7, 43) = -0.978989;
        weights[2]->coeffRef(8, 43) = -0.705279;
        weights[2]->coeffRef(9, 43) = 0.491908;
        weights[2]->coeffRef(10, 43) = 0.0554542;
        weights[2]->coeffRef(11, 43) = 0.0579497;
        weights[2]->coeffRef(12, 43) = -0.514546;
        weights[2]->coeffRef(13, 43) = -0.555725;
        weights[2]->coeffRef(14, 43) = 0.0917689;
        weights[2]->coeffRef(15, 43) = 0.808617;
        weights[2]->coeffRef(16, 43) = 0.688145;
        weights[2]->coeffRef(17, 43) = -0.528786;
        weights[2]->coeffRef(18, 43) = -0.169701;
        weights[2]->coeffRef(19, 43) = 0.476293;
        weights[2]->coeffRef(20, 43) = -0.0404083;
        weights[2]->coeffRef(0, 44) = -0.807439;
        weights[2]->coeffRef(1, 44) = -0.0615498;
        weights[2]->coeffRef(2, 44) = 0.852546;
        weights[2]->coeffRef(3, 44) = 0.744504;
        weights[2]->coeffRef(4, 44) = 0.281182;
        weights[2]->coeffRef(5, 44) = -0.447338;
        weights[2]->coeffRef(6, 44) = -0.56794;
        weights[2]->coeffRef(7, 44) = -0.457471;
        weights[2]->coeffRef(8, 44) = -0.0912163;
        weights[2]->coeffRef(9, 44) = -0.078642;
        weights[2]->coeffRef(10, 44) = -0.409499;
        weights[2]->coeffRef(11, 44) = 0.200099;
        weights[2]->coeffRef(12, 44) = 0.732113;
        weights[2]->coeffRef(13, 44) = 0.0168998;
        weights[2]->coeffRef(14, 44) = 0.963907;
        weights[2]->coeffRef(15, 44) = 0.344688;
        weights[2]->coeffRef(16, 44) = 0.290406;
        weights[2]->coeffRef(17, 44) = 0.970944;
        weights[2]->coeffRef(18, 44) = 0.5865;
        weights[2]->coeffRef(19, 44) = -0.22896;
        weights[2]->coeffRef(20, 44) = 0.0457829;
        weights[2]->coeffRef(0, 45) = -0.354325;
        weights[2]->coeffRef(1, 45) = 0.27472;
        weights[2]->coeffRef(2, 45) = 0.476748;
        weights[2]->coeffRef(3, 45) = 0.726287;
        weights[2]->coeffRef(4, 45) = 0.0675669;
        weights[2]->coeffRef(5, 45) = 0.170838;
        weights[2]->coeffRef(6, 45) = -0.8005;
        weights[2]->coeffRef(7, 45) = 0.900194;
        weights[2]->coeffRef(8, 45) = -0.351119;
        weights[2]->coeffRef(9, 45) = 0.153585;
        weights[2]->coeffRef(10, 45) = -0.914982;
        weights[2]->coeffRef(11, 45) = 0.577596;
        weights[2]->coeffRef(12, 45) = 0.0332858;
        weights[2]->coeffRef(13, 45) = 0.846704;
        weights[2]->coeffRef(14, 45) = -0.1431;
        weights[2]->coeffRef(15, 45) = 0.572656;
        weights[2]->coeffRef(16, 45) = -0.722023;
        weights[2]->coeffRef(17, 45) = 0.410724;
        weights[2]->coeffRef(18, 45) = -0.536198;
        weights[2]->coeffRef(19, 45) = 0.193128;
        weights[2]->coeffRef(20, 45) = -0.982539;
        weights[2]->coeffRef(0, 46) = 0.639302;
        weights[2]->coeffRef(1, 46) = -0.0502927;
        weights[2]->coeffRef(2, 46) = 0.0363866;
        weights[2]->coeffRef(3, 46) = 0.576084;
        weights[2]->coeffRef(4, 46) = -0.745842;
        weights[2]->coeffRef(5, 46) = -0.660593;
        weights[2]->coeffRef(6, 46) = 0.552013;
        weights[2]->coeffRef(7, 46) = 0.847252;
        weights[2]->coeffRef(8, 46) = 0.116905;
        weights[2]->coeffRef(9, 46) = -0.41524;
        weights[2]->coeffRef(10, 46) = -0.508272;
        weights[2]->coeffRef(11, 46) = -0.60494;
        weights[2]->coeffRef(12, 46) = -0.940947;
        weights[2]->coeffRef(13, 46) = -0.779275;
        weights[2]->coeffRef(14, 46) = 0.459551;
        weights[2]->coeffRef(15, 46) = 0.242215;
        weights[2]->coeffRef(16, 46) = -0.57488;
        weights[2]->coeffRef(17, 46) = 0.356618;
        weights[2]->coeffRef(18, 46) = 0.875803;
        weights[2]->coeffRef(19, 46) = 0.573108;
        weights[2]->coeffRef(20, 46) = 0.446995;
        weights[2]->coeffRef(0, 47) = 0.462187;
        weights[2]->coeffRef(1, 47) = -0.376048;
        weights[2]->coeffRef(2, 47) = 0.260068;
        weights[2]->coeffRef(3, 47) = -0.70914;
        weights[2]->coeffRef(4, 47) = -0.818805;
        weights[2]->coeffRef(5, 47) = 0.590179;
        weights[2]->coeffRef(6, 47) = 0.721885;
        weights[2]->coeffRef(7, 47) = -0.368892;
        weights[2]->coeffRef(8, 47) = -0.224047;
        weights[2]->coeffRef(9, 47) = 0.739491;
        weights[2]->coeffRef(10, 47) = -0.731789;
        weights[2]->coeffRef(11, 47) = 0.737775;
        weights[2]->coeffRef(12, 47) = -0.241473;
        weights[2]->coeffRef(13, 47) = 0.851916;
        weights[2]->coeffRef(14, 47) = 0.984605;
        weights[2]->coeffRef(15, 47) = 0.153778;
        weights[2]->coeffRef(16, 47) = 0.418154;
        weights[2]->coeffRef(17, 47) = 0.814914;
        weights[2]->coeffRef(18, 47) = -0.78294;
        weights[2]->coeffRef(19, 47) = -1.00813;
        weights[2]->coeffRef(20, 47) = -0.67448;
        weights[2]->coeffRef(0, 48) = -0.379604;
        weights[2]->coeffRef(1, 48) = -0.93016;
        weights[2]->coeffRef(2, 48) = -0.482935;
        weights[2]->coeffRef(3, 48) = -0.939371;
        weights[2]->coeffRef(4, 48) = 0.295572;
        weights[2]->coeffRef(5, 48) = -0.0304541;
        weights[2]->coeffRef(6, 48) = 0.428154;
        weights[2]->coeffRef(7, 48) = 0.166497;
        weights[2]->coeffRef(8, 48) = -0.458643;
        weights[2]->coeffRef(9, 48) = -0.127934;
        weights[2]->coeffRef(10, 48) = -0.380377;
        weights[2]->coeffRef(11, 48) = 0.161934;
        weights[2]->coeffRef(12, 48) = -0.851397;
        weights[2]->coeffRef(13, 48) = -0.0710937;
        weights[2]->coeffRef(14, 48) = 0.338591;
        weights[2]->coeffRef(15, 48) = 0.751372;
        weights[2]->coeffRef(16, 48) = -0.344434;
        weights[2]->coeffRef(17, 48) = 0.972421;
        weights[2]->coeffRef(18, 48) = -0.513238;
        weights[2]->coeffRef(19, 48) = -0.614457;
        weights[2]->coeffRef(20, 48) = -0.735822;
        weights[2]->coeffRef(0, 49) = -0.799817;
        weights[2]->coeffRef(1, 49) = 0.165773;
        weights[2]->coeffRef(2, 49) = -0.868366;
        weights[2]->coeffRef(3, 49) = -0.828499;
        weights[2]->coeffRef(4, 49) = -0.715883;
        weights[2]->coeffRef(5, 49) = 0.531633;
        weights[2]->coeffRef(6, 49) = 0.978486;
        weights[2]->coeffRef(7, 49) = -0.482425;
        weights[2]->coeffRef(8, 49) = 0.534284;
        weights[2]->coeffRef(9, 49) = -0.712992;
        weights[2]->coeffRef(10, 49) = 0.13364;
        weights[2]->coeffRef(11, 49) = 0.592157;
        weights[2]->coeffRef(12, 49) = -0.17087;
        weights[2]->coeffRef(13, 49) = 0.202984;
        weights[2]->coeffRef(14, 49) = -0.110002;
        weights[2]->coeffRef(15, 49) = 0.768007;
        weights[2]->coeffRef(16, 49) = -0.376083;
        weights[2]->coeffRef(17, 49) = -0.929349;
        weights[2]->coeffRef(18, 49) = -0.680928;
        weights[2]->coeffRef(19, 49) = 0.497892;
        weights[2]->coeffRef(20, 49) = -0.305811;
        weights[3]->coeffRef(0, 0) = 0.793662;
        weights[3]->coeffRef(1, 0) = 0.329318;
        weights[3]->coeffRef(2, 0) = -0.0305746;
        weights[3]->coeffRef(3, 0) = -0.305574;
        weights[3]->coeffRef(4, 0) = 0.126365;
        weights[3]->coeffRef(5, 0) = 0.600499;
        weights[3]->coeffRef(6, 0) = -0.221423;
        weights[3]->coeffRef(7, 0) = 0.440723;
        weights[3]->coeffRef(8, 0) = 0.102671;
        weights[3]->coeffRef(9, 0) = 0.496002;
        weights[3]->coeffRef(10, 0) = -0.0868364;
        weights[3]->coeffRef(11, 0) = -0.243265;
        weights[3]->coeffRef(12, 0) = 0.120088;
        weights[3]->coeffRef(13, 0) = 0.682777;
        weights[3]->coeffRef(14, 0) = -0.84648;
        weights[3]->coeffRef(15, 0) = 0.401137;
        weights[3]->coeffRef(16, 0) = -0.546601;
        weights[3]->coeffRef(17, 0) = -0.991634;
        weights[3]->coeffRef(18, 0) = -0.910166;
        weights[3]->coeffRef(19, 0) = 0.467298;
        weights[3]->coeffRef(20, 0) = -0.894753;
        weights[3]->coeffRef(21, 0) = 0.930104;
        weights[3]->coeffRef(22, 0) = -0.39471;
        weights[3]->coeffRef(23, 0) = -0.650131;
        weights[3]->coeffRef(24, 0) = -0.928632;
        weights[3]->coeffRef(25, 0) = 0.257193;
        weights[3]->coeffRef(26, 0) = -0.442206;
        weights[3]->coeffRef(27, 0) = -0.271845;
        weights[3]->coeffRef(28, 0) = 0.471065;
        weights[3]->coeffRef(29, 0) = 0.0815357;
        weights[3]->coeffRef(30, 0) = -0.497914;
        weights[3]->coeffRef(31, 0) = 0.292679;
        weights[3]->coeffRef(32, 0) = -0.586678;
        weights[3]->coeffRef(33, 0) = 0.473525;
        weights[3]->coeffRef(34, 0) = -1.01628;
        weights[3]->coeffRef(35, 0) = 0.52791;
        weights[3]->coeffRef(36, 0) = 0.085422;
        weights[3]->coeffRef(37, 0) = -0.214971;
        weights[3]->coeffRef(38, 0) = -0.0183104;
        weights[3]->coeffRef(39, 0) = -0.798851;
        weights[3]->coeffRef(40, 0) = -0.727471;
        weights[3]->coeffRef(41, 0) = 0.837963;
        weights[3]->coeffRef(42, 0) = -0.0848289;
        weights[3]->coeffRef(43, 0) = 0.398267;
        weights[3]->coeffRef(44, 0) = 0.530074;
        weights[3]->coeffRef(45, 0) = 0.0497415;
        weights[3]->coeffRef(46, 0) = -0.210233;
        weights[3]->coeffRef(47, 0) = 0.996709;
        weights[3]->coeffRef(48, 0) = 0.0829995;
        weights[3]->coeffRef(49, 0) = -0.0975635;
        weights[3]->coeffRef(50, 0) = 0.43914;
        weights[3]->coeffRef(0, 1) = 0.143377;
        weights[3]->coeffRef(1, 1) = -0.188805;
        weights[3]->coeffRef(2, 1) = -0.976328;
        weights[3]->coeffRef(3, 1) = 0.484985;
        weights[3]->coeffRef(4, 1) = -0.102354;
        weights[3]->coeffRef(5, 1) = 0.295533;
        weights[3]->coeffRef(6, 1) = -0.941881;
        weights[3]->coeffRef(7, 1) = 0.61888;
        weights[3]->coeffRef(8, 1) = -0.21878;
        weights[3]->coeffRef(9, 1) = 0.148699;
        weights[3]->coeffRef(10, 1) = -0.873829;
        weights[3]->coeffRef(11, 1) = -0.926019;
        weights[3]->coeffRef(12, 1) = 0.552238;
        weights[3]->coeffRef(13, 1) = 0.585546;
        weights[3]->coeffRef(14, 1) = -0.912263;
        weights[3]->coeffRef(15, 1) = 0.0744195;
        weights[3]->coeffRef(16, 1) = -0.331795;
        weights[3]->coeffRef(17, 1) = -0.144322;
        weights[3]->coeffRef(18, 1) = -0.956961;
        weights[3]->coeffRef(19, 1) = -0.164071;
        weights[3]->coeffRef(20, 1) = 0.135986;
        weights[3]->coeffRef(21, 1) = 0.890841;
        weights[3]->coeffRef(22, 1) = 0.756114;
        weights[3]->coeffRef(23, 1) = -0.48108;
        weights[3]->coeffRef(24, 1) = 0.433232;
        weights[3]->coeffRef(25, 1) = -0.207943;
        weights[3]->coeffRef(26, 1) = 0.296516;
        weights[3]->coeffRef(27, 1) = 0.418994;
        weights[3]->coeffRef(28, 1) = 0.859613;
        weights[3]->coeffRef(29, 1) = -0.816851;
        weights[3]->coeffRef(30, 1) = -0.147636;
        weights[3]->coeffRef(31, 1) = 0.025091;
        weights[3]->coeffRef(32, 1) = -0.00370482;
        weights[3]->coeffRef(33, 1) = -0.12237;
        weights[3]->coeffRef(34, 1) = -0.4926;
        weights[3]->coeffRef(35, 1) = 0.884628;
        weights[3]->coeffRef(36, 1) = -0.817823;
        weights[3]->coeffRef(37, 1) = -0.416507;
        weights[3]->coeffRef(38, 1) = 0.513832;
        weights[3]->coeffRef(39, 1) = -0.0262796;
        weights[3]->coeffRef(40, 1) = 0.725473;
        weights[3]->coeffRef(41, 1) = 0.59502;
        weights[3]->coeffRef(42, 1) = 0.0139252;
        weights[3]->coeffRef(43, 1) = 0.282177;
        weights[3]->coeffRef(44, 1) = 0.187945;
        weights[3]->coeffRef(45, 1) = 0.0866771;
        weights[3]->coeffRef(46, 1) = -0.651022;
        weights[3]->coeffRef(47, 1) = 0.866616;
        weights[3]->coeffRef(48, 1) = 0.96204;
        weights[3]->coeffRef(49, 1) = -0.589928;
        weights[3]->coeffRef(50, 1) = -0.317117;
        weights[3]->coeffRef(0, 2) = 0.0784832;
        weights[3]->coeffRef(1, 2) = -0.710404;
        weights[3]->coeffRef(2, 2) = -0.572851;
        weights[3]->coeffRef(3, 2) = 0.587659;
        weights[3]->coeffRef(4, 2) = 0.722111;
        weights[3]->coeffRef(5, 2) = 0.226054;
        weights[3]->coeffRef(6, 2) = -0.113927;
        weights[3]->coeffRef(7, 2) = 0.136219;
        weights[3]->coeffRef(8, 2) = 0.0912124;
        weights[3]->coeffRef(9, 2) = 0.0620983;
        weights[3]->coeffRef(10, 2) = 0.984049;
        weights[3]->coeffRef(11, 2) = -0.882726;
        weights[3]->coeffRef(12, 2) = -0.940442;
        weights[3]->coeffRef(13, 2) = -0.135278;
        weights[3]->coeffRef(14, 2) = -0.360786;
        weights[3]->coeffRef(15, 2) = 0.947224;
        weights[3]->coeffRef(16, 2) = 0.036231;
        weights[3]->coeffRef(17, 2) = 0.227203;
        weights[3]->coeffRef(18, 2) = 0.444435;
        weights[3]->coeffRef(19, 2) = 0.984565;
        weights[3]->coeffRef(20, 2) = -0.0542109;
        weights[3]->coeffRef(21, 2) = 0.0553222;
        weights[3]->coeffRef(22, 2) = 0.00101656;
        weights[3]->coeffRef(23, 2) = -0.783608;
        weights[3]->coeffRef(24, 2) = -0.751711;
        weights[3]->coeffRef(25, 2) = -0.905744;
        weights[3]->coeffRef(26, 2) = -0.431396;
        weights[3]->coeffRef(27, 2) = -0.900453;
        weights[3]->coeffRef(28, 2) = -0.956953;
        weights[3]->coeffRef(29, 2) = -0.0419299;
        weights[3]->coeffRef(30, 2) = -0.219974;
        weights[3]->coeffRef(31, 2) = 0.117203;
        weights[3]->coeffRef(32, 2) = 0.247285;
        weights[3]->coeffRef(33, 2) = 0.206865;
        weights[3]->coeffRef(34, 2) = -0.294615;
        weights[3]->coeffRef(35, 2) = -0.0287796;
        weights[3]->coeffRef(36, 2) = -0.568846;
        weights[3]->coeffRef(37, 2) = 0.587938;
        weights[3]->coeffRef(38, 2) = -0.894582;
        weights[3]->coeffRef(39, 2) = 0.520344;
        weights[3]->coeffRef(40, 2) = -0.348649;
        weights[3]->coeffRef(41, 2) = -0.901723;
        weights[3]->coeffRef(42, 2) = 0.644237;
        weights[3]->coeffRef(43, 2) = -0.289965;
        weights[3]->coeffRef(44, 2) = -0.0384461;
        weights[3]->coeffRef(45, 2) = -0.713616;
        weights[3]->coeffRef(46, 2) = -0.34125;
        weights[3]->coeffRef(47, 2) = 0.995736;
        weights[3]->coeffRef(48, 2) = 0.509733;
        weights[3]->coeffRef(49, 2) = -0.900349;
        weights[3]->coeffRef(50, 2) = 0.98415;
        weights[3]->coeffRef(0, 3) = -0.529236;
        weights[3]->coeffRef(1, 3) = 0.161175;
        weights[3]->coeffRef(2, 3) = -0.00924415;
        weights[3]->coeffRef(3, 3) = -0.313237;
        weights[3]->coeffRef(4, 3) = 0.400547;
        weights[3]->coeffRef(5, 3) = 0.0802661;
        weights[3]->coeffRef(6, 3) = 0.247667;
        weights[3]->coeffRef(7, 3) = 0.501472;
        weights[3]->coeffRef(8, 3) = 0.117918;
        weights[3]->coeffRef(9, 3) = -0.803096;
        weights[3]->coeffRef(10, 3) = -0.723811;
        weights[3]->coeffRef(11, 3) = -0.76442;
        weights[3]->coeffRef(12, 3) = 0.450336;
        weights[3]->coeffRef(13, 3) = 0.492652;
        weights[3]->coeffRef(14, 3) = -0.0684798;
        weights[3]->coeffRef(15, 3) = -0.573615;
        weights[3]->coeffRef(16, 3) = 0.919822;
        weights[3]->coeffRef(17, 3) = -0.468523;
        weights[3]->coeffRef(18, 3) = -0.469448;
        weights[3]->coeffRef(19, 3) = 0.445883;
        weights[3]->coeffRef(20, 3) = 0.174839;
        weights[3]->coeffRef(21, 3) = -0.368484;
        weights[3]->coeffRef(22, 3) = 0.0885946;
        weights[3]->coeffRef(23, 3) = 0.887537;
        weights[3]->coeffRef(24, 3) = 0.588705;
        weights[3]->coeffRef(25, 3) = 0.386211;
        weights[3]->coeffRef(26, 3) = -0.445134;
        weights[3]->coeffRef(27, 3) = 0.582645;
        weights[3]->coeffRef(28, 3) = -0.102206;
        weights[3]->coeffRef(29, 3) = -0.347257;
        weights[3]->coeffRef(30, 3) = 0.56881;
        weights[3]->coeffRef(31, 3) = 0.353786;
        weights[3]->coeffRef(32, 3) = 0.812613;
        weights[3]->coeffRef(33, 3) = -0.441498;
        weights[3]->coeffRef(34, 3) = -0.957662;
        weights[3]->coeffRef(35, 3) = 0.219384;
        weights[3]->coeffRef(36, 3) = 0.632745;
        weights[3]->coeffRef(37, 3) = 0.27799;
        weights[3]->coeffRef(38, 3) = -0.286048;
        weights[3]->coeffRef(39, 3) = -0.256238;
        weights[3]->coeffRef(40, 3) = 0.479387;
        weights[3]->coeffRef(41, 3) = 0.0202109;
        weights[3]->coeffRef(42, 3) = 0.00191788;
        weights[3]->coeffRef(43, 3) = -0.0732615;
        weights[3]->coeffRef(44, 3) = -0.492069;
        weights[3]->coeffRef(45, 3) = 0.943455;
        weights[3]->coeffRef(46, 3) = 0.358214;
        weights[3]->coeffRef(47, 3) = -0.579246;
        weights[3]->coeffRef(48, 3) = -0.538229;
        weights[3]->coeffRef(49, 3) = 0.876696;
        weights[3]->coeffRef(50, 3) = 0.879782;
        weights[3]->coeffRef(0, 4) = 0.676857;
        weights[3]->coeffRef(1, 4) = -0.474735;
        weights[3]->coeffRef(2, 4) = -0.0159188;
        weights[3]->coeffRef(3, 4) = 0.565428;
        weights[3]->coeffRef(4, 4) = -0.907114;
        weights[3]->coeffRef(5, 4) = -0.64239;
        weights[3]->coeffRef(6, 4) = -0.898421;
        weights[3]->coeffRef(7, 4) = 0.679794;
        weights[3]->coeffRef(8, 4) = 0.241437;
        weights[3]->coeffRef(9, 4) = -0.265267;
        weights[3]->coeffRef(10, 4) = 0.236882;
        weights[3]->coeffRef(11, 4) = -0.403878;
        weights[3]->coeffRef(12, 4) = -0.438255;
        weights[3]->coeffRef(13, 4) = 0.817621;
        weights[3]->coeffRef(14, 4) = -0.386967;
        weights[3]->coeffRef(15, 4) = 0.792004;
        weights[3]->coeffRef(16, 4) = 0.44303;
        weights[3]->coeffRef(17, 4) = 0.918729;
        weights[3]->coeffRef(18, 4) = -0.493653;
        weights[3]->coeffRef(19, 4) = -0.794465;
        weights[3]->coeffRef(20, 4) = 0.380483;
        weights[3]->coeffRef(21, 4) = 0.529745;
        weights[3]->coeffRef(22, 4) = 0.203315;
        weights[3]->coeffRef(23, 4) = -0.684104;
        weights[3]->coeffRef(24, 4) = -0.973729;
        weights[3]->coeffRef(25, 4) = 0.172185;
        weights[3]->coeffRef(26, 4) = 0.693885;
        weights[3]->coeffRef(27, 4) = -0.55414;
        weights[3]->coeffRef(28, 4) = 0.641045;
        weights[3]->coeffRef(29, 4) = 0.570523;
        weights[3]->coeffRef(30, 4) = -0.669073;
        weights[3]->coeffRef(31, 4) = 0.283605;
        weights[3]->coeffRef(32, 4) = -0.907239;
        weights[3]->coeffRef(33, 4) = 0.312535;
        weights[3]->coeffRef(34, 4) = -0.146813;
        weights[3]->coeffRef(35, 4) = -0.799904;
        weights[3]->coeffRef(36, 4) = 0.65616;
        weights[3]->coeffRef(37, 4) = -0.0731302;
        weights[3]->coeffRef(38, 4) = 0.863868;
        weights[3]->coeffRef(39, 4) = -0.118427;
        weights[3]->coeffRef(40, 4) = 0.672032;
        weights[3]->coeffRef(41, 4) = 0.17056;
        weights[3]->coeffRef(42, 4) = 0.530108;
        weights[3]->coeffRef(43, 4) = -0.773155;
        weights[3]->coeffRef(44, 4) = -0.023268;
        weights[3]->coeffRef(45, 4) = -0.833606;
        weights[3]->coeffRef(46, 4) = -0.969327;
        weights[3]->coeffRef(47, 4) = -0.596483;
        weights[3]->coeffRef(48, 4) = -0.94543;
        weights[3]->coeffRef(49, 4) = -0.490997;
        weights[3]->coeffRef(50, 4) = -0.360432;
        weights[3]->coeffRef(0, 5) = 0.439467;
        weights[3]->coeffRef(1, 5) = -0.952743;
        weights[3]->coeffRef(2, 5) = 0.853846;
        weights[3]->coeffRef(3, 5) = 0.775711;
        weights[3]->coeffRef(4, 5) = -0.904793;
        weights[3]->coeffRef(5, 5) = 0.0231339;
        weights[3]->coeffRef(6, 5) = 0.483285;
        weights[3]->coeffRef(7, 5) = -0.452303;
        weights[3]->coeffRef(8, 5) = -0.334643;
        weights[3]->coeffRef(9, 5) = 0.0880789;
        weights[3]->coeffRef(10, 5) = -0.100275;
        weights[3]->coeffRef(11, 5) = 0.945978;
        weights[3]->coeffRef(12, 5) = 0.164561;
        weights[3]->coeffRef(13, 5) = -0.815606;
        weights[3]->coeffRef(14, 5) = -0.208417;
        weights[3]->coeffRef(15, 5) = 0.347642;
        weights[3]->coeffRef(16, 5) = 0.870898;
        weights[3]->coeffRef(17, 5) = 0.682192;
        weights[3]->coeffRef(18, 5) = 0.247525;
        weights[3]->coeffRef(19, 5) = -0.209823;
        weights[3]->coeffRef(20, 5) = 0.386109;
        weights[3]->coeffRef(21, 5) = -0.619693;
        weights[3]->coeffRef(22, 5) = -0.681301;
        weights[3]->coeffRef(23, 5) = 0.630024;
        weights[3]->coeffRef(24, 5) = 0.35692;
        weights[3]->coeffRef(25, 5) = -0.553512;
        weights[3]->coeffRef(26, 5) = 0.63476;
        weights[3]->coeffRef(27, 5) = 0.795179;
        weights[3]->coeffRef(28, 5) = -0.476694;
        weights[3]->coeffRef(29, 5) = -0.811078;
        weights[3]->coeffRef(30, 5) = -0.56503;
        weights[3]->coeffRef(31, 5) = -0.995051;
        weights[3]->coeffRef(32, 5) = -0.760094;
        weights[3]->coeffRef(33, 5) = -0.708145;
        weights[3]->coeffRef(34, 5) = 0.775555;
        weights[3]->coeffRef(35, 5) = -0.682656;
        weights[3]->coeffRef(36, 5) = 0.332189;
        weights[3]->coeffRef(37, 5) = 0.29314;
        weights[3]->coeffRef(38, 5) = -0.115259;
        weights[3]->coeffRef(39, 5) = 1.01725;
        weights[3]->coeffRef(40, 5) = -0.631606;
        weights[3]->coeffRef(41, 5) = 0.698618;
        weights[3]->coeffRef(42, 5) = 0.898768;
        weights[3]->coeffRef(43, 5) = 0.541478;
        weights[3]->coeffRef(44, 5) = 0.897093;
        weights[3]->coeffRef(45, 5) = -0.338241;
        weights[3]->coeffRef(46, 5) = -0.125421;
        weights[3]->coeffRef(47, 5) = 0.787969;
        weights[3]->coeffRef(48, 5) = -0.618482;
        weights[3]->coeffRef(49, 5) = -0.843438;
        weights[3]->coeffRef(50, 5) = -0.45938;
        weights[3]->coeffRef(0, 6) = 0.740454;
        weights[3]->coeffRef(1, 6) = -0.481192;
        weights[3]->coeffRef(2, 6) = -0.160392;
        weights[3]->coeffRef(3, 6) = 0.349841;
        weights[3]->coeffRef(4, 6) = 0.866329;
        weights[3]->coeffRef(5, 6) = 0.296145;
        weights[3]->coeffRef(6, 6) = -0.0182747;
        weights[3]->coeffRef(7, 6) = 0.652552;
        weights[3]->coeffRef(8, 6) = 0.826211;
        weights[3]->coeffRef(9, 6) = 0.147986;
        weights[3]->coeffRef(10, 6) = -0.926622;
        weights[3]->coeffRef(11, 6) = 0.833596;
        weights[3]->coeffRef(12, 6) = 0.395767;
        weights[3]->coeffRef(13, 6) = -0.619921;
        weights[3]->coeffRef(14, 6) = 0.631042;
        weights[3]->coeffRef(15, 6) = 0.723709;
        weights[3]->coeffRef(16, 6) = 0.686525;
        weights[3]->coeffRef(17, 6) = -0.0558004;
        weights[3]->coeffRef(18, 6) = -0.427261;
        weights[3]->coeffRef(19, 6) = 0.655981;
        weights[3]->coeffRef(20, 6) = 0.291174;
        weights[3]->coeffRef(21, 6) = -0.693224;
        weights[3]->coeffRef(22, 6) = 0.558665;
        weights[3]->coeffRef(23, 6) = -0.189088;
        weights[3]->coeffRef(24, 6) = -0.789545;
        weights[3]->coeffRef(25, 6) = -0.755955;
        weights[3]->coeffRef(26, 6) = 0.699839;
        weights[3]->coeffRef(27, 6) = 0.964849;
        weights[3]->coeffRef(28, 6) = -0.400377;
        weights[3]->coeffRef(29, 6) = 0.811926;
        weights[3]->coeffRef(30, 6) = -0.497748;
        weights[3]->coeffRef(31, 6) = -0.682033;
        weights[3]->coeffRef(32, 6) = -0.671217;
        weights[3]->coeffRef(33, 6) = 0.340266;
        weights[3]->coeffRef(34, 6) = 0.670484;
        weights[3]->coeffRef(35, 6) = -0.795573;
        weights[3]->coeffRef(36, 6) = -0.372605;
        weights[3]->coeffRef(37, 6) = -0.365772;
        weights[3]->coeffRef(38, 6) = 0.84665;
        weights[3]->coeffRef(39, 6) = -0.556723;
        weights[3]->coeffRef(40, 6) = 0.788936;
        weights[3]->coeffRef(41, 6) = 0.965028;
        weights[3]->coeffRef(42, 6) = -0.689339;
        weights[3]->coeffRef(43, 6) = 0.180237;
        weights[3]->coeffRef(44, 6) = -0.662277;
        weights[3]->coeffRef(45, 6) = 0.95669;
        weights[3]->coeffRef(46, 6) = -0.0884332;
        weights[3]->coeffRef(47, 6) = -0.986225;
        weights[3]->coeffRef(48, 6) = -0.118802;
        weights[3]->coeffRef(49, 6) = 0.466243;
        weights[3]->coeffRef(50, 6) = 0.689426;
        weights[3]->coeffRef(0, 7) = -0.783767;
        weights[3]->coeffRef(1, 7) = 0.792826;
        weights[3]->coeffRef(2, 7) = 0.266905;
        weights[3]->coeffRef(3, 7) = 0.0319938;
        weights[3]->coeffRef(4, 7) = 0.984713;
        weights[3]->coeffRef(5, 7) = 0.496854;
        weights[3]->coeffRef(6, 7) = -0.285672;
        weights[3]->coeffRef(7, 7) = 0.955341;
        weights[3]->coeffRef(8, 7) = -0.918249;
        weights[3]->coeffRef(9, 7) = -0.488396;
        weights[3]->coeffRef(10, 7) = -0.551063;
        weights[3]->coeffRef(11, 7) = -0.599868;
        weights[3]->coeffRef(12, 7) = -0.147206;
        weights[3]->coeffRef(13, 7) = 0.807866;
        weights[3]->coeffRef(14, 7) = -0.957978;
        weights[3]->coeffRef(15, 7) = 0.0657232;
        weights[3]->coeffRef(16, 7) = -0.567004;
        weights[3]->coeffRef(17, 7) = -0.300782;
        weights[3]->coeffRef(18, 7) = -0.0805873;
        weights[3]->coeffRef(19, 7) = -0.0967406;
        weights[3]->coeffRef(20, 7) = -0.52484;
        weights[3]->coeffRef(21, 7) = -0.119096;
        weights[3]->coeffRef(22, 7) = 0.2092;
        weights[3]->coeffRef(23, 7) = 0.667816;
        weights[3]->coeffRef(24, 7) = 0.206407;
        weights[3]->coeffRef(25, 7) = 0.18598;
        weights[3]->coeffRef(26, 7) = -0.404236;
        weights[3]->coeffRef(27, 7) = 0.225322;
        weights[3]->coeffRef(28, 7) = -0.921151;
        weights[3]->coeffRef(29, 7) = -0.929751;
        weights[3]->coeffRef(30, 7) = -0.0795573;
        weights[3]->coeffRef(31, 7) = -0.733849;
        weights[3]->coeffRef(32, 7) = 0.860519;
        weights[3]->coeffRef(33, 7) = -0.814733;
        weights[3]->coeffRef(34, 7) = 0.301649;
        weights[3]->coeffRef(35, 7) = 0.857418;
        weights[3]->coeffRef(36, 7) = 0.670317;
        weights[3]->coeffRef(37, 7) = 0.992447;
        weights[3]->coeffRef(38, 7) = 0.799247;
        weights[3]->coeffRef(39, 7) = 0.738556;
        weights[3]->coeffRef(40, 7) = -0.487153;
        weights[3]->coeffRef(41, 7) = -0.692931;
        weights[3]->coeffRef(42, 7) = -0.817099;
        weights[3]->coeffRef(43, 7) = 0.359797;
        weights[3]->coeffRef(44, 7) = -0.894723;
        weights[3]->coeffRef(45, 7) = -0.755464;
        weights[3]->coeffRef(46, 7) = -0.564507;
        weights[3]->coeffRef(47, 7) = -0.47543;
        weights[3]->coeffRef(48, 7) = -0.0820138;
        weights[3]->coeffRef(49, 7) = 0.331272;
        weights[3]->coeffRef(50, 7) = 0.453568;
        weights[3]->coeffRef(0, 8) = 0.418786;
        weights[3]->coeffRef(1, 8) = -0.772993;
        weights[3]->coeffRef(2, 8) = -0.321705;
        weights[3]->coeffRef(3, 8) = 0.099345;
        weights[3]->coeffRef(4, 8) = 0.434307;
        weights[3]->coeffRef(5, 8) = 0.855295;
        weights[3]->coeffRef(6, 8) = 0.692583;
        weights[3]->coeffRef(7, 8) = -0.333973;
        weights[3]->coeffRef(8, 8) = 0.926862;
        weights[3]->coeffRef(9, 8) = 0.772103;
        weights[3]->coeffRef(10, 8) = 0.592373;
        weights[3]->coeffRef(11, 8) = -0.808257;
        weights[3]->coeffRef(12, 8) = 0.631132;
        weights[3]->coeffRef(13, 8) = 0.773706;
        weights[3]->coeffRef(14, 8) = 0.474328;
        weights[3]->coeffRef(15, 8) = 0.484606;
        weights[3]->coeffRef(16, 8) = 0.457959;
        weights[3]->coeffRef(17, 8) = 0.460959;
        weights[3]->coeffRef(18, 8) = 0.305586;
        weights[3]->coeffRef(19, 8) = 0.229753;
        weights[3]->coeffRef(20, 8) = 0.982772;
        weights[3]->coeffRef(21, 8) = 0.591914;
        weights[3]->coeffRef(22, 8) = 0.409338;
        weights[3]->coeffRef(23, 8) = 0.357721;
        weights[3]->coeffRef(24, 8) = 0.690599;
        weights[3]->coeffRef(25, 8) = 0.64535;
        weights[3]->coeffRef(26, 8) = 0.789028;
        weights[3]->coeffRef(27, 8) = -0.764747;
        weights[3]->coeffRef(28, 8) = -0.419326;
        weights[3]->coeffRef(29, 8) = 0.147239;
        weights[3]->coeffRef(30, 8) = 0.691977;
        weights[3]->coeffRef(31, 8) = -0.994955;
        weights[3]->coeffRef(32, 8) = 0.37474;
        weights[3]->coeffRef(33, 8) = -0.629326;
        weights[3]->coeffRef(34, 8) = 0.103714;
        weights[3]->coeffRef(35, 8) = -0.193306;
        weights[3]->coeffRef(36, 8) = -0.771753;
        weights[3]->coeffRef(37, 8) = -0.199161;
        weights[3]->coeffRef(38, 8) = 0.47533;
        weights[3]->coeffRef(39, 8) = -0.842281;
        weights[3]->coeffRef(40, 8) = -0.428755;
        weights[3]->coeffRef(41, 8) = 0.0563356;
        weights[3]->coeffRef(42, 8) = -0.659073;
        weights[3]->coeffRef(43, 8) = -0.796494;
        weights[3]->coeffRef(44, 8) = -0.168093;
        weights[3]->coeffRef(45, 8) = 0.811472;
        weights[3]->coeffRef(46, 8) = 0.686185;
        weights[3]->coeffRef(47, 8) = -0.70749;
        weights[3]->coeffRef(48, 8) = 0.277403;
        weights[3]->coeffRef(49, 8) = -0.00366594;
        weights[3]->coeffRef(50, 8) = 0.517294;
        weights[3]->coeffRef(0, 9) = 0.274874;
        weights[3]->coeffRef(1, 9) = -0.407755;
        weights[3]->coeffRef(2, 9) = -0.0707337;
        weights[3]->coeffRef(3, 9) = -0.373825;
        weights[3]->coeffRef(4, 9) = -0.732915;
        weights[3]->coeffRef(5, 9) = -0.429511;
        weights[3]->coeffRef(6, 9) = -0.596923;
        weights[3]->coeffRef(7, 9) = -0.498177;
        weights[3]->coeffRef(8, 9) = 0.145151;
        weights[3]->coeffRef(9, 9) = 0.530927;
        weights[3]->coeffRef(10, 9) = -0.818002;
        weights[3]->coeffRef(11, 9) = 0.151562;
        weights[3]->coeffRef(12, 9) = -0.0830759;
        weights[3]->coeffRef(13, 9) = -0.429021;
        weights[3]->coeffRef(14, 9) = -0.752405;
        weights[3]->coeffRef(15, 9) = 0.733706;
        weights[3]->coeffRef(16, 9) = -0.213921;
        weights[3]->coeffRef(17, 9) = 0.0717626;
        weights[3]->coeffRef(18, 9) = 0.197092;
        weights[3]->coeffRef(19, 9) = -0.0613367;
        weights[3]->coeffRef(20, 9) = 0.625204;
        weights[3]->coeffRef(21, 9) = -0.732598;
        weights[3]->coeffRef(22, 9) = 0.278461;
        weights[3]->coeffRef(23, 9) = 0.826464;
        weights[3]->coeffRef(24, 9) = 0.0947651;
        weights[3]->coeffRef(25, 9) = 0.113122;
        weights[3]->coeffRef(26, 9) = 0.529314;
        weights[3]->coeffRef(27, 9) = 0.375132;
        weights[3]->coeffRef(28, 9) = -0.6141;
        weights[3]->coeffRef(29, 9) = -0.489551;
        weights[3]->coeffRef(30, 9) = -0.105524;
        weights[3]->coeffRef(31, 9) = 0.632812;
        weights[3]->coeffRef(32, 9) = 0.100226;
        weights[3]->coeffRef(33, 9) = 0.821728;
        weights[3]->coeffRef(34, 9) = -0.737623;
        weights[3]->coeffRef(35, 9) = 0.37909;
        weights[3]->coeffRef(36, 9) = -0.619186;
        weights[3]->coeffRef(37, 9) = -0.35729;
        weights[3]->coeffRef(38, 9) = 0.867854;
        weights[3]->coeffRef(39, 9) = 0.512905;
        weights[3]->coeffRef(40, 9) = -0.817861;
        weights[3]->coeffRef(41, 9) = -0.893232;
        weights[3]->coeffRef(42, 9) = -0.292802;
        weights[3]->coeffRef(43, 9) = 0.0934116;
        weights[3]->coeffRef(44, 9) = -0.331588;
        weights[3]->coeffRef(45, 9) = -0.026247;
        weights[3]->coeffRef(46, 9) = -0.163241;
        weights[3]->coeffRef(47, 9) = 0.441247;
        weights[3]->coeffRef(48, 9) = -0.97939;
        weights[3]->coeffRef(49, 9) = -0.988992;
        weights[3]->coeffRef(50, 9) = -0.595211;

    }

    data in_dat;
    in_dat.push_back(new RowVector(2));
    in_dat.back()->coeffRef(0) = _v;
    in_dat.back()->coeffRef(1) = _w;
    // Lineas para evaluar datos con la red entrenada
    evaluar(*in_dat[0]);

    Scalar a = neuronLayers[5]->coeffRef(0, 0);


	perf_begin(_loop_perf);

	// only run controller if attitude changed
	vehicle_attitude_s att;

	if (_att_sub.update(&att)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		const float dt = math::constrain((att.timestamp - _last_run) * 1e-6f, 0.002f, 0.04f);
		_last_run = att.timestamp;

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(att.q);

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_rates_sub.copy(&angular_velocity);
		float rollspeed = angular_velocity.xyz[0];
		float pitchspeed = angular_velocity.xyz[1];
		float yawspeed = angular_velocity.xyz[2];

		if (_is_tailsitter) {
			/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
			 *
			 * Since the VTOL airframe is initialized as a multicopter we need to
			 * modify the estimated attitude for the fixed wing operation.
			 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
			 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
			 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
			 * Additionally, in order to get the correct sign of the pitch, we need to multiply
			 * the new x axis of the rotation matrix with -1
			 *
			 * original:			modified:
			 *
			 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
			 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
			 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
			 * */
			matrix::Dcmf R_adapted = R;		//modified rotation matrix

			/* move z to x */
			R_adapted(0, 0) = R(0, 2);
			R_adapted(1, 0) = R(1, 2);
			R_adapted(2, 0) = R(2, 2);

			/* move x to z */
			R_adapted(0, 2) = R(0, 0);
			R_adapted(1, 2) = R(1, 0);
			R_adapted(2, 2) = R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			R = R_adapted;

			/* lastly, roll- and yawspeed have to be swaped */
			float helper = rollspeed;
			rollspeed = -yawspeed;
			yawspeed = helper;
		}

		const matrix::Eulerf euler_angles(R);

		vehicle_attitude_setpoint_poll();

		// vehicle status update must be before the vehicle_control_mode_poll(), otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_control_mode_poll();
		vehicle_manual_poll();
		vehicle_land_detected_poll();

		// the position controller will not emit attitude setpoints in some modes
		// we need to make sure that this flag is reset
		_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

		bool wheel_control = false;

		// TODO: manual wheel_control on ground?
		if (_param_fw_w_en.get() && _att_sp.fw_control_yaw) {
			wheel_control = true;
		}

		/* lock integrator until control is started or for long intervals (> 20 ms) */
		bool lock_integrator = !_vcontrol_mode.flag_control_rates_enabled
				       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && ! _vehicle_status.in_transition_mode)
				       || (dt > 0.02f);

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		control_flaps(dt);

		/* decide if in stabilized or full manual control */
		if (_vcontrol_mode.flag_control_rates_enabled) {

			const float airspeed = get_airspeed_and_update_scaling();

			/* reset integrals where needed */
			if (_att_sp.roll_reset_integral) {
				_roll_ctrl.reset_integrator();
			}

			if (_att_sp.pitch_reset_integral) {
				_pitch_ctrl.reset_integrator();
			}

			if (_att_sp.yaw_reset_integral) {
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Reset integrators if the aircraft is on ground
			 * or a multicopter (but not transitioning VTOL)
			 */
			if (_landed
			    || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				&& !_vehicle_status.in_transition_mode)) {

				_roll_ctrl.reset_integrator();
				_pitch_ctrl.reset_integrator();
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Prepare data for attitude controllers */
			ECL_ControlData control_input{};
			control_input.roll = euler_angles.phi();
			control_input.pitch = euler_angles.theta();
			control_input.yaw = euler_angles.psi();
            _roll = euler_angles.phi();
            _pitch = euler_angles.theta();
            _yaw = euler_angles.psi();
			control_input.body_x_rate = rollspeed;
			control_input.body_y_rate = pitchspeed;
			control_input.body_z_rate = yawspeed;
			control_input.roll_setpoint = _att_sp.roll_body;
			control_input.pitch_setpoint = _att_sp.pitch_body;
			control_input.yaw_setpoint = _att_sp.yaw_body;
			control_input.airspeed_min = _param_fw_airspd_min.get();
			control_input.airspeed_max = _param_fw_airspd_max.get();
			control_input.airspeed = airspeed;
			control_input.scaler = _airspeed_scaling;
			control_input.lock_integrator = lock_integrator;

			if (wheel_control) {
				_local_pos_sub.update(&_local_pos);

				/* Use min airspeed to calculate ground speed scaling region.
				* Don't scale below gspd_scaling_trim
				*/
				float groundspeed = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
				float gspd_scaling_trim = (_param_fw_airspd_min.get() * 0.6f);
				float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

				control_input.groundspeed = groundspeed;
				control_input.groundspeed_scaler = groundspeed_scaler;
			}



			/* reset body angular rate limits on mode change */
			if ((_vcontrol_mode.flag_control_attitude_enabled != _flag_control_attitude_enabled_last) || params_updated) {
				if (_vcontrol_mode.flag_control_attitude_enabled
				    || _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					_roll_ctrl.set_max_rate(radians(_param_fw_r_rmax.get()));
					_pitch_ctrl.set_max_rate_pos(radians(_param_fw_p_rmax_pos.get()));
					_pitch_ctrl.set_max_rate_neg(radians(_param_fw_p_rmax_neg.get()));
					_yaw_ctrl.set_max_rate(radians(_param_fw_y_rmax.get()));

				} else {
					_roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
					_pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
					_pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
					_yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));
				}
			}

			_flag_control_attitude_enabled_last = _vcontrol_mode.flag_control_attitude_enabled;

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			float trim_roll = _param_trim_roll.get();
			float trim_pitch = _param_trim_pitch.get();
			float trim_yaw = _param_trim_yaw.get();

			if (airspeed < _param_fw_airspd_trim.get()) {
				trim_roll += gradual(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_r_vmin.get(),
						     0.0f);
				trim_pitch += gradual(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_p_vmin.get(),
						      0.0f);
				trim_yaw += gradual(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_y_vmin.get(),
						    0.0f);

			} else {
				trim_roll += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						     _param_fw_dtrim_r_vmax.get());
				trim_pitch += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						      _param_fw_dtrim_p_vmax.get());
				trim_yaw += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						    _param_fw_dtrim_y_vmax.get());
			}

			/* add trim increment if flaps are deployed  */
			trim_roll += _flaps_applied * _param_fw_dtrim_r_flps.get();
			trim_pitch += _flaps_applied * _param_fw_dtrim_p_flps.get();

			/* Run attitude controllers */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {
					_roll_ctrl.control_attitude(dt, control_input);
					_pitch_ctrl.control_attitude(dt, control_input);

					if (wheel_control) {
						_wheel_ctrl.control_attitude(dt, control_input);
						_yaw_ctrl.reset_integrator();

					} else {
						// runs last, because is depending on output of roll and pitch attitude
						_yaw_ctrl.control_attitude(dt, control_input);
						_wheel_ctrl.reset_integrator();
					}

					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_euler_rate(dt, control_input);
					_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

					if (!PX4_ISFINITE(roll_u)) {
						_roll_ctrl.reset_integrator();
					}

					float pitch_u = _pitch_ctrl.control_euler_rate(dt, control_input);
					_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

					if (!PX4_ISFINITE(pitch_u)) {
						_pitch_ctrl.reset_integrator();
					}

					float yaw_u = 0.0f;

					if (wheel_control) {
						yaw_u = _wheel_ctrl.control_bodyrate(dt, control_input);

					} else {
						yaw_u = _yaw_ctrl.control_euler_rate(dt, control_input);
					}

					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

					/* add in manual rudder control in manual modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						_actuators.control[actuator_controls_s::INDEX_YAW] += _manual_control_setpoint.r;
					}

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						_wheel_ctrl.reset_integrator();
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
                    _actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust_body[0])
                            && !_vehicle_status.engine_failure) ? _att_sp.thrust_body[0] : 0.0f;
                    //_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust_body[0])
                    //        && !_vehicle_status.engine_failure) ? 0.0f : 0.0f;

					/* scale effort by battery status */
					if (_param_fw_bat_scale_en.get() &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {

						if (_battery_status_sub.updated()) {
							battery_status_s battery_status{};

							if (_battery_status_sub.copy(&battery_status)) {
								if (battery_status.scale > 0.0f) {
									_battery_scale = battery_status.scale;
								}
							}
						}

						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_scale;
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_bodyrate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_bodyrate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

				_rates_sp.timestamp = hrt_absolute_time();

				_rate_sp_pub.publish(_rates_sp);

			} else {
				vehicle_rates_setpoint_poll();

				_roll_ctrl.set_bodyrate_setpoint(_rates_sp.roll);
				_yaw_ctrl.set_bodyrate_setpoint(_rates_sp.yaw);
				_pitch_ctrl.set_bodyrate_setpoint(_rates_sp.pitch);

				float roll_u = _roll_ctrl.control_bodyrate(dt, control_input);
				_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

				float pitch_u = _pitch_ctrl.control_bodyrate(dt, control_input);
				_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

				float yaw_u = _yaw_ctrl.control_bodyrate(dt, control_input);
				_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ?
						_rates_sp.thrust_body[0] : 0.0f;
			}

			rate_ctrl_status_s rate_ctrl_status{};
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.rollspeed_integ = _roll_ctrl.get_integrator();
			rate_ctrl_status.pitchspeed_integ = _pitch_ctrl.get_integrator();

			if (wheel_control) {
				rate_ctrl_status.additional_integ1 = _wheel_ctrl.get_integrator();

			} else {
				rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();
			}

			_rate_ctrl_status_pub.publish(rate_ctrl_status);
		}

		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		_actuators.control[actuator_controls_s::INDEX_YAW] += _param_fw_rll_to_yaw_ff.get()
				* constrain(_actuators.control[actuator_controls_s::INDEX_ROLL], -1.0f, 1.0f);

		_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
		_actuators.control[5] = _manual_control_setpoint.aux1;
		_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
		// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
		_actuators.control[7] = _manual_control_setpoint.aux3;

		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = att.timestamp;

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			_actuators_0_pub.publish(_actuators);
		}


        _local_pos_sub.update(&_local_pos);

        matrix::Dcmf Rotation_m;
                 Rotation_m(0, 0) = cosf(_pitch) * cosf(_yaw);
                 Rotation_m(0, 1) = -cosf(_roll) * sinf(_yaw) + sinf(_roll) * sinf(_pitch) * cosf(_yaw);
                 Rotation_m(0, 2) = sinf(_roll) * sinf(_yaw) + cosf(_roll) * sinf(_pitch) * cosf(_yaw);
                 Rotation_m(1, 0) = cosf(_pitch) * sinf(_yaw);
                 Rotation_m(1, 1) = cosf(_roll) * cosf(_yaw) + sinf(_roll) * sinf(_pitch) * sinf(_yaw);
                 Rotation_m(1, 2) = -sinf(_roll) * cosf(_yaw) + cosf(_roll) * sinf(_pitch) * sinf(_yaw);
                 Rotation_m(2, 0) = -sinf(_pitch);
                 Rotation_m(2, 1) = sinf(_roll) * cosf(_pitch);
                 Rotation_m(2, 2) = cosf(_roll) * cosf(_pitch);
                 matrix::Vector3f vel_NED = {_local_pos.vx, _local_pos.vy, _local_pos.vz};
                 matrix::Vector3f vel_body = Rotation_m.transpose() * vel_NED;
                 _v = vel_body(0);
                 _w = -vel_body(2);
                 //float angle_of_attack = atan2f(vel_body(2), vel_body(0));


        struct debug_vect_s dbg_vect;
        strncpy(dbg_vect.name, "vec", 10);

        //_contador += dt;
        dbg_vect.x = _v;
        dbg_vect.y = _w;
        dbg_vect.z = a;
        orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);
        uint64_t timestamp_us = hrt_absolute_time();
        dbg_vect.timestamp = timestamp_us;
        orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);
	}



	perf_end(_loop_perf);
}

void FixedwingAttitudeControl::control_flaps(const float dt)
{
	/* default flaps to center */
	float flap_control = 0.0f;

	/* map flaps by default to manual if valid */
	if (PX4_ISFINITE(_manual_control_setpoint.flaps) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_param_fw_flaps_scl.get()) > 0.01f) {
		flap_control = 0.5f * (_manual_control_setpoint.flaps + 1.0f) * _param_fw_flaps_scl.get();

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_param_fw_flaps_scl.get()) > 0.01f) {

		switch (_att_sp.apply_flaps) {
		case vehicle_attitude_setpoint_s::FLAPS_OFF:
			flap_control = 0.0f; // no flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_LAND:
			flap_control = 1.0f * _param_fw_flaps_scl.get() * _param_fw_flaps_lnd_scl.get();
			break;

		case vehicle_attitude_setpoint_s::FLAPS_TAKEOFF:
			flap_control = 1.0f * _param_fw_flaps_scl.get() * _param_fw_flaps_to_scl.get();
			break;
		}
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaps_applied - flap_control) > 0.01f) {
		_flaps_applied += (_flaps_applied - flap_control) < 0 ? dt : -dt;

	} else {
		_flaps_applied = flap_control;
	}

	/* default flaperon to center */
	float flaperon_control = 0.0f;

	/* map flaperons by default to manual if valid */
	if (PX4_ISFINITE(_manual_control_setpoint.aux2) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_param_fw_flaperon_scl.get()) > 0.01f) {

		flaperon_control = 0.5f * (_manual_control_setpoint.aux2 + 1.0f) * _param_fw_flaperon_scl.get();

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_param_fw_flaperon_scl.get()) > 0.01f) {

		if (_att_sp.apply_flaps == vehicle_attitude_setpoint_s::FLAPS_LAND) {
			flaperon_control = _param_fw_flaperon_scl.get();

		} else {
			flaperon_control = 0.0f;
		}
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaperons_applied - flaperon_control) > 0.01f) {
		_flaperons_applied += (_flaperons_applied - flaperon_control) < 0 ? dt : -dt;

	} else {
		_flaperons_applied = flaperon_control;
	}
}

int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


bool FixedwingAttitudeControl::NeuralNetwork(std::vector<uint> topolog, Scalar learningRat)
{
    topology = topolog;
    learningRate = learningRat;


    for (uint i = 0; i < topolog.size(); i++) {
        // initialze neuron layers
        if (i == topolog.size() - 1)
            neuronLayers.push_back(new RowVector(topolog[i]));
        else
            neuronLayers.push_back(new RowVector(topolog[i] + 1));

        // initialize cache and delta vectors
        cacheLayers.push_back(new RowVector(neuronLayers.back()->size()));
        deltas.push_back(new RowVector(neuronLayers.size()));

        // vector.back() gives the handle to recently added element
        // coeffRef gives the reference of value at that place
        // (using this as we are using pointers here)
        if (i != topolog.size() - 1) {
            neuronLayers.back()->coeffRef(topolog[i]) = 1.0f;
            cacheLayers.back()->coeffRef(topolog[i]) = 1.0f;
        }

        // initialze weights matrix
        if (i > 0) {
            if (i != topolog.size() - 1) {
                weights.push_back(new Matrix(topolog[i - 1] + 1, topolog[i] + 1));
                weights.back()->setRandom();
                weights.back()->col(topolog[i]).setZero();
                weights.back()->coeffRef(topolog[i - 1], topolog[i]) = 1.0f;
            }
            else {
                weights.push_back(new Matrix(topolog[i - 1] + 1, topolog[i]));
                weights.back()->setRandom();
            }
        }

    }

/*
    // Para leer los weights
     std::ifstream file("weights");
     std::string line;
     while(!file.is_open()){}

     if (file.is_open()) {
         _contador = 1.f;
         for (uint i = 0; i < topolog.size() - 1; i++) {
             // in this loop we are iterating over the different layers (from first hidden to output layer)
             // if this layer is the output layer, there is no bias neuron there, number of neurons specified = number of cols
             // if this layer not the output layer, there is a bias neuron and number of neurons specified = number of cols -1
             if (i != topolog.size() - 1) {
                 for (uint c = 0; c < weights[i]->cols() - 1; c++) {
                     for (uint r = 0; r < weights[i]->rows(); r++) {
                         getline(file, line, '\n');
                         weights[i]->coeffRef(r, c) = Scalar(std::stof(&line[0]));
                         //std::cout << "weights[0] : " << Scalar(std::stof(&line[0])) << std::endl;
                     }
                 }
             }
             else {
                 for (uint c = 0; c < weights[i]->cols(); c++) {
                     for (uint r = 0; r < weights[i]->rows(); r++) {
                         getline(file, line, '\n');
                         weights[i]->coeffRef(r, c) = Scalar(std::stof(&line[0]));
                         //std::cout << "weights[0] : " << Scalar(std::stof(&line[0])) << std::endl;
                     }
                 }
             }
         }
     }
*/
     return false;
}


void FixedwingAttitudeControl::evaluar(RowVector& input){
    // set the input to input layer
    // block returns a part of the given vector or matrix
    // block takes 4 arguments : startRow, startCol, blockRows, blockCols
    //std::cout << "Neuronas primera capa : " << cacheLayers[0]->size() << std::endl;
    cacheLayers.front()->block(0, 0, 1, cacheLayers[0]->size()-1 ) = input;
    //std::cout << "Valores 1ra capa : " << cacheLayers.front()->block(0, 0, 1, 3) << std::endl;

    // propagate the data forawrd
    for (uint i = 1; i < topology.size(); i++) {
        // already explained above
    (*cacheLayers[i]) = (*cacheLayers[i - 1]) * (*weights[i - 1]);
        //std::cout << "Valores " << i+1 << " capa : " << cacheLayers[i]->block(0, 0, 1, cacheLayers[i]->size()) << std::endl;
    }

    // apply the activation function to your network
    // unaryExpr applies the given function to all elements of CURRENT_LAYER
    for (uint i = 1; i < topology.size(); i++) {
        neuronLayers[i]->block(0, 0, 1, topology[i]) = cacheLayers[i]->block(0, 0, 1, topology[i]).unaryExpr([](Scalar x) { return 1.f*x; });
        //std::cout << "Salida : " << neuronLayers[i]->block(0, 0, 1, topology[i]) << std::endl;
    }
}





int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}
