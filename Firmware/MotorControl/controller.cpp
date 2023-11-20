
#include "odrive_main.h"
#include <algorithm>

#include <algorithm>

const float leg_gear_reduction = 3;
const float pi = 3.141592653589f;
const float tau = pi*2.0f;
const float leg_angle_1 = -130*pi/180;
const float upper_leg_length = 125;
const float lower_leg_length = 91.3685f;
//float wheel_side_distance = odrv.wheel_side_distance_;
const float wheel_side_distance = 178.0f;

Controller::Controller(Config_t& config) :
    config_(config)
{
    update_filter_gains();
}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
}

void Controller::set_error(Error error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_CONTROLLER_FAILED;
}

//--------------------------------
// Command Handling
//--------------------------------


bool Controller::select_encoder(size_t encoder_num) {
    if (encoder_num < AXIS_COUNT) {
        Axis* ax = axes[encoder_num];
        pos_estimate_circular_src_ = &ax->encoder_.pos_circular_;
        pos_wrap_src_ = &config_.circular_setpoint_range;
        pos_estimate_linear_src_ = &ax->encoder_.pos_estimate_;
        pos_estimate_valid_src_ = &ax->encoder_.pos_estimate_valid_;
        vel_estimate_src_ = &ax->encoder_.vel_estimate_;
        vel_estimate_valid_src_ = &ax->encoder_.vel_estimate_valid_;
        return true;
    } else {
        return set_error(Controller::ERROR_INVALID_LOAD_ENCODER), false;
    }
}

void Controller::move_to_pos(float goal_point) {
    axis_->trap_traj_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_traj_.config_.vel_limit,
                                 axis_->trap_traj_.config_.accel_limit,
                                 axis_->trap_traj_.config_.decel_limit);
    axis_->trap_traj_.t_ = 0.0f;
    trajectory_done_ = false;
}

void Controller::move_incremental(float displacement, bool from_input_pos = true){
    if(from_input_pos){
        input_pos_ += displacement;
    } else{
        input_pos_ = pos_setpoint_ + displacement;
    }

    input_pos_updated();
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (axis_->error_ == Axis::ERROR_NONE) {
        config_.anticogging.calib_anticogging = true;
    }
}

float Controller::get_anticogging_map(int32_t index) {
    return config_.anticogging.cogging_map[index];
}


/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    float pos_err = input_pos_ - pos_estimate;
    if (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold / (float)axis_->encoder_.config_.cpr &&
        std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold / (float)axis_->encoder_.config_.cpr) {
        config_.anticogging.cogging_map[std::clamp<uint32_t>(config_.anticogging.index++, 0, 3600)] = vel_integrator_torque_;
    }
    if (config_.anticogging.index < 3600) {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = config_.anticogging.index * axis_->encoder_.getCoggingRatio();
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    } else {
        config_.anticogging.index = 0;
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = 0.0f;  // Send the motor home
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        anticogging_valid_ = true;
        config_.anticogging.calib_anticogging = false;
        return true;
    }
}

void Controller::update_filter_gains() {
    float bandwidth = std::min(config_.input_filter_bandwidth, 0.25f * current_meas_hz);
    input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

float sq(float x) {
	return x*x;
}

float lerp(float a, float b, float t)
{
	return a + (b-a)*t;
}

void ODrive::set_side_angle(float value) {
    uint32_t current_counter = axes[0]->loop_counter_;
    float new_side_angle_d = (value-side_angle_) / (current_counter-last_side_angle_counter_) * 8000.0f;

    // target_gain_d sometimes causes oscillation, where it flips almost every jetson frame
    // the sign of the velocity. We decrease the value when the sign flips here, to prevent that.
    // Kind of a hack, but it seems to fix the oscillation.
    /*if ((new_side_angle_d > 0) != (side_angle_d_ > 0))
        side_angle_d_ = new_side_angle_d * 0.01f;
    else
        side_angle_d_ = new_side_angle_d;*/

    side_angle_d_ = lerp(
			new_side_angle_d,
			side_angle_d_,
			side_balance_error_d_smooth_factor_);

    side_angle_ = value;
    last_side_angle_counter_ = current_counter;
}

void Controller::handle_side_balancing() {
    float x_angle_hip_angle_l = odrv.x_angle_hip_angle_l_;
    float x_angle_hip_angle_r = odrv.x_angle_hip_angle_r_;
    float leg_pos_l =  (axes[1]->encoder_.pos_estimate_+axes[1]->motor_.leg_base_angle_);
    float leg_pos_r = -(axes[0]->encoder_.pos_estimate_+axes[0]->motor_.leg_base_angle_);
    float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
    float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;
    float leg_height_upper_l = our_arm_cos_f32(x_angle_hip_angle_l) * upper_leg_length;
    float leg_height_upper_r = our_arm_cos_f32(x_angle_hip_angle_r) * upper_leg_length;
    float leg_height_lower_l = our_arm_cos_f32(x_angle_hip_angle_l + leg_angle_l) * lower_leg_length;
    float leg_height_lower_r = our_arm_cos_f32(x_angle_hip_angle_r + leg_angle_r) * lower_leg_length;
    float leg_height_l = leg_height_upper_l + leg_height_lower_l;
    float leg_height_r = leg_height_upper_r + leg_height_lower_r;

    // Skip atan for performance reasons (argument should be small enough)
    //float side_angle_motor = fast_atan2((leg_height_r-leg_height_l)/wheel_side_distance, 1);
    float side_angle_motor = (leg_height_r-leg_height_l)/wheel_side_distance;

    float side_angle_motor_dl = -lower_leg_length*wheel_side_distance*(tau / leg_gear_reduction)*our_arm_sin_f32(x_angle_hip_angle_l + leg_angle_l) / (sq(leg_height_upper_l+leg_height_lower_l-leg_height_r)+sq(wheel_side_distance));
    float side_angle_motor_dr = -lower_leg_length*wheel_side_distance*(tau / leg_gear_reduction)*our_arm_sin_f32(x_angle_hip_angle_r + leg_angle_r) / (sq(leg_height_upper_r+leg_height_lower_r-leg_height_l)+sq(wheel_side_distance));

    float leg_vel_r = axes[0]->encoder_.vel_estimate_; // TODO: Missing minus sign?
    float leg_vel_l = axes[1]->encoder_.vel_estimate_;

    float side_angle_motor_d = side_angle_motor_dr*leg_vel_r +
                               side_angle_motor_dl*leg_vel_l;

    float average_leg_pos = (leg_pos_l+leg_pos_r)*0.5f;

	float average_leg_pos_dl = 0.5f;
    float average_leg_pos_dr = -0.5f;
    float average_leg_pos_d = average_leg_pos_dr*leg_vel_r + average_leg_pos_dl*leg_vel_l;

	float side_angle_motor_force;
    if (odrv.side_balance_mode_ == 0) {
	    side_angle_motor_force = 
			    (odrv.side_angle_motor_target_  -side_angle_motor  ) * odrv.side_gain_p_ +
			    (odrv.side_angle_motor_target_d_-side_angle_motor_d) * odrv.side_gain_d_;
    } else {
        side_angle_motor_force  = (odrv.side_angle_target_-odrv.side_angle_  ) * odrv.target_gain_p_;
        side_angle_motor_force += (0                      -side_angle_d_) * odrv.target_gain_d_;
    }

    if (side_angle_motor_force_limit_current < odrv.side_angle_motor_force_limit_) {
    	const float side_angle_motor_force_limit_time = 1;
        side_angle_motor_force_limit_current +=
                odrv.side_angle_motor_force_limit_ *
                (1/8000.0f) /
                side_angle_motor_force_limit_time;
    }
    if (side_angle_motor_force_limit_current > odrv.side_angle_motor_force_limit_) {
        side_angle_motor_force_limit_current = odrv.side_angle_motor_force_limit_;
    }
    side_angle_motor_force = std::clamp(side_angle_motor_force,
            -side_angle_motor_force_limit_current,
             side_angle_motor_force_limit_current);
    //odrv.side_angle_motor_force_ = side_angle_motor_force;

	odrv.average_leg_pos_integrator_ += (odrv.average_leg_pos_target_-average_leg_pos) * odrv.average_gain_i_ * (1/8000.0f);
    float average_leg_pos_force =
			odrv.average_leg_pos_integrator_ +
			(odrv.average_leg_pos_target_-average_leg_pos  ) * odrv.average_gain_p_ +
			(0                           -average_leg_pos_d) * odrv.average_gain_d_;
    //odrv.average_leg_pos_force_ = average_leg_pos_force;

	// multiplying by jacobian transpose to get the force in motor space
    float force_l = side_angle_motor_force * side_angle_motor_dl + average_leg_pos_force * average_leg_pos_dl;
    float force_r = side_angle_motor_force * side_angle_motor_dr + average_leg_pos_force * average_leg_pos_dr;
    
    // calculate estimate of acceleration
    //float smooth = odrv.acc_smooth_factor_;
    const float smooth = 0.999f;
    static float last_leg_vel_r;
    static float acc_r_;
	acc_r_ = acc_r_ * smooth + (leg_vel_r-last_leg_vel_r) * (1-smooth);
    last_leg_vel_r = leg_vel_r;
    static float last_leg_vel_l;
    static float acc_l_;
	acc_l_ = acc_l_ * smooth + (leg_vel_l-last_leg_vel_l) * (1-smooth);
    last_leg_vel_l = leg_vel_l;

    if (force_r > 0) {
        float acc_current = odrv.acc_current_gain_ * acc_r_;
        if (force_r+acc_current < 0 && leg_vel_r < 0)
            force_r = 0;
        else
            force_r += acc_current;

        if (-leg_vel_r < odrv.side_balance_down_limit_threshold_)
		    force_r += (-leg_vel_r-odrv.side_balance_down_limit_threshold_) * odrv.side_balance_down_limit_factor_;
    }
    if (force_l < 0) {
        float acc_current = odrv.acc_current_gain_ * acc_l_;
        if (force_l+acc_current > 0 && leg_vel_l > 0)
            force_l = 0;
        else
            force_l += acc_current;

        if (leg_vel_l < odrv.side_balance_down_limit_threshold_)
		    force_l -= (leg_vel_l-odrv.side_balance_down_limit_threshold_) * odrv.side_balance_down_limit_factor_;
    }
    
	if (x_angle_hip_angle_l+leg_angle_l > odrv.side_balance_max_vertical_angle_)
		force_l += (x_angle_hip_angle_l+leg_angle_l - odrv.side_balance_max_vertical_angle_) * odrv.side_balance_max_vertical_angle_gain_;
	if (x_angle_hip_angle_r+leg_angle_r > odrv.side_balance_max_vertical_angle_)
		force_r -= (x_angle_hip_angle_r+leg_angle_r - odrv.side_balance_max_vertical_angle_) * odrv.side_balance_max_vertical_angle_gain_;

	// Stay within leg limits
    const float stand_control_max_leg_angle = 0.97f;
	if (force_l > 0 && leg_pos_l > stand_control_max_leg_angle)
		force_l = 0;

	if (force_r < 0 && leg_pos_r > stand_control_max_leg_angle)
		force_r = 0;

    // Torque limiting
    float Tlim = axis_->motor_.config_.current_lim;
    force_l = std::clamp(force_l, -Tlim, Tlim);
    force_r = std::clamp(force_r, -Tlim, Tlim);

	torque_setpoint_ = force_r;
	axes[1]->controller_.torque_setpoint_ = force_l;
}

void Controller::handle_landing_mode() {
    float leg_pos;
	if (axis_->axis_num_ == 1)
        leg_pos =  (axes[1]->encoder_.pos_estimate_+axes[1]->motor_.leg_base_angle_);
    else
        leg_pos = -(axes[0]->encoder_.pos_estimate_+axes[0]->motor_.leg_base_angle_);

    float leg_pos_target;
	if (axis_->axis_num_ == 1)
        leg_pos_target =  (pos_setpoint_+axes[1]->motor_.leg_base_angle_);
    else
        leg_pos_target = -(pos_setpoint_+axes[0]->motor_.leg_base_angle_);
        
    float leg_vel;
	if (axis_->axis_num_ == 1)
        leg_vel =  axes[1]->encoder_.vel_estimate_;
    else
        leg_vel = -axes[0]->encoder_.vel_estimate_;

    if (odrv.l_do_straightening_) {
        float leg_angle        = (1 - leg_pos)        * (tau / leg_gear_reduction) + leg_angle_1;
        float leg_angle_target = (1 - leg_pos_target) * (tau / leg_gear_reduction) + leg_angle_1;
        float leg_angle_vel    = -leg_vel             * (tau / leg_gear_reduction);

        float l_base_angle = odrv.l_base_angle_;
        leg_pos        = -our_arm_cos_f32(l_base_angle + leg_angle);
        leg_pos_target = -our_arm_cos_f32(l_base_angle + leg_angle_target);
        leg_vel        =  our_arm_sin_f32(l_base_angle + leg_angle) * leg_angle_vel;
    }

    float pos_err = leg_pos_target - leg_pos;
    bool stop_landing = false;
    if (pos_err < 0.005f) {
        stop_landing = true;
        pos_err = 0.005f; // prevent division by zero below
        l_frame_counter_ = -10;
    }

    l_max_vel_ = std::max(l_max_vel_, 2.0f * pos_err / odrv.l_max_time_);
    if (odrv.l_do_max_vel_) {
        l_max_vel_ = std::max(l_max_vel_, leg_vel);
    }

    l_vel_target_delta_ -= 0.5f * sq(l_vel_target_) / pos_err * CURRENT_MEAS_PERIOD;
    l_vel_target_ = l_max_vel_ + l_vel_target_delta_;
    if (l_vel_target_ < 0) {
        stop_landing = true;
        l_frame_counter_ = -20;
    }

    float vel_error = l_vel_target_ - leg_vel;
    float leg_current = vel_error * odrv.l_vel_gain_ + l_vel_integrator_;
    l_vel_integrator_ += vel_error * odrv.l_vel_integrator_gain_ * current_meas_period;

    // PD Controller to synchronize left and right leg
    /*float sync_err_pos = leg_pos_l - leg_pos_r;
    float sync_err_vel = leg_vel_l - leg_vel_r;
    leg_current_l -= sync_err_pos*odrv.l_sync_gain_pos_ + sync_err_vel*odrv.l_sync_gain_vel_;
    leg_current_r += sync_err_pos*odrv.l_sync_gain_pos_ + sync_err_vel*odrv.l_sync_gain_vel_;*/

    l_frame_counter_++;
    if (l_frame_counter_ < odrv.l_initial_measure_time_frames_) {
        // In the first few frames of landing, the leg is still accelerating and we don't yet
        // know the maximum velocity. This means the velocity is usually much lower than l_max_vel
        // (which has a minimum value due to l_max_time). This causes a quite large positive current,
        // which we don't want. It would prevent solid grip of the wheels with the ground.
        leg_current = std::min(leg_current, odrv.l_initial_measure_current_);
        l_vel_integrator_ = 0;
    }

    //float initial_vel_target = l_initial_vel_ + l_frame_counter_*odrv.l_initial_expected_accel_per_frame_;
    //float initial_leg_current = (initial_vel_target-leg_vel) * odrv.l_initial_accel_gain_;
    //leg_current = std::min(leg_current, initial_leg_current);

    // Torque limiting
    float Tlim = axis_->motor_.config_.current_lim;
    leg_current = std::clamp(leg_current, -Tlim, Tlim);



    if (stop_landing) {
        axes[0]->motor_.trigger_stop_landing_ = true;
        axes[1]->motor_.trigger_stop_landing_ = true;
        leg_current = 0;
        odrv.enable_landing_mode_ = false;
    }

	if (axis_->axis_num_ == 1)
	    torque_setpoint_ =  leg_current;
    else
        torque_setpoint_ = -leg_current;
}

bool Controller::update(float* torque_setpoint_output) {

    if (odrv.enable_side_balance_) {
        if (std::abs(axis_->encoder_.vel_estimate_) > config_.vel_limit) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
        if (axes[0]->current_state_== Axis::AXIS_STATE_CLOSED_LOOP_CONTROL &&
			axes[1]->current_state_== Axis::AXIS_STATE_CLOSED_LOOP_CONTROL) {

	        if (axis_->axis_num_ == 0) {
			    handle_side_balancing();
			    //osSignalSet(axes[1]->thread_id_, Axis::M_SIGNAL_TORQUE_TARGET_DONE_AXIS_0);
            } /*else {
			    if (osSignalWait(Axis::M_SIGNAL_TORQUE_TARGET_DONE_AXIS_0, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
	                set_error(ERROR_INVALID_MIRROR_AXIS);
				    return false;
                }
            }*/
        } else {
            torque_setpoint_ = 0;
			odrv.average_leg_pos_integrator_ = -6;
        }
		if (torque_setpoint_output) *torque_setpoint_output = torque_setpoint_;
		return true;
    }
    odrv.average_leg_pos_integrator_ = -6;
    
    if (enable_landing_mode_) {
        if (axes[0]->current_state_== Axis::AXIS_STATE_CLOSED_LOOP_CONTROL &&
			axes[1]->current_state_== Axis::AXIS_STATE_CLOSED_LOOP_CONTROL) {
            handle_landing_mode();
        } else {
            torque_setpoint_ = 0;
        }
        if (torque_setpoint_output) *torque_setpoint_output = torque_setpoint_;
        return true;
    }

    float* pos_estimate_linear = (pos_estimate_valid_src_ && *pos_estimate_valid_src_)
            ? pos_estimate_linear_src_ : nullptr;
    float* pos_estimate_circular = (pos_estimate_valid_src_ && *pos_estimate_valid_src_)
            ? pos_estimate_circular_src_ : nullptr;
    float* vel_estimate_src = (vel_estimate_valid_src_ && *vel_estimate_valid_src_)
            ? vel_estimate_src_ : nullptr;

    // Calib_anticogging is only true when calibration is occurring, so we can't block anticogging_pos
    float anticogging_pos = axis_->encoder_.pos_estimate_ / axis_->encoder_.getCoggingRatio();
    if (config_.anticogging.calib_anticogging) {
        if (!axis_->encoder_.pos_estimate_valid_ || !axis_->encoder_.vel_estimate_valid_) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(axis_->encoder_.pos_estimate_, axis_->encoder_.vel_estimate_);
    }

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (config_.circular_setpoints) {
        // Keep pos setpoint from drifting
        input_pos_ = fmodf_pos(input_pos_, config_.circular_setpoint_range);
    }

    // Update inputs
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;
        } break;
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_MIRROR: {
            if (config_.axis_to_mirror < AXIS_COUNT) {
                pos_setpoint_ = axes[config_.axis_to_mirror]->encoder_.pos_estimate_ * config_.mirror_ratio;
                vel_setpoint_ = axes[config_.axis_to_mirror]->encoder_.vel_estimate_ * config_.mirror_ratio;
            } else {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;
        // case INPUT_MODE_MIX_CHANNELS: {
        //     // NOT YET IMPLEMENTED
        // } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
                pos_setpoint_ = input_pos_;
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += current_meas_period;
            }
            anticogging_pos = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        float pos_err;

        if (config_.circular_setpoints) {
            if(!pos_estimate_circular) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap_src_);
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_circular;
            pos_err = wrap_pm(pos_err, 0.5f * *pos_wrap_src_);
        } else {
            if(!pos_estimate_linear) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            pos_err = pos_setpoint_ - *pos_estimate_linear;
        }

        vel_des += config_.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate_src) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float effective_flux = axis_->motor_.current_control_.acim_rotor_flux;
        float minflux = axis_->motor_.config_.acim_gain_min_flux;
        if (fabsf(effective_flux) < minflux)
            effective_flux = std::copysignf(minflux, effective_flux);
        vel_gain /= effective_flux;
        vel_integrator_gain /= effective_flux;
        // TODO: also scale the integral value which is also changing units.
        // (or again just do control in torque units)
    }

    // Velocity control
    float torque = torque_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
        torque += config_.anticogging.cogging_map[std::clamp(mod((int)anticogging_pos, 3600), 0, 3600)];
    }

    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate_src;
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;
    }

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_current_mode_vel_limit) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate_src, vel_gain, torque);
    }

    // Torque limiting
    bool limited = false;
    float Tlim = axis_->motor_.max_available_torque();
    if (torque > Tlim) {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) {
        limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
    }

    if (torque_setpoint_output) *torque_setpoint_output = torque;
    return true;
}

void Controller::set_input_pos(float value) {
    input_pos_ = value;
    input_pos_updated();
}
