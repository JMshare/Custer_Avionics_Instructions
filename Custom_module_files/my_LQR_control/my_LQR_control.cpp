/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

  /**
 * @file my_LQR_control.cpp
 *
 * my_LQR_control module
 *
 * @author Juraj Mihalik <jm1e16@soton.ac.uk>
 */

#include "my_LQR_control.hpp"


int My_LQR_control::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description
This implements the LQR controller. 
So far only attitude control.

### Implementation
The control loop polls on the sensors combined topic.

### Examples
To load S500 quadcopter gains:
$ my_LQR_control start -p 1
To load Custer gains:
$ my_LQR_control start -p 2
To load Custer HITL gains:
$ my_LQR_control start -p 3

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("my_LQR_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional flag", true);
        PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Vehicle ID.\n\t\tQuadS500: 1\n\t\tCuster: 2", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

int My_LQR_control::print_status()
{
        PX4_INFO("Running in mode %d", vehicle_id);
        // TODO: print additional runtime information about the state of the module

        return 0;
}

int My_LQR_control::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("my_LQR_control",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_ATTITUDE_CONTROL,
                                      4000, // stack size
                                      (px4_main_t)&run_trampoline,
                                      (char *const *)argv);

        if (_task_id < 0) {
                _task_id = -1;
                return -errno;
        }

        return 0;
}

My_LQR_control *My_LQR_control::instantiate(int argc, char *argv[])
{
        int arg_param = 0;
        bool arg_flag = false;
        bool error_flag = false;

        int myoptind = 1;
        int ch;
        const char *myoptarg = nullptr;

        // parse CLI arguments
        while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
                switch (ch) {
                case 'p':
                        arg_param = (int)strtol(myoptarg, nullptr, 10);
                        break;

                case 'f':
                        arg_flag = true;
                        break;

                case '?':
                        error_flag = true;
                        break;

                default:
                        PX4_WARN("unrecognized flag");
                        error_flag = true;
                        break;
                }
        }

        if (error_flag) {
                return nullptr;
        }

        My_LQR_control *instance = new My_LQR_control(arg_param, arg_flag);

        if (instance == nullptr) {
                PX4_ERR("alloc failed");
        }

        return instance;
}

My_LQR_control::My_LQR_control(int arg_param, bool arg_flag)
        : ModuleParams(nullptr), _loop_perf(perf_alloc(PC_ELAPSED, "my_LQR_control"))
{
    vehicle_id = arg_param;
}

int My_LQR_control::custom_command(int argc, char *argv[])
{
        /*
        if (!is_running()) {
                print_usage("not running");
                return 1;
        }

        // additional custom commands can be handled like this:
        if (!strcmp(argv[0], "do-something")) {
                get_instance()->do_something();
                return 0;
        }
         */

        return print_usage("unknown command");
}



int My_LQR_control::parameter_update_poll()
{
    bool parameter_update_updated;
    orb_check(parameter_update_sub, &parameter_update_updated);
    if(parameter_update_updated){
        orb_copy(ORB_ID(parameter_update), parameter_update_sub, &parameter_update);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::vehicle_attitude_poll(){
    orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &vehicle_attitude);
    return PX4_OK;
}

int My_LQR_control::vehicle_local_position_poll(){
    bool vehicle_local_position_updated;
    orb_check(vehicle_local_position_sub, &vehicle_local_position_updated);
    if(vehicle_local_position_updated){
        orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &vehicle_local_position);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::vehicle_local_position_setpoint_poll(){
    bool vehicle_local_position_setpoint_updated;
    orb_check(vehicle_local_position_setpoint_sub, &vehicle_local_position_setpoint_updated);
    if(vehicle_local_position_setpoint_updated){
        orb_copy(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_setpoint_sub, &vehicle_local_position_setpoint);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::manual_control_setpoint_poll(){
    bool manual_control_setpoint_updated;
    orb_check(manual_control_setpoint_sub, &manual_control_setpoint_updated);
    if(manual_control_setpoint_updated){
        orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual_control_setpoint);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::rc_channels_poll(){
    bool rc_channels_updated;
    int ret = PX4_ERROR;
    orb_check(rc_channels_sub, &rc_channels_updated);
    if(rc_channels_updated){
        orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_channels);
        ret = PX4_OK;
    }
    rc_loss_failsafe();
    return ret;
}

int My_LQR_control::actuator_controls_virtual_poll(){
    bool actuator_controls_virtual_updated;
    orb_check(actuator_controls_virtual_sub, &actuator_controls_virtual_updated);
    if(actuator_controls_virtual_updated){
        orb_copy(actuator_controls_virtual_id, actuator_controls_virtual_sub, &actuator_controls_virtual);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::home_position_poll(){
    bool home_position_updated;
    orb_check(home_position_sub, &home_position_updated);
    if(home_position_updated){
        orb_copy(ORB_ID(home_position), home_position_sub, &home_position);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::airspeed_poll(){
    bool airspeed_updated;
    orb_check(airspeed_sub, &airspeed_updated);
    if(airspeed_updated){
        orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed);
        return PX4_OK;
    }
    return PX4_ERROR;
}

int My_LQR_control::actuator_controls_publish(){
    bound_controls(); // bounds pqr control to (-1,1) and thrust to (0,1)

    for(int i=0; i < 4; i++){
        actuator_controls_0.control[i] = cf(i,0);
    }
    actuator_controls_0.control[4] = 0.0f;
    actuator_controls_0.control[5] = 0.0f;
    actuator_controls_0.control[6] = 0.0f;
    actuator_controls_0.control[7] = -1.0f;
    actuator_controls_0.timestamp = hrt_absolute_time();
    actuator_controls_0.timestamp_sample = vehicle_attitude.timestamp;

    orb_publish(ORB_ID(actuator_controls_0), actuator_controls_0_pub, &actuator_controls_0);

    for(int i=0; i < 8; i++){
        actuator_controls_1.control[i] = uf(i,0);
    }
    actuator_controls_1.timestamp = hrt_absolute_time();
    actuator_controls_1.timestamp_sample = vehicle_attitude.timestamp;

    orb_publish(ORB_ID(actuator_controls_1), actuator_controls_1_pub, &actuator_controls_1);
    
    return PX4_OK;
}

int My_LQR_control::angular_rates_filtered_publish(){
    angular_rates_filtered.rollspeed = omg_filtered(0);
    angular_rates_filtered.pitchspeed = omg_filtered(1);
    angular_rates_filtered.yawspeed = omg_filtered(2);
    angular_rates_filtered.roll = eps_filtered(0);
    angular_rates_filtered.pitch = eps_filtered(1);
    angular_rates_filtered.yaw = eps_filtered(2);
    angular_rates_filtered.rc_roll = RC_filtered(0);
    angular_rates_filtered.rc_pitch = RC_filtered(1);
    angular_rates_filtered.rc_yaw = RC_filtered(2);
    angular_rates_filtered.loop_update_freqn = loop_update_freqn;
    angular_rates_filtered.cutoff_freqn_omg = cutoff_freqn_omg;
    angular_rates_filtered.filter_status_omg = filter_status_omg;
    angular_rates_filtered.cutoff_freqn_eps = cutoff_freqn_eps;
    angular_rates_filtered.filter_status_eps = filter_status_eps;
    angular_rates_filtered.cutoff_freqn_rc = cutoff_freqn_RC;
    angular_rates_filtered.filter_status_rc = filter_status_RC;
    angular_rates_filtered.lpf_order = lpf_order;
    
    angular_rates_filtered.timestamp = hrt_absolute_time();
    angular_rates_filtered.timestamp_sample = vehicle_attitude.timestamp;

    if(orb_publish(ORB_ID(angular_rates_filtered), angular_rates_filtered_pub, &angular_rates_filtered) != PX4_OK){
        return PX4_ERROR;
    }
    
    return PX4_OK;
}

int My_LQR_control::setpoints_publish(){
    setpoints_struct.y0 = y(0,0);
    setpoints_struct.y1 = y(1,0);
    setpoints_struct.y2 = y(2,0);
    setpoints_struct.y3 = y(3,0);
    setpoints_struct.y4 = y(4,0);
    setpoints_struct.y5 = y(5,0);
    setpoints_struct.y6 = y(6,0);
    setpoints_struct.y7 = y(7,0);
    setpoints_struct.y8 = y(8,0);
    setpoints_struct.y9 = y(9,0);
    setpoints_struct.y10 = y(10,0);
    setpoints_struct.y11 = y(11,0);

    setpoints_struct.sp_y0 = y_setpoint(0,0);
    setpoints_struct.sp_y1 = y_setpoint(1,0);
    setpoints_struct.sp_y2 = y_setpoint(2,0);
    setpoints_struct.sp_y3 = y_setpoint(3,0);
    setpoints_struct.sp_y4 = y_setpoint(4,0);
    setpoints_struct.sp_y5 = y_setpoint(5,0);
    setpoints_struct.sp_y6 = y_setpoint(6,0);
    setpoints_struct.sp_y7 = y_setpoint(7,0);
    setpoints_struct.sp_y8 = y_setpoint(8,0);
    setpoints_struct.sp_y9 = y_setpoint(9,0);
    setpoints_struct.sp_y10 = y_setpoint(10,0);
    setpoints_struct.sp_y11 = y_setpoint(11,0);

    setpoints_struct.c0 = c_setpoint(0,0);
    setpoints_struct.c1 = c_setpoint(1,0);
    setpoints_struct.c2 = c_setpoint(2,0);
    setpoints_struct.c3 = c_setpoint(3,0);

    setpoints_struct.k_omg_pp = K_feedback_y_sc_tun_sched(0,6);
    setpoints_struct.k_omg_pr = K_feedback_y_sc_tun_sched(0,8);
    setpoints_struct.k_omg_rp = K_feedback_y_sc_tun_sched(2,6);
    setpoints_struct.k_omg_rr = K_feedback_y_sc_tun_sched(2,8);
    setpoints_struct.k_omg_qq = K_feedback_y_sc_tun_sched(1,7);

    setpoints_struct.k_eps_phiphi = K_feedback_y_sc_tun_sched(0,9);
    setpoints_struct.k_eps_phipsi = K_feedback_y_sc_tun_sched(0,11);
    setpoints_struct.k_eps_psiphi = K_feedback_y_sc_tun_sched(2,9);
    setpoints_struct.k_eps_psipsi = K_feedback_y_sc_tun_sched(2,11);
    setpoints_struct.k_eps_thttht = K_feedback_y_sc_tun_sched(1,10);

    setpoints_struct.del_c_omg_p = Del_c_omg(0,0);
    setpoints_struct.del_c_omg_q = Del_c_omg(1,0);
    setpoints_struct.del_c_omg_r = Del_c_omg(2,0);
    setpoints_struct.del_c_eps_phi = Del_c_eps(0,0);
    setpoints_struct.del_c_eps_tht = Del_c_eps(1,0);
    setpoints_struct.del_c_eps_psi = Del_c_eps(2,0);

    setpoints_struct.del_c_x_q = Del_c_x(1,0);
    setpoints_struct.del_c_x_m = Del_c_x(3,0);
    setpoints_struct.del_c_v_q = Del_c_v(1,0);
    setpoints_struct.del_c_v_m = Del_c_v(3,0);
    setpoints_struct.k_x_qz = K_feedback_y_sc_tun_sched(1,2);
    setpoints_struct.k_x_mz = K_feedback_y_sc_tun_sched(3,2);
    setpoints_struct.k_v_qvz = K_feedback_y_sc_tun_sched(1,5);
    setpoints_struct.k_v_mvz = K_feedback_y_sc_tun_sched(3,5);
    setpoints_struct.alt_mode = altitude_mode;
    setpoints_struct.c_alt_support = c_alt_support;


    setpoints_struct.pitch_setpoint = rad2deg(pitch_setpoint);
    setpoints_struct.pitch_setpoint_ramp = rad2deg(pitch_setpoint_ramp);

    setpoints_struct.proj_dpsi_status = proj_dpsi_status;
    setpoints_struct.dpsi = rad2deg(Del_y_eps(2,0));
    setpoints_struct.proj_theta_status = proj_theta_status;

    setpoints_struct.tuner_status = tuner_status;
    setpoints_struct.gain_scale_p_p = tune_p_p;
    setpoints_struct.gain_scale_d_p = tune_d_p;
    setpoints_struct.gain_scale_p_q = tune_p_q;
    setpoints_struct.gain_scale_d_q = tune_d_q;
    setpoints_struct.gain_scale_p_r = tune_p_r;
    setpoints_struct.gain_scale_d_r = tune_d_r;
    setpoints_struct.tuner_mode = tune_mode;

    setpoints_struct.case_int_f_int = 1.0f*case_int + f_int;
    setpoints_struct.scheduler_status = schedule_K_status;

    setpoints_struct.dt = dt;

    setpoints_struct.control_status = control_status;

    setpoints_struct.do_recursivels = do_recursiveLS;
    setpoints_struct.x_rls1 = X_RLS(0,0);
    setpoints_struct.x_rls2 = X_RLS(1,0);
    setpoints_struct.do_adaptive = do_adaptive;
    setpoints_struct.kp_rls = K_p_adapt;
    setpoints_struct.kphi_rls = K_phi_adapt;

    setpoints_struct.gain_limiter_p = gain_limiter(0,0);
    setpoints_struct.gain_limiter_q = gain_limiter(1,0);
    setpoints_struct.gain_limiter_r = gain_limiter(2,0);
    setpoints_struct.oscillating_p = oscillating(0,0);
    setpoints_struct.oscillating_q = oscillating(1,0);
    setpoints_struct.oscillating_r = oscillating(2,0);

    setpoints_struct.c_eps_satur_p = c_eps_satur(0,0);
    setpoints_struct.c_eps_satur_q = c_eps_satur(1,0);
    setpoints_struct.c_eps_satur_r = c_eps_satur(2,0);
    setpoints_struct.c_eps_satur_m = c_eps_satur(3,0);


    setpoints_struct.timestamp = hrt_absolute_time();
    setpoints_struct.timestamp_sample = vehicle_attitude.timestamp;

    if(orb_publish(ORB_ID(my_LQR_setpoints), setpoints_pub, &setpoints_struct) != PX4_OK){
        return PX4_ERROR;
    }
    
    return PX4_OK;
}

int My_LQR_control::debug_publish(){
    dbg_val.ind = 0;
    //dbg_val.value = y(6,0); // roll rate to see filtered vibrations
    //dbg_val.value = X_RLS(1,0); // identified values
    dbg_val.value = gain_limiter(0,0); // limiter on p gain
    
    dbg_val.timestamp = hrt_absolute_time();

    if(orb_publish(ORB_ID(debug_value), dbg_val_pub, &dbg_val) != PX4_OK){
        return PX4_ERROR;
    }
    
    return PX4_OK;
}

int My_LQR_control::publish_topics(){
    actuator_controls_publish();
    angular_rates_filtered_publish();
    setpoints_publish();
    debug_publish();
    return PX4_OK;
}




/*----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------*/
void My_LQR_control::run(){
    // subscribe to topics
    parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    actuator_controls_virtual_sub = orb_subscribe(actuator_controls_virtual_id);
    home_position_sub = orb_subscribe(ORB_ID(home_position));
    airspeed_sub = orb_subscribe(ORB_ID(airspeed));
    
    // advertise topics
    actuator_controls_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuator_controls_0);    
    actuator_controls_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &actuator_controls_1);
    angular_rates_filtered_pub = orb_advertise(ORB_ID(angular_rates_filtered), &angular_rates_filtered);
    setpoints_pub = orb_advertise(ORB_ID(my_LQR_setpoints), &setpoints_struct);
    dbg_val_pub = orb_advertise(ORB_ID(debug_value), &dbg_val);
    
    // initialize parameters
    initialize_variables();
    

    // Run the loop synchronized to the vehicle_attitude topic.
    px4_pollfd_struct_t fds[1];
    fds[0].fd = vehicle_attitude_sub;
    fds[0].events = POLLIN;
    int pret;

    while (!should_exit()) {
        
        // wait for up to 100ms for data
        pret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 100);

        if (pret == 0) {
            continue;
        }
        else if (pret < 0) {
            // this is undesirable but not much we can do
            PX4_ERR("poll error %d, %d", pret, errno);
            px4_usleep(50000);
            continue;
        }
        else {
            perf_begin(_loop_perf);
            if (fds[0].revents & POLLIN) {
                timer_clock(); // manages time to have a reliable dt variable
                
                read_y_state();
                
                read_setpoints();

                gains_tune(); // gains tune based on RC knobs
                gains_schedule(); // gains schedule based on pitch setpoint

                recursiveLS();
                adaptive_control();
                gains_limiter_fun();
                
                control_fun(); // computes the actuator controls

                px4_override(); // overrides the controls by the PX4 controller based on RC switches
                manual_override(); // overrides the controls by manual RC input based on RC switches

                supporting_outputs(); // front engine and tailerons and differential thrust to mixer based on RC switches
                
                publish_topics();

                debug_printouts();
                
            }
            perf_end(_loop_perf);
        }
        update_parameters();
    }
    
    
    // Unsubscribe
    orb_unsubscribe(parameter_update_sub);
    orb_unsubscribe(vehicle_attitude_sub);
    orb_unsubscribe(vehicle_local_position_sub);
    orb_unsubscribe(vehicle_local_position_setpoint_sub);
    orb_unsubscribe(manual_control_setpoint_sub);
    orb_unsubscribe(rc_channels_sub);
    orb_unsubscribe(actuator_controls_virtual_sub);
    orb_unsubscribe(home_position_sub);
    orb_unsubscribe(airspeed_sub);

    // unadvertise topics
    orb_unadvertise(actuator_controls_0_pub);    
    orb_unadvertise(actuator_controls_1_pub);
    orb_unadvertise(angular_rates_filtered_pub);
    orb_unadvertise(setpoints_pub);
    orb_unadvertise(dbg_val_pub);
}
/* -----------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------*/




int My_LQR_control::timer_clock(){
    if(t_start < 0.0f){
        t_start = vehicle_attitude.timestamp;
    }
    dt = (vehicle_attitude.timestamp - time_last_run)/1000000.0f;
    time_last_run = vehicle_attitude.timestamp;
    
    loop_counter = loop_counter + 1.0f;
    dt_loop = dt_loop + dt;
    if(dt_loop >= 10.0f){ // use 10 seconds to compute the loop update rate
        if(fabsf(loop_update_freqn - loop_counter/dt_loop) > 10.0f){ // if loop freqn change by more than 10 Hz update the filter
            loop_update_freqn = loop_counter/dt_loop;
            lp2_filter_omg.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_omg);
            lp2_filter_eps.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_eps);
            lp3_filter_omg.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_omg);
            lp3_filter_eps.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_eps);
            lp2_filter_RC.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_RC);
        }
        dt_loop = 0.0f;
        loop_counter = 0.0f;
    }
    return PX4_OK;
}




int My_LQR_control::read_y_state(){
    vehicle_attitude_poll();
    vehicle_local_position_poll();
    convert_quaternions();
    filter_omg();
    filter_eps();
        
    y( 0,0) = 0.0f*vehicle_local_position.x;
    y( 1,0) = 0.0f*vehicle_local_position.y;
    y( 2,0) =-1.0f*vehicle_local_position.z;
    y( 3,0) = 0.0f*vehicle_local_position.vx;
    y( 4,0) = 0.0f*vehicle_local_position.vy;
    y( 5,0) =-1.0f*vehicle_local_position.vz;
    y( 6,0) = 1.0f*omg_filtered(0);
    y( 7,0) = 1.0f*omg_filtered(1);
    y( 8,0) = 1.0f*omg_filtered(2);
    y( 9,0) = 1.0f*eps_filtered(0);
    y(10,0) = 1.0f*eps_filtered(1);
    y(11,0) = 1.0f*eps_filtered(2);

    return PX4_OK;
}
int My_LQR_control::convert_quaternions(){
    Qdcm = Quatf(vehicle_attitude.q);
    euler_angles = Qdcm; // this is how you convert the Dcm into Euler angles (readme.md in matrix lib)
    project_theta();

    eps(0) = euler_angles.phi();
    eps(1) = euler_angles.theta();
    eps(2) = euler_angles.psi();
    return PX4_OK;
}
int My_LQR_control::project_theta(){
// Extending the -90 to +90 deg range on theta to -120 to 120 deg
    proj_theta_status = 0;
    theta0 = euler_angles.theta();
    if(proj_theta){
        proj_theta_status = 1;
        Qdcm_proj = Qdcm; // predefine
        if(euler_angles.theta() > deg2rad(70.0f)){ 
            // rotate the device by -90 deg
            Qdcm_proj(0,0) = Qdcm(0,2); // z to x
            Qdcm_proj(1,0) = Qdcm(1,2);
            Qdcm_proj(2,0) = Qdcm(2,2);

            Qdcm_proj(0,2) = -Qdcm(0,0); // x to -z
            Qdcm_proj(1,2) = -Qdcm(1,0);
            Qdcm_proj(2,2) = -Qdcm(2,0);

            euler_angles_proj = Qdcm_proj; // get the corresponding euler angles
            euler_angles.theta() = euler_angles_proj.theta() + deg2rad(90.0f); // bring back the unrotated theta
            proj_theta_status = 10; // log status
            euler_angles.phi() = euler_angles_proj.phi();
            euler_angles.psi() = euler_angles_proj.psi();
        }
        else if(euler_angles.theta() < -deg2rad(70.0f)){ 
            // rotate the device by 90 deg
            Qdcm_proj(0,0) = -Qdcm(0,2); // z to -x
            Qdcm_proj(1,0) = -Qdcm(1,2);
            Qdcm_proj(2,0) = -Qdcm(2,2);

            Qdcm_proj(0,2) = Qdcm(0,0); // x to z
            Qdcm_proj(1,2) = Qdcm(1,0);
            Qdcm_proj(2,2) = Qdcm(2,0);

            euler_angles_proj = Qdcm_proj; // get the corresponding euler angles
            euler_angles.theta() = euler_angles_proj.theta() - deg2rad(90.0f); // bring back the unrotated theta
            proj_theta_status = -10; // log status
            euler_angles.phi() = euler_angles_proj.phi();
            euler_angles.psi() = euler_angles_proj.psi();
        }
    }
    return PX4_OK;
}

int My_LQR_control::filter_omg(){
    omg(0) = vehicle_attitude.rollspeed;
    omg(1) = vehicle_attitude.pitchspeed;
    omg(2) = vehicle_attitude.yawspeed;

    if(cutoff_freqn_omg <= 100.0f){
        omg_filtered_temp_lp2 = lp2_filter_omg.apply(omg);
        omg_filtered_temp_lp3 = lp3_filter_omg.apply(omg);
        filter_status_omg = 0; // ok
        if((lpf_order == 3) && (isbound(omg_filtered_temp_lp3(0)) && isbound(omg_filtered_temp_lp3(1)) && isbound(omg_filtered_temp_lp3(2)))){
            omg_filtered = omg_filtered_temp_lp3;
        }
        else if((lpf_order == 2) && (isbound(omg_filtered_temp_lp2(0)) && isbound(omg_filtered_temp_lp2(1)) && isbound(omg_filtered_temp_lp2(2)))){
            omg_filtered = omg_filtered_temp_lp2;
        }
        else if(isbound(omg_filtered_temp_lp3(0)) && isbound(omg_filtered_temp_lp3(1)) && isbound(omg_filtered_temp_lp3(2))){ // not looking anymore on the filter preference
            omg_filtered = omg_filtered_temp_lp3;
            filter_status_omg = 23; // overriden by lpf3
        }
        else if(isbound(omg_filtered_temp_lp2(0)) && isbound(omg_filtered_temp_lp2(1)) && isbound(omg_filtered_temp_lp2(2))){
            omg_filtered = omg_filtered_temp_lp2;
            filter_status_omg = 22; // overriden by lpf2
        }
        else{
            omg_filtered = omg*0.0f; // turn it off to prevent feeding vibrations to servos
            filter_status_omg = 1; // whops
        }
    }
    else{
        omg_filtered = omg; 
        filter_status_omg = 2; // no cutoff
    }

    return PX4_OK;
}
int My_LQR_control::filter_eps(){
    if(cutoff_freqn_eps <= 100.0f){
        eps_filtered_temp_lp2 = lp2_filter_eps.apply(eps);
        eps_filtered_temp_lp3 = lp3_filter_eps.apply(eps);
        filter_status_eps = 0; // ok
        if((lpf_order == 3) && (isbound(eps_filtered_temp_lp3(0)) && isbound(eps_filtered_temp_lp3(1)) && isbound(eps_filtered_temp_lp3(2)))){
            eps_filtered = eps_filtered_temp_lp3;
        }
        else if((lpf_order == 2) && (isbound(eps_filtered_temp_lp2(0)) && isbound(eps_filtered_temp_lp2(1)) && isbound(eps_filtered_temp_lp2(2)))){
            eps_filtered = eps_filtered_temp_lp2;
        }
        else if(isbound(eps_filtered_temp_lp3(0)) && isbound(eps_filtered_temp_lp3(1)) && isbound(eps_filtered_temp_lp3(2))){ // not looking anymore on the filter preference
            eps_filtered = eps_filtered_temp_lp3;
            filter_status_eps = 23; // overriden by lpf3
        }
        else if(isbound(eps_filtered_temp_lp2(0)) && isbound(eps_filtered_temp_lp2(1)) && isbound(eps_filtered_temp_lp2(2))){
            eps_filtered = eps_filtered_temp_lp2;
            filter_status_eps = 22; // overriden by lpf2
        }
        else if(isbound(eps(0)) && isbound(eps(1)) && isbound(eps(2))){ // check the orig eps
                eps_filtered = eps; // better to use the non-filtered than nothing
                filter_status_eps = 10; // overriden by non-filtered
        }
        else{ // if they are not bounded
                eps_filtered = eps*0.0f; // then no point feeding them back 
                filter_status_eps = 1; // whops
        }
    }
    else{
        eps_filtered = eps; 
        filter_status_eps = 2; // no cutoff
    }

    return PX4_OK;
}
int My_LQR_control::filter_RC(){
    RC(0) = rc_channels.channels[0];
    RC(1) = rc_channels.channels[1];
    RC(2) = rc_channels.channels[2];

    if(cutoff_freqn_RC <= 100.0f){
        RC_filtered = lp2_filter_RC.apply(RC);
        filter_status_RC = 0; // ok
        if(!isbound(RC_filtered(0)) || !isbound(RC_filtered(1)) || !isbound(RC_filtered(2))){ 
            filter_status_RC = 1; // whops
            RC_filtered = RC; // definitely better to use the non-filtered than nothing
        }
    }
    else{
        RC_filtered = RC; 
        filter_status_RC = 2; // no cutoff
    }

    return PX4_OK;
}





int My_LQR_control::read_setpoints(){
    // manual_control_setpoint_poll();
    // vehicle_local_position_setpoint_poll();
    rc_channels_poll();
    filter_RC();
    read_y_setpoint();
    read_c_setpoint();
    return PX4_OK;
}
int My_LQR_control::read_c_setpoint(){
    c_setpoint = c_nominal_control;

    // Position/ velocity control
    /* all c_setpoints are equal to nominal control */

    // Attitude control
    /* we control thrust directly by rc */
    thrust_setpoint = rc_channels.channels[3] - 0.5f; // [0,1] to [-0.5, 0.5]
    c_setpoint(3,0) = math::constrain(c_nominal_control(3,0) + thrust_setpoint, 0.0f, 1.0f);

    return PX4_OK;
}
int My_LQR_control::read_y_setpoint(){
    y_setpoint.setAll(0.0f);

    pitch_setpoint = deg2rad(pitch_sp_min) + ((rc_channels.channels[9] + 1.0f)/2.0f)*deg2rad(pitch_sp_max); // 0 to pitch_sp_max deg based on RS stick input
    //pitch_setpoint = rc_channels.channels[9]*deg2rad(pitch_sp_max); // -90 to 90 deg based on RS stick input just for test
    pitch_setpoint_ramp = pitch_setpoint_ramp + (dt/1.0f)*(pitch_setpoint - pitch_setpoint_ramp); // 1st order lag for use with gain scheduler
    pitch_setpoint_ramp = pitch_setpoint_ramp + dt*(deg2rad(20.0f)/2.0f)*((pitch_setpoint - pitch_setpoint_ramp) > (dt*(deg2rad(20.0f)/2.0f))) - dt*(deg2rad(20.0f)/2.0f)*((pitch_setpoint - pitch_setpoint_ramp) < (-dt*(deg2rad(20.0f)/2.0f))); // or linear ramp

    /* 
    // Position/ velocity control
    y_setpoint(3,0) = rc_deadband(manual_control_setpoint.x)*y_setpoint_scale(3,0);
    y_setpoint(4,0) = rc_deadband(manual_control_setpoint.y)*y_setpoint_scale(4,0);
    y_setpoint(5,0) = -(2.0f*rc_deadband(manual_control_setpoint.z) - 1.0f)*y_setpoint_scale(5,0);
    y_setpoint(8,0) = rc_deadband(manual_control_setpoint.r)*y_setpoint_scale(8,0);
    */

    // Attitude control
    y_setpoint(6,0) =  RC_filtered(0);
    y_setpoint(7,0) = -RC_filtered(1);
    y_setpoint(8,0) =  RC_filtered(2);
    y_setpoint(9,0) = 0.0f;
    y_setpoint(10,0) = pitch_setpoint_ramp;
    if(fabsf(y_setpoint(8,0)) > 0.1f || ((fabsf(y_setpoint(6,0)) > 0.1f) && ((vehicle_id == 2) || (vehicle_id == 3))) || fabsf(yaw_setpoint) <= 0.0f){ // at yaw||(roll if fw) rate command or at the startup
        yaw_setpoint = y(11,0);
    }
    if((fabsf(y(11,0) - setpoints_struct.y11) > 0.08f) && (fabsf(y(11,0) - setpoints_struct.y11) < 6.0f)){ // if jump in psi more than 4.5 degrees, but not the 360 deg jump when crossing the 180 limit
        yaw_setpoint = y(11,0) - (setpoints_struct.y11 - setpoints_struct.sp_y11); // preserve the last del_y
    }
    y_setpoint(11,0) = yaw_setpoint;


    // Altitude control
    if(altitude_mode == 1){
        alt_rate_setpoint = 0.0f;
        if(fabsf(y_setpoint(7,0)) > 0.1f){
            alt_rate_setpoint = y(5,0); // don't limit the climb/descend now
            if(k_sw(0,0) > 0.001f){
                alt_setpoint = y(2,0) + Del_c_x(1,0)/k_sw(0,0); // preserve the last control contribution
                alt_setpoint = math::constrain(alt_setpoint, y(2,0) - Del_c_lim(0,0)/k_sw(0,0), y(2,0) + Del_c_lim(0,0)/k_sw(0,0)); // integral windup this
            }
            else{
                altitude_mode = -10;
                alt_setpoint = y(2,0);
            }
        }
    }
    if(altitude_mode == 2){
        alt_rate_setpoint = alt_rate_rc_scale*(c_setpoint(3,0)-0.5f);
        if(fabsf(c_setpoint(3,0) - 0.5f) > 0.1f){
            if(k_sw(2,1) > 0.001f){
                alt_setpoint = y(2,0) + Del_c_x(3,0)/k_sw(2,1); // preserve the last control contribution
                alt_setpoint = math::constrain(alt_setpoint, y(2,0), y(2,0) + Del_c_lim(0,0)/k_sw(2,1)); // integral windup this from up side and don't use from bottom
            }
            else{
                altitude_mode = -20;
                alt_setpoint = y(2,0);
            }
        }
    }
    if(altitude_mode == 0){
        alt_setpoint = y(2,0);
        alt_rate_setpoint = y(5,0);
    }

    y_setpoint(2,0) = alt_setpoint;
    y_setpoint(5,0) = alt_rate_setpoint;

    return PX4_OK;
}
int My_LQR_control::omg_setpoints_scale(){
// RC pass through scaled by RC_scale cos i.e. for multicopters we don't want such high rates
// also p,q,r can react differently to cp,cq,cr in fixed-wing plane

    RC_scale = RC_scale_base;
    if(do_rc_scale){
        for(int i=0; i<3; i++){ // scaling the RC input up based on K_eps gains, so that if K_eps high, I can stil move the plane without the KP-compensation pushing me back to zero
            f_scale = K_feedback_y_sc_tun_sched(i,9+i); // this scales the RC such that I reach exactly eps==1[rad]~57.3[deg] at c==1 (omg==0).
            
            p_scale = fabsf(Del_y_eps(i,0))/RC_scale_base(i,0); // however it may make the RC scale so small that it would give very small inputs at eps==0 if Keps->0. Same way, if keps->large, I get overreactions at eps==0.
            if(p_scale < 1.0f){
                f_scale = 1.0f + (K_feedback_y_sc_tun_sched(i,9+i) - 1)*p_scale; // so I do a linear interpolation between the base RC at eps==0 and the scaled RC at eps==1 (eps=1*RC_scale_base)
            }

            RC_scale(i,0) *= f_scale;
        }
    }

    y_setpoint(6,0) *= RC_scale(0,0);
    y_setpoint(7,0) *= RC_scale(1,0);
    y_setpoint(8,0) *= RC_scale(2,0);

    return PX4_OK;
}




int My_LQR_control::gains_scale(){
    k_sc_vec(0,0) = k_sc_x.get();
    k_sc_vec(1,0) = k_sc_v.get();
    k_sc_vec(2,0) = k_sc_p.get();
    k_sc_vec(3,0) = k_sc_q.get();
    k_sc_vec(4,0) = k_sc_r.get();
    k_sc_vec(5,0) = k_sc_phi.get();
    k_sc_vec(6,0) = k_sc_tht.get();
    k_sc_vec(7,0) = k_sc_psi.get();
    k_sc_vec(8,0) = k_sc_ccd.get();
    k_sc_vec(9,0) = k_sc_ccp.get();
    k_sc_vec(10,0) = k_sc_cf.get();
    k_sc_vec(11,0) = k_sc_ri.get();

    K_feedback_y_scaled = K_feedback_y;
    for(int i=0; i<4; i++){
        for(int j=0; j<3; j++){
            K_feedback_y_scaled(i,j) *= k_sc_vec(0,0); // x
        }
        for(int j=3; j<6; j++){
            K_feedback_y_scaled(i,j) *= k_sc_vec(1,0); // v
        }
        K_feedback_y_scaled(i,6) *= k_sc_vec(2,0); // p
        K_feedback_y_scaled(i,7) *= k_sc_vec(3,0); // q
        K_feedback_y_scaled(i,8) *= k_sc_vec(4,0); // r
        K_feedback_y_scaled(i,9) *= k_sc_vec(5,0); // phi
        K_feedback_y_scaled(i,10) *= k_sc_vec(6,0); // tht
        K_feedback_y_scaled(i,11) *= k_sc_vec(7,0); // psi
    }
    K_feedback_y_scaled(0,8) *= k_sc_vec(8,0); // cross-coupling d
    K_feedback_y_scaled(2,6) *= k_sc_vec(8,0);
    K_feedback_y_scaled(0,11) *= k_sc_vec(9,0); // cross-coupling p
    K_feedback_y_scaled(2,9) *= k_sc_vec(9,0);


    K_feedback_cf_scaled = K_feedback_cf;
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            K_feedback_cf_scaled *= k_sc_vec(10,0);
        }
    }

    K_feedback_int_scaled = K_feedback_int;
    for(int i=0; i<4; i++){
        for(int j=0; j<6; j++){
            K_feedback_int_scaled *= k_sc_vec(11,0);
        }
    }


    k_scheds_sc = k_scheds;
    for(int j=0; j<n_int+1; j++){ // for all thetas
        k_scheds_sc(0,j) *= k_sc_vec(2,0); // p
        k_scheds_sc(1,j) *= k_sc_vec(2,0); // pr
        k_scheds_sc(2,j) *= k_sc_vec(4,0); // rp
        k_scheds_sc(3,j) *= k_sc_vec(4,0); // r
        k_scheds_sc(4,j) *= k_sc_vec(5,0); // phi
        k_scheds_sc(5,j) *= k_sc_vec(5,0); // phipsi
        k_scheds_sc(6,j) *= k_sc_vec(7,0); // psiphi
        k_scheds_sc(7,j) *= k_sc_vec(7,0); // psi
        k_scheds_sc(1,j) *= k_sc_vec(8,0); // cross-coupling [pr, rp]
        k_scheds_sc(2,j) *= k_sc_vec(8,0); 
        k_scheds_sc(5,j) *= k_sc_vec(9,0); // cross-coupling [phipsi, psiphi]
        k_scheds_sc(6,j) *= k_sc_vec(9,0); 
        k_scheds_sc(8,j) *= k_sc_vec(3,0); // q
        k_scheds_sc(9,j) *= k_sc_vec(6,0); // tht
    }

    rc_sc_eps_last = 0.0f; // to wake up the tuner as well
    rc_sc_omg_last = 0.0f;
    K_feedback_y_scaled_tuned = K_feedback_y_scaled;
    K_feedback_y_sc_tun_sched = K_feedback_y_scaled_tuned;
    k_scheds_sc_tun = k_scheds_sc;

    k_sw(0,0) = k_sw1_qz.get(); //// cruise&fast [qz, qvz, mz, mvz];
    k_sw(1,0) = k_sw1_qvz.get();
    k_sw(2,0) = k_sw1_mz.get();
    k_sw(3,0) = k_sw1_mvz.get();
    k_sw(0,1) = k_sw2_qz.get(); //// slow&hover [qz, qvz, mz, mvz];
    k_sw(1,1) = k_sw2_qvz.get();
    k_sw(2,1) = k_sw2_mz.get();
    k_sw(3,1) = k_sw2_mvz.get();

    return PX4_OK;
}
int My_LQR_control::gains_tune(){
// Tune the gains using the radio knobs
    tuner_status = 0;
    if(fabsf(rc_sc_omg_last - rc_channels.channels[11]) > 0.002f || fabsf(rc_sc_eps_last - rc_channels.channels[10]) > 0.002f){
        tuner_status = 1;

        tune_d = powf(tune_expo, rc_channels.channels[11]);
        tune_p = powf(tune_expo, rc_channels.channels[10]);

        rc_sc_omg_last = rc_channels.channels[11];
        rc_sc_eps_last = rc_channels.channels[10];

        // updating only those scales that are currently selected, the rest stays as it was
        if((tune_mode == 0) | (tune_mode == 10) | (tune_mode == 20)  | (tune_mode == 210)){
            tune_d_p = tune_d;
            tune_p_p = tune_p;
        }
        if((tune_mode == 1) | (tune_mode == 10) | (tune_mode == 21)  | (tune_mode == 210)){
            tune_d_q = tune_d;
            tune_p_q = tune_p;
        }
        if((tune_mode == 2) | (tune_mode == 20) | (tune_mode == 21)  | (tune_mode == 210)){
            tune_d_r = tune_d;
            tune_p_r = tune_p;
        }

        K_feedback_y_scaled_tuned = K_feedback_y_scaled;
        for(int j=6; j<9; j++){
            K_feedback_y_scaled_tuned(0,j) *= tune_d_p;
        }
        for(int j=9; j<12; j++){
            K_feedback_y_scaled_tuned(0,j) *= tune_p_p;
        }
        for(int j=6; j<9; j++){
            K_feedback_y_scaled_tuned(1,j) *= tune_d_q;
        }
        for(int j=9; j<12; j++){
            K_feedback_y_scaled_tuned(1,j) *= tune_p_q;
        }
        for(int j=6; j<9; j++){
            K_feedback_y_scaled_tuned(2,j) *= tune_d_r;
        }
        for(int j=9; j<12; j++){
            K_feedback_y_scaled_tuned(2,j) *= tune_p_r;
        }


        k_scheds_sc_tun = k_scheds_sc; //[pp, pr, rp, rr, ..., q, tht]
        for(int j=0; j<n_int+1; j++){
            k_scheds_sc_tun(0,j) *= tune_d_p;
            k_scheds_sc_tun(1,j) *= tune_d_p;
            k_scheds_sc_tun(2,j) *= tune_d_r;
            k_scheds_sc_tun(3,j) *= tune_d_r;
            k_scheds_sc_tun(4,j) *= tune_p_p;
            k_scheds_sc_tun(5,j) *= tune_p_p;
            k_scheds_sc_tun(6,j) *= tune_p_r;
            k_scheds_sc_tun(7,j) *= tune_p_r;
            k_scheds_sc_tun(8,j) *= tune_d_q;
            k_scheds_sc_tun(9,j) *= tune_p_q;
        }

        // trigger the scheduler as well if tuning changed
        case_int_last = -1;
        K_feedback_y_sc_tun_sched = K_feedback_y_scaled_tuned; // if schedule_K == 0 it updates here 
    }

    return PX4_OK;
}
int My_LQR_control::gains_schedule(){
    if(schedule_K == 1){ 
        schedule_K_status = 1;
        // tht_sched = y(10,0); // set the scheduling variable to theta state
        tht_sched = pitch_setpoint_ramp; // set the scheduling variable to the theta setpoint
        for(int i = 0; i < n_int; i++){ // find interval
            f_int = (tht_sched - tht_ints(0,i))/(tht_ints(0,i+1) - tht_ints(0,i));
            if(f_int >= 0.0f && f_int <= 1.0f){
                case_int = i;
                break;
            }
        }
        airspeed_poll();
        if((airspeed.true_airspeed_m_s <= 50.0f) && (airspeed.true_airspeed_m_s > 15.0f)){ // checking if <50 as a safety check for infs or nans or bad readings. You control the gains by pitch setpoint now, but if it happens for example in transitions that the airspeed is still high, then keep them lower.
            case_int = 1;
            f_int = 0.0f;
            schedule_K_status = 2; // blocked by airspeed
        }
        case_int_last = 100; // comment if you just want a step function
        if(case_int_last != case_int){ // interpolate
            // f_int = 0.0f; // zero order interpolation
            case_int_last = case_int;
            for(int i=0; i<10; i++){
                k_scheds_sc_tun_int(i,0) = (1.0f-f_int)*k_scheds_sc_tun(i,case_int) + f_int*k_scheds_sc_tun(i,case_int+1);
            }
            K_feedback_y_sc_tun_sched = K_feedback_y_scaled_tuned;
            K_feedback_y_sc_tun_sched(0,6) = k_scheds_sc_tun_int(0,0);
            K_feedback_y_sc_tun_sched(0,8) = k_scheds_sc_tun_int(1,0);
            K_feedback_y_sc_tun_sched(1,7) = k_scheds_sc_tun_int(8,0);
            K_feedback_y_sc_tun_sched(2,6) = k_scheds_sc_tun_int(2,0);
            K_feedback_y_sc_tun_sched(2,8) = k_scheds_sc_tun_int(3,0);
            K_feedback_y_sc_tun_sched(0,9) = k_scheds_sc_tun_int(4,0);
            K_feedback_y_sc_tun_sched(0,11) = k_scheds_sc_tun_int(5,0);
            K_feedback_y_sc_tun_sched(1,10) = k_scheds_sc_tun_int(9,0);
            K_feedback_y_sc_tun_sched(2,9) = k_scheds_sc_tun_int(6,0);
            K_feedback_y_sc_tun_sched(2,11) = k_scheds_sc_tun_int(7,0);
        }


        // Switches between the two altitude stabilisation modes based on flight mode.
        if(c_alt_bool < 0.5f){ // == 0 but it's a float
            altitude_mode = 0;
        }
        else{
            if(pitch_setpoint_ramp < deg2rad(10.0f)){
                altitude_mode = 1; // use elevator
                /* Let it stall if the pilot decides to go so low throttle and hold altitude.
                if(airspeed.true_airspeed_m_s < 15.0f){ //  don't cause stall
                    altitude_mode = -1; // reject alt mode 1
                    K_feedback_y_sc_tun_sched(1,2) = 0.0f; // qz
                    K_feedback_y_sc_tun_sched(1,5) = 0.0f; // qvz
                    K_feedback_y_sc_tun_sched(3,2) = 0.0f; // mz
                    K_feedback_y_sc_tun_sched(3,5) = 0.0f; // mvz
                }
                else{ */
                    K_feedback_y_sc_tun_sched(1,2) = k_sw(0,0); // qz
                    K_feedback_y_sc_tun_sched(1,5) = k_sw(1,0); // qvz
                    K_feedback_y_sc_tun_sched(3,2) = k_sw(2,0); // mz
                    K_feedback_y_sc_tun_sched(3,5) = k_sw(3,0); // mvz
                //}
            }
            else{
                altitude_mode = 2; // use motors
                if(c_setpoint(3,0) < 0.10f){ // don't add throttle if not already using the motors
                    altitude_mode = -2; // reject alt mode 2
                    K_feedback_y_sc_tun_sched(1,2) = 0.0f; // qz
                    K_feedback_y_sc_tun_sched(1,5) = 0.0f; // qvz
                    K_feedback_y_sc_tun_sched(3,2) = 0.0f; // mz
                    K_feedback_y_sc_tun_sched(3,5) = 0.0f; // mvz
                }
                else{
                    K_feedback_y_sc_tun_sched(1,2) = k_sw(0,1); // qz
                    K_feedback_y_sc_tun_sched(1,5) = k_sw(1,1); // qvz
                    K_feedback_y_sc_tun_sched(3,2) = k_sw(2,1); // mz
                    K_feedback_y_sc_tun_sched(3,5) = k_sw(3,1); // mvz
                }
            }
        }

    }
    else{
        schedule_K_status = 0;
        case_int = -1;
        f_int = 0.0f;
    }


    return PX4_OK;
}



int My_LQR_control::recursiveLS(){
    if(do_recursiveLS){
        M_RLS(0,0) = p_prev_RLS(0,0);
        M_RLS(0,1) = cf(0,0);
        Y_RLS(0,0) = (omg_filtered(0) - p_prev_RLS(0,0))/dt;
        E_RLS = (Y_RLS - M_RLS*X_RLS);
        E_RLS(0,0) = math::constrain(E_RLS(0,0), -5.0f, 5.0f);
        if(fabsf(Y_RLS(0,0)) < 50.0f && fabsf(Y_RLS(0,0)) > 1.0f && dt < 0.3f && dt > 0.002f){
            fract_RLS = M_RLS*P_RLS*M_RLS.T();
            P_RLS = (P_RLS - (P_RLS*M_RLS.T()*M_RLS*P_RLS)/(lambda_RLS + fract_RLS(0,0))) / lambda_RLS;
            X_RLS = X_RLS + P_RLS*M_RLS.T()*E_RLS;
            X_RLS(0,0) = math::constrain(X_RLS(0,0), -30.0f, 0.0f);
            X_RLS(1,0) = math::constrain(X_RLS(1,0), 0.0f, 60.0f);
        }
        p_prev_RLS(0,0) = omg_filtered(0);
    }
    else{
        X_RLS = X0_RLS;
        P_RLS = P0_RLS;
    }
    return PX4_OK;
}
int My_LQR_control::adaptive_control(){
    K_p_adapt = -(ap_adapt - bp_adapt*kp_adapt - X_RLS(0,0))/X_RLS(1,0);
    K_phi_adapt = -(0.0f - bp_adapt*kphi_adapt - 0.0f)/X_RLS(1,0);
    K_p_adapt = math::constrain(K_p_adapt, 0.0f, 10.0f);
    K_phi_adapt = math::constrain(K_phi_adapt, 0.0f, 10.0f);
    if(do_adaptive){
        K_feedback_y_sc_tun_sched(0,6) = K_p_adapt;
        K_feedback_y_sc_tun_sched(0,9) = K_phi_adapt;
    }
    return PX4_OK;
}
int My_LQR_control::gains_limiter_fun(){
    if(gains_limiter_on){
        oscillating = detected_oscillations.apply(Del_c_omg.slice<3,1>(0,0), vehicle_attitude.timestamp);
        for(int i=0; i<3; i++){
            if(oscillating(i,0) == 1){
                gain_limiter(i,0) *= 0.8f;
            }
            else{
                gain_limiter(i,0) = math::min(gain_limiter(i,0) + (1.0f/(loop_update_freqn*20.0f)), 1.0f); 
            }
        }
    }
    else{
        gain_limiter.setAll(1.0f);
    }

    return PX4_OK;
}



int My_LQR_control::control_fun(){
// Main controller function
    Del_y  = y  - y_setpoint;
    //Del_y(11,0) = math::constrain(Del_y(11,0), -0.26f, 0.26f); // limit the yaw error to +-15 deg (0.26rad) not to freak out when heading too off course

    //Del_cf = cf - c_setpoint; // not used if not filter
    //Del_r  = r  - r_setpoint; // not used if no integral compensation

    //Del_c = K_feedback_y*Del_y ... ; // not used if not filter

    //if(dt < 0.5f){ // ignore too large dt steps (probably some glitches, startup etc)
        //r  = r  + dt*Ci*Del_y; // not used if no integral compensation
        //cf = cf + dt*Tf*(-Del_cf + Del_c); // not used if not filter
    //}
    
    Del_y_eps = Del_y.slice<3,1>(9,0);
    project_del_psi();
    omg_setpoints_scale();
    // del_epsilon_to_body_frame(); // depreciated
    
    /* depreciated, this is already limited later
    // Let's constrain Del_eps so I dont get overreaction at large perturbs. The Del_c is limited but if this is too large then the omg compensation wont be able to react
    math::constrain(Del_y_eps(0,0), -deg2rad(30.0f), deg2rad(30.0f));
    math::constrain(Del_y_eps(1,0), -deg2rad(30.0f), deg2rad(30.0f));
    math::constrain(Del_y_eps(2,0), -deg2rad(30.0f), deg2rad(30.0f));
    */ 

    Del_c_x   = -K_feedback_y_sc_tun_sched.T().slice<3,4>(0,0).T()*Del_y.slice<3,1>(0,0); // slice x contribution
    Del_c_v   = -K_feedback_y_sc_tun_sched.T().slice<3,4>(3,0).T()*Del_y.slice<3,1>(3,0); // slice v contribution
    c_alt_support = math::constrain(Del_c_v(3,0) - Del_c_lim(1,0), 0.0f, 1.0f) + math::constrain(Del_c_x(3,0) - Del_c_lim(0,0), 0.0f, 1.0f); // store the throttle remainder after bounding to its upper limit (this will only add throttle, never reduce)
    // Del_c_omg = -K_feedback_y_sc_tun_sched.T().slice<3,4>(6,0).T()*Del_y.slice<3,1>(6,0); // slice omg contribution
    Del_c_omg = -K_feedback_y_sc_tun_sched.T().slice<3,4>(6,0).T()*(y.slice<3,1>(6,0).emult(gain_limiter)); // slice omg contribution, this way RC input indep of K
    Del_c_omg(0,0) += y_setpoint(6,0); // p
    Del_c_omg(1,0) += y_setpoint(7,0); // q
    Del_c_omg(2,0) += y_setpoint(8,0); // r
    Del_c_eps = -K_feedback_y_sc_tun_sched.T().slice<3,4>(9,0).T()*Del_y_eps; // sliced eps contribution
    for(int i = 0; i < 4; i++){ // get the reminders after the constraining that follows
        c_eps_satur(i,0) = copysignf(1.0f, Del_c_eps(i,0))*math::constrain(fabsf(Del_c_eps(i,0)) - Del_c_lim(3,0), 0.0f, 1.0f);
    }
    for(int i = 0; i < 4; i++){ // not necessarily (-1,1), just a sanity check against NaNs
        Del_c_x(i,0)   = math::constrain(Del_c_x(i,0)  , -Del_c_lim(0,0), Del_c_lim(0,0)*(altitude_mode != 1)); // only downward elev trim for our purpose now
        Del_c_v(i,0)   = math::constrain(Del_c_v(i,0)  , -Del_c_lim(1,0), Del_c_lim(1,0)); // should be more careful here cos the throttle is actually [0,1] not [-1,1], but will do for now as it doesn't mix anywhere
        Del_c_omg(i,0) = math::constrain(Del_c_omg(i,0), -Del_c_lim(2,0), Del_c_lim(2,0));
        Del_c_eps(i,0) = math::constrain(Del_c_eps(i,0), -Del_c_lim(3,0), Del_c_lim(3,0));
    }

    stabilisation_mode();
    Del_c = Del_c_eps.emult(c_eps_bool) + Del_c_omg + (Del_c_v + Del_c_x)*c_alt_bool;

    if(abs(proj_theta_status) == 10){ // swap roll and yaw proportional compensation if above the treshold pitch deg
        Del_c(0,0) = -Del_c_eps(2,0)*c_eps_bool(2,0) + Del_c_omg(0,0);
        Del_c(2,0) = Del_c_eps(0,0)*c_eps_bool(0,0) + Del_c_omg(2,0);
    }

    c_eps_satur = c_eps_satur.emult(c_eps_bool); // put the component to zero if stabilisation disabled
    if((Del_c(2,0) > 0.0f) && (c_eps_satur(0,0) < 0.0f)){
        Del_c(2,0) = math::constrain(Del_c(2,0) + c_eps_satur(0,0), 0.0f, 1.0f);  // add the negative saturated roll from the positive yaw compensation that is likely causing this by diff thrust
    }
    else if((Del_c(2,0) < 0.0f) && (c_eps_satur(0,0) > 0.0f)){
        Del_c(2,0) = math::constrain(Del_c(2,0) + c_eps_satur(0,0), -1.0f, 0.0f); // add the positive saturated roll from the negative yaw compensation that is likely causing this by diff thrust
    }

    cf = c_setpoint + Del_c;
    c_alt_support = math::constrain(c_alt_support + math::constrain(cf(3,0)-1.0f, 0.0f, 1.0f), 0.0f, 1.0f); // add what will be scrapped by cf limit and then limit the total to [0,1]
        
    return PX4_OK;
}
int My_LQR_control::project_del_psi(){
// If the heading error is more than 180 degree, it is faster to correct for it by turning the opposite way.
    proj_dpsi_status = 0;
    if(proj_dpsi){
        proj_dpsi_status = 1;
        if(Del_y_eps(2,0) >  MY_PI){
            Del_y_eps(2,0) = -2*MY_PI + Del_y_eps(2,0);
            proj_dpsi_status = 10;
        }
        else if(Del_y_eps(2,0) < -MY_PI){
            Del_y_eps(2,0) =  2*MY_PI + Del_y_eps(2,0);
            proj_dpsi_status = -10;
        }
    }
    return PX4_OK;
}
int My_LQR_control::stabilisation_mode(){
// Decide whether to disable pitch or yaw compensation
    c_eps_bool.setAll(1.0f);
    if(rc_channels.channels[14] < 0.5f){ // pitch/roll rate compensation only
        if(rc_channels.channels[6] > 0.0f){ // roll
            c_eps_bool(0,0) = 0.0f;
        }
        else{ // pitch 
            c_eps_bool(1,0) = 0.0f;
        }
    }
    if(rc_channels.channels[5] < 0.5f){ // yaw rate compensation only
        c_eps_bool(2,0) = 0.0f;
    }


    c_alt_bool = 0.0f;
    if(rc_channels.channels[13] > 0.5f){ // altitude stab on
        c_alt_bool = 1.0f;
    }

    return PX4_OK;
}




int My_LQR_control::manual_override(){
    cm.setAll(0.0f);
    cm(0,0) =  rc_channels.channels[0] * RC_scale_base(0,0) + c_setpoint(0,0);
    cm(1,0) = -rc_channels.channels[1] * RC_scale_base(1,0) + c_setpoint(1,0);
    cm(2,0) =  rc_channels.channels[2] * RC_scale_base(2,0) + c_setpoint(2,0);
    cm(3,0) =  c_setpoint(3,0);
    
    if(rc_channels.channels[14] < -0.5f){ // manual override pitch/roll
        if(rc_channels.channels[6] > 0.0f){ // roll override
            cf(0,0) = cm(0,0);
        }
        else{ // pitch override
            cf(1,0) = cm(1,0);
        }
    }
    if(rc_channels.channels[5] < -0.5f){ // manual override yaw
        cf(2,0) = cm(2,0);
    }
    control_status = 0;
    if(!isbound(cf(0,0)) || !isbound(cf(1,0)) || !isbound(cf(2,0)) || !isbound(cf(3,0))){ // check for errors in the stabilised control
        control_status = 1;
    }
    if(rc_channels.channels[13] < -0.5f || control_status == 1){ // manual override all
        cf.setAll(0.0f);
        cf(0,0) = cm(0,0);
        cf(1,0) = cm(1,0);
        cf(2,0) = cm(2,0);
        cf(3,0) = cm(3,0);
    }

    return PX4_OK;
}

int My_LQR_control::px4_override(){
    if(rc_channels.channels[13] > 0.5f){ // PX4 override
        if((vehicle_id != 2) && (vehicle_id != 3)){ // don't let this happen in custer, px4 not set up yet and could be in some weird hold mode
            actuator_controls_virtual_poll();
            for(int i=0; i<8; i++){
                cf(i,0) = actuator_controls_virtual.control[i];
            }
        }
    }
    return PX4_OK;
}

int My_LQR_control::supporting_outputs(){
// Outputs to the actuator_controls_1

    uf.setAll(0.0f);

    // front propeller thrust
    uf(3,0) = math::constrain(c_nominal_control(3,0) + rc_channels.channels[8]/2.0f + c_alt_support*c_alt_bool, 0.0f, 1.0f); // adding alto the alt_hold support not divided by 2 because I want up to full throttle

    // roll support to the spilt elevators
    if(rc_channels.channels[12] > -0.5f){
        uf(0,0) = tailerons_scaling*cf(0,0);
    }

    // roll and yaw support to the wing motors
    if(rc_channels.channels[12] > 0.5f){
        uf(6,0) = motorons_p_scaling*cf(0,0);
        uf(7,0) = motorons_r_scaling*cf(2,0);
    }
    
    return PX4_OK;
}

int My_LQR_control::rc_loss_failsafe(){
    if(rc_channels.signal_lost == true){
        dt_rcloss = dt_rcloss + dt;
        f_rcloss = math::constrain(dt_rcloss/4.0f, 0.0f, 1.0f);
        for(int i=0; i<16; i++){
            rc_channels.channels[i] = (1.0f - f_rcloss)*rc_channels_prev.channels[i] + (f_rcloss)*rc_channels_fail.channels[i];
        }
        if(dt_rcloss >= 1000000.0f){ // not to get an overflow
            dt_rcloss = 5.0f;
        }
    }
    else{
        rc_channels_prev = rc_channels;
        dt_rcloss = 0.0f;
    }

    return PX4_OK;
}

int My_LQR_control::bound_controls(){
    
    for(int i = 0; i < 3; i++){
        cf(i,0) = math::constrain(cf(i,0), -1.0f, 1.0f);
    }
    cf(3,0) = math::constrain(cf(3,0), 0.0f, 1.0f); // thrust only positive

    for(int i = 0; i < 3; i++){
        uf(i,0) = math::constrain(uf(i,0), -1.0f, 1.0f);
    }
    uf(3,0) = math::constrain(uf(3,0), 0.0f, 1.0f); // thrust only positive
    for(int i = 4; i < 8; i++){
        uf(i,0) = math::constrain(uf(i,0), -1.0f, 1.0f);
    }

    return PX4_OK;
}





int My_LQR_control::debug_printouts(){
    if(do_printouts){ // Never ever do this in flight, only for debug. The messaging is slow and blocking
        dt_print = dt_print + dt;
        if(dt_print > 2.0f){
            PX4_INFO(" ");
            PX4_INFO("KD p = %5.2f, KD q = %5.2f, KD r = %5.2f", (double)K_feedback_y_sc_tun_sched(0,6), (double)K_feedback_y_sc_tun_sched(1,7), (double)K_feedback_y_sc_tun_sched(2,8));
            PX4_INFO("KP p = %5.2f, KP q = %5.2f, KP r = %5.2f", (double)K_feedback_y_sc_tun_sched(0,9), (double)K_feedback_y_sc_tun_sched(1,10), (double)K_feedback_y_sc_tun_sched(2,11));
            dt_print = 0.0f;

            PX4_INFO("c_alt_support for throttle: %2.5f", (double)c_alt_support);
        }
    }
    return PX4_OK;
}




/*--------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------*/
int My_LQR_control::initialize_variables(){
    y_setpoint.setAll(0.0f);
    c_setpoint.setAll(0.0f);
    r_setpoint.setAll(0.0f);
    pitch_setpoint = 0.0f;
    control_status = 0;
    altitude_mode = 0;

    y.setAll(0.0f);
    r.setAll(0.0f);
    cf.setAll(0.0f);
    uf.setAll(0.0f);

    K_feedback_y.setAll(0.0f);
    K_feedback_int.setAll(0.0f);
    K_feedback_cf.setAll(0.0f);
    c_nominal_control.setAll(0.0f);

    tune_p_p = 1.0f;
    tune_d_p = 1.0f;
    tune_p_q = 1.0f;
    tune_d_q = 1.0f;
    tune_p_r = 1.0f;
    tune_d_r = 1.0f;

    k_scheds.setAll(0.0f);
    k_sw.setAll(0.0f);

    if(vehicle_id == 1){ // S500 quad
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.11f; K_feedback_y(0,7) =   0.00f; K_feedback_y(0,8) =   0.00f; K_feedback_y(0,9) =   0.50f; K_feedback_y(0,10) =   0.00f; K_feedback_y(0,11) =   0.00f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =   0.00f; K_feedback_y(1,7) =   0.11f; K_feedback_y(1,8) =   0.00f; K_feedback_y(1,9) =   0.00f; K_feedback_y(1,10) =   0.50f; K_feedback_y(1,11) =   0.00f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =   0.00f; K_feedback_y(2,7) =   0.00f; K_feedback_y(2,8) =   0.22f; K_feedback_y(2,9) =   0.00f; K_feedback_y(2,10) =   0.00f; K_feedback_y(2,11) =   0.50f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.00f; K_feedback_y(3,7) =   0.00f; K_feedback_y(3,8) =   0.00f; K_feedback_y(3,9) =   0.00f; K_feedback_y(3,10) =   0.00f; K_feedback_y(3,11) =   0.00f; 
    }
    else if(vehicle_id == 2){ // Custer
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.1000f; K_feedback_y(0,7) =  -0.0000f; K_feedback_y(0,8) =   0.0000f; K_feedback_y(0,9) =   1.7500f; K_feedback_y(0,10) =  -0.0000f; K_feedback_y(0,11) =   0.0000f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =  -0.0000f; K_feedback_y(1,7) =   0.0500f; K_feedback_y(1,8) =  -0.0000f; K_feedback_y(1,9) =  -0.0000f; K_feedback_y(1,10) =   1.7500f; K_feedback_y(1,11) =  -0.0000f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =  -0.0000f; K_feedback_y(2,7) =  -0.0000f; K_feedback_y(2,8) =   0.1000f; K_feedback_y(2,9) =  -0.0000f; K_feedback_y(2,10) =  -0.0000f; K_feedback_y(2,11) =   1.7500f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.0000f; K_feedback_y(3,7) =   0.0000f; K_feedback_y(3,8) =   0.0000f; K_feedback_y(3,9) =   0.0000f; K_feedback_y(3,10) =   0.0000f; K_feedback_y(3,11) =   0.0000f; 
k_scheds(0,0) =   0.10f; k_scheds(0,1) =   0.10f; k_scheds(0,2) =   0.70f; k_scheds(0,3) =   1.20f; k_scheds(0,4) =   1.45f; k_scheds(0,5) =   1.45f; 
k_scheds(1,0) =   0.00f; k_scheds(1,1) =   0.00f; k_scheds(1,2) =   0.00f; k_scheds(1,3) =   0.00f; k_scheds(1,4) =   0.00f; k_scheds(1,5) =   0.00f;  
k_scheds(2,0) =   0.00f; k_scheds(2,1) =   0.00f; k_scheds(2,2) =   0.00f; k_scheds(2,3) =   0.00f; k_scheds(2,4) =   0.00f; k_scheds(2,5) =   0.00f;  
k_scheds(3,0) =   0.10f; k_scheds(3,1) =   0.10f; k_scheds(3,2) =   0.70f; k_scheds(3,3) =   1.20f; k_scheds(3,4) =   1.20f; k_scheds(3,4) =   1.20f;  
k_scheds(4,0) =   1.30f; k_scheds(4,1) =   2.00f; k_scheds(4,2) =   5.00f; k_scheds(4,3) =   6.00f; k_scheds(4,4) =   6.00f; k_scheds(4,5) =   6.00f; 
k_scheds(5,0) =   0.00f; k_scheds(5,1) =   0.00f; k_scheds(5,2) =   0.00f; k_scheds(5,3) =   0.00f; k_scheds(5,4) =   0.00f; k_scheds(5,5) =   0.00f; 
k_scheds(6,0) =   0.00f; k_scheds(6,1) =   0.00f; k_scheds(6,2) =   0.00f; k_scheds(6,3) =   0.00f; k_scheds(6,4) =   0.00f; k_scheds(6,5) =   0.00f; 
k_scheds(7,0) =   1.30f; k_scheds(7,1) =   1.30f; k_scheds(7,2) =   1.75f; k_scheds(7,3) =   1.75f; k_scheds(7,4) =   1.75f; k_scheds(7,5) =   1.75f; 
k_scheds(8,0) =   0.05f; k_scheds(8,1) =   0.05f; k_scheds(8,2) =   0.12f; k_scheds(8,3) =   0.15f; k_scheds(8,4) =   0.15f; k_scheds(8,5) =   0.15f; 
k_scheds(9,0) =   1.40f; k_scheds(9,1) =   2.00f; k_scheds(9,2) =   3.00f; k_scheds(9,3) =   4.00f; k_scheds(9,4) =   4.00f; k_scheds(9,5) =   4.00f; 
        tht_ints(0,0) =  -1.5708f; tht_ints(0,1) =   0.0000f; tht_ints(0,2) =   0.3491f; tht_ints(0,3) =   0.6981f; tht_ints(0,4) =   1.0472f; tht_ints(0,5) =   1.5708f; 
        // pitch angles (0.35, 0.52, 0.70, 0.78, 0.87, 1.04, 1.57 rad = 20, 30, 40, 45, 50, 60, 90 deg)
        //// [pp, pr, rp, rr, ..., q, tht]
        k_sw(0,0) = 0.02f; k_sw(0,1) = 0.00f;
        k_sw(1,0) = 0.02f; k_sw(1,1) = 0.00f;
        k_sw(2,0) = 0.00f; k_sw(2,1) = 0.01f;
        k_sw(3,0) = 0.00f; k_sw(3,1) = 0.01f;
        // at fast&cruise vs slow&hover
        //// [qz, qvz, mz, mvz];
    }
    else if(vehicle_id == 3){ // Custer HITL
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.1000f; K_feedback_y(0,7) =  -0.0000f; K_feedback_y(0,8) =   0.2000f; K_feedback_y(0,9) =   1.2910f; K_feedback_y(0,10) =  -0.0000f; K_feedback_y(0,11) =   0.2000f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =  -0.0000f; K_feedback_y(1,7) =   0.0500f; K_feedback_y(1,8) =  -0.0000f; K_feedback_y(1,9) =  -0.0000f; K_feedback_y(1,10) =   1.2910f; K_feedback_y(1,11) =  -0.0000f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =  -0.0000f; K_feedback_y(2,7) =  -0.0000f; K_feedback_y(2,8) =   0.1000f; K_feedback_y(2,9) =  -0.0000f; K_feedback_y(2,10) =  -0.0000f; K_feedback_y(2,11) =   1.2910f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.0000f; K_feedback_y(3,7) =   0.0000f; K_feedback_y(3,8) =   0.0000f; K_feedback_y(3,9) =   0.0000f; K_feedback_y(3,10) =   0.0000f; K_feedback_y(3,11) =   0.0000f; 
k_scheds(0,0) =   0.01f; k_scheds(0,1) =   0.01f; k_scheds(0,2) =   0.01f; k_scheds(0,3) =   0.10f; k_scheds(0,4) =   0.50f; k_scheds(0,5) =   0.50f; k_scheds(0,6) =   0.50f; 
k_scheds(1,0) =   0.00f; k_scheds(1,1) =   0.00f; k_scheds(1,2) =   0.00f; k_scheds(1,3) =   0.00f; k_scheds(1,4) =   0.00f; k_scheds(1,5) =   0.00f; k_scheds(1,6) =   0.00f; 
k_scheds(2,0) =   0.00f; k_scheds(2,1) =   0.00f; k_scheds(2,2) =   0.00f; k_scheds(2,3) =   0.00f; k_scheds(2,4) =   0.00f; k_scheds(2,5) =   0.00f; k_scheds(2,6) =   0.00f;
k_scheds(3,0) =   0.01f; k_scheds(3,1) =   0.01f; k_scheds(3,2) =   0.01f; k_scheds(3,3) =   0.80f; k_scheds(3,4) =   1.00f; k_scheds(3,5) =   4.00f; k_scheds(3,6) =   4.00f;
k_scheds(4,0) =   1.20f; k_scheds(4,1) =   1.20f; k_scheds(4,2) =   1.30f; k_scheds(4,3) =   1.30f; k_scheds(4,4) =   1.00f; k_scheds(4,5) =   1.00f; k_scheds(4,6) =   0.20f;
k_scheds(5,0) =  -0.00f; k_scheds(5,1) =  -0.00f; k_scheds(5,2) =  -0.00f; k_scheds(5,3) =  -0.00f; k_scheds(5,4) =  -0.00f; k_scheds(5,5) =  -0.00f; k_scheds(5,6) =  -0.00f;
k_scheds(6,0) =  -0.00f; k_scheds(6,1) =  -0.00f; k_scheds(6,2) =  -0.00f; k_scheds(6,3) =  -0.00f; k_scheds(6,4) =  -0.00f; k_scheds(6,5) =  -0.00f; k_scheds(6,6) =  -0.00f;
k_scheds(7,0) =   0.35f; k_scheds(7,1) =   0.35f; k_scheds(7,2) =   0.35f; k_scheds(7,3) =   0.35f; k_scheds(7,4) =   0.35f; k_scheds(7,5) =   0.06f; k_scheds(7,6) =   0.60f;
k_scheds(8,0) =   0.01f; k_scheds(8,1) =   0.01f; k_scheds(8,2) =   0.01f; k_scheds(8,3) =   0.01f; k_scheds(8,4) =   0.01f; k_scheds(8,5) =   0.01f; k_scheds(8,6) =   0.01f;
k_scheds(9,0) =   1.30f; k_scheds(9,1) =   1.30f; k_scheds(9,2) =   2.40f; k_scheds(9,3) =   3.50f; k_scheds(9,4) =   3.50f; k_scheds(9,5) =   3.50f; k_scheds(9,6) =   2.60f; 
        tht_ints(0,0) =  -1.5708f; tht_ints(0,1) =   0.0000f; tht_ints(0,2) =   0.3491f; tht_ints(0,3) =   0.6981f; tht_ints(0,4) =   0.87f; tht_ints(0,5) =   1.04f; tht_ints(0,6) = 1.57f;
        // pitch angles (0.35, 0.52, 0.70, 0.87 rad = 20, 30, 40, 50 deg)
        //// [pp, pr, rp, rr, ..., q, tht]
        k_sw(0,0) = 0.02f; k_sw(0,1) = 0.00f;
        k_sw(1,0) = 0.02f; k_sw(1,1) = 0.00f;
        k_sw(2,0) = 0.00f; k_sw(2,1) = 0.01f;
        k_sw(3,0) = 0.00f; k_sw(3,1) = 0.01f;
        // at fast&cruise vs slow&hover
        //// [qz, qvz, mz, mvz];
    }
    else{ // Not specified
        PX4_WARN("No airframe specified, using unit gains K");
        K_feedback_y(0,0) =   0.0000f; K_feedback_y(0,1) =   0.0000f; K_feedback_y(0,2) =   0.0000f; K_feedback_y(0,3) =   0.0000f; K_feedback_y(0,4) =   0.0000f; K_feedback_y(0,5) =   0.0000f; K_feedback_y(0,6) =   0.01f; K_feedback_y(0,7) =   0.00f; K_feedback_y(0,8) =   0.00f; K_feedback_y(0,9) =   0.01f; K_feedback_y(0,10) =   0.00f; K_feedback_y(0,11) =   0.00f; 
        K_feedback_y(1,0) =   0.0000f; K_feedback_y(1,1) =   0.0000f; K_feedback_y(1,2) =   0.0000f; K_feedback_y(1,3) =   0.0000f; K_feedback_y(1,4) =   0.0000f; K_feedback_y(1,5) =   0.0000f; K_feedback_y(1,6) =   0.00f; K_feedback_y(1,7) =   0.01f; K_feedback_y(1,8) =   0.00f; K_feedback_y(1,9) =   0.00f; K_feedback_y(1,10) =   0.01f; K_feedback_y(1,11) =   0.00f; 
        K_feedback_y(2,0) =   0.0000f; K_feedback_y(2,1) =   0.0000f; K_feedback_y(2,2) =   0.0000f; K_feedback_y(2,3) =   0.0000f; K_feedback_y(2,4) =   0.0000f; K_feedback_y(2,5) =   0.0000f; K_feedback_y(2,6) =   0.00f; K_feedback_y(2,7) =   0.00f; K_feedback_y(2,8) =   0.01f; K_feedback_y(2,9) =   0.00f; K_feedback_y(2,10) =   0.00f; K_feedback_y(2,11) =   0.01f; 
        K_feedback_y(3,0) =   0.0000f; K_feedback_y(3,1) =   0.0000f; K_feedback_y(3,2) =   0.0000f; K_feedback_y(3,3) =   0.0000f; K_feedback_y(3,4) =   0.0000f; K_feedback_y(3,5) =   0.0000f; K_feedback_y(3,6) =   0.00f; K_feedback_y(3,7) =   0.00f; K_feedback_y(3,8) =   0.00f; K_feedback_y(3,9) =   0.00f; K_feedback_y(3,10) =   0.00f; K_feedback_y(3,11) =   0.00f; 
    }

    c_nominal_control(3,0) = 0.5f; // don't change this, otherwise the RC is not able to go to full 1 or full 0. Add a trim variable to c_setpoint after rc is added if you want to change it

    K_feedback_y_scaled = K_feedback_y;
    K_feedback_y_scaled_tuned = K_feedback_y_scaled;
    K_feedback_y_sc_tun_sched = K_feedback_y_scaled_tuned;

    k_scheds_sc = k_scheds;
    k_scheds_sc_tun = k_scheds_sc;

    Ci.setAll(0.0f);
    Ci(0, 0) = 1.0f;
    Ci(1, 1) = 1.0f;
    Ci(2, 2) = 1.0f;
    Ci(3, 9) = 1.0f;
    Ci(4,10) = 1.0f;
    Ci(5,11) = 1.0f;

    manual_control_setpoint.x = 0.0f;
    manual_control_setpoint.y = 0.0f;
    manual_control_setpoint.z = 0.0f;
    manual_control_setpoint.r = 0.0f;
    manual_control_setpoint.aux1 = 0.0f;
    manual_control_setpoint.aux2 = 0.0f;

    rc_channels_fail.channels[0] = 0.1f;
    rc_channels_fail.channels[1] = 0.0f;
    rc_channels_fail.channels[2] = 0.0f;
    rc_channels_fail.channels[9] = -1.0f; // pitch setpoint 0 deg
    rc_channels_fail.channels[12] = 0.0f; // mixer setting - use tailerons 
    rc_channels_fail.channels[13] = 0.0f; // manual/px4 override - off, keep stabilisation on
    rc_channels_fail.channels[14] = 1.0f; // manual override pitch - off, keep pitch hold
    rc_channels_fail.channels[5] = 0.0f; // manual override yaw - damp, keep yaw damping only
    rc_channels_fail.channels[11] = 0.0f; // gains scale omg
    rc_channels_fail.channels[10] = 0.0f; // gains scale eps
    rc_channels_fail.channels[3] = 0.0f; // motors off
    rc_channels_fail.channels[8] = -1.0f; // front engine off
    rc_channels_fail.channels[4] = -1.0f; // kill channel, not used here
    rc_channels_fail.channels[6] = -1.0f; // px modes, not used here
    rc_channels_fail.channels[7] = 1.0f; // arming channel as well, not used here
    rc_channels_fail.channels[15] = 1.0f; // arming channel, not used here

    rc_channels = rc_channels_fail;
    rc_channels_prev = rc_channels_fail;


    vehicle_attitude.rollspeed = 0.0f;
    vehicle_attitude.pitchspeed = 0.0f;
    vehicle_attitude.yawspeed = 0.0f;
    vehicle_attitude.q[0] = 0.0f;
    vehicle_attitude.q[1] = 0.0f;
    vehicle_attitude.q[2] = 0.0f;
    vehicle_attitude.q[3] = 0.0f;

    vehicle_local_position.x = 0.0f;
    vehicle_local_position.y = 0.0f;
    vehicle_local_position.z = 0.0f;
    vehicle_local_position.vx = 0.0f;
    vehicle_local_position.vy = 0.0f;
    vehicle_local_position.vz = 0.0f;

    actuator_controls_virtual.control[0] = 0.0f;
    actuator_controls_virtual.control[1] = 0.0f;
    actuator_controls_virtual.control[2] = 0.0f;
    actuator_controls_virtual.control[3] = 0.0f;

    if(vehicle_id == 3){
        loop_update_freqn = 20.0f;
    }
    else{
        loop_update_freqn = 250.0f;
    }
    omg.setAll(0.0f);
    omg_filtered.setAll(0.0f);
    omg_filtered_temp_lp2.setAll(0.0f);
    omg_filtered_temp_lp3.setAll(0.0f);
    angular_rates_filtered.rollspeed = 0.0f;
    angular_rates_filtered.pitchspeed = 0.0f;
    angular_rates_filtered.yawspeed = 0.0f;
    angular_rates_filtered.loop_update_freqn = loop_update_freqn;
    angular_rates_filtered.cutoff_freqn_omg = cutoff_freqn_omg;
    eps.setAll(0.0f);
    eps_filtered.setAll(0.0f);
    eps_filtered_temp_lp2.setAll(0.0f);
    eps_filtered_temp_lp3.setAll(0.0f);
    angular_rates_filtered.roll = 0.0f;
    angular_rates_filtered.pitch = 0.0f;
    angular_rates_filtered.yaw = 0.0f;
    angular_rates_filtered.cutoff_freqn_eps = cutoff_freqn_eps;
    RC.setAll(0.0f);
    RC_filtered.setAll(0.0f);
    angular_rates_filtered.rc_roll = 0.0f;
    angular_rates_filtered.rc_pitch = 0.0f;
    angular_rates_filtered.rc_yaw = 0.0f;
    angular_rates_filtered.cutoff_freqn_rc = cutoff_freqn_RC;

    Del_c_lim.setAll(1.0f);
    c_eps_satur.setAll(0.0f);

    E2B.setAll(0.0f);
    E2B(0,0) = 1.0f;
    E2B(1,1) = 1.0f;
    E2B(2,2) = 1.0f;


    X0_RLS(0,0) = -7.0f;
    X0_RLS(1,0) = 25.0f;
    P0_RLS(0,0) = 0.1481f/1000.0f;
    P0_RLS(0,1) = -0.1784f/1000.0f;
    P0_RLS(1,0) = -0.1784f/1000.0f;
    P0_RLS(1,1) = 0.7632f/1000.0f;
    X_RLS = X0_RLS;
    P_RLS = P0_RLS;
    K_p_adapt = kp_adapt;
    K_phi_adapt = kphi_adapt;

    oscillating.setAll(0);
    gain_limiter.setAll(1.0f);

    update_parameters(true);

    return PX4_OK;
}

int My_LQR_control::local_parameters_update(){
    y_max.setAll(0.0f);
    y_max(3,0) = param_max_u.get();
    y_max(4,0) = param_max_v.get();
    y_max(5,0) = param_max_w.get();
    y_max(6,0) = param_max_p.get();
    y_max(7,0) = param_max_q.get();
    y_max(8,0) = param_max_r.get();
    y_max(9,0) = param_max_phi.get();
    y_max(10,0) = param_max_theta.get();
    y_max(11,0) = param_max_psi.get();

    RC_scale_base(0,0) = rc_scale_p.get();
    RC_scale_base(1,0) = rc_scale_q.get();
    RC_scale_base(2,0) = rc_scale_r.get();
    alt_rate_rc_scale = rc_scale_m.get();

    c_nominal_control(1,0) = cq_trim.get();
    
    Tf.setAll(0.0f);
    for(int i = 0; i < 4; i++){
        Tf(i,i) = f_lag.get();
    }

    gains_scale();

    Del_c_lim(0,0) = dx_lim.get();
    Del_c_lim(1,0) = dv_lim.get();
    Del_c_lim(2,0) = domg_lim.get();
    Del_c_lim(3,0) = deps_lim.get();

    lpf_order = lpf_ord.get();
    if(fabsf(cutoff_freqn_omg - math::min(cutoff_fn_omg.get(), 300.0f)) > 0.1f){
        cutoff_freqn_omg = math::min(cutoff_fn_omg.get(), 300.0f);
        lp2_filter_omg.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_omg);
        lp3_filter_omg.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_omg);
    }
    if(fabsf(cutoff_freqn_eps - math::constrain(cutoff_fn_eps.get(), 1.0f, 300.0f)) > 0.1f){
        cutoff_freqn_eps = math::constrain(cutoff_fn_eps.get(), 1.0f, 300.0f);
        lp2_filter_eps.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_eps);
        lp3_filter_eps.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_eps);
    }
    if(fabsf(cutoff_freqn_RC - math::constrain(cutoff_fn_RC.get(), 1.0f, 300.0f)) > 0.1f){
        cutoff_freqn_RC = math::constrain(cutoff_fn_RC.get(), 1.0f, 300.0f);
        lp2_filter_RC.set_cutoff_frequency(loop_update_freqn, cutoff_freqn_RC);
    }

    tailerons_scaling = tailerons_sc.get();
    motorons_p_scaling = motorons_p_sc.get();
    motorons_r_scaling = motorons_r_sc.get();


    e2b = bool_e2b.get() == 1;

    schedule_K = bool_K_sched.get() == 1;

    do_rc_scale = bool_rc_sc.get() == 1;

    proj_theta = bool_proj_tht.get() == 1;

    proj_dpsi = bool_proj_dpsi.get() == 1;

    do_printouts = bool_printouts.get() == 1;

    tune_expo = tune_ex.get();
    tune_mode = tune_mod.get();

    pitch_sp_max = tht_sp_m.get();
    pitch_sp_min = tht_sp_min.get();

    do_recursiveLS = bool_recursiveLS.get() == 1;
    lambda_RLS = lambda_rls.get();
    do_adaptive = bool_adaptive.get() == 1;

    gains_limiter_on = bool_gains_limiter.get() == 1;
    if(fabsf(pksz - glm_pksz.get()) > 0.0001f || fabsf(pksz - glm_pksz.get()) > 0.0001f){
        pksz = glm_pksz.get();
        dtlim = glm_dtlim.get();
        detected_oscillations.reset(pksz, dtlim);
    }
    
    return PX4_OK;
}

int My_LQR_control::update_parameters(bool force){
    bool updated = false;
    updated = parameter_update_poll() == PX4_OK;
    if(updated || force){
        updateParams();
        local_parameters_update();
    }
    return PX4_OK;
}

float My_LQR_control::deg2rad(float degs){
    return degs*0.01745329252f;
}

float My_LQR_control::rad2deg(float rads){
    return rads/0.01745329252f;
}

bool My_LQR_control::isbound(float val){
    return (val >= -100) && (val <= 100); // bounds for angular rates
}



/*--------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------*/
int My_LQR_control::del_epsilon_to_body_frame(){
// Non-linear transformation on the Del_epsilon     
    if(e2b){
        /*E2B(0,0) = 1.0f;
        E2B(0,2) = -sin(y(10,0));
        E2B(1,1) = cos(y(9,0));
        E2B(1,2) = sin(y(9,0))*cos(y(10,0));
        E2B(2,1) = -sin(y(9,0));
        E2B(2,2) = cos(y(9,0))*cos(y(10,0)); */

        E2B(0,0) = 1.0f;
        E2B(0,2) = -sin(y(10,0));
        E2B(1,1) = 1.0f;
        E2B(1,2) = 0.0f;
        E2B(2,1) = 0.0f;
        E2B(2,2) = cos(y(10,0));

        Del_y_eps = E2B*Del_y.slice<3,1>(9,0);
    }

    return PX4_OK;
}


/*--------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------*/
int my_LQR_control_main(int argc, char *argv[])
{
    return My_LQR_control::main(argc, argv);
}
