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
 * @file my_LQR_printouts.cpp
 *
 * My_LQR_printouts module
 *
 * @author Juraj Mihalik <jm1e16@soton.ac.uk>
 */

#include "my_LQR_printouts.hpp"


int My_LQR_printouts::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description
This implements the printouts for LQR controller. 

### Examples
To start the printouts:
$ my_LQR_printouts start


)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("my_LQR_printouts", "printouts");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional flag", true);
        PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Vehicle ID.\n\t\tQuadS500: 1\n\t\tCuster: 2", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

int My_LQR_printouts::print_status()
{
        PX4_INFO("Running in mode %d", vehicle_id);
        // TODO: print additional runtime information about the state of the module

        return 0;
}

int My_LQR_printouts::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("my_LQR_printouts",
                                      SCHED_DEFAULT,
                                      50, // priority
                                      1500, // stack size
                                      (px4_main_t)&run_trampoline,
                                      (char *const *)argv);

        if (_task_id < 0) {
                _task_id = -1;
                return -errno;
        }

        return 0;
}

My_LQR_printouts *My_LQR_printouts::instantiate(int argc, char *argv[])
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

        My_LQR_printouts *instance = new My_LQR_printouts(arg_param, arg_flag);

        if (instance == nullptr) {
                PX4_ERR("alloc failed");
        }

        return instance;
}

My_LQR_printouts::My_LQR_printouts(int arg_param, bool arg_flag)
        : ModuleParams(nullptr), _loop_perf(perf_alloc(PC_ELAPSED, "my_LQR_printouts"))
{
    vehicle_id = arg_param;
}

int My_LQR_printouts::custom_command(int argc, char *argv[])
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



int My_LQR_printouts::parameter_update_poll()
{
    bool parameter_update_updated;
    orb_check(parameter_update_sub, &parameter_update_updated);
    if(parameter_update_updated){
        orb_copy(ORB_ID(parameter_update), parameter_update_sub, &parameter_update);
        return PX4_OK;
    }
    return PX4_ERROR;
}


int My_LQR_printouts::my_rpm_topic_poll(){
    bool my_rpm_topic_updated;
    orb_check(my_rpm_topic_sub, &my_rpm_topic_updated);
    if(my_rpm_topic_updated){
        orb_copy(ORB_ID(my_rpm_topic), my_rpm_topic_sub, &my_rpm_topic);
        current1 = 0.9f*current1 + 0.1f*my_rpm_topic.current1;
        current2 = 0.9f*current2 + 0.1f*my_rpm_topic.current2;
        return PX4_OK;
    }
    if((hrt_absolute_time() - my_rpm_topic.timestamp) > 2000000){ // if no new data for 2 seconds
        my_rpm_topic.status = 5; // set different to zero
        current1 = 0.0f;
        current2 = 0.0f;
    }
    return PX4_ERROR;
}

int My_LQR_printouts::actuator_controls_poll(){
    bool actuator_controls_0_updated;
    orb_check(actuator_controls_0_sub, &actuator_controls_0_updated);
    if(actuator_controls_0_updated){
        orb_copy(ORB_ID(actuator_controls_0), actuator_controls_0_sub, &actuator_controls_0);
    }

    bool actuator_controls_1_updated;
    orb_check(actuator_controls_1_sub, &actuator_controls_1_updated);
    if(actuator_controls_1_updated){
        orb_copy(ORB_ID(actuator_controls_1), actuator_controls_1_sub, &actuator_controls_1);
        return PX4_OK;
    }
    
    return PX4_ERROR;
}

int My_LQR_printouts::my_LQR_setpoints_poll(){
    bool my_LQR_setpoints_updated;
    orb_check(my_LQR_setpoints_sub, &my_LQR_setpoints_updated);
    if(my_LQR_setpoints_updated){
        orb_copy(ORB_ID(my_LQR_setpoints), my_LQR_setpoints_sub, &my_LQR_setpoints);
        return PX4_OK;
    }
    
    return PX4_ERROR;
}

int My_LQR_printouts::angular_rates_filtered_poll(){
    bool angular_rates_filtered_updated;
    orb_check(angular_rates_filtered_sub, &angular_rates_filtered_updated);
    if(angular_rates_filtered_updated){
        orb_copy(ORB_ID(angular_rates_filtered), angular_rates_filtered_sub, &angular_rates_filtered);
        return PX4_OK;
    }
    
    return PX4_ERROR;
}




/*----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------*/
void My_LQR_printouts::run(){
    // subscribe to topics
    parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
    my_rpm_topic_sub = orb_subscribe(ORB_ID(my_rpm_topic));
    actuator_controls_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));    
    actuator_controls_1_sub = orb_subscribe(ORB_ID(actuator_controls_1));
    my_LQR_setpoints_sub = orb_subscribe(ORB_ID(my_LQR_setpoints));
    angular_rates_filtered_sub = orb_subscribe(ORB_ID(angular_rates_filtered));
    
    // initialize parameters
    initialize_variables();
    

    // Run the loop synchronized to the actuator_controls_0 topic.
    px4_pollfd_struct_t fds[1];
    fds[0].fd = actuator_controls_0_sub;
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

                my_rpm_topic_poll();
                my_LQR_setpoints_poll();
                actuator_controls_poll();
                angular_rates_filtered_poll();

                printouts();
                
            }
            perf_end(_loop_perf);
        }
        update_parameters();
    }
    
    
    // Unsubscribe
    orb_unsubscribe(parameter_update_sub);
    orb_unsubscribe(my_rpm_topic_sub);
    orb_unsubscribe(actuator_controls_0_sub);    
    orb_unsubscribe(actuator_controls_1_sub);
    orb_unsubscribe(my_LQR_setpoints_sub);
    orb_unsubscribe(angular_rates_filtered_sub);
}
/* -----------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------*/




int My_LQR_printouts::timer_clock(){
    dt = (actuator_controls_0.timestamp - time_last_run)/1000000.0f;
    time_last_run = actuator_controls_0.timestamp;
    
    return PX4_OK;
}

int My_LQR_printouts::printouts(){
    my_rpm_topic_poll();
    dt_print = dt_print + dt;

    if(dt_print > 2.0f){
        PX4_INFO(" ");
        PX4_INFO(" ");
        PX4_INFO(" ");
        //PX4_INFO("dt:%2.5f", (double)dt);

        if(my_rpm_topic.status == 0){
            print_my_rpm();
        }
        else if(my_rpm_topic.status == 5){
            PX4_INFO("my_rpm_topic sensors not updated");
        }
        else if(my_rpm_topic.status == 1){
            PX4_INFO("my_rpm_topic sensors reading failed");
        }

        
        PX4_INFO(" ");
        if(angular_rates_filtered.filter_status_omg == 1){
            PX4_ERR("Filtering rates results in NANs!");
        }
        if(angular_rates_filtered.filter_status_omg == 2){
            PX4_WARN("Filtering omg freqn off range 100Hz, disabled!");
        }
        if(angular_rates_filtered.filter_status_eps == 1){
            PX4_ERR("Filtering angles results in NANs!");
        }
        if(angular_rates_filtered.filter_status_rc == 1){
            PX4_ERR("Filtering RC results in NANs!");
        }
        if(my_LQR_setpoints.control_status == 1){
            PX4_ERR("Control resulted in NANs! Using manual.");
        }

        PX4_INFO(" ");
        PX4_INFO("c1(roll): %2.2f, c2(pitch): %2.2f, c3(yaw): %2.2f, c4(thrust): %2.4f", (double)actuator_controls_0.control[0], (double)actuator_controls_0.control[1], (double)actuator_controls_0.control[2], (double)actuator_controls_0.control[3]);

        PX4_INFO(" ");
        PX4_INFO("Alt_setpoint = %4.2f, Altitude = %4.2f", (double)my_LQR_setpoints.sp_y2, (double)my_LQR_setpoints.y2);
        PX4_INFO("Altitude mode = %d, c_qx = %4.3f, c_qv = %4.3f, c_mx = %4.3f, c_mv = %4.3f", my_LQR_setpoints.alt_mode, (double)my_LQR_setpoints.del_c_x_q, (double)my_LQR_setpoints.del_c_v_q, (double)my_LQR_setpoints.del_c_x_m, (double)my_LQR_setpoints.del_c_v_m);
        
        PX4_INFO(" ");
        PX4_WARN("pitch setpoint [deg]: %3.1f", (double)my_LQR_setpoints.pitch_setpoint);   
        PX4_INFO("Scheduler interval: %2.4f", (double)my_LQR_setpoints.case_int_f_int);

        PX4_INFO(" ");
        PX4_INFO("KD p = %5.2f, KD q = %5.2f, KD r = %5.2f", (double)my_LQR_setpoints.k_omg_pp, (double)my_LQR_setpoints.k_omg_qq, (double)my_LQR_setpoints.k_omg_rr);
        PX4_INFO("KP p = %5.2f, KP q = %5.2f, KP r = %5.2f", (double)my_LQR_setpoints.k_eps_phiphi, (double)my_LQR_setpoints.k_eps_thttht, (double)my_LQR_setpoints.k_eps_psipsi);
        
        if(my_LQR_setpoints.oscillating_p){
            PX4_INFO(" ");
            PX4_INFO("glm_p: %1.2f", (double)my_LQR_setpoints.gain_limiter_p);
        }

        if(my_LQR_setpoints.oscillating_q){
            PX4_INFO(" ");
            PX4_INFO("glm_q: %1.2f", (double)my_LQR_setpoints.gain_limiter_q);
        }

        if(my_LQR_setpoints.oscillating_r){
            PX4_INFO(" ");
            PX4_INFO("glm_r: %1.2f", (double)my_LQR_setpoints.gain_limiter_r);
        }

        PX4_INFO(" ");
        PX4_INFO("c_alt_support for front engine: %2.5f", (double)my_LQR_setpoints.c_alt_support);

        dt_print = 0.0f;
    }
    return PX4_OK;
}




/*--------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------*/
int My_LQR_printouts::initialize_variables(){

    my_rpm_topic.status = 1; // manually preset different to zero to make sure it's faulty if no updates

    update_parameters(true);

    return PX4_OK;
}

int My_LQR_printouts::local_parameters_update(){
    printouts_dt = prnt_dt.get();

    return PX4_OK;
}

int My_LQR_printouts::update_parameters(bool force){
    bool updated = false;
    updated = parameter_update_poll() == PX4_OK;
    if(updated || force){
        updateParams();
        local_parameters_update();
    }
    return PX4_OK;
}

float My_LQR_printouts::deg2rad(float degs){
    return degs*0.01745329252f;
}

float My_LQR_printouts::rad2deg(float rads){
    return rads/0.01745329252f;
}


int My_LQR_printouts::print_my_rpm(){
    PX4_INFO("RPM front: %llu", my_rpm_topic.rpm);
    PX4_INFO("current1: %3.0f [A], current2: %3.0f [A]", (double) current1, (double) current2);

    telem_string1[0] = my_rpm_topic.telem1_1;
    telem_string1[1] = my_rpm_topic.telem1_2;
    telem_string1[2] = my_rpm_topic.telem1_3;
    telem_string1[3] = my_rpm_topic.telem1_4;
    telem_string1[4] = my_rpm_topic.telem1_5;
    telem_string1[5] = my_rpm_topic.telem1_6;
    telem_string1[6] = my_rpm_topic.telem1_7;
    telem_string1[7] = my_rpm_topic.telem1_8;
    telem_string1[8] = my_rpm_topic.telem1_9;
    telem_string1[9] = my_rpm_topic.telem1_10;
    telem_string1[10] = my_rpm_topic.telem1_11;
    telem_string1[11] = my_rpm_topic.telem1_12;
    telem_string1[12] = my_rpm_topic.telem1_13;
    telem_string1[13] = my_rpm_topic.telem1_14;
    telem_string1[14] = my_rpm_topic.telem1_15;
    telem_string1[15] = my_rpm_topic.telem1_16;
    telem_string1[16] = my_rpm_topic.telem1_17;
    telem_string1[17] = my_rpm_topic.telem1_18;
    telem_string1[18] = my_rpm_topic.telem1_19;
    telem_string1[19] = my_rpm_topic.telem1_20;
    telem_string1[20] = my_rpm_topic.telem1_21;
    telem_string1[21] = my_rpm_topic.telem1_22;
    telem_string1[22] = my_rpm_topic.telem1_23;
    telem_string1[23] = my_rpm_topic.telem1_24;
    telem_string1[24] = my_rpm_topic.telem1_25;
    telem_string1[25] = my_rpm_topic.telem1_26;
    telem_string1[26] = my_rpm_topic.telem1_27;
    telem_string1[27] = my_rpm_topic.telem1_28;
    telem_string1[28] = my_rpm_topic.telem1_29;
    telem_string1[29] = my_rpm_topic.telem1_30;
    telem_string1[30] = my_rpm_topic.telem1_31;
    telem_string1[31] = my_rpm_topic.telem1_32;
    telem_string1[32] = my_rpm_topic.telem1_33;
    telem_string1[33] = my_rpm_topic.telem1_34;
    telem_string1[34] = '\0';
    PX4_INFO("\ttelem 1: %s", telem_string1);

    telem_string2[0] = my_rpm_topic.telem2_1;
    telem_string2[1] = my_rpm_topic.telem2_2;
    telem_string2[2] = my_rpm_topic.telem2_3;
    telem_string2[3] = my_rpm_topic.telem2_4;
    telem_string2[4] = my_rpm_topic.telem2_5;
    telem_string2[5] = my_rpm_topic.telem2_6;
    telem_string2[6] = my_rpm_topic.telem2_7;
    telem_string2[7] = my_rpm_topic.telem2_8;
    telem_string2[8] = my_rpm_topic.telem2_9;
    telem_string2[9] = my_rpm_topic.telem2_10;
    telem_string2[10] = my_rpm_topic.telem2_11;
    telem_string2[11] = my_rpm_topic.telem2_12;
    telem_string2[12] = my_rpm_topic.telem2_13;
    telem_string2[13] = my_rpm_topic.telem2_14;
    telem_string2[14] = my_rpm_topic.telem2_15;
    telem_string2[15] = my_rpm_topic.telem2_16;
    telem_string2[16] = my_rpm_topic.telem2_17;
    telem_string2[17] = my_rpm_topic.telem2_18;
    telem_string2[18] = my_rpm_topic.telem2_19;
    telem_string2[19] = my_rpm_topic.telem2_20;
    telem_string2[20] = my_rpm_topic.telem2_21;
    telem_string2[21] = my_rpm_topic.telem2_22;
    telem_string2[22] = my_rpm_topic.telem2_23;
    telem_string2[23] = my_rpm_topic.telem2_24;
    telem_string2[24] = my_rpm_topic.telem2_25;
    telem_string2[25] = my_rpm_topic.telem2_26;
    telem_string2[26] = my_rpm_topic.telem2_27;
    telem_string2[27] = my_rpm_topic.telem2_28;
    telem_string2[28] = my_rpm_topic.telem2_29;
    telem_string2[29] = my_rpm_topic.telem2_30;
    telem_string2[30] = my_rpm_topic.telem2_31;
    telem_string2[31] = my_rpm_topic.telem2_32;
    telem_string2[32] = my_rpm_topic.telem2_33;
    telem_string2[33] = my_rpm_topic.telem2_34;
    telem_string2[34] = '\0';
    PX4_INFO("\ttelem 2: %s", telem_string2);

    return PX4_OK;
}



/*--------------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------------*/
int my_LQR_printouts_main(int argc, char *argv[])
{
    return My_LQR_printouts::main(argc, argv);
}
