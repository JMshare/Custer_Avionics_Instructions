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
 * @file my_LQR_printouts.hpp
 *
 * my_LQR_printouts module header file
 *
 * @author Juraj Mihalik <jm1e16@soton.ac.uk>
 */

#include <px4_module.h> // to exist as a module
#include <px4_module_params.h> // to exist as a module
#include <px4_getopt.h> // for the module::instantiate functionality
//#include <px4_log.h>

//#include <px4_defines.h>
//#include <platforms/px4_defines.h>
//#include <px4_config.h>
//#include <px4_tasks.h>
//#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h> // for the ioctl definition
//#include <drivers/drv_mixer.h> // for the ioctl mixer defines (MIXERIOCLOADBUF)
//#include "systemlib/err.h"

#include <px4_posix.h> // for the module polling
//#include <matrix/math.hpp> // for the matrix:: library
//#include <mathlib/mathlib.h>
//#include <matrix/Vector.hpp>
//#include <matrix/Matrix.hpp>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h> // for the hrt_absolute_time fn
//#include <lib/mixer/mixer.h> // for the load_mixer_file function

#include <perf/perf_counter.h> // for performance counter

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/my_LQR_setpoints.h>
#include <uORB/topics/my_rpm_topic.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/angular_rates_filtered.h>



extern "C" __EXPORT int my_LQR_printouts_main(int argc, char *argv[]);

class My_LQR_printouts : public ModuleBase<My_LQR_printouts>, public ModuleParams
{
public:
	My_LQR_printouts(int arg_param, bool arg_flag);

	virtual ~My_LQR_printouts() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static My_LQR_printouts *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:


        /**
        * My functions
        */
        int parameter_update_poll();
		int my_rpm_topic_poll();
		int my_LQR_setpoints_poll();
		int actuator_controls_poll();
		int angular_rates_filtered_poll();
		int timer_clock();
		int update_parameters(bool force = false);
		int initialize_variables();
		int local_parameters_update();
		int printouts();
		int print_my_rpm();
		float deg2rad(float);
		float rad2deg(float);
		
		

		/**
        * My subscription/publication topics structures
		*/
        struct parameter_update_s parameter_update{};
		struct my_LQR_setpoints_s my_LQR_setpoints{};
		struct my_rpm_topic_s my_rpm_topic{};
		struct actuator_controls_s actuator_controls_0{};        
		struct actuator_controls_s actuator_controls_1{};
		struct angular_rates_filtered_s angular_rates_filtered{};

        /**
        * My subscription topics subsriptors
        */
        int parameter_update_sub = -1;
		int my_rpm_topic_sub = -1;
		int my_LQR_setpoints_sub = -1;
		int actuator_controls_0_sub = -1;
		int actuator_controls_1_sub = -1;
		int angular_rates_filtered_sub = -1;
		
		/**
		* My performance counters
		*/
		perf_counter_t	_loop_perf;			/**< loop performance counter */

        /**
        * My global variables
        */
        int vehicle_id = 0;

		uint64_t time_last_run = 0.0f;
		float dt = 0.0f;

		float dt_print = 0.0f;
		float printouts_dt = 2.0f;


    	float current1 = 0.0f;
    	float current2 = 0.0f;
    	char telem_string1[35];
    	char telem_string2[35];

		
	
		


	DEFINE_PARAMETERS(
        (ParamFloat<px4::params::MY_LQR_PRNT_DT>) prnt_dt
        )// Just the handles, need to .get() them
};

