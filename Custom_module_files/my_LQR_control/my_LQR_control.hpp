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
 * @file my_LQR_control.hpp
 *
 * my_LQR_control module header file
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
#include <matrix/math.hpp> // for the matrix:: library
//#include <mathlib/mathlib.h>
//#include <matrix/Vector.hpp>
//#include <matrix/Matrix.hpp>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h> // for the hrt_absolute_time fn
//#include <lib/mixer/mixer.h> // for the load_mixer_file function

#include <perf/perf_counter.h> // for performance counter

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/angular_rates_filtered.h>
#include <uORB/topics/my_LQR_setpoints.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/airspeed.h>

using matrix::Matrix;
using matrix::Vector3f;
using matrix::Eulerf;
using matrix::Dcmf;
using matrix::Quatf;

#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <mathlib/math/my_lib/LowPassFilter3pVector3f.hpp>
#include <mathlib/math/my_lib/Detect_oscillations_Vector3f.hpp>

#define MY_PI 3.14159265359f


extern "C" __EXPORT int my_LQR_control_main(int argc, char *argv[]);

class My_LQR_control : public ModuleBase<My_LQR_control>, public ModuleParams
{
public:
	My_LQR_control(int arg_param, bool arg_flag);

	virtual ~My_LQR_control() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static My_LQR_control *instantiate(int argc, char *argv[]);

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
        int vehicle_attitude_poll();
        int vehicle_local_position_poll();
        int vehicle_local_position_setpoint_poll();
        int manual_control_setpoint_poll();
        int rc_channels_poll();
        int actuator_controls_virtual_poll();
		int home_position_poll();
		int airspeed_poll();
		int actuator_controls_publish();
		int angular_rates_filtered_publish();
		int setpoints_publish();
		int debug_publish();
		int publish_topics();
		int timer_clock();
		int update_parameters(bool force = false);
		int initialize_variables();
		int local_parameters_update();
		int control_fun();
        int read_y_state();
        int convert_quaternions();
        int filter_omg();
        int filter_eps();
        int filter_RC();
        int read_setpoints();
        int read_y_setpoint();
        int read_c_setpoint();
        int gains_schedule();
        int gains_scale();
        int gains_tune();
        int omg_setpoints_scale();
        int supporting_outputs();
		int bound_controls();
		int rc_loss_failsafe();
		int debug_printouts();
		int stabilisation_mode();
		int manual_override();
		int px4_override();
		int project_theta();
		int project_del_psi();
		int del_epsilon_to_body_frame();
		int adaptive_control();
		int recursiveLS();
		int gains_limiter_fun();
		float deg2rad(float);
		float rad2deg(float);
		bool isbound(float);
		
		

		/**
        * My subscription/publication topics structures
		*/
        struct parameter_update_s parameter_update{};
        struct vehicle_attitude_s vehicle_attitude{};
        struct vehicle_local_position_s vehicle_local_position{};
        struct vehicle_local_position_setpoint_s vehicle_local_position_setpoint{};
        struct manual_control_setpoint_s manual_control_setpoint{};
        struct rc_channels_s rc_channels{};
        struct rc_channels_s rc_channels_prev{};
        struct rc_channels_s rc_channels_fail{};
		struct actuator_controls_s actuator_controls_0{};        
		struct actuator_controls_s actuator_controls_1{};
		struct actuator_controls_s actuator_controls_virtual{};
		struct home_position_s home_position{};		
		struct angular_rates_filtered_s angular_rates_filtered{};
		struct my_LQR_setpoints_s setpoints_struct{};
		struct debug_value_s dbg_val{};
		struct airspeed_s airspeed{};

        /**
        * My subscription topics subsriptors
        */
        int parameter_update_sub = -1;
        int vehicle_attitude_sub = -1;
        int vehicle_local_position_sub = -1;
        int vehicle_local_position_setpoint_sub = -1;
        int manual_control_setpoint_sub = -1;
        int rc_channels_sub = -1;
        int actuator_controls_virtual_sub = -1;
		int home_position_sub = -1;
		int airspeed_sub = -1;
		
		/**
		* My publication topics publictors
		*/
		orb_advert_t actuator_controls_0_pub = nullptr;		
		orb_advert_t actuator_controls_1_pub = nullptr;
		orb_advert_t angular_rates_filtered_pub = nullptr;
		orb_advert_t setpoints_pub = nullptr;
		orb_advert_t dbg_val_pub = nullptr;

		/**
		* My ORB IDs
		*/
		orb_id_t actuator_controls_virtual_id = ORB_ID(actuator_controls_virtual_fw);

		/**
		* My performance counters
		*/
		perf_counter_t	_loop_perf;			/**< loop performance counter */

        /**
        * My global variables
        */
		uint64_t time_last_run = 0.0f;
		float dt = 0.0f;
		float dt_print = 0.0f;
		float dt_perturb = 0.0f;
		float dt_rcloss = 0.0f;
		float f_rcloss = 0.0f;
		float dt_loop = 0.0f;
		float loop_counter = 0.0f;
		uint64_t t_start = -1;
		float loop_update_freqn = 250.0f;

        Matrix<float,12,1> y; // state vector
        Matrix<float,4,1> cf; // filtered control
        Matrix<float,6,1> r; // integral of the error in states 
        Matrix<float,8,1> uf; // filtered control surfaces
        Matrix<float,4,1> cm; // manual control
        int control_status = 0; // check for the controller outputs, 0 ok, 1 nans

        Matrix<float,12,1> y_setpoint; 
        Matrix<float,4,1> c_setpoint;
        Matrix<float,6,1> r_setpoint; 
		
		Matrix<float,12,1> Del_y; 
		Matrix<float,4,1> Del_cf; 
		Matrix<float,6,1> Del_r; 
		Matrix<float,4,1> Del_c; 

		Matrix<float,4,12> K_feedback_y; // feedback matrix for the state terms
		Matrix<float,4,6> K_feedback_int; // feedback matrix for the integral terms
		Matrix<float,4,4> K_feedback_cf; // feedback matrix for the filtered control terms
		
		Matrix<float,4,4> Tf; // diagonal matrix of control time delays inverted
		Matrix<float,6,12> Ci; // matrix to pick the states from y to integrate in r

		Matrix<float,4,1> c_nominal_control;

		Matrix<float,12,1> y_max;
		Matrix<float,3,1> RC_scale_base; // base scale for cp,cq,cr inputs. 
		Matrix<float,3,1> RC_scale;
		float f_scale = 1.0f;
		float p_scale = 1.0f;
		bool do_rc_scale;

		Vector3f eps;
		Eulerf euler_angles;
		Eulerf euler_angles_proj;
		Dcmf Qdcm;
		Dcmf Qdcm_proj;
		Vector3f eps_filtered;
		float cutoff_freqn_eps = 5.0f;
		math::LowPassFilter2pVector3f lp2_filter_eps{loop_update_freqn, cutoff_freqn_eps};
		math::LowPassFilter3pVector3f lp3_filter_eps{loop_update_freqn, cutoff_freqn_eps};
		int filter_status_eps = 0;
		Vector3f eps_filtered_temp_lp2;
		Vector3f eps_filtered_temp_lp3;

		float thrust_setpoint = 0.0f;
		float pitch_setpoint = 0.0f;
		float pitch_sp_max = 40.0f;
		float yaw_setpoint = 0.0f;
		float pitch_sp_min = 0.0f;
		float pitch_setpoint_ramp = 0.0f;

		Vector3f omg;
		Vector3f omg_filtered;
		float cutoff_freqn_omg = 5.0f;
		math::LowPassFilter2pVector3f lp2_filter_omg{loop_update_freqn, cutoff_freqn_omg};
		math::LowPassFilter3pVector3f lp3_filter_omg{loop_update_freqn, cutoff_freqn_omg};
		int filter_status_omg = 0;
		int lpf_order = 3; // order of the lp filter to use
		Vector3f omg_filtered_temp_lp2;
		Vector3f omg_filtered_temp_lp3;

		Vector3f RC;
		Vector3f RC_filtered;
		float cutoff_freqn_RC = 5.0f;
		math::LowPassFilter2pVector3f lp2_filter_RC{loop_update_freqn, cutoff_freqn_RC};
		int filter_status_RC = 0;


		int case_int = 1;
		int case_int_last = 1;
		float f_int = 0.0f;
		float tht_sched = 0;
		const static int n_int = 6;
		Matrix<float,1,n_int+1> tht_ints;
		Matrix<float,10,n_int+1> k_scheds; // [pp, pr, rp, rr, ..., q, tht]
		Matrix<float,10,n_int+1> k_scheds_sc;
		Matrix<float,10,n_int+1> k_scheds_sc_tun;
		Matrix<float,10,1> k_scheds_sc_tun_int;
		Matrix<float,4,12> K_feedback_y_sc_tun_sched;
		float rc_sc_omg_last = 0;
		float rc_sc_eps_last = 0;
		int tuner_status = 0;
		float tune_expo = 10.0f;
		int tune_mode = 123;
		float tune_p = 1.0f;
		float tune_d = 1.0f;
		float tune_p_p = 1.0f;
		float tune_d_p = 1.0f;
		float tune_p_q = 1.0f;
		float tune_d_q = 1.0f;
		float tune_p_r = 1.0f;
		float tune_d_r = 1.0f;

		Matrix<float,4,2> k_sw; 
		// at fast&cruise vs slow&hover
        //// [qz, qvz, mz, mvz];
        int altitude_mode = 0;
        float c_alt_bool = 0.0f;
        float alt_setpoint = 0.0f;
        float alt_rate_setpoint = 0.0f;
        float alt_rate_rc_scale = 5.0f;
        float c_alt_support = 0.0f;


		Matrix<float,4,12> K_feedback_y_scaled;
		Matrix<float,4,6> K_feedback_int_scaled; 
		Matrix<float,4,4> K_feedback_cf_scaled; 
		Matrix<float,4,12> K_feedback_y_scaled_tuned;
		Matrix<float,4,6> K_feedback_int_scaled_tuned; 
		Matrix<float,4,4> K_feedback_cf_scaled_tuned; 
		Matrix<float,12,1> k_sc_vec; // x,v, p,q,r,phi,theta,psi, ccp, ccd, cf,ri

		Matrix<float,4,1> Del_c_x; 
		Matrix<float,4,1> Del_c_v; 
		Matrix<float,4,1> Del_c_omg; 
		Matrix<float,4,1> Del_c_eps; 
		Matrix<float,4,1> Del_c_lim; // limits on max Del_c for x,v,omg,eps
		Matrix<float,4,1> c_eps_bool; // stabilisation mode, will be zeros or ones based on RC switch
		Matrix<float,4,1> c_eps_satur; // store the remainders after bounding to [-1,1]

		float theta0 = 0.0f;
		bool proj_theta = 0;
		int proj_theta_status = 0;
		bool proj_dpsi = 0;
		int proj_dpsi_status = 0;
		bool schedule_K = 0;
		int schedule_K_status = 0;

		Matrix<float,3,3> E2B; // matrix to convert from euler to body frame
		bool e2b;
		Matrix<float,3,1> Del_y_eps; // the Del eps to be used in feedback

		float tailerons_scaling = 0.3f;
		float motorons_p_scaling = 0.15f;
		float motorons_r_scaling = 0.15f;

		bool do_printouts = 0;

		/**
        * My global decision variables
        */
		int vehicle_id = 0;
		

		bool do_recursiveLS = 1;
		Matrix<float,1,2> M_RLS;
		Matrix<float,1,1> Y_RLS;
		Matrix<float,1,1> E_RLS;
		Matrix<float,2,2> P_RLS;
		Matrix<float,2,1> X_RLS;
		Matrix<float,1,1> p_prev_RLS;
		float lambda_RLS = 0.97; // forgetting factor, 0.97 in HITL, 0.999 in real cos faster sampling
		Matrix<float,2,2> P0_RLS;
		Matrix<float,2,1> X0_RLS;
		Matrix<float,1,1> fract_RLS; 

		bool do_adaptive = 0; 
		float K_p_adapt = 0.0f;
		float K_phi_adapt = 0.0f;
		float ap_adapt = -8.0f;
    	float bp_adapt = 25.0f;
    	float kp_adapt = 0.06f;
    	float kphi_adapt = 1.4f;

    	bool gains_limiter_on = 1;
    	Matrix<int,3,1> oscillating;
    	Matrix<float,3,1> gain_limiter;
    	float pksz = 0.08f;
    	float dtlim = 0.2f;
    	math::Detect_oscillations_Vector3f detected_oscillations{pksz, dtlim};

  

		
		


		
		
		
		


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MY_LQR_CQ_TRM>) cq_trim,
		(ParamFloat<px4::params::MY_LQR_RC_SC_P>) rc_scale_p,
		(ParamFloat<px4::params::MY_LQR_RC_SC_Q>) rc_scale_q,
		(ParamFloat<px4::params::MY_LQR_RC_SC_R>) rc_scale_r,
		(ParamFloat<px4::params::MY_LQR_RC_SC_M>) rc_scale_m,
		(ParamFloat<px4::params::MY_LQR_MAX_U>) param_max_u,   
		(ParamFloat<px4::params::MY_LQR_MAX_V>) param_max_v,
		(ParamFloat<px4::params::MY_LQR_MAX_W>) param_max_w,
		(ParamFloat<px4::params::MY_LQR_MAX_P>) param_max_p,
		(ParamFloat<px4::params::MY_LQR_MAX_Q>) param_max_q,
		(ParamFloat<px4::params::MY_LQR_MAX_R>) param_max_r,
		(ParamFloat<px4::params::MY_LQR_MAX_PHI>) param_max_phi,
		(ParamFloat<px4::params::MY_LQR_MAX_THETA>) param_max_theta,
		(ParamFloat<px4::params::MY_LQR_MAX_PSI>) param_max_psi,
        (ParamFloat<px4::params::MY_LQR_F_LAG>) f_lag,
        (ParamFloat<px4::params::MY_LQR_K_SC_X>) k_sc_x,
        (ParamFloat<px4::params::MY_LQR_K_SC_V>) k_sc_v,
        (ParamFloat<px4::params::MY_LQR_K_SC_P>) k_sc_p,
        (ParamFloat<px4::params::MY_LQR_K_SC_Q>) k_sc_q,
        (ParamFloat<px4::params::MY_LQR_K_SC_R>) k_sc_r, 
        (ParamFloat<px4::params::MY_LQR_K_SC_PHI>) k_sc_phi, 
        (ParamFloat<px4::params::MY_LQR_K_SC_THT>) k_sc_tht, 
        (ParamFloat<px4::params::MY_LQR_K_SC_PSI>) k_sc_psi, 
        (ParamFloat<px4::params::MY_LQR_K_SC_CCP>) k_sc_ccp, 
        (ParamFloat<px4::params::MY_LQR_K_SC_CCD>) k_sc_ccd,
        (ParamFloat<px4::params::MY_LQR_K_SC_CF>) k_sc_cf,
        (ParamFloat<px4::params::MY_LQR_K_SC_RI>) k_sc_ri,
        (ParamFloat<px4::params::MY_LQR_K_SW1QZ>) k_sw1_qz,
        (ParamFloat<px4::params::MY_LQR_K_SW1QVZ>) k_sw1_qvz,
        (ParamFloat<px4::params::MY_LQR_K_SW1MZ>) k_sw1_mz,
        (ParamFloat<px4::params::MY_LQR_K_SW1MVZ>) k_sw1_mvz,
        (ParamFloat<px4::params::MY_LQR_K_SW2QZ>) k_sw2_qz,
        (ParamFloat<px4::params::MY_LQR_K_SW2QVZ>) k_sw2_qvz,
        (ParamFloat<px4::params::MY_LQR_K_SW2MZ>) k_sw2_mz,
        (ParamFloat<px4::params::MY_LQR_K_SW2MVZ>) k_sw2_mvz,
        (ParamFloat<px4::params::MY_LQR_TUNE_EX>) tune_ex,
        (ParamInt<px4::params::MY_LQR_TUNE_MOD>) tune_mod,
        (ParamFloat<px4::params::MY_LQR_THT_SP_M>) tht_sp_m,
        (ParamFloat<px4::params::MY_LQR_THT_SP_MN>) tht_sp_min,
        (ParamFloat<px4::params::MY_LQR_DX_LIM>) dx_lim,
        (ParamFloat<px4::params::MY_LQR_DV_LIM>) dv_lim,
        (ParamFloat<px4::params::MY_LQR_DOMG_LIM>) domg_lim,
        (ParamFloat<px4::params::MY_LQR_DEPS_LIM>) deps_lim,
        (ParamFloat<px4::params::MY_LQR_CTF_OMG>) cutoff_fn_omg,
        (ParamFloat<px4::params::MY_LQR_CTF_EPS>) cutoff_fn_eps,
        (ParamFloat<px4::params::MY_LQR_CTF_RC>) cutoff_fn_RC,
        (ParamInt<px4::params::MY_LQR_LPF_ORD>) lpf_ord,
        (ParamFloat<px4::params::MY_LQR_TAILERONS>) tailerons_sc,
        (ParamFloat<px4::params::MY_LQR_MOTORONSP>) motorons_p_sc,
        (ParamFloat<px4::params::MY_LQR_MOTORONSR>) motorons_r_sc,
        (ParamInt<px4::params::MY_LQR_BOOL_SCHD>) bool_K_sched,
        (ParamInt<px4::params::MY_LQR_BOOL_RCSC>) bool_rc_sc,
        (ParamInt<px4::params::MY_LQR_BOOL_PRNT>) bool_printouts,
        (ParamInt<px4::params::MY_LQR_BOOL_PTT>) bool_proj_tht,
        (ParamInt<px4::params::MY_LQR_BOOL_PDP>) bool_proj_dpsi,
        (ParamInt<px4::params::MY_LQR_BOOL_ADP>) bool_adaptive,
        (ParamInt<px4::params::MY_LQR_BOOL_RLS>) bool_recursiveLS,
        (ParamFloat<px4::params::MY_LQR_LMBD_RLS>) lambda_rls,
        (ParamInt<px4::params::MY_LQR_BOOL_GLM>) bool_gains_limiter,
        (ParamFloat<px4::params::MY_LQR_GLM_PKSZ>) glm_pksz,
        (ParamFloat<px4::params::MY_LQR_GLM_DTLIM>) glm_dtlim,
        (ParamInt<px4::params::MY_LQR_BOOL_E2B>) bool_e2b
        )// Just the handles, need to .get() them
};

