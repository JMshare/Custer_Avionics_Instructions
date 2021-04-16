/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file my_LQR_control_params.c
 *
 * Parameters defined by the my_LQR_control task
 *
 * @author Juraj Mihalik <jm1e16@soton.ac.uk>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */


/**
 * RC tuning of roll/pitch/yaw or combinations
 *
 * 0/1/2/10/20/21/210
 * @unit []
 * @min 1
 * @max 210
 * @decimal 0
 * @increment 1
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_TUNE_MOD, 210);


/**
 * Pitch trim 
 *
 * @unit []
 * @min -1
 * @max 1
 * @decimal 1
 * @increment 0.1
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_CQ_TRM, 0.0f);


/**
 * Cutoff frequency for angular rates filter
 *
 * @unit [Hz]
 * @min 0
 * @max 250
 * @decimal 1
 * @increment 0.1
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_CTF_OMG, 3.0f);

/**
 * Cutoff frequency for angles filter
 *
 * @unit [Hz]
 * @min 0
 * @max 250
 * @decimal 1
 * @increment 0.1
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_CTF_EPS, 250.0f);

/**
 * Cutoff frequency for RC
 *
 * @unit [Hz]
 * @min 0
 * @max 250
 * @decimal 1
 * @increment 0.1
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_CTF_RC, 3.0f);

/**
 * LPF order
 * set to 2 or 3
 *
 * @unit none
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_LPF_ORD, 3);

/**
 * Pitch setpoint max
 *
 * @unit []
 * @min 0
 * @max 90
 * @decimal 1
 * @increment 0.1
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_THT_SP_M, 40.0f);

/**
 * Pitch setpoint min
 *
 * @unit []
 * @min -90
 * @max 90
 * @decimal 1
 * @increment 0.1
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_THT_SP_MN, 0.0f);

/**
 * Feedback matrix scaling
 * The feedback on p will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_P, 1.0f);

/**
 * Feedback matrix scaling
 * The feedback on q will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_Q, 1.0f);

/**
 * Feedback matrix scaling
 * The feedback on r will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_R, 1.0f);

/**
 * Feedback matrix scaling
 * The feedback on phi will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_PHI, 1.0f);

/**
 * Feedback matrix scaling
 * The feedback on theta will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_THT, 1.0f);

/**
 * Feedback matrix scaling
 * The feedback on psi will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_PSI, 1.0f);

/**
 * Feedback matrix scaling, cross coupling P term
 * The feedback matrix will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_CCP, 1.0f);

/**
 * Feedback matrix scaling, cross coupling D term
 * The feedback matrix will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_CCD, 1.0f);

/**
 * Feedback matrix scaling
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SW1QZ, 0.02f);
/**
 * Feedback matrix scaling
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SW1QVZ, 0.00f);
/**
 * Feedback matrix scaling
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SW1MZ, 0.0f);
/**
 * Feedback matrix scaling
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SW1MVZ, 0.0f);
/**
 * Feedback matrix scaling
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SW2QZ, 0.0f);
/**
 * Feedback matrix scaling
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SW2QVZ, 0.0f);
/**
 * Feedback matrix scaling
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SW2MZ, 0.08f);
/**
 * Feedback matrix scaling
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SW2MVZ, 0.2f);

/**
 * Motorons roll mixer scaling
 * The supporting outputs on motorons in the mixer will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MOTORONSP, 0.0f);

/**
 * Motorons yaw mixer scaling
 * The supporting outputs on motorons in the mixer will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MOTORONSR, 0.3f);

/**
 * Tailerons mixer scaling
 * The supporting outputs on tailerons in the mixer will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_TAILERONS, 0.3f);

/**
 * Tuner exponential base
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_TUNE_EX, 10.0f);

/**
 * gains schedule bool
 * Schedule the feedback gains matrix K based on pitch angle theta
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_SCHD, 1);

/**
 * adaptive control bool
 * Scale the feedback control
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_ADP, 0);

/**
 * recursive LS 
 * system identification
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_RLS, 1);

/**
 * 
 * forgetting factor for the RLS
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 3
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_LMBD_RLS, 0.999f);

/**
 * RC adaptive scaling bool
 * adaptive RC scaling based on current attitude
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_RCSC, 1);

/**
 * Gains limiter
 * from detected oscillations
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_GLM, 1);

/**
 * 
 * peak size threshold for gains limiter
 *
 * @unit []
 * @min 0
 * @max 1
 * @decimal 3
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_GLM_PKSZ, 0.08f);

/**
 * 
 * dt oscillations threshold for gains limiter
 *
 * @unit []
 * @min 0
 * @max 1
 * @decimal 3
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_GLM_DTLIM, 0.2f);

/**
 * RC setpoint scaling on P
 * Scale to change the responsivness of the plane
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 2
 * @increment 0.05
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_RC_SC_P, 1.0f);

/**
 * RC setpoint scaling on Q
 * Scale to change the responsivness of the plane
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 2
 * @increment 0.05
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_RC_SC_Q, 1.0f);

/**
 * RC setpoint scaling on R
 * Scale to change the responsivness of the plane
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 2
 * @increment 0.05
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_RC_SC_R, 1.0f);

/**
 * RC setpoint scaling on M
 * Scale to change the responsivness of the plane
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 2
 * @increment 0.05
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_RC_SC_M, 10.0f);


/**
 * PTT bool
 * Project the pitch theta to extend to -110 to +110 deg
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_PTT, 1);

/**
 * PDP bool
 * Project the heading psi error to 180 deg
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_PDP, 1);

/**
 * Printouts bool
 * Do I want to generate the custom printouts?
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_PRNT, 0);







// ------------------------------------------------------------------------------------------------------
// Active but don't use, don't change
/**
 * Limit on max Del_c from Del_y_omg contribution
 *
 * @unit []
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_DX_LIM, 0.8f);

/**
 * Limit on max Del_c from Del_y_omg contribution
 *
 * @unit []
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_DV_LIM, 0.8f);

/**
 * Limit on max Del_c from Del_y_omg contribution
 *
 * @unit []
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_DOMG_LIM, 1.0f);

/**
 * Limit on max Del_c from Del_y_eps contribution
 *
 * @unit []
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_DEPS_LIM, 1.0f);







// ------------------------------------------------------------------------------------------------------
// Depreciated parameters, not used at the moment
/**
 * E2B bool
 * Use the nonlinear transformation on epsilon to get it to body frame
 *
 * @unit bool
 * @group MY LQR Control
 */
PARAM_DEFINE_INT32(MY_LQR_BOOL_E2B, 0);

/**
 * Maximum velocity in body x direction
 *
 * @unit m/s
 * @min 0
 * @max 8
 * @decimal 2
 * @increment 0.05
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_U, 6.0f);

/**
 * Maximum velocity in body y direction
 *
 * @unit m/s
 * @min 0
 * @max 8
 * @decimal 2
 * @increment 0.05
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_V, 6.0f);

/**
 * Maximum velocity in body z direction
 *
 * @unit m/s
 * @min 0
 * @max 4
 * @decimal 2
 * @increment 0.05
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_W, 4.0f);

/**
 * Maximum pitch rate
 *
 * @unit rad/s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_P, 1.0f);

/**
 * Maximum roll rate
 *
 * @unit rad/s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_Q, 1.0f);

/**
 * Maximum yaw rate
 *
 * @unit rad/s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_R, 1.0f);

/**
 * Maximum pitch 
 *
 * @unit rad
 * @min 0.0
 * @max 1.57
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_PHI, 1.5f);

/**
 * Maximum roll 
 *
 * @unit rad
 * @min 0.0
 * @max 1.57
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_THETA, 1.5f);

/**
 * Maximum yaw 
 *
 * @unit rad
 * @min 0.0
 * @max 1.57
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_MAX_PSI, 1.5f);

/**
 * Time lags filter parameter
 * How much the motors should be delayed comparing to the actuator, inverted.
 *
 * @unit [1/s]
 * @min 0
 * @max 1000
 * @decimal 2
 * @increment 0.01
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_F_LAG, 10.0f);

/**
 * Feedback matrix scaling
 * The feedback matrix will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_X, 1.0f);

/**
 * Feedback matrix scaling
 * The feedback matrix will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_V, 1.0f);

/**
 * Feedback matrix scaling
 * The feedback matrix will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_CF, 0.0f);

/**
 * Feedback matrix scaling
 * The feedback matrix will be scaled by this
 *
 * @unit []
 * @min 0
 * @max 1000
 * @decimal 4
 * @increment 0.001
 * @group MY LQR Control
 */
PARAM_DEFINE_FLOAT(MY_LQR_K_SC_RI, 0.0f);

