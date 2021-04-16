# Publishing the setpoints used in my_LQR_control

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

float32 y0		
float32 y1		
float32 y2
float32 y3
float32 y4
float32 y5
float32 y6
float32 y7
float32 y8
float32 y9
float32 y10
float32 y11

float32 c0
float32 c1
float32 c2
float32 c3		

float32 pitch_setpoint 		# [deg]

int32 proj_dpsi_status 		# status of the dpsi projection (0 off, 1 on, 10 projected+, -10 projected-)
float32 dpsi 				# the dpsi value after projection [deg]

int32 proj_theta_status 	# status of the theta projection (0 off, 1 on, 10 projected+, -10 projected-)

int32 tuner_status 
float32 gain_scale_p
float32 gain_scale_d
float32 case_int_f_int		# scheduler interval case + f_int

# TOPICS my_LQR_setpoints
