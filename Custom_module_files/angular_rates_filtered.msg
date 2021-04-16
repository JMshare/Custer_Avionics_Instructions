# Taking the rates from the vehicle_attitude and filtering them using the Lowpass filter with cutoff freqn set up by MY_LQR_RATES_CUTOFFFN

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

float32 rollspeed		# filtered angular velocity about X body axis in rad/s
float32 pitchspeed		# filtered angular velocity about Y body axis in rad/s
float32 yawspeed		# filtered angular velocity about Z body axis in rad/s

float32 loop_update_freqn # update rate param in LowPassFilter2pVector3f
float32 cutoff_freqn # cutoff param in LowPassFilter2pVector3f

int32 filter_status 		# filter status (0 = ok, 1 = NaNs, 2 = too high cutofffqn)

# TOPICS angular_rates_filtered
