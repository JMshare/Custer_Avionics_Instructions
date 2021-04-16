-- FILE:        stability_analysis.lua
-- VERSION:     0.9
-- AUTHOR:      Marco Ottaviano
-- DESCRIPTION: Calculates Control & Stability derivatives for fixed-wing aircrafts in X-Plane

-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
-- !!!!!!!!!!!!!!! SET "FLIGHT MODEL PER FRAME" TO 1 IN X-PLANE !!!!!!!!!!!!!!!
-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
-- !!! SOME DATAREFS AND INPUT DATA MUST BE CHANGED ON A PER-AIRCRAFT BASIS !!!
-- !!!!!!!!!!!!!!! READ COMMENTS BELOW FOR FURTHER EXPLANATIONS !!!!!!!!!!!!!!!
-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

-- time
dataref("t", "sim/time/total_running_time_sec", 'readonly') -- simulation time

-- euler coordinates representing the orientation of the aircraft (degrees)
dataref("psi", "sim/flightmodel/position/psi", "readonly") -- yaw angle
dataref("theta", "sim/flightmodel/position/theta", "readonly") -- pitch angle
dataref("phi", "sim/flightmodel/position/phi", "readonly") -- roll angle

-- rotation rates of the aircraft (deg/sec)
dataref("P", "sim/flightmodel/position/P", "writable") -- roll rate (positive for right roll) in Deg/s!!
dataref("Q", "sim/flightmodel/position/Q", "writable") -- pitch rate (positive for pitch up)
dataref("R", "sim/flightmodel/position/R", "writable") -- yaw rate (positive for right yaw)

-- torques acting on the aircraft along its 3 axes (Nm)
dataref("L_aero", "sim/flightmodel/forces/L_aero", "readonly") -- torque acting on aircraft roll (X) axis (positive for right roll)
dataref("M_aero", "sim/flightmodel/forces/M_aero", "readonly") -- torque acting on aircraft pitch (Y) axis (positive for pitch up)
dataref("N_aero", "sim/flightmodel/forces/N_aero", "readonly") -- torque acting on aircraft yaw (Z) axis (positive for right yaw)

dataref("L_prop", "sim/flightmodel/forces/L_prop", "readonly")
dataref("M_prop", "sim/flightmodel/forces/M_prop", "readonly")
dataref("N_prop", "sim/flightmodel/forces/N_prop", "readonly")

dataref("L_total", "sim/flightmodel/forces/L_total", "writable")
dataref("M_total", "sim/flightmodel/forces/M_total", "writable")
dataref("N_total", "sim/flightmodel/forces/N_total", "writable")

-- forces acting on the aircraft along its 3 axes (N)
dataref("fside_aero", "sim/flightmodel/forces/fside_aero", "readonly") -- force acting along aircraft lateral axis (does not include weight)
dataref("fnrml_aero", "sim/flightmodel/forces/fnrml_aero", "readonly") -- force acting along aircraft vertical axis does not include weight)
dataref("faxil_aero", "sim/flightmodel/forces/faxil_aero", "readonly") -- force acting along aircraft longitudinal axis (does not include weight)

dataref("fside_prop", "sim/flightmodel/forces/fside_prop", "readonly")
dataref("fnrml_prop", "sim/flightmodel/forces/fnrml_prop", "readonly")
dataref("faxil_prop", "sim/flightmodel/forces/faxil_prop", "readonly")

dataref("fside_total", "sim/flightmodel/forces/fside_total", "writable")
dataref("fnrml_total", "sim/flightmodel/forces/fnrml_total", "writable")
dataref("faxil_total", "sim/flightmodel/forces/faxil_total", "writable")

-- local velocity components of the aircraft (Earth-fixed axes) (m/s)
dataref("vx", "sim/flightmodel/position/local_vx", "writable")
dataref("vy", "sim/flightmodel/position/local_vy", "writable")
dataref("vz", "sim/flightmodel/position/local_vz", "writable")

-- coordinates of the aircraft (m)
dataref("local_x", "sim/flightmodel/position/local_x", "writable")
dataref("local_y", "sim/flightmodel/position/local_y", "writable")
dataref("local_z", "sim/flightmodel/position/local_z", "writable")

-- orientation quaternion
dataref("q0", "sim/flightmodel/position/q", "writable",0)
dataref("q1", "sim/flightmodel/position/q", "writable",1)
dataref("q2", "sim/flightmodel/position/q", "writable",2)
dataref("q3", "sim/flightmodel/position/q", "writable",3)

dataref("dynamic_pressure", "sim/flightmodel/misc/Qstatic", "readonly") -- instantaneous dynamic pressure (Pa) (NOTE: the unit of measure is erroneously listed as "psf" on the dataref reference webpage)
dataref("U", "sim/flightmodel/position/true_airspeed", "readonly") -- true airspeed (m/s)

-- angle of attack and angle of sideslip (degrees)
dataref("alpha", "sim/flightmodel/position/alpha", "readonly")
dataref("beta", "sim/flightmodel/position/beta", "readonly")

-- datarefs used to override autopilot and artificial stability inputs
dataref("override_flight_controls", "sim/operation/override/override_flightcontrol", "writable")
dataref("override_control_surfaces", "sim/operation/override/override_control_surfaces", "writable")
dataref("override_throttles", "sim/operation/override/override_throttles", "writable")
dataref("override_forces", "sim/operation/override/override_forces", "writable")

-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! CONTROL SURFACES DEFLECTIONS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
-- !!! DEPEND ON HOW THE AIRCRAFT IS DESIGNED IN PLANE-MAKER!!! MUST BE CHANGED ON A PER-AIRCRAFT BASIS!!!
-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
dataref("left_elev_def", "sim/flightmodel/controls/hstab1_elv1def", "writable") --(degrees) (set for Jason Chandler's freeware C182)
dataref("right_elev_def", "sim/flightmodel/controls/hstab2_elv1def", "writable") --(degrees)
dataref("left_ail_def", "sim/flightmodel/controls/wing1l_ail1def", "writable") --(degrees)
dataref("right_ail_def", "sim/flightmodel/controls/wing1r_ail1def", "writable") --(degrees)
dataref("left_rudder_def", "sim/flightmodel/controls/vstab1_rud1def", "writable") --(degrees)
dataref("right_rudder_def", "sim/flightmodel/controls/vstab2_rud1def", "writable") --(degrees)
dataref("throttleL", "sim/flightmodel/engine/ENGN_thro_use", "writable", 1)
dataref("throttleR", "sim/flightmodel/engine/ENGN_thro_use", "writable", 2)



-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! REFERENCE DIMENSIONS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
-- !!! DEPEND ON THE SPECIFIC AIRCRAFT BEING ANALYZED!!! MUST BE CHANGED FOR EACH DIFFERENT AIRCRAFT!!!
-- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
minv = 0.0625
Iinvp = 0.2489
Iinvq = 0.1795
Iinvr = 0.1084

dAmax = 30
dEmax = 30
dRmax = 30
dESmax = 13
dPmax = 1

-- perturbations applied to angles, rotation rates or control deflections
-------------------------------------------------------------------------
delta_alpha = 1 -- nominal perturbation in angle of attack (degrees)
delta_beta = 1 -- nominal perturbation in angle of sideslip (degrees)

delta_p = 0.1 -- perturbation in roll rate (rad/second)
delta_q = 0.1 -- perturbation in pitch rate (rad/second)
delta_r = 0.1 -- perturbation in yaw rate (rad/second)

delta_dA = 3 -- ailerons deflection (average deflection of left + right) (positive for right aileron up / roll right) (degrees)
delta_dE = 3 -- elevator deflection (positive for trailing edge down / pitch down) (degrees)
delta_dR = 3 -- rudder deflection (positive for trailing edge right / yaw right) (degrees)
delta_dES = 3 -- tailerons
delta_dP = 0.1 -- motorons

pitch_moment_before = 0
pitch_moment_after = 0

yaw_moment_before = 0
yaw_moment_after = 0

roll_moment_before = 0
roll_moment_after = 0

eng_pitch_moment_before = 0
eng_pitch_moment_after = 0

eng_yaw_moment_before = 0
eng_yaw_moment_after = 0

drag_before = 0
drag_after = 0

lift_before = 0
lift_after = 0

sideforce_before = 0
sideforce_after = 0

alpha_before = 0
alpha_after = 0

beta_before = 0
beta_after = 0

phi_before = 0

p_before = 0
q_before = 0
r_before = 0
vx_before = 0
vy_before = 0
vz_before = 0
q0_before = 0
q1_before = 0
q2_before = 0
q3_before = 0
faxil_total_before = 0
fside_total_before = 0
fnrml_total_before = 0
L_total_before = 0
M_total_before = 0
N_total_before = 0
LE0 = 0
RE0 = 0
LA0 = 0
RA0 = 0
LR0 = 0
RR0 = 0
RP0 = 0
LP0 = 0

faxil_aero_before = 0
fside_aero_before = 0
fnrml_aero_before = 0
L_aero_before = 0
M_aero_before = 0
N_aero_before = 0
faxil_prop_before = 0
fside_prop_before = 0
fnrml_prop_before = 0
L_prop_before = 0
M_prop_before = 0
N_prop_before = 0

effective_delta_alpha = 0
effective_delta_beta = 0
efdLA = 0
efdRA = 0
efdLE = 0
efdRE = 0
efdLR = 0
efdRR = 0
efdp = 0
efdq = 0
efdr = 0
efdP = 0

-- Since X-Plane integrates rotation rates at every flight cycle, when pitch, yaw or roll rates are perturbated, the actual aircraft attitude changes a little
-- during the hold phase. So, we calculate the gap between current aircraft attitude (during the hold) and aircraft attitude before the perturbation, then we
-- modify the initial reference attitude of the aircraft in order to make the attitude during the hold coincident with the attitude before the perturbation

alpha_gap = 0 -- difference between the alpha during the perturbation and the alpha before the perturbation, when pitch rate is perturbated (is brought to zero during hold)
beta_gap = 0 -- same for beta, when yaw rate is perturbated
phi_gap = 0 -- same for bank angle, when roll rate is perturbated

-- Pitch axis vector in OGL coordinates (will be calculated before the perturbation and during the hold)
YxOGL = 0
YyOGL = 0
YzOGL = 0

-- Pitch axis vector in body fixed axes (will be calculated before the perturbation and during the hold)
Yx1 = 0
Yy1 = 0
Yz1 = 0

-- Yaw axis vector in OGL coordinates (will be calculated before the perturbation and during the hold)
ZxOGL = 0
ZyOGL = 0
ZzOGL = 0

-- Yaw axis vector in body fixed axes (will be calculated before the perturbation and during the hold)
Zx1 = 0
Zy1 = 0
Zz1 = 0

-- Roll axis vector in body fixed axes (will be calculated during the hold)
Xx1 = 0
Xy1 = 0
Xz1 = 0

-- Variables used to store aircraft position, orientation, velocity and rotation rates during hold 
temp_x = 0
temp_y = 0
temp_z = 0
    
temp_q0 = 0
temp_q1 = 0
temp_q2 = 0
temp_q3 = 0
    
temp_vx = 0
temp_vy = 0
temp_vz = 0
    
temp_P = 0
temp_Q = 0
temp_R = 0



DegToRad = 3.14159 / 180 -- multiplier to convert from degrees to radians

-- variables used to store the state of the plugin
mode = 'off'
query = 'off'

-- overriding datarefs
override_flight_controls = 0
override_forces = 0
override_throttles = 0


-- My mode function
my_mode = 'off'
dAlpha = 0
dBeta = 0
dp = 0
dq = 0
dr = 0
dA = 0
dE = 0
dR = 0
dES = 0
dP = 0
dx = 0
add_macro("run calculations", "my_mode = 'run_calculations'")
dt_query_max = 0
tquery = 0
iter = 0
iter_perturb = 0
iter_calcul = 0
tpause = 0

dFx = 0
dFy = 0
dFz = 0
dl = 0
dm = 0
dn = 0

dFxp = 0
dFyp = 0
dFzp = 0
dlp = 0
dmp = 0
dnp = 0

-- //////////////////////////////////////////////

-- create plugin commands
--add_macro("pitch perturbation","mode = 'pitch'")
--add_macro("pitch rate perturbation","mode = 'pitch_rate'")
--add_macro("elevator deflection","mode = 'elevator'")
--add_macro("yaw perturbation","mode = 'yaw'")
--add_macro("yaw rate perturbation","mode = 'yaw_rate'")
--add_macro("rudder deflection","mode = 'rudder'")
--add_macro("roll rate perturbation","mode = 'roll_rate'")
--add_macro("aileron deflection","mode = 'aileron'")
--add_macro("calculate","mode = 'calculate'")

dataref("cmd_p", "sim/joystick/FC_roll", 'readonly')
dataref("cmd_q", "sim/joystick/FC_ptch", 'readonly')
dataref("cmd_r", "sim/joystick/FC_hdng", 'readonly')
LA = 0
RA = 0
LE = 0
RE = 0
LR = 0
RR = 0
en_tailerons = 1
en_motorons = 1
en_rudders = 1
en_ailerons = 1
dmode = 'off'
add_macro("disable tailerons", "dmode = 'tailerons_off'")
add_macro("disable ailerons", "dmode = 'ailerons_off'")
add_macro("disable rudders", "dmode = 'rudders_off'")
add_macro("enable all", "dmode = 'off'")
function mixer()
    LA =  cmd_p*dAmax*en_ailerons
    RA = -cmd_p*dAmax*en_ailerons
    LE = -cmd_q*dEmax + cmd_p*dESmax*en_tailerons
    RE = -cmd_q*dEmax - cmd_p*dESmax*en_tailerons
    LR =  cmd_r*dRmax*en_rudders
    RR =  cmd_r*dRmax*en_rudders

    left_ail_def = LA
    right_ail_def = RA
    left_elev_def = LE
    right_elev_def = RE
    left_rudder_def = LR
    right_rudder_def = RR
end
function control_loop()
    if dmode=='off' then
        en_tailerons = 1
        en_motorons = 1
        en_rudders = 1
        en_ailerons = 1
    end

    if dmode == 'tailerons_off' then
        en_tailerons = 0
        en_motorons = 1
        en_rudders = 1
        en_ailerons = 1
    end

    if dmode == 'ailerons_off' then
        en_tailerons = 1
        en_motorons = 1
        en_rudders = 1
        en_ailerons = 0
    end

    if dmode == 'rudders_off' then
        en_tailerons = 1
        en_motorons = 1
        en_rudders = 0
        en_ailerons = 1
    end

    override_control_surfaces = 1
    mixer()
end




-- main loop
function loop()

    if my_mode ~= 'run_calculations' then
        control_loop()
    end

    iter = iter + 1
    if iter > 1000 and iter_perturb == 0 then 
      iter = 0
    end

    if t < tpause then
        P = p_before
        Q = q_before
        R = r_before
        vx = vx_before
        vy = vy_before
        vz = vz_before
        q0 = q0_before
        q1 = q1_before
        q2 = q2_before
        q3 = q3_before
        faxil_total = faxil_total_before
        fside_total = fside_total_before
        fnrml_total = fnrml_total_before
        L_total = L_total_before
        M_total = M_total_before
        N_total = N_total_before
        left_ail_def = LA0
        right_ail_def = RA0
        left_elev_def = LE0
        right_elev_def = RE0
        left_rudder_def = LR0
        right_rudder_def = RR0
        throttleL = LP0
        throttleR = RP0
        return
    end

    if mode=='off' then
        if my_mode == 'run_calculations' then
            if dAlpha == 0 then
              outstring = string.format("\n\n\n-------------------------\n")
              logMsg(outstring)
              outstring = string.format("\n-------------------------\n")
              logMsg(outstring)
              outstring = string.format("\n-------------------------\n| STARTING run_calculations at t= %d |\n-------------------------\n\n", t)
              logMsg(outstring)

              override_flight_controls = 1
              override_forces = 1
              override_throttles = 1

              mode = 'pitch' -- (alpha)
              dAlpha = 1
              return
            end
            if dBeta == 0 then
              mode = 'yaw' -- beta
              dBeta = 1
              return
            end

            if dp == 0 then
              mode = 'roll_rate' 
              dp = 1
              return
            end
            if dq == 0 then
              mode = 'pitch_rate' 
              dq = 1
              return
            end
            if dr == 0 then
              mode = 'yaw_rate' 
              dr = 1
              return
            end

            if dA == 0 then
              mode = 'aileron' 
              dA = 1
              return
            end
            if dE == 0 then
              mode = 'elevator' 
              dE = 1
              return
            end
            if dR == 0 then
              mode = 'rudder' 
              dR = 1
              return
            end
            if dES == 0 then
              mode = 'taileron'
              dES = 1
              return
            end
            if dP == 0 then
              mode = 'motoron'
              dP = 1
              return
            end
            
            dAlpha = 0
            dBeta = 0
            dp = 0
            dq = 0
            dr = 0
            dA = 0
            dE = 0
            dR = 0
            dES = 0
            dP = 0
            my_mode = 'off'

            override_flight_controls = 0
            override_forces = 0
            override_throttles = 0
            return
        end

        return
    end
    
    if mode=='calculate' then -- if calculation mode has been selected, calculate the derivatives
       calculate()
       return
    end

    if query ~= 'off' and (t - tquery) > dt_query_max then
        calculate()
        return
    end

    if mode=='hold' then -- after the perturbation is set, hold it
       hold()
       return
    end

    if query == 'off' then
        perturbate() -- if a perturbation mode has been selected, do the perturbation
        return
    end

end

-- hold the perturbation
function hold()

    -- freeze the aircraft in the position, attitude and velocity calculated during the perturbation
    local_x = temp_x
    local_y = temp_y
    local_z = temp_z
    
    q0 = temp_q0
    q1 = temp_q1
    q2 = temp_q2
    q3 = temp_q3
    
    if query=='pitch_rate' then   
    
       --find the coordinates of the pitch (Y) axis of the aircraft (in conventional earth-fixed axes)
       --see e.g. Roskam J., "Airplane flight dynamics and automatic flight controls, part I", page 18, Eq. (1.36)
       --pitch axis vector is (0,1,0) in body-fixed axes 
       Yx1 = math.sin(phi * DegToRad) * math.sin(theta * DegToRad) * math.cos(psi * DegToRad) - (math.cos(phi * DegToRad) * math.sin(psi * DegToRad))
       Yy1 = math.sin(phi * DegToRad) * math.sin(theta * DegToRad) * math.cos(psi * DegToRad) + (math.cos(phi * DegToRad) * math.cos(psi * DegToRad))
       Yz1 = math.sin(phi * DegToRad) * math.cos(theta * DegToRad)
       
       --calculate the gap in delta_alpha
       alpha_gap = alpha - alpha_before
       
       --calculate the quaternion representing a rotation of -alpha_gap/5 around pitch axis
       gap_q0 = math.cos(-alpha_gap / 10 * DegToRad)
       gap_q1 = Yx1 * math.sin(-alpha_gap / 10 * DegToRad)
       gap_q2 = Yy1 * math.sin(-alpha_gap / 10 * DegToRad)
       gap_q3 = Yz1 * math.sin(-alpha_gap / 10 * DegToRad)
       
       --multiply the rotation quaternion and the orientation quaternion to obtain the final orientation quaternion, in order to progressively cancel alpha_gap
       temp2_q0 = gap_q0 * temp_q0 - gap_q1 * temp_q1 - gap_q2 * temp_q2 - gap_q3 * temp_q3
       temp2_q1 = gap_q0 * temp_q1 + gap_q1 * temp_q0 + gap_q2 * temp_q3 - gap_q3 * temp_q2
       temp2_q2 = gap_q0 * temp_q2 - gap_q1 * temp_q3 + gap_q2 * temp_q0 + gap_q3 * temp_q1
       temp2_q3 = gap_q0 * temp_q3 + gap_q1 * temp_q2 - gap_q2 * temp_q1 + gap_q3 * temp_q0
       
       temp_q0 = temp2_q0
       temp_q1 = temp2_q1
       temp_q2 = temp2_q2
       temp_q3 = temp2_q3
       
    end
    
    if query=='yaw_rate' then
    
       --find the coordinates of the yaw (Z) stability axis of the aircraft (in conventional earth-fixed axes)
       --see e.g. Roskam J., "Airplane flight dynamics and automatic flight controls, part I", page 18, Eq. (1.36)
       --yaw stability axis is (-sin(alpha),0,cos(alpha)) in body fixed axes
       Zx1 = (-math.sin(alpha * DegToRad) * math.cos(theta * DegToRad) + math.cos(alpha * DegToRad) * math.cos(phi * DegToRad) * math.sin(theta * DegToRad))*math.cos(psi * DegToRad) + (math.cos(alpha * DegToRad) * math.sin(phi * DegToRad) * math.sin(psi * DegToRad))
       Zy1 = (-math.sin(alpha * DegToRad) * math.cos(theta * DegToRad) + math.cos(alpha * DegToRad) * math.cos(phi * DegToRad) * math.sin(theta * DegToRad))*math.sin(psi * DegToRad) - (math.cos(alpha * DegToRad) * math.sin(phi * DegToRad) * math.cos(psi * DegToRad))
       Zz1 = math.sin(alpha * DegToRad) * math.sin(theta * DegToRad) + math.cos(alpha * DegToRad) * math.cos(phi * DegToRad) * math.cos(theta * DegToRad)
       
       --calculate the gap in delta_beta
       beta_gap = beta - beta_before
       
       --calculate the quaternion representing a rotation of -beta_gap/5 around pitch axis
       gap_q0 = math.cos(-beta_gap / 10 * DegToRad)
       gap_q1 = Zx1 * math.sin(-beta_gap / 10 * DegToRad)
       gap_q2 = Zy1 * math.sin(-beta_gap / 10 * DegToRad)
       gap_q3 = Zz1 * math.sin(-beta_gap / 10 * DegToRad)
       
       --multiply the rotation quaternion and the orientation quaternion to obtain the final orientation quaternion, in order to progressively cancel beta_gap
       temp2_q0 = gap_q0 * temp_q0 - gap_q1 * temp_q1 - gap_q2 * temp_q2 - gap_q3 * temp_q3
       temp2_q1 = gap_q0 * temp_q1 + gap_q1 * temp_q0 + gap_q2 * temp_q3 - gap_q3 * temp_q2
       temp2_q2 = gap_q0 * temp_q2 - gap_q1 * temp_q3 + gap_q2 * temp_q0 + gap_q3 * temp_q1
       temp2_q3 = gap_q0 * temp_q3 + gap_q1 * temp_q2 - gap_q2 * temp_q1 + gap_q3 * temp_q0
       
       temp_q0 = temp2_q0
       temp_q1 = temp2_q1
       temp_q2 = temp2_q2
       temp_q3 = temp2_q3
       
    end
    
    if query=='roll_rate' then       
       
       --find the coordinates of the roll (X) stability axis of the aircraft (in conventional earth-fixed axes)
       --see e.g. Roskam J., "Airplane flight dynamics and automatic flight controls, part I", page 18, Eq. (1.36)
       --yaw stability axis is (cos(alpha),0,sin(alpha)) in body fixed axes
       Xx1 = (math.cos(alpha * DegToRad) * math.cos(theta * DegToRad) + math.sin(alpha * DegToRad) * math.cos(phi * DegToRad) * math.sin(theta * DegToRad))*math.cos(psi * DegToRad) + (math.sin(alpha * DegToRad) * math.sin(phi * DegToRad) * math.sin(psi * DegToRad))
       Xy1 = (math.cos(alpha * DegToRad) * math.cos(theta * DegToRad) + math.sin(alpha * DegToRad) * math.cos(phi * DegToRad) * math.sin(theta * DegToRad))*math.sin(psi * DegToRad) - (math.sin(alpha * DegToRad) * math.sin(phi * DegToRad) * math.cos(psi * DegToRad))
       Xz1 = -math.cos(alpha * DegToRad) * math.sin(theta * DegToRad) + math.sin(alpha * DegToRad) * math.cos(phi * DegToRad) * math.cos(theta * DegToRad)
       
       --calculate the gap in delta_phi
       phi_gap = phi-phi_before
       
       --calculate the quaternion representing a rotation of -phi_gap/5 around roll axis
       gap_q0 = math.cos(-phi_gap / 10 * DegToRad)
       gap_q1 = Xx1 * math.sin(-phi_gap / 10 * DegToRad)
       gap_q2 = Xy1 * math.sin(-phi_gap / 10 * DegToRad)
       gap_q3 = Xz1 * math.sin(-phi_gap / 10 * DegToRad)
       
       --multiply the rotation quaternion and the orientation quaternion to obtain the final orientation quaternion, in order to progressively cancel phi_gap
       temp2_q0 = gap_q0 * temp_q0 - gap_q1 * temp_q1 - gap_q2 * temp_q2 - gap_q3 * temp_q3
       temp2_q1 = gap_q0 * temp_q1 + gap_q1 * temp_q0 + gap_q2 * temp_q3 - gap_q3 * temp_q2
       temp2_q2 = gap_q0 * temp_q2 - gap_q1 * temp_q3 + gap_q2 * temp_q0 + gap_q3 * temp_q1
       temp2_q3 = gap_q0 * temp_q3 + gap_q1 * temp_q2 - gap_q2 * temp_q1 + gap_q3 * temp_q0
       
       temp_q0 = temp2_q0
       temp_q1 = temp2_q1
       temp_q2 = temp2_q2
       temp_q3 = temp2_q3
       
    end
    
    vx = temp_vx
    vy = temp_vy
    vz = temp_vz
    
    P = temp_P
    Q = temp_Q
    R = temp_R
    
end

-- calculate the derivatives
function calculate()
        
        effective_delta_alpha = alpha - alpha_before
        effective_delta_beta = beta - beta_before    
        efdLA = left_ail_def - LA0
        efdRA = right_ail_def - RA0
        efdLE = left_elev_def - LE0
        efdRE = right_elev_def - RE0
        efdLR = left_rudder_def - LR0
        efdRR = right_rudder_def - RR0  
        efdP = throttleL - LP0
        efdp = DegToRad*(P - p_before)
        efdq = DegToRad*(Q - q_before)
        efdr = DegToRad*(R - r_before)                  
        
        
        dFx = faxil_aero - faxil_aero_before
        dFy = fside_aero - fside_aero_before
        dFz = fnrml_aero - fnrml_aero_before
        dl = L_aero - L_aero_before
        dm = M_aero - M_aero_before
        dn = N_aero - N_aero_before

        dFxp = faxil_prop - faxil_prop_before
        dFyp = fside_prop - fside_prop_before
        dFzp = fnrml_prop - fnrml_prop_before
        dlp = L_prop - L_prop_before
        dmp = M_prop - M_prop_before
        dnp = N_prop - N_prop_before

        -- Angle of attack (alpha) derivatives
        if query=='pitch' then      
           outstring = string.format("\n-------------------------\n| PITCH ANALYSIS \n-------------------------\n| effective_delta_alpha = %f deg", effective_delta_alpha)
           logMsg(outstring)
           dx = effective_delta_alpha
           write_forces()
        end
        
        -- Angle of sideslip (beta) derivatives
        if query=='yaw' then 
           outstring = string.format("\n-------------------------\n| YAW ANALYSIS \n-------------------------\n| effective_delta_beta = %f deg", effective_delta_beta)
           logMsg(outstring)
           dx = effective_delta_beta
           write_forces()
        end
        
        -- Pitch rate (q) derivatives
        if query=='pitch_rate' then          
           outstring = string.format("\n-------------------------\n| PITCH RATE ANALYSIS \n-------------------------\n| Delta q = %f rad/s, \n effectively %f rad/s", delta_q, efdq)
           logMsg(outstring)
           dx = efdq
           write_forces()
        end
        
        -- Yaw rate (r) derivatives
        if query=='yaw_rate' then
           outstring = string.format("\n-------------------------\n| YAW RATE ANALYSIS \n-------------------------\n| delta_r = %f rad/s, \n effectively %f rad/s", delta_r, efdr)
           logMsg(outstring)
           dx = efdr
           write_forces()
        end
        
        -- Roll rate (p) derivatives
        if query=='roll_rate' then
           outstring = string.format("\n-------------------------\n| ROLL RATE ANALYSIS \n-------------------------\n| delta_p = %f rad/s, \n effectively %f rad/s", delta_p, efdp)
           logMsg(outstring)
           dx = efdp
           write_forces()
        end
        
        -- Elevator (dE) derivatives
        if query=='elevator' then
           outstring = string.format("\n-------------------------\n| DELTA ELEVATOR ANALYSIS \n-------------------------\n| delta_elev = %f deg, \n effectively %f deg", delta_dE, efdLE)
           logMsg(outstring)
           dx = efdLE/dEmax
           write_forces()
        end
        
        -- Rudder (dR) derivatives
        if query=='rudder' then
           outstring = string.format("\n-------------------------\n| DELTA RUDDER ANALYSIS \n-------------------------\n| delta_rud = %f deg, \n effectively %f deg", delta_dR, efdLR)
           logMsg(outstring)
           dx = -efdLR/dRmax
           write_forces()
        end
        
        -- Ailerons (dA) derivatives
        if query=='aileron' then
           outstring = string.format("\n-------------------------\n| DELTA AILERONS ANALYSIS \n-------------------------\n| delta_ail = %f deg, \n effectively %f deg", delta_dA, efdRA)
           logMsg(outstring)
           dx = -efdLA/dAmax
           write_forces()
        end
        -- Tailerons (dES) derivatives
        if query=='taileron' then
           outstring = string.format("\n-------------------------\n| DELTA TAILERONS ANALYSIS \n-------------------------\n| delta_tail = %f deg, \n effectively %f deg", delta_dES, efdRE)
           logMsg(outstring)
           dx = -efdLE/dESmax
           write_forces()
        end
        -- Motorons (dEP) derivatives
        if query=='motoron' then
           outstring = string.format("\n-------------------------\n| DELTA MOTORONS ANALYSIS \n-------------------------\n| delta_moto = %f, \n effectively %f ", delta_dP, efdP)
           logMsg(outstring)
           dx = efdP/dPmax
           write_forces()
        end


      outstring = string.format("iter: %d \n num iterations perturbed: %d \n", iter, iter - iter_perturb)
      logMsg(outstring)
      iter_perturb = 0
      iter_calcul = iter

      -- put the state back to before perturb
      P = p_before
      Q = q_before
      R = r_before
      vx = vx_before
      vy = vy_before
      vz = vz_before
      q0 = q0_before
      q1 = q1_before
      q2 = q2_before
      q3 = q3_before
      faxil_total = faxil_total_before
      fside_total = fside_total_before
      fnrml_total = fnrml_total_before
      L_total = L_total_before
      M_total = M_total_before
      N_total = N_total_before
      left_ail_def = LA0
      right_ail_def = RA0
      left_elev_def = LE0
      right_elev_def = RE0
      left_rudder_def = LR0
      right_rudder_def = RR0
      throttleL = LP0
      throttleR = RP0

      tpause = t + 0
        
      mode = 'off'
      query = 'off'
 end

function write_forces()
    outstring = string.format("| alpha before=%f deg\n| alpha after=%f deg\n| beta before=%f deg\n| beta after=%f deg\n| phi before=%f deg\n| phi after=%f deg\n| true airspeed=%f m/s\n| initial_dynamic_pressure=%f Pa\n",
                                      alpha_before,
                                      alpha,
                                      beta_before,
                                      beta,
                                      phi_before,
                                      phi,
                                      U,
                                      initial_dynamic_pressure)
    logMsg(outstring)

    logMsg("\n")
    outstring = string.format("Fx0 = %9.4f \t Fx = %9.4f \t delFx = %9.4f [N] \t dFx = %9.4f \t dfx = %9.4f", faxil_aero_before, faxil_aero, dFx, dFx/dx, minv*dFx/dx)
    logMsg(outstring)
    outstring = string.format("Fy0 = %9.4f \t Fy = %9.4f \t delFy = %9.4f [N] \t dFy = %9.4f \t dfy = %9.4f", fside_aero_before, fside_aero, dFy, dFy/dx, minv*dFy/dx)
    logMsg(outstring)
    outstring = string.format("Fz0 = %9.4f \t Fz = %9.4f \t delFz = %9.4f [N] \t dFz = %9.4f \t dfz = %9.4f", fnrml_aero_before, fnrml_aero, dFz, dFz/dx, minv*dFz/dx)
    logMsg(outstring)
    outstring = string.format("l0  = %9.4f \t l  = %9.4f \t dell  = %9.4f [NM] \t dl = %9.4f \t dfl = %9.4f", L_aero_before, L_aero, dl, dl/dx, Iinvp*dl/dx)
    logMsg(outstring)
    outstring = string.format("m0  = %9.4f \t m  = %9.4f \t delm  = %9.4f [NM] \t dm = %9.4f \t dfm = %9.4f", M_aero_before, M_aero, dm, dm/dx, Iinvq*dm/dx)
    logMsg(outstring)
    outstring = string.format("n0  = %9.4f \t n  = %9.4f \t deln  = %9.4f [NM] \t dn = %9.4f \t dfn = %9.4f", N_aero_before, N_aero, dn, dn/dx, Iinvr*dn/dx)
    logMsg(outstring)

    logMsg("\n")
    outstring = string.format("Fxp0 = %9.4f \t Fxp = %9.4f \t delFxp = %9.4f [N] \t dFxp = %9.4f \t dfpx = %9.4f", faxil_prop_before, faxil_prop, dFxp, dFxp/dx, minv*dFxp/dx)
    logMsg(outstring)
    outstring = string.format("Fyp0 = %9.4f \t Fyp = %9.4f \t delFyp = %9.4f [N] \t dFyp = %9.4f \t dfpy = %9.4f", fside_prop_before, fside_prop, dFyp, dFyp/dx, minv*dFyp/dx)
    logMsg(outstring)
    outstring = string.format("Fzp0 = %9.4f \t Fzp = %9.4f \t delFzp = %9.4f [N] \t dFzp = %9.4f \t dfpz = %9.4f", fnrml_prop_before, fnrml_prop, dFzp, dFzp/dx, minv*dFzp/dx)
    logMsg(outstring)
    outstring = string.format("lp0  = %9.4f \t lp  = %9.4f \t dellp  = %9.4f [NM] \t dlp = %9.4f \t dfpl = %9.4f", L_prop_before, L_prop, dlp, dlp/dx, Iinvp*dlp/dx)
    logMsg(outstring)
    outstring = string.format("mp0  = %9.4f \t mp  = %9.4f \t delmp  = %9.4f [NM] \t dmp = %9.4f \t dfpm = %9.4f", M_prop_before, M_prop, dmp, dmp/dx, Iinvq*dmp/dx)
    logMsg(outstring)
    outstring = string.format("np0  = %9.4f \t np  = %9.4f \t delnp  = %9.4f [NM] \t dnp = %9.4f \t dfpn = %9.4f", N_prop_before, N_prop, dnp, dnp/dx, Iinvr*dnp/dx)
    logMsg(outstring)
end

-- do the perturbation of the selected variable
function perturbate()
    
    initial_dynamic_pressure = dynamic_pressure
    
    alpha_before = alpha
    beta_before = beta
    phi_before = phi
    
    p_before = P
    q_before = Q
    r_before = R
    p_before = 0
    q_before = 0
    r_before = 0
    vx_before = vx
    vy_before = vy
    vz_before = vz
    q0_before = q0
    q1_before = q1
    q2_before = q2
    q3_before = q3
    faxil_total_before = faxil_total
    fside_total_before = fside_total
    fnrml_total_before = fnrml_total
    L_total_before = L_total
    M_total_before = M_total
    N_total_before = N_total
    LA0 = left_ail_def
    RA0 = right_ail_def
    LE0 = left_elev_def
    RE0 = right_elev_def
    LR0 = left_rudder_def
    RR0 = right_rudder_def
    LP0 = throttleL
    RP0 = throttleR

    faxil_aero_before = faxil_aero
    fside_aero_before = fside_aero
    fnrml_aero_before = fnrml_aero
    L_aero_before = L_aero
    M_aero_before = M_aero
    N_aero_before = N_aero

    faxil_prop_before = faxil_prop
    fside_prop_before = fside_prop
    fnrml_prop_before = fnrml_prop
    L_prop_before = L_prop
    M_prop_before = M_prop
    N_prop_before = N_prop

    dt_query_max = 0
	
    if mode=='pitch' then
       --find the coordinates of the pitch (Y) axis of the aircraft (in conventional earth-fixed axes)
       --see e.g. Roskam J., "Airplane flight dynamics and automatic flight controls, part I", page 18, Eq. (1.36)
       --pitch axis vector is (0,1,0) in body-fixed axes
       Yx1 = math.sin(phi * DegToRad) * math.sin(theta * DegToRad) * math.cos(psi * DegToRad) - (math.cos(phi * DegToRad) * math.sin(psi * DegToRad))
       Yy1 = math.sin(phi * DegToRad) * math.sin(theta * DegToRad) * math.cos(psi * DegToRad) + (math.cos(phi * DegToRad) * math.cos(psi * DegToRad))
       Yz1 = math.sin(phi * DegToRad) * math.cos(theta * DegToRad)
       
       --transform from conventional earth-fixed coordinates to OGL coordinates
       YxOGL = Yy1
       YyOGL = -Yz1
       YzOGL = -Yx1
       
       --rotate the velocity vector around the pitch axis of amount -delta_alpha
       --Rodrigues' rotation formula, see https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
       vx1 = vx * math.cos(-delta_alpha * DegToRad) + (YyOGL*vz - YzOGL*vy) * math.sin(-delta_alpha * DegToRad) + YxOGL*vx*(1-math.cos(-delta_alpha * DegToRad))*YxOGL
       vy1 = vy * math.cos(-delta_alpha * DegToRad) + (YzOGL*vx - YxOGL*vz) * math.sin(-delta_alpha * DegToRad) + YyOGL*vy*(1-math.cos(-delta_alpha * DegToRad))*YyOGL
       vz1 = vz * math.cos(-delta_alpha * DegToRad) + (YxOGL*vy - YyOGL*vx) * math.sin(-delta_alpha * DegToRad) + YzOGL*vz*(1-math.cos(-delta_alpha * DegToRad))*YzOGL
       
       vx = vx1
       vy = vy1
       vz = vz1
    end
    
    if mode=='yaw' then
       --find the coordinates of the yaw (Z) stability axis of the aircraft (in conventional earth-fixed axes)
       --see e.g. Roskam J., "Airplane flight dynamics and automatic flight controls, part I", page 18, Eq. (1.36)
       --yaw stability axis is (-sin(alpha),0,cos(alpha)) in body fixed axes
       Zx1 = (-math.sin(alpha * DegToRad) * math.cos(theta * DegToRad) + math.cos(alpha * DegToRad) * math.cos(phi * DegToRad) * math.sin(theta * DegToRad))*math.cos(psi * DegToRad) + (math.cos(alpha * DegToRad) * math.sin(phi * DegToRad) * math.sin(psi * DegToRad))
       Zy1 = (-math.sin(alpha * DegToRad) * math.cos(theta * DegToRad) + math.cos(alpha * DegToRad) * math.cos(phi * DegToRad) * math.sin(theta * DegToRad))*math.sin(psi * DegToRad) - (math.cos(alpha * DegToRad) * math.sin(phi * DegToRad) * math.cos(psi * DegToRad))
       Zz1 = math.sin(alpha * DegToRad) * math.sin(theta * DegToRad) + math.cos(alpha * DegToRad) * math.cos(phi * DegToRad) * math.cos(theta * DegToRad)
       
       --transform from conventional earth-fixed coordinates to OGL coordinates
       ZxOGL = Zy1
       ZyOGL = -Zz1
       ZzOGL = -Zx1
       
       --rotate the velocity vector around the yaw stability axis of amount delta_beta
       --Rodrigues' rotation formula, see https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
       vx1 = vx * math.cos(-delta_beta * DegToRad) + (ZyOGL*vz - ZzOGL*vy) * math.sin(-delta_beta * DegToRad) + ZxOGL*vx*(1-math.cos(-delta_beta * DegToRad))*ZxOGL
       vy1 = vy * math.cos(-delta_beta * DegToRad) + (ZzOGL*vx - ZxOGL*vz) * math.sin(-delta_beta * DegToRad) + ZyOGL*vy*(1-math.cos(-delta_beta * DegToRad))*ZyOGL
       vz1 = vz * math.cos(-delta_beta * DegToRad) + (ZxOGL*vy - ZyOGL*vx) * math.sin(-delta_beta * DegToRad) + ZzOGL*vz*(1-math.cos(-delta_beta * DegToRad))*ZzOGL
       
       vx = vx1
       vy = vy1
       vz = vz1
    end
    
    if mode=='roll_rate' then
       --add an amount delta_p to roll rate
       P = P + delta_p/DegToRad
    end
    
    if mode=='yaw_rate' then
       --add an amount delta_r to yaw rate
       R = R + delta_r/DegToRad
    end
    
    if mode=='pitch_rate' then
       --add an amount delta_q to pitch rate (stability pitch axis coincides with body-fixed pitch axis)
       Q = Q + delta_q/DegToRad    
    end
    
    if mode=='elevator' then
       -- add an amount delta_de to elevator deflection
       left_elev_def = left_elev_def + delta_dE
       right_elev_def = right_elev_def + delta_dE
    end        
        
    if mode=='rudder' then
       -- add an amount delta_dr to rudder deflection
       left_rudder_def = left_rudder_def + delta_dR
       right_rudder_def = right_rudder_def + delta_dR
    end
    
    if mode=='aileron' then
       -- add a total amount delta_da to ailerons deflection
        left_ail_def = left_ail_def - delta_dA
        right_ail_def = right_ail_def + delta_dA
    end

    if mode=='taileron' then
       -- add a total amount delta_de to tailerons deflection
        left_elev_def = left_elev_def - delta_dES
        right_elev_def = right_elev_def + delta_dES
    end

    if mode=='motoron' then
       -- add a total amount delta_dp to motorons u
        throttleL = throttleL + delta_dP
        throttleR = throttleR - delta_dP
        dt_query_max = 1
    end
    
    
    -- store position, attitude, velocity and rotation rates after perturbation
    temp_x = local_x
    temp_y = local_y
    temp_z = local_z
    temp_q0 = q0
    temp_q1 = q1
    temp_q2 = q2
    temp_q3 = q3
    temp_vx = vx
    temp_vy = vy
    temp_vz = vz
    temp_P = P
    temp_Q = Q
    temp_R = R
    
    L_total = 0 -- probably for xplane not to try to move the aircraft in the iteration step, even though we are resetting the states anyway
    M_total = 0
    N_total = 0
    --faxil_total = 0
    --fside_total = 0
    --fnrml_total = 0
    
    query = mode -- store the requested type of perturbation, it will be used in the "calculate" routine
    
   	mode = 'hold' -- set plugin mode to "hold"
    tquery = t
    iter_perturb = iter
    
end

function print_text()

   if query=='pitch_rate' or query=='roll_rate' or query=='yaw_rate' then
   
      outstring = string.format("alpha_gap=%f , beta_gap=%f , phi_gap=%f",
                             alpha_gap,
                             beta_gap,
                             phi_gap)

      draw_string(16,16,outstring)
      
   end

   if mode ~= 'off' then
      outstring = string.format("mode= %s, query= %s, tquery= %d, my mode= %s", mode, query, tquery, my_mode)
      draw_string(16,100,outstring)
   end

end

do_every_frame("loop()")

do_every_draw("print_text()")
