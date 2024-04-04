// *********************************************** //
//    Ascent Program Evolution: V4.0   //
// *********************************************** //

RUNONCEPATH("Library/lib_derivator.ks").
RUNONCEPATH("Library/lib_integrator.ks").
RUNONCEPATH("Library/lib_VerticalAccelCalcs.ks").
RUNONCEPATH("Library/lib_MachNumber.ks").
RUNONCEPATH("Library/lib_BisectionSolver.ks").
RUNONCEPATH("Library/lib_execnode.ks").

// Main Inputs
parameter // these values can be passed as args from the command line.
	TargetOrbit is body:atm:height+20000, // The default target orbital altitude.
	TargetInclination is 0, // The defualt target orbital inclination.
	ExecuteCircularizeManeuver is true. // default to circularize

set TargetOrbit to max(TargetOrbit, body:atm:height+20000). // enforce minimum orbit altitude

local report_mode is 0.
	// 0 = print run indicator only.
	// 1 = also print basic report.
	// 2 = also print ascent program analysis report.

// Functions

function makePitch_rate_function {
	parameter vertspeed_min.
	local pitch_des to 0.
	local pitch_final to 90.
	local begin_pitch to false.
	local timeLast to time:seconds.
	return {
		parameter time_to_alt.
		if not(begin_pitch) AND verticalspeed > vertspeed_min {
			set begin_pitch to true.
		}
		local timeNow to time:seconds.
		local dt to timeNow - timeLast.
		set timeLast to timeNow.
		if begin_pitch AND (machNumber() < 0.85 OR machNumber() > 1.1) {
			local pitch_rate to max(0,(pitch_final - pitch_des)/time_to_alt).
			set pitch_des to min(pitch_final,max(0,pitch_des + dt*pitch_rate)).
		}
		return pitch_des.
	}.
}

local v_accel_func to makeDerivator_N(0,10).
local v_jerk_func to makeDerivator_N(0,20).

function getVertAccel {
	return v_accel_func:call(verticalspeed).
}

function getVertJerk {
	return v_jerk_func:call(getVertAccel).
}

function AltIntegration_Jerk {
	parameter time_input.
	local x is time_input.
	local d is altitude.
	local c is verticalspeed.
	local b is getVertAccel()/2.
	local a is getVertJerk()/6.
	return d + c*x + b*x^2 + a*x^3.
}

function T2Alt_Score {
	parameter time_input.
	return ship:body:atm:height - AltIntegration_Jerk(time_input).
}

function Calculate_DeltaV {
	parameter DeltaV_Data.
	local thrust_accel_1 to DeltaV_Data["Thrust_Accel"].
	local thrust_accel_2 to throttle*availablethrust/mass.
	local a_vec1 to DeltaV_Data["Accel_Vec"].
	local a_vec2 to throttle*ship:sensors:acc.
	local time1 to DeltaV_Data["Time"].
	local time2 to time:seconds.
	local dt to max(0.0001,time2 - time1).
	local thrust_accel to (thrust_accel_1 + thrust_accel_2)/2.
	local a_vec to (a_vec1 + a_vec2)/2.
	set DeltaV_Data["Total"] to DeltaV_Data["Total"] + thrust_accel*dt.
	local obt_vel_norm to ship:velocity:orbit:normalized.
	set DeltaV_Data["Gain"] to DeltaV_Data["Gain"] + dt*(VDOT(obt_vel_norm,a_vec)).
	set DeltaV_Data["Time"] to time2.
	set DeltaV_Data["Accel_Vec"] to a_vec2.
	set DeltaV_Data["Thrust_Accel"] to thrust_accel_2.
	return DeltaV_Data.
}

function circular_speed {
	parameter R_param.
	local R_val to ship:body:radius + R_param.
	return sqrt(ship:body:mu/R_val).
}

function vis_viva_speed {
	parameter R_param, a is ship:orbit:semimajoraxis.
	local R_val to ship:body:radius + R_param.
	return sqrt(ship:body:mu*(2/R_val - 1/a)).
}

function Create_Circularization_Maneuver{
	local circ_dv to circular_speed(apoapsis) - vis_viva_speed(apoapsis).
	local circ_maneuver to NODE(TIME:seconds + eta:apoapsis, 0, 0, circ_dv).
	ADD circ_maneuver.
	return circ_maneuver:deltav:mag.
}

function compute_heading {
	// Compute heading required to achieve the target orbit inclination
	parameter inc. // target inclination
	local V_orb is max(ship:velocity:orbit:mag + 1,sqrt( body:mu / ( ship:altitude + body:radius))).
	local az_orb is arcsin ( max(-1,min(1,cos(inc) / cos(ship:latitude)))).
	if (inc < 0) {set az_orb to 180 - az_orb.}
	local V_star is heading(az_orb, 0)*v(0, 0, V_orb).
	local V_ship_h is ship:velocity:orbit - vdot(ship:velocity:orbit, up:vector:normalized)*up:vector:normalized.
	local V_corr is V_star - V_ship_h.
	local vel_n is vdot(V_corr, ship:north:vector).
	local vel_e is vdot(V_corr, heading(90,0):vector).
	return arctan2(vel_e, vel_n).
}

// Ignition
set ship:control:pilotmainthrottle to 0.
local pitch_ang to 0.
local compass to compute_heading(TargetInclination).
lock throttle to 1.
lock steering to lookdirup(heading(compass,90-pitch_ang):vector,ship:facing:upvector).
stage.

// Basic Staging:
local current_max to maxthrust.
when maxthrust < current_max OR availablethrust = 0 then {
	lock throttle to 0.
	stage.
	lock throttle to 1.
	set current_max to maxthrust.
	preserve.
}

// Pitch Program Parameters: Jerk Integration
local T2Alt_Solver to makeBiSectSolver(T2Alt_Score@,100,101).
local T2Alt_TestPoints to T2Alt_Solver:call().
local pitch_controller to makePitch_rate_function(10).

// Run Mode Variables
local AscentStage is 1.
local ThrottleStage is 1.

// Delta V Variables
set DeltaV_Data to lexicon().
DeltaV_Data:ADD("Total",0).
DeltaV_Data:ADD("Gain",0).
DeltaV_Data:ADD("Time",time:seconds).
DeltaV_Data:ADD("Thrust_Accel",throttle*availablethrust/mass).
DeltaV_Data:ADD("Accel_Vec",throttle*ship:sensors:acc).

local line is 1.
local FPA is VANG(UP:vector,ship:velocity:surface).
clearscreen.

// Main Ascent Guidance Loop
until AscentStage = 2 AND altitude > ship:body:ATM:height {

	if apoapsis > TargetOrbit AND ThrottleStage = 1 {
		lock throttle to 0.
		set ThrottleStage to 2.
		set AscentStage to 2.
	}

	if AscentStage = 1 {
		set T2Alt_TestPoints to T2Alt_Solver:call().
		set pitch_ang to pitch_controller:call(T2Alt_TestPoints[2][0]).
	}

	if AscentStage = 2 {
		set pitch_ang to FPA.
	}

	set FPA to VANG(UP:vector,ship:velocity:surface).
	set compass to compute_heading(TargetInclination).
	set DeltaV_Data to Calculate_DeltaV(DeltaV_Data).

	// Print Report
	set line to 1.
	print "ASCENT GUIDANCE PROGRAM RUNNING" at(0,line). set line to line + 1.

	if report_mode >= 1 { // print basic  report
		set line to line + 1.
		print "Target Apo    = " + TargetOrbit + "   " at(0,line). set line to line + 1.
		print "Target Inc    = " + TargetInclination + "   " at(0,line). set line to line + 1.
		print "Apoapsis      = " + round(apoapsis) + "   " at(0,line). set line to line + 1.
		print "Compass       = " + round(compass,2) + "   " at(0,line). set line to line + 1.
		print "pitch_ang     = " + round(pitch_ang,2) + "   " at(0,line). set line to line + 1.
	}

	if report_mode >= 2 { // print analysis report
		set line to line + 1.
		print "ASCENT GUIDANCE ANALYSIS".
		print "ThrottleStage = " + ThrottleStage + "   " at(0,line). set line to line + 1.
		print "AscentStage   = " + AscentStage + "   " at(0,line). set line to line + 1.
		print "Vert Accel  = " + round(getVertAccel(),3) + "     " at(0,line). set line to line + 1.
		print "Vert Jerk   = " + round(getVertJerk(),3) + "     " at(0,line). set line to line + 1.
		print "Time to Alt = " + round(T2Alt_TestPoints[2][0],2) + "     " at(0,line). set line to line + 1.
		print "Gamma         = " + round(FPA,2) + "   " at(0,line). set line to line + 1.
		print "Altitude      = " + round(altitude) + "   " at(0,line). set line to line + 1.
		print "Periapsis     = " + round(periapsis) + "   " at(0,line). set line to line + 1.
		set line to line + 1.
		print "DeltaV_total  = " + round(DeltaV_Data["Total"]) + "   " at(0,line). set line to line + 1.
		print "DeltaV_gain   = " + round(DeltaV_Data["Gain"]) + "   " at(0,line). set line to line + 1.
		print "DeltaV_Losses = " + round(DeltaV_Data["Total"] - DeltaV_Data["Gain"]) + "   " at(0,line). set line to line + 1.
		print "DeltaV_Eff    = " + round(100*DeltaV_Data["Gain"]/DeltaV_Data["Total"]) + "%   " at(0,line). set line to line + 1.
	}

	wait 0.
}

local circ_maneuver_magnitude to Create_Circularization_Maneuver().
set line to line + 1.
print "Total Delta V for Circularization " + round(DeltaV_Data["Total"] + circ_maneuver_magnitude) + "    " at(0,line).

wait 3.

if ExecuteCircularizeManeuver{
	ExecuteNode().
}

print "Ascent Guidance Complete".
