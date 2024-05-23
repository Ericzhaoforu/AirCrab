#include "Copter.h"
#define GROUND_PID 1
#define FLY_PID 0
/*
 * Init and run calls for pendulum flight mode
 * Pendulum aims to let EricCopter to land while maintaining it's attitude after landed
 * The process has two stages:
 * First Copter descent to 2m(if it is above 2m) with a speed of max_down_speed, below 2m speed change to Land_Speed
 * Then when copter touches the ground, the param of the alltitude controller is overwrited to balance the Copter
 */
bool ModePendulum::init(bool ignore_checks)
{   
    Read_Cur_PID_Param();
    Param_Change(FLY_PID);
    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), 
                    wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), 
                    wp_nav->get_wp_acceleration());
    // initialise the horizontal position controller if using GPS to crtl x,y
    if(Use_GPS_Pos_Ctrl&&!pos_control->is_active_xy())
    {
        pos_control->init_xy_controller();
    }
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), 
                    wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), 
                    wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }
    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    Descented=false;
    motors->is_Ground_Rolling=false;
    motors->take_fly=false;
    attitude_control->Grouding = false;
    manual_thr=false;
    float ground_thr = float(Eric.ERIC_G_THR);
    if(ground_thr>=0.35)
    {
        ground_thr=0.0;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "STOP!!!!!!!");
    }
    motors->g_thr = ground_thr;
    first_loop=0;
    
    return true;
}

// pendulum_run - runs the main pendulum controller
// should be called at 100hz or more(400Hz actually)
void ModePendulum::run()
{
    printf("takeof throttle%f\n",float(Eric.ERIC_TO_THR));
    //printf("gthr%f\n",motors->g_thr);
    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttltarget_roll is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        if(motors->get_spool_state()==AP_Motors::SpoolState::SHUT_DOWN)
        {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        }
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
    
    //implement the first descent stage
    if(!Descented)
    {
        Descent();
    }

    if(Descented)
    {
        Ground_Rolling();
    }

    
}
// The decent control function
void ModePendulum::Descent()
{   
    //int above_ground=get_alt_above_ground_cm();
    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AG: %d\n",above_ground);
    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "DC: %d",Descented);
    //check if armed first
    if (!motors->armed())
        {
            // Motors should be Stopped
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }
    else{
        //in the air
        //int has_manual = copter.flightmode->has_manual_throttle(); 
        switch (motors->get_spool_state())
        {
            
            //if the motor is shuted down or  in ground idle, we want our throttle control to spooling it up
            case AP_Motors::SpoolState::SHUT_DOWN:
            case AP_Motors::SpoolState::GROUND_IDLE:
            case AP_Motors::SpoolState::SPOOLING_UP:
            case AP_Motors::SpoolState::SPOOLING_DOWN:
                //To make sure using the controller with throttle input
                if(motors->is_Ground_Rolling ==true)
                {motors->is_Ground_Rolling =false;}
                //enable manual throttle input
                if(manual_thr==false)
                {
                manual_thr = true;}
                
                
                //printf("armed but not spooling,mthr status:%d\n",has_manual);
                break;
            case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
                //if(!copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED)
                if(motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED)
                {   //ensure not use manual throttle
                    if(manual_thr ==true)
                    {
                        manual_thr=false;
                    }
                    //run horizontal controller
                    Horizontal_Ctrl();
                    //run vertical controller
                    Vertical_Ctrl_v2();
                    if(LD_Detector())
                    {
                        Count_Land_End+=1;
                        //printf("%d",Count_Land_End);
                        if(Count_Land_End>=Eric.ERIC_LD_W_T ||motors->limit.throttle_lower)
                        {
                            // float ground_thr=float(Eric.ERIC_G_THR);
                            // if(ground_thr>=0.35)
                            // {
                            //     ground_thr=0.0;
                            //     GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "STOP!!!!!!!");
                            // }
                            manual_thr=true;
                            //motors->set_throttle(ground_thr);
                            Descented=true;
                            motors->is_Ground_Rolling=true;
                            attitude_control->Grouding = true;
                            //change the param of controller
                            Param_Change(GROUND_PID);
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GROUNDED");
                            attitude_control->reset_yaw_target_and_rate();
                            pos_control->relax_velocity_controller_xy();
                            pos_control->update_xy_controller();
                            pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
                            pos_control->update_z_controller();
                            attitude_control->reset_rate_controller_I_terms();
                            //attitude_control->reset_rate_controller_I_terms();
                        }
                    }
                    else
                    {
                        Count_Land_End=0;
                    }
                }
                break;
        }
        
    }
}

void ModePendulum::Ground_Rolling()
{   
    if(first_loop==0)
    {
        float ground_thr=float(Eric.ERIC_G_THR);
                            if(ground_thr>=0.4)
                            {
                                ground_thr=0.0;
                                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "STOP!!!!!!!");
                            }
    attitude_control->set_throttle_out(motors->get_throttle_hover(),true, g.throttle_filt);
        first_loop=1;
    }
    //printf("thr:%f\n",motors->get_throttle());
    //ensure using manual throttle, to enable throttle output while taking off
    if (manual_thr ==false)
    {
        manual_thr=true;
    }
    // stabilize roll,pitch
    float target_roll = 0.0f, target_pitch = 0.0f;

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, 
                auto_yaw.get_heading().yaw_rate_cds);
    if(attitude_control->err_flag)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ACC_PARAM_Err!");
    }
    float pilot_desired_throttle=(float) channel_throttle->get_control_in()*0.001f;
    //printf("input_thr:%f\n",pilot_desired_throttle);
    //float pilot_desired_throttle = get_pilot_desired_throttle();
    //NEVER GO INTO THIS LOOP TO DEBUG
    if(pilot_desired_throttle > float(Eric.ERIC_TO_THR) )
    {
        //printf("in_takeoff_loop\n");
        //output with input throttle
        // if(motors->take_fly==false)
        // {
        //     motors->take_fly = true;
        // }
        // output pilot's throttle
        //attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
        //motors->set_throttle(pilot_desired_throttle);
        //takeoff?
        // if (get_alt_above_ground_cm()>int(Eric.ERIC_D_TAKEOFF))
        // {
        //     Count_Takeoff_End+=1;
        //     if(Count_Takeoff_End>=int(Eric.ERIC_TO_W_T))
        //     {
                Param_Change(FLY_PID);
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FLY!");
                //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PIT:   P_ANG:%f P: %f I: %f D:%f\n",float(attitude_control->get_angle_pitch_p().kP()),float(attitude_control->get_rate_pitch_pid().kP()),float(attitude_control->get_rate_pitch_pid().kI()),float(attitude_control->get_rate_pitch_pid().kD()));
                //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ROL:   P_ANG:%f P: %f I: %f D:%f\n",float(attitude_control->get_angle_roll_p().kP()),float(attitude_control->get_rate_roll_pid().kP()),float(attitude_control->get_rate_roll_pid().kI()),float(attitude_control->get_rate_roll_pid().kD()));
                //CLear the flag
                Descented=false;
                motors->is_Ground_Rolling=false;
                manual_thr=false;
                Count_Land_End=0;
                motors->take_fly=false;
                attitude_control->Grouding = false;
                first_loop = 0;
                copter.set_mode(Mode::Number::LOITER,ModeReason::UNKNOWN);

                
            //}
        // }
        // else
        // {
            
        //     Count_Takeoff_End=0;
        // }
    }
    // else
    // {
    //     if(motors->take_fly==true)
    //     motors->take_fly=false;
    // }
    
}


bool ModePendulum::LD_Detector()
{
    //situation based check to determin whether landed
    //distance
    bool is_below_d_l = (get_alt_above_ground_cm()<=int(Eric.ERIC_D_L));
    //accel
    // check that the airframe is not accelerating (not falling or braking after fast forward flight)
    bool accel_stationary = (copter.land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX);
    // check that vertical speed is within 0.8cm/s of zero

    // check that vertical speed is within desired land speed
    bool descent_rate_low = (inertial_nav.get_velocity_z_up_cms() > -(int(Eric.ERIC_LD_SPEED)-2));
    //float velo= inertial_nav.get_velocity_z_up_cms();
    //printf("velo:%f\n",velo);
    motors->accel_l = accel_stationary;
    motors->distance_l = is_below_d_l;
    motors->rate_l = descent_rate_low;
    //printf("d_l:%d  ,accel:%d  ,d_rate:%d  \n",is_below_d_l,accel_stationary,descent_rate_low);
    if(is_below_d_l && accel_stationary && descent_rate_low)
    {
        return true;
    } 
    else{
    return false;
    }
}

void ModePendulum::Vertical_Ctrl()
{
    float cmb_rate = 0;
    float max_land_descent_velocity;
    
    if (g.land_speed_high > 0) {
        max_land_descent_velocity = -g.land_speed_high;
    } else {
        max_land_descent_velocity = pos_control->get_max_speed_down_cms();
    }

    // Don't speed up for landing. LAND_SPPED is also param
    max_land_descent_velocity = MIN(max_land_descent_velocity, -float(Eric.ERIC_LD_SPEED));

    // Compute a vertical velocity demand such that the vehicle approaches D_h. 
    cmb_rate = sqrt_controller(int(Eric.ERIC_D_H)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);

    // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
    cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -float(Eric.ERIC_LD_SPEED));
    // update altitude target and call position controller
    pos_control->input_vel_accel_z(cmb_rate,0.0);
    // pos_control->land_at_climb_rate_cm(cmb_rate, ignore_descent_limit);
    pos_control->update_z_controller();
    
    
}

void ModePendulum::Vertical_Ctrl_v2()
{
    float cmb_rate = 0;

    // Compute a vertical velocity demand such that the vehicle approaches D_h. 
    //cmb_rate = sqrt_controller(int(Eric.ERIC_D_L)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);

    // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
    //cmb_rate = constrain_float(cmb_rate, -int(Eric.ERIC_LD_SPEED), -1.0);
    //printf("desired velocity:%f\n",cmb_rate);
    // update altitude target and call position controller
    cmb_rate = -int(Eric.ERIC_LD_SPEED);
    pos_control->input_vel_accel_z(cmb_rate,0.0);
    // pos_control->land_at_climb_rate_cm(cmb_rate, ignore_descent_limit);
    if(Descented==false)
    {
        pos_control->update_z_controller();
    }
    
    
}


void ModePendulum::Horizontal_Ctrl()
{
    Vector2f accel,vel_correction;
    vel_correction.zero();
    accel.zero();
    pos_control->input_vel_accel_xy(vel_correction, accel);
    // run pos controller
    pos_control->update_xy_controller();
    Vector3f thrust_vector = pos_control->get_thrust_vector();

    if (g2.wp_navalt_min > 0) {
        // user has requested an altitude below which navigation
        // attitude is limited. This is used to prevent commanded roll
        // over on landing, which particularly affects helicopters if
        // there is any position estimate drift after touchdown. We
        // limit attitude to 7 degrees below this limit and linearly
        // interpolate for 1m above that
        const float attitude_limit_cd = linear_interpolate(700, copter.aparm.angle_max, get_alt_above_ground_cm(),
                                                     g2.wp_navalt_min*100U, (g2.wp_navalt_min+1)*100U);
        const float thrust_vector_max = sinf(radians(attitude_limit_cd * 0.01f)) * GRAVITY_MSS * 100.0f;
        const float thrust_vector_mag = thrust_vector.xy().length();
        if (thrust_vector_mag > thrust_vector_max) {
            float ratio = thrust_vector_max / thrust_vector_mag;
            thrust_vector.x *= ratio;
            thrust_vector.y *= ratio;

            // tell position controller we are applying an external limit
            pos_control->set_externally_limited_xy();
        }
    }
    // call attitude controller
    //attitude_control->reset_yaw_target_and_rate();
    attitude_control->input_thrust_vector_heading(thrust_vector, auto_yaw.get_heading());
}

void ModePendulum::Param_Change(int type)
{
    if (type==1){
        //pitch
        attitude_control->get_angle_roll_p().kP(float(Eric.ERIC_ROL_ANG_P));
        //Roll
        attitude_control->get_angle_pitch_p().kP(float(Eric.ERIC_PIT_ANG_P));

        //pitch rate
        attitude_control->get_rate_roll_pid().kP(float(Eric.ERIC_ROL_RAT_P));
        attitude_control->get_rate_roll_pid().kI(float(Eric.ERIC_ROL_RAT_I));
        attitude_control->get_rate_roll_pid().kD(float(Eric.ERIC_ROL_RAT_D));

        //row rate
        attitude_control->get_rate_pitch_pid().kP(float(Eric.ERIC_PIT_RAT_P));
        attitude_control->get_rate_pitch_pid().kI(float(Eric.ERIC_PIT_RAT_I));
        attitude_control->get_rate_pitch_pid().kD(float(Eric.ERIC_PIT_RAT_D));

        attitude_control->get_rate_yaw_pid().kP(float(Eric.ERIC_YAW_RAT_P));
        attitude_control->get_rate_yaw_pid().kI(float(Eric.ERIC_YAW_RAT_I));
        attitude_control->get_rate_yaw_pid().kD(float(Eric.ERIC_YAW_RAT_D));

    }
    if(type==0)
    {
        
        //pitch
        attitude_control->get_angle_roll_p().kP(P_Ang_R);
        //pitch
        attitude_control->get_angle_pitch_p().kP(P_Ang_P);
        //pitch rate
        attitude_control->get_rate_roll_pid().kP(P_Rate_R);
        attitude_control->get_rate_roll_pid().kI(I_Rate_R);
        attitude_control->get_rate_roll_pid().kD(D_Rate_R);
        //row rate
        attitude_control->get_rate_pitch_pid().kP(P_Rate_P);
        attitude_control->get_rate_pitch_pid().kI(I_Rate_P);
        attitude_control->get_rate_pitch_pid().kI(D_Rate_P);
        //yaw rate
        attitude_control->get_rate_yaw_pid().kP(P_Rate_Y);
        attitude_control->get_rate_yaw_pid().kI(I_Rate_Y);
        attitude_control->get_rate_yaw_pid().kD(D_Rate_Y);
    }
}

void ModePendulum::Read_Cur_PID_Param()
{
    // P for pitch & roll
    P_Ang_P= attitude_control->get_angle_pitch_p().kP();
    P_Ang_R= attitude_control->get_angle_roll_p().kP();
    // P,I for pitch rate & angle rate
    P_Rate_R=attitude_control->get_rate_roll_pid().kP();
    I_Rate_R=attitude_control->get_rate_roll_pid().kI();
    D_Rate_R=attitude_control->get_rate_roll_pid().kD();

    P_Rate_P=attitude_control->get_rate_pitch_pid().kP();
    I_Rate_P=attitude_control->get_rate_pitch_pid().kI();
    D_Rate_P=attitude_control->get_rate_pitch_pid().kD();

    P_Rate_Y = attitude_control->get_rate_yaw_pid().kP();
    I_Rate_Y = attitude_control->get_rate_yaw_pid().kI();
    D_Rate_Y = attitude_control->get_rate_yaw_pid().kD();

}

void ModePendulum::exit()
{
    Param_Change(FLY_PID);
    //CLear the flag
    Descented=false;
    motors->is_Ground_Rolling=false;
    motors->take_fly=false;
    attitude_control->Grouding = false;
    Count_Land_End=0;
    Count_Takeoff_End=0;
    manual_thr=false;
    first_loop=0;
}

