#include "Copter.h"
unsigned char vel[9] = {0xFF,0xFF,0x01,0x05,0x03,0x2E,0x00,0x00,0x00};
short int velocity=0;
uint16_t v_former=0;
unsigned char velo_buf[2]={0x00,0x00};
// static void setup_uart(AP_HAL::UARTDriver *uart, const char *name, int baudrate)
// {
//     if (uart == nullptr) {
//         // that UART doesn't exist on this platform
//         return;
//     }
//     uart->begin(baudrate);
// }

void Copter::Velocity_Cmd_Send(AP_HAL::UARTDriver *uart, const char *name, short int Vel)
{
    
    int sign=0;
    if (Vel<0)
    {
        sign=1;
        Vel=-Vel;
    }
    unsigned char high_byte_vel,low_byte_vel;
    high_byte_vel=(Vel & 0xFF00)>> 8;
    low_byte_vel=(Vel & 0x00FF);
    //deal with the negative number
    if(sign==1)
    {
        high_byte_vel=high_byte_vel|(1<<7);
    }
    vel[6]=low_byte_vel;
    vel[7]=high_byte_vel;
    vel[8]= ~(vel[2]+vel[3]+vel[4]+vel[5]+vel[6]+vel[7]);
    uart->write(vel,9);
    //hal.console->printf("Vel Cmd:%s",vel);
}

short int Copter::velo_map(uint16_t v,int sacle)
{   
    short int mapped_v=0;
    if (v>=2000)
    {
        v=2000;
    }
    if(v<1000)
    {
        v=1000;
    }
    mapped_v=(v-1500)*4*sacle;
    if (mapped_v>=-50&&mapped_v<=50)
    {
        mapped_v=0;
    }
    return mapped_v;
}

void Copter::LogMotorSpinningState()
{
    switch (motors->get_spool_state())
        {
            case AP_Motors::SpoolState::SHUT_DOWN:
                motor_state=0;
                //printf("Motors SHUT DOWN\n");
                break;
            case AP_Motors::SpoolState::GROUND_IDLE:
                motor_state=1;
                //printf("Motors IDLE\n");
                break;
            case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
                motor_state=4;
                //printf("Motors UNLIMITED\n");
                break;
            case AP_Motors::SpoolState::SPOOLING_UP:
                motor_state=2;
                //printf("Motors SPOOLING_UP\n");
                break;
            case AP_Motors::SpoolState::SPOOLING_DOWN:
                motor_state=3;
                //printf("Motors SPOOLING_DOWN\n");
                break;
        }
}
bool is_setup=0;
#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // float a=attitude_control->ec_get_accel_pitch_max_radss();
    // printf("pit max:%f\n",a);
    // a=attitude_control->ec_get_accel_roll_max_radss();
    // printf("roll max:%f\n",a);
    // a=attitude_control->ec_get_accel_yaw_max_radss();
    // printf("yaw max:%f\n",a);
    if(is_setup==0)
    {
        //hal.scheduler->delay(1000); //Ensure that the uartA can be initialized
        //setup_uart(hal.serial(0), "SERIAL0",57600);  // console
        hal.serial(1)->begin(1000000);
        
        //setup_uart(hal.serial(1), "SERIAL1",1000000);  // Rudder Tele 1 
        vel[8]= ~(vel[2]+vel[3]+vel[4]+vel[5]+vel[6]+vel[7]);
        is_setup=1;
        return;
    }
    if(is_setup==1)
    {
        uint8_t nchannels = hal.rcin->num_channels();  // Get the numbers channels detected by RC_INPUT.
        if (nchannels == 0) {
            hal.console->printf("No channels detected\n");
        }
        else{
            //RC6
            uint16_t v = hal.rcin->read(5);
            //changed
            if (v!=v_former)
            {
                //hal.console->printf("val ch6:%d\n",v);
                velocity=velo_map(v,1);
                //hal.console->printf("Vel:%d\n",velocity);
                Velocity_Cmd_Send(hal.serial(1), "SERIAL1", velocity);
            }
        }
        LogMotorSpinningState();
    }
    return;
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    //put your 3.3Hz code here
    if(hal.rcin->read(4)>=1200)
    {
        //printf("colosed\n");
        return;
    }
    else{
        //printf("yaw Input: %hd\n",channel_yaw->get_control_in());
        if(copter.is_first_usr_log==1)
        {
            switch(copter.air_mode)
            {
                case(AirMode::AIRMODE_ENABLED):
                
                printf("Airmode Enabled\n");
                break;
                case(AirMode::AIRMODE_DISABLED):
                printf("Airmode Disabled\n");
                break;
                case(AirMode::AIRMODE_NONE):
                printf("Airmode Unconfig\n");
                break;
            }
            copter.is_first_usr_log=0;
        }
        // if(motors->limit.throttle_lower)
        // {
        //     printf("motors throttle lower\n");
        // }
        // if(ap.land_complete)
        // {
        //     printf("landed\n");
        // }
        // else
        // {
        //     printf("fly\n");
        //}
    }
    //float  a = AP_Motors::get_throttle_out();
    //hal.console->printf("throttle_out:%f\n",a);
    return;
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
