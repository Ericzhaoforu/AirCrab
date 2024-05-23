/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>

void setup();
void loop();
// 0XFF 0XFF | ID | Length=N+2 N:number of param | Instruction | Param1 Param 2 ... | Check Sum=~(ID+Length+Instruction+Pararm)
unsigned char vel[9] = {0xFF,0xFF,0x01,0x05,0x03,0x2E,0x00,0x00,0x00};
short int velocity=0;
unsigned char velo_buf[2]={0x00,0x00};
int increase=1;
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name, int baudrate)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(baudrate);
}
void Velocity_Cmd_Send(AP_HAL::UARTDriver *uart, const char *name, short int Vel)
{
    unsigned high_byte_vel,low_byte_vel;
    high_byte_vel=(Vel & 0xFF00)>> 8;
    low_byte_vel=(Vel & 0x00FF);
    vel[6]=low_byte_vel;
    vel[7]=high_byte_vel;
    vel[8]= ~(vel[2]+vel[3]+vel[4]+vel[5]+vel[6]+vel[7]);
    uart->write(vel,9);
    //uart->printf("Vel Cmd:%s",
                 //vel);
}

void setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized

    setup_uart(hal.serial(0), "SERIAL0",57600);  // console
    setup_uart(hal.serial(1), "SERIAL1",1000000);  // Rudder Tele 1 
    // setup_uart(hal.serial(2), "SERIAL2");  // telemetry 2
    // setup_uart(hal.serial(3), "SERIAL3");  // 1st GPS
    // setup_uart(hal.serial(4), "SERIAL4");  // 2nd GPS
    //cal the Sumcheck of Initial cmd
    vel[8]= ~(vel[2]+vel[3]+vel[4]+vel[5]+vel[6]+vel[7]);
}

// static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
// {
//     if (uart == nullptr) {
//         // that UART doesn't exist on this platform
//         return;
//     }
//     uart->printf("Hello on UART %s at %.3f seconds\n",
//                  name, (double)(AP_HAL::millis() * 0.001f));
// }

void loop(void)
{
    // uint8_t nchannels = hal.rcin->num_channels();  // Get the numbers channels detected by RC_INPUT.
    // if (nchannels == 0) {
    //     hal.console->printf("No channels detected\n");
    // }
    // else{
    //     uint16_t v = hal.rcin->read(4);
    //     hal.console->printf("val ch5:%d\n",v);
    // }
    //test_uart(hal.serial(0), "SERIAL0");
    //Velocity between -32766~ +32766
    if(hal.serial(0)->read(velo_buf,2))

    {
        velocity=(velo_buf[0]<<8)|velo_buf[1];
        hal.serial(0)->printf("Vel:%d\n",velocity);
        Velocity_Cmd_Send(hal.serial(0), "SERIAL0", velocity);
        Velocity_Cmd_Send(hal.serial(1), "SERIAL1", velocity);
    }
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
