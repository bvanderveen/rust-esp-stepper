#![no_std]
#![no_main]

extern crate esp32_sys;

use core::panic::PanicInfo;
use esp32_sys::*;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

mod motor;
use motor::*;
mod uart;
use uart::log;

static mut MILLIS_PER_TICK: u32 = 0;

fn get_millis_per_tick() {
    unsafe {
        MILLIS_PER_TICK = 1000 / xPortGetTickRateHz();
    }
}

fn sleep(millis: u32) {
    unsafe {
        let ticks = millis / MILLIS_PER_TICK;
        vTaskDelay(ticks);
    }
}

#[no_mangle]
pub fn app_main() {
    get_millis_per_tick();

    let data_callback = |motor_controller: &mut MotorController| {
        // TODO move this into the implementation
        let duration = 64;

        // TODO make a constructor that doesn't care about duration
        // and abstract the rmt_item32_t type away
        let data = [
            MotorData::new(
                [
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,1,duration,0)
                ]
            ),
            MotorData::new(
                [
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,0,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,0,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,0,duration,0),
                    MotorData::item(duration,1,duration,0),
                    MotorData::item(duration,0,duration,0)
                ]
            )
        ];
        motor_controller.write(data);
    };

    let motors = [MotorConfig::new(
        gpio_num_t_GPIO_NUM_15,
        gpio_num_t_GPIO_NUM_16,
        rmt_channel_t_RMT_CHANNEL_0
    ), MotorConfig::new( 
        gpio_num_t_GPIO_NUM_17,
        gpio_num_t_GPIO_NUM_18,
        rmt_channel_t_RMT_CHANNEL_1
    )];

    let motor_x = &motors[0];
    let motor_y = &motors[1];

    motor_x.set_direction(false);
    motor_y.set_direction(true);

    let clock_divider = 255;
    let mut motor_controller = MotorController::new(clock_divider, motors, data_callback);

    let motor_success = motor_controller.initialize();
    motor_controller.start();

    uart::initialize();
    log("Started motor controller\n");

    loop {
        let ping = if motor_success { "." } else { "-" } ;
        log(ping);
        sleep(1000);
    }
}

