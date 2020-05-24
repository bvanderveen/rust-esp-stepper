#![no_std]
#![no_main]

extern crate esp32_sys;

use core::panic::PanicInfo;
use core::ptr;
use esp32_sys::*;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const STEP_GPIO: gpio_num_t = gpio_num_t_GPIO_NUM_17;
const DIRECTION_GPIO: gpio_num_t = gpio_num_t_GPIO_NUM_18;

const BLINK_GPIO: gpio_num_t = gpio_num_t_GPIO_NUM_2;
const UART_NUM: uart_port_t = uart_port_t_UART_NUM_0;
//const ECHO_TEST_TXD: i32 = gpio_num_t_GPIO_NUM_17 as i32;
//const ECHO_TEST_RXD: i32 = gpio_num_t_GPIO_NUM_16 as i32;
const ECHO_TEST_TXD: i32 = gpio_num_t_GPIO_NUM_1 as i32;
const ECHO_TEST_RXD: i32 = gpio_num_t_GPIO_NUM_3 as i32;
const ECHO_TEST_RTS: i32 = UART_PIN_NO_CHANGE;
const ECHO_TEST_CTS: i32 = UART_PIN_NO_CHANGE;

const BUF_SIZE: i32 = 1024;

#[no_mangle]
pub fn app_main() {
    unsafe {
        rust_blink_and_write();
    }
}

mod motor {
    struct Config{step_gpio: gpio_num_t, direction_gpio: gpio_num_t, rmt_channel: rmt_channel_t)
    struct LevelDuration(level: u32, duration: u32)

    impl Config {
        pub fn initialize(&self) {
            gpio_pad_select_gpio(self.step_gpio as u8);
            gpio_set_direction(self.step_gpio, gpio_mode_t_GPIO_MODE_OUTPUT);

            gpio_pad_select_gpio(self.direction_gpio as u8);
            gpio_set_direction(self.direction_gpio, gpio_mode_t_GPIO_MODE_OUTPUT);
            
            rmt_init(self.rmt_channel, self.step_gpio);
        }

        pub fn set_direction(&self, bool direction) {
            gpio_set_level(self.direction_gpio, if direction { 1 } else { 0 } as u32);
        }

        pub fn write_items(&self, items: &[rmt_item32_t]) {
            rmt_write_items(self.rmt_channel, items.as_ptr(), items.len(), false);  
        }
    }
    
}

pub use self::motor::Config
unsafe fn rust_blink_and_write() {
    //gpio_pad_select_gpio(STEP_GPIO as u8);
    //gpio_set_direction(STEP_GPIO, gpio_mode_t_GPIO_MODE_OUTPUT);

    //gpio_pad_select_gpio(DIRECTION_GPIO as u8);
    //gpio_set_direction(DIRECTION_GPIO, gpio_mode_t_GPIO_MODE_OUTPUT);

    let motor_x = motor::Config { 
        step_gpio: gpio_num_t_GPIO_NUM_17; 
        direction_gpio: gpio_num_t_GPIO_NUM_18;
    };

    let motor_y = motor::Config { 
        step_gpio: gpio_num_t_GPIO_NUM_17; 
        direction_gpio: gpio_num_t_GPIO_NUM_18;
    };

    motor_x.initialize();
    motor_y.initialize();

    let uart_config = uart_config_t {
        baud_rate: 115200,
        data_bits: uart_word_length_t_UART_DATA_8_BITS,
        parity: uart_parity_t_UART_PARITY_DISABLE,
        stop_bits: uart_stop_bits_t_UART_STOP_BITS_1,
        flow_ctrl: uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE,
        rx_flow_ctrl_thresh: 0,
        use_ref_tick: false,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, ptr::null_mut(), 0);

    let tick_period_ms: u32 = 1000 / xPortGetTickRateHz();

    let channel = rmt_channel_t_RMT_CHANNEL_0;

    //rmt_init(channel, STEP_GPIO);
    let steps_per_second = 40 * 2;
    let milliseconds_per_second = 1000;
    let milliseconds_per_step = milliseconds_per_second / steps_per_second;
    let step_period = milliseconds_per_step / tick_period_ms;

    let mut i = 0 as u32;
    let mut direction = false;

    //gpio_set_level(DIRECTION_GPIO, direction);
    motor_x.set_direction(direction);
    motor_y.set_direction(direction);
    loop {
        i = i + 1;
        if i > 10 {
            i = 0;
            direction = !direction; 
            motor_x.set_direction(direction);
            motor_y.set_direction(direction);
        }


        //gpio_set_level(STEP_GPIO, 0);

        //vTaskDelay(step_period);


        //gpio_set_level(STEP_GPIO, 1);
        let msg = "direction is ";
        log(msg.as_ptr(), msg.len());

        //vTaskDelay(step_period);

        let messages: [rmt_item32_t; 8] = [
                item(32767,1,32767,0),
                item(32767,1,32767,0),
                item(32767,0,32767,0),
                item(32767,0,32767,0),
                item(32767,1,32767,0),
                item(32767,1,32767,0),
                item(32767,0,32767,0),
                item(32767,0,32767,0)
        ];


       motor_x.write_items(&messages);
       motor_y.write_items(&messages);

//rmt_write_items(channel, messages.as_ptr(), 8, false);
    }
}

unsafe fn item(a: u32, b: u32, c: u32, d: u32) -> rmt_item32_t {
        let message = rmt_item32_t {
            __bindgen_anon_1: rmt_item32_t__bindgen_ty_1 {
                __bindgen_anon_1: rmt_item32_t__bindgen_ty_1__bindgen_ty_1 { _bitfield_1: rmt_item32_t__bindgen_ty_1__bindgen_ty_1::new_bitfield_1(a, b, c ,d) },
                }
        };
        return message;
}

unsafe fn log(message: *const u8, len: usize){
    uart_write_bytes(UART_NUM, message as *const _, len);
}

unsafe fn rmt_init(channel_id: rmt_channel_t, gpio: gpio_num_t) {
    let config = rmt_config_t {
        rmt_mode: rmt_mode_t_RMT_MODE_TX,
        channel: channel_id,
        gpio_num: gpio,
        clk_div: 255,
        mem_block_num: 1,
        //flags: 0,
            __bindgen_anon_1: rmt_config_t__bindgen_ty_1 {
                tx_config: rmt_tx_config_t {
                        loop_en: false,
                        carrier_freq_hz: 100,
                        carrier_duty_percent: 50,
                        carrier_level: rmt_carrier_level_t_RMT_CARRIER_LEVEL_HIGH,
                        idle_level: rmt_idle_level_t_RMT_IDLE_LEVEL_LOW,
                        carrier_en: true,
                        idle_output_en: true
                }
            },
    };
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

unsafe fn rmt_write() {
}

