use core::ptr;
use esp32_sys::*;


const UART_NUM: uart_port_t = uart_port_t_UART_NUM_0;
const ECHO_TEST_TXD: i32 = gpio_num_t_GPIO_NUM_1 as i32;
const ECHO_TEST_RXD: i32 = gpio_num_t_GPIO_NUM_3 as i32;
const ECHO_TEST_RTS: i32 = UART_PIN_NO_CHANGE;
const ECHO_TEST_CTS: i32 = UART_PIN_NO_CHANGE;

const BUF_SIZE: i32 = 1024;

pub fn initialize() {
    unsafe {
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
    }
}


pub fn log(message: &str) {
    unsafe {
        uart_write_bytes(UART_NUM, message.as_ptr() as *const _, message.len() as u32);
    }
}