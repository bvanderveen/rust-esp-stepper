
use esp32_sys::*;

pub struct Config {
        step_gpio: super::gpio_num_t, 
        direction_gpio: super::gpio_num_t, 
        rmt_channel: super::rmt_channel_t 
}
// struct LevelDuration { level: u32, duration: u32 }

pub unsafe fn item(a: u32, b: u32, c: u32, d: u32) -> rmt_item32_t {
    let message = rmt_item32_t {
        __bindgen_anon_1: rmt_item32_s__bindgen_ty_1 {
            __bindgen_anon_1: rmt_item32_s__bindgen_ty_1__bindgen_ty_1 { _bitfield_1: rmt_item32_s__bindgen_ty_1__bindgen_ty_1::new_bitfield_1(a, b, c ,d) },
            }
    };
    return message;
}

impl Config {
    pub fn new(step_gpio: gpio_num_t, direction_gpio: gpio_num_t, rmt_channel: rmt_channel_t) -> Config {
        Config {
            step_gpio, direction_gpio, rmt_channel
        }
    }

    unsafe fn rmt_init(&self, channel_id: esp32_sys::rmt_channel_t, gpio: esp32_sys::gpio_num_t) {
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
                        carrier_freq_hz: 0,
                        carrier_duty_percent: 0,
                        carrier_level: rmt_carrier_level_t_RMT_CARRIER_LEVEL_HIGH,
                        idle_level: rmt_idle_level_t_RMT_IDLE_LEVEL_LOW,
                        carrier_en: false,
                        idle_output_en: true
                }
            },
        };
        rmt_config(&config);
        rmt_driver_install(config.channel, 0, 0);
    }

    pub unsafe fn initialize(&self) {
        gpio_pad_select_gpio(self.step_gpio as u8);
        gpio_set_direction(self.step_gpio, gpio_mode_t_GPIO_MODE_OUTPUT);

        gpio_pad_select_gpio(self.direction_gpio as u8);
        gpio_set_direction(self.direction_gpio, gpio_mode_t_GPIO_MODE_OUTPUT);
        
        self.rmt_init(self.rmt_channel, self.step_gpio);
    }

    pub unsafe fn set_direction(&self, direction: bool) {
        gpio_set_level(self.direction_gpio, if direction { 1 } else { 0 } as u32);
    }

    pub unsafe fn write_items(&self, items: &[esp32_sys::rmt_item32_t]) {
        rmt_write_items(self.rmt_channel, items.as_ptr(), items.len() as i32, false);  
    }
}
