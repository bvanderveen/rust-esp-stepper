
use esp32_sys::*;
use core::mem::MaybeUninit;

use crate::uart::log;

const MEM_OWNER_SW: u32 = 0;
const MEM_OWNER_HW: u32 = 1;
const ITEM_COUNT: usize = 8;

pub struct MotorConfig {
    step_gpio: gpio_num_t, 
    direction_gpio: gpio_num_t, 
    rmt_channel: rmt_channel_t
}

// type MotorWriteCallback = fn(u8, &[rmt_item32_t]);
type MotorDataCallback = fn(&mut MotorController);

impl MotorConfig {
    pub fn new(step_gpio: gpio_num_t, direction_gpio: gpio_num_t, rmt_channel: rmt_channel_t) -> MotorConfig {
        MotorConfig {
            step_gpio, direction_gpio, rmt_channel
        }
    }

    pub fn set_direction(&self, direction: bool) {
        unsafe {
            gpio_set_level(self.direction_gpio, if direction { 1 } else { 0 } as u32);
        }
    }

    pub fn start(&self) {
        unsafe {
            rmt_ll_reset_tx_pointer(self.rmt_channel);

            // maybe this last step should be a separate one
            // to ensure minimum delay between channel starts?
            rmt_ll_start_tx(self.rmt_channel);
        }
    }
}

pub struct MotorController {
        // an item always takes a fixed amount of time to be consumed, so this thing has another config param that says how many items/sec to consume
        clock_divider: u8,
        motors: [MotorConfig; 2],
        handle: Option<rmt_isr_handle_t>,
        callback: MotorDataCallback,
        offset: u16
}

pub struct MotorData {
    items: [rmt_item32_t; ITEM_COUNT]
}

impl MotorData {
    pub fn new(items: [rmt_item32_t; ITEM_COUNT]) -> MotorData {
        MotorData {
            items
        }
    }

    pub fn item(a: u32, b: u32, c: u32, d: u32) -> rmt_item32_t {
        let message = rmt_item32_t {
            __bindgen_anon_1: rmt_item32_s__bindgen_ty_1 {
                __bindgen_anon_1: rmt_item32_s__bindgen_ty_1__bindgen_ty_1 { _bitfield_1: rmt_item32_s__bindgen_ty_1__bindgen_ty_1::new_bitfield_1(a, b, c ,d) },
                }
        };
        return message;
    }
}


unsafe fn rmt_ll_enable_mem_access(enabled: bool) {
    RMT.apb_conf.__bindgen_anon_1.set_fifo_mask(enabled as u32);
}

unsafe fn rmt_ll_enable_tx_pingpong(enabled: bool) {
    RMT.apb_conf.__bindgen_anon_1.set_mem_tx_wrap_en(enabled as u32);
}

unsafe fn rmt_ll_set_counter_clock_src(channel: u32, src: u32) {
    RMT.conf_ch[channel as usize].conf1.__bindgen_anon_1.set_ref_always_on(src);
}

unsafe fn rmt_ll_set_mem_blocks(channel: u32, num: u32) {
    RMT.conf_ch[channel as usize].conf0.__bindgen_anon_1.set_mem_size(num);
}

unsafe fn rmt_ll_set_mem_owner(channel: u32, owner: u32) {
    RMT.conf_ch[channel as usize].conf1.__bindgen_anon_1.set_mem_owner(owner);
}

unsafe fn rmt_ll_enable_tx_loop(channel: u32, enabled: bool) {
    RMT.conf_ch[channel as usize].conf1.__bindgen_anon_1.set_tx_conti_mode(enabled as u32);
}

unsafe fn rmt_ll_reset_tx_pointer(channel: u32) {
    RMT.conf_ch[channel as usize].conf1.__bindgen_anon_1.set_mem_rd_rst(1);
    RMT.conf_ch[channel as usize].conf1.__bindgen_anon_1.set_mem_rd_rst(0);
}
unsafe fn rmt_ll_start_tx(channel: u32) {
    RMT.conf_ch[channel as usize].conf1.__bindgen_anon_1.set_tx_start(1);
}

unsafe fn rmt_ll_enable_carrier(channel: u32, enabled: bool) {
    RMT.conf_ch[channel as usize].conf0.__bindgen_anon_1.set_carrier_en(enabled as u32);
}

unsafe fn rmt_ll_enable_tx_idle(channel: u32, enabled: bool) {
    RMT.conf_ch[channel as usize].conf1.__bindgen_anon_1.set_idle_out_en(enabled as u32);
}

unsafe fn rmt_ll_set_tx_idle_level(channel: u32, level: u32) {
    RMT.conf_ch[channel as usize].conf1.__bindgen_anon_1.set_idle_out_lv(level);
}

impl MotorController {
    pub fn new(clock_divider: u8, motors: [MotorConfig; 2], callback: MotorDataCallback) -> MotorController {
        let handle = None;
        let offset = 0;

        MotorController {
            clock_divider, motors, handle, callback, offset
        }
    }

    pub fn initialize(&mut self) -> bool {
        unsafe {
            let flags = 0;
            let mut handle = MaybeUninit::uninit();
            let context: *mut core::ffi::c_void = self as *mut _ as *mut core::ffi::c_void;
            let result = rmt_isr_register(Some(self::interrupt_handler), context, flags, handle.as_mut_ptr());

            if result != ESP_OK as i32 {
                return false;
            }

            self.handle = Some(handle.assume_init());

            rmt_ll_enable_mem_access(true);
            rmt_ll_enable_tx_pingpong(true);

            for m in &self.motors {
                gpio_pad_select_gpio(m.direction_gpio as u8);
                gpio_set_direction(m.direction_gpio, gpio_mode_t_GPIO_MODE_OUTPUT);

                rmt_set_pin(m.rmt_channel, rmt_mode_t_RMT_MODE_TX, m.step_gpio);

                rmt_set_clk_div(m.rmt_channel, self.clock_divider);

                rmt_ll_set_counter_clock_src(m.rmt_channel, rmt_source_clk_t_RMT_BASECLK_APB);

                rmt_ll_set_mem_blocks(m.rmt_channel, 1);
                rmt_ll_set_mem_owner(m.rmt_channel, MEM_OWNER_HW);

                rmt_ll_enable_tx_loop(m.rmt_channel, true);

                rmt_ll_enable_tx_idle(m.rmt_channel, true);
                rmt_ll_set_tx_idle_level(m.rmt_channel, rmt_idle_level_t_RMT_IDLE_LEVEL_LOW);
                
                rmt_ll_enable_carrier(m.rmt_channel, false);
            }

            return true;
        }
    }

    pub fn write(&mut self, motor_data: [MotorData; 2]) {
        if self.offset == 64 {
            self.offset = 0;
        }
        unsafe {
            for (i, d) in motor_data.iter().enumerate() {
                rmt_ll_set_mem_owner(i as u32, MEM_OWNER_SW);
                for (j, item) in d.items.iter().enumerate() {
                    RMTMEM.chan[i as usize].__bindgen_anon_1.data32[self.offset as usize + j as usize] = *item;
                }
                rmt_ll_set_mem_owner(i as u32, MEM_OWNER_HW);
            }
        }
        self.offset += ITEM_COUNT as u16;
    }

    pub fn start(&mut self) {
        (self.callback)(self);
        for m in &self.motors {
            unsafe {
                rmt_set_tx_intr_en(m.rmt_channel, true);
                rmt_set_tx_thr_intr_en(m.rmt_channel, true, ITEM_COUNT as u16);
                rmt_set_err_intr_en(m.rmt_channel, true);
            }

            m.start();
        }
    }
}

impl Drop for MotorController {
    fn drop (&mut self) {
        match self.handle {
            Some(p) => unsafe { rmt_isr_deregister(p) },
            None => 0
        };
    }


    // unsafe fn rmt_init_janky(&self, channel_id: esp32_sys::rmt_channel_t, gpio: esp32_sys::gpio_num_t) {
    //     let config = rmt_config_t {
    //         rmt_mode: rmt_mode_t_RMT_MODE_TX,
    //         channel: channel_id,
    //         gpio_num: gpio,
    //         clk_div: 255,
    //         mem_block_num: 1,
    //         //flags: 0,
    //         __bindgen_anon_1: rmt_config_t__bindgen_ty_1 {
    //             tx_config: rmt_tx_config_t {
    //                     loop_en: false,
    //                     carrier_freq_hz: 0,
    //                     carrier_duty_percent: 0,
    //                     carrier_level: rmt_carrier_level_t_RMT_CARRIER_LEVEL_HIGH,
    //                     idle_level: rmt_idle_level_t_RMT_IDLE_LEVEL_LOW,
    //                     carrier_en: false,
    //                     idle_output_en: true
    //             }
    //         },
    //     };
    //     rmt_config(&config);
    //     rmt_driver_install(config.channel, 0, 0);
    // }


    // pub unsafe fn write_items(&self, items: &[esp32_sys::rmt_item32_t]) {
    //     rmt_write_items(self.rmt_channel, items.as_ptr(), items.len() as i32, false);  
    // }
}




extern "C" fn interrupt_handler(context_ptr: *mut ::core::ffi::c_void) {
    unsafe {
        let context: &mut MotorController = &mut *(context_ptr as *mut MotorController);

        if RMT.int_st.__bindgen_anon_1.ch0_tx_end() == 1 {
            RMT.int_clr.__bindgen_anon_1.set_ch0_tx_end(1);
            log("0:e\n");
        }
        if RMT.int_st.__bindgen_anon_1.ch0_tx_thr_event() == 1 {
            RMT.int_clr.__bindgen_anon_1.set_ch0_tx_thr_event(1);
            log("0:t\n");
            (context.callback)(context);
        }
        if RMT.int_st.__bindgen_anon_1.ch0_err() == 1 {
            RMT.int_clr.__bindgen_anon_1.set_ch0_err(1);
            log("0:x\n");
        }
    }
}