For usage with the [smart-leds](https://github.com/smart-leds-rs/smart-leds) 
crate.

An [stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal) driver 
for ws2812 leds using a pwm timer and dma streaming.

It uses a buffer to store the signal & transmits it automatically in
the background over dma.

# Why
Compared to packages like [ws2812-spi](https://github.com/smart-leds-rs/ws2812-spi-rs) 
this implementation requires less timing constraints and has no problems with 
being interrupted by interrupts.

# Why not
a big problem of the current implementation is storage requirements.
this needs 48 or 96 bytes per LED (depending on the timers ARR size).

the code is also not very portable since there is no general
abstraction over DMA transfers. therefore currently only the
[stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal) is supported.

# Example
```
let led_buf={
    static mut led_buf:[u16;24*LED_COUNT+8]=[0;24*LED_COUNT+8];
    unsafe{&mut led_buf};
}

let gpiob=dp.GPIOB.split();
let dma1=dp.DMA1.split();

let ws_pin=gpiob.pb5.into_alternate();

// uses stream channel 5 and Timer channel 2
let mut ws=Ws2812Pwm::new(dp.TIM3,ws_pin,dma1.5,led_buf,&clocks);

ws.write((0..=LED_COUNT).map(|_|RGB8::new(255,255,255)));
```
