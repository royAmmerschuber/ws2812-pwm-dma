#![no_std]
#![doc = include_str!("../README.md")]
use core::mem::{self, size_of};
use bitvec::prelude::{BitArray, Msb0};
use stm32f4xx_hal::{timer::{Pins,CCR, PwmExt,Pwm, Ch}, dma::{MemoryToPeripheral, traits::{PeriAddress, Stream, Channel, DMASet}, ChannelX}, rcc::Clocks};
use smart_leds_trait::{SmartLedsWrite, RGB8};
use embedded_hal::PwmPin;
use embedded_dma::Word;
use fugit::ExtU32;

#[allow(unused_imports)] //for doc link
use stm32f4xx_hal::timer::pwm::CPin;

mod dma_ccr_timer;
pub use dma_ccr_timer::DmaCcrTimer;


/// Handles the stream and timer used for writing to the leds
///
/// # Example
/// ```
/// let led_buf={
///     static mut led_buf:[u16;24*LED_COUNT+8]=[0;24*LED_COUNT+8];
///     unsafe{&mut led_buf};
/// }
///
/// let gpiob=dp.GPOB.split();
/// let dma1=dp.DMA1.split();
///
/// let ws_pin=gpiob.pb5.into_alternate();
///
/// // uses stream channel 5 and Timer channel 2
/// let mut ws=Ws2812Pwm::new(dp.TIM3,ws_pin,dma1.5,led_buf,&clocks);
///
/// ws.write((0..=LED_COUNT).map(|_|RGB8::new(255,255,255)));
/// ```
pub struct Ws2812Pwm<TIM,STREAM,PINS,const STR_CHAN:u8,const TIM_CHAN:u8,const FREQ:u32>
    where PINS: Pins<TIM,Ch<TIM_CHAN,false>>,
          STREAM: Stream,
          TIM:PwmExt+DmaCcrTimer<TIM_CHAN>,
          PINS::Channels:PwmPin,
          <PINS::Channels as PwmPin>::Duty:From<u16>,
          ChannelX<STR_CHAN>: Channel,
          CCR<TIM,TIM_CHAN>: PeriAddress + DMASet<STREAM, STR_CHAN, MemoryToPeripheral>,
          <CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize:From<u16>+Word+'static+Copy
        {
    stream:STREAM,
    tim:Pwm<TIM, Ch<TIM_CHAN,false>, PINS, FREQ>,
    pins:PINS::Channels,
    buf:&'static mut [<CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize],
    duty_0:<CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize,
    duty_1:<CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize,
    break_length:usize
}

impl<TIM,STREAM,PINS,const STR_CHAN:u8,const TIM_CHAN:u8,const FREQ:u32>
    Ws2812Pwm<TIM,STREAM,PINS,STR_CHAN,TIM_CHAN,FREQ>
    where PINS: Pins<TIM,Ch<TIM_CHAN,false>>,
          STREAM: Stream,
          TIM:PwmExt+DmaCcrTimer<TIM_CHAN>,
          PINS::Channels:PwmPin,
          <PINS::Channels as PwmPin>::Duty:From<u16>,
          ChannelX<STR_CHAN>: Channel,
          CCR<TIM,TIM_CHAN>: PeriAddress + DMASet<STREAM, STR_CHAN, MemoryToPeripheral>,
          <CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize:From<u16>+Word+'static+Copy
{
    /// Creates a new [`Ws2812Pwm<...>`](Ws2812Pwm).
    ///
    /// consult the implementations of [`DMASet`] to figure
    /// out wich Stream channel corresponds to which Timer Channel. And consult the [`CPin`]
    /// implementations to figure out which gpio pins in which alternate mode correspond to
    /// which Timer Channel (look at the source, the generated docs seem to have issues with
    /// the macros). you can also of course instead reference the processors Documentation.
    ///
    /// the buffer must at least have a `len()` of `24*NUM_LEDS+8`
    pub fn new(
        tim:TIM,
        pins:PINS,
        stream:STREAM,
        buf:&'static mut[<CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize],
        clocks:&Clocks
    )-> Self{
        let p_addr=Self::tim_to_ccr(&tim);

        let mut tim=tim.pwm(pins, 1250.nanos(), clocks);
        tim.set_polarity(stm32f4xx_hal::timer::Channel::C2, stm32f4xx_hal::timer::Polarity::ActiveHigh);
        let mut pins=PINS::split();
        pins.set_duty(0.into());
        pins.enable();
        TIM::enable_dma();

        let max_duty=tim.get_max_duty();
        let duty_0=((max_duty as u32*35/125) as u16).into();
        let duty_1=((max_duty as u32*70/125) as u16).into();
        //NOTE: spec says 50us but in reality 8us should be fine
        // let break_length=51_000/1_250;
        let break_length=10_000/1_250;

        let stream=Self::configure_stream(stream,p_addr.address());
        Self{
            // transfer,
            stream,
            buf,
            tim,
            pins,
            duty_0,
            duty_1,
            break_length
        }
    }
    /// Same as [`Ws2812Pwm::new`] but also enables selected interrupts
    pub fn new_with_interrupts(
        tim:TIM,
        pins:PINS,
        stream:STREAM,
        buf:&'static mut[<CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize],
        clocks:&Clocks,
        transfer_complete_interrupt:bool,
        half_transfer_interrupt:bool,
        transfer_error_interrupt:bool
    )->Self{
      let mut s=Self::new(tim, pins, stream, buf, clocks);
      s.stream.set_interrupts_enable(transfer_complete_interrupt, half_transfer_interrupt, transfer_error_interrupt, false);
      s
    }

    /// releases the held resources to be used for something else
    pub fn release(mut self)->(TIM,STREAM,&'static mut[<CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize]){
        self.pins.disable();
        self.disable_stream();
        TIM::disable_dma();
        return (self.tim.release().release(),self.stream,self.buf)
    }


    fn configure_stream(mut stream:STREAM,p_addr:u32)->STREAM{
        stream.disable();
        stream.clear_interrupts();
        stream.set_channel::<STR_CHAN>();
        stream.set_direct_mode_error_interrupt_enable(false);
        stream.set_transfer_complete_interrupt_enable(false);
        stream.set_transfer_error_interrupt_enable(false);
        stream.set_fifo_error_interrupt_enable(false);
        stream.set_direction(MemoryToPeripheral);
        stream.set_double_buffer(false);
        stream.set_fifo_enable(true);

        stream.set_memory_increment(true);
        stream.set_memory_burst(stm32f4xx_hal::dma::config::BurstMode::NoBurst);
        //NOTE: set in enable function
        //stream.set_memory_address(value);
        // stream.set_number_of_transfers(value)

        stream.set_peripheral_address(p_addr);
        stream.set_peripheral_increment(false);
        stream.set_peripheral_burst(stm32f4xx_hal::dma::config::BurstMode::NoBurst);
        unsafe{
            let memsize=mem::size_of::<<CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize>() as u8-1;
            stream.set_memory_size(memsize);
            stream.set_peripheral_size(memsize);
        }
        stream.set_priority(stm32f4xx_hal::dma::config::Priority::Medium);
        stream
    }
    fn disable_stream(&mut self){
        self.stream.disable();
    }
    fn enable_stream(&mut self,ptr:u32,len:u16){
        self.stream.set_memory_address(ptr);
        self.stream.set_number_of_transfers(len);
        unsafe{ self.stream.enable();}
    }
    fn tim_to_ccr(tim:&TIM)->CCR<TIM,TIM_CHAN>{
        //NOTE: since CRR is just a direct wrapper of TIM this should never fail
        assert_eq!(size_of::<TIM>(),size_of::<CCR<TIM,TIM_CHAN>>());
        unsafe{mem::transmute_copy(&tim)}
    }
}

impl<TIM,STREAM,PINS,const STR_CHAN:u8,const TIM_CHAN:u8,const FREQ:u32> SmartLedsWrite
    for Ws2812Pwm<TIM,STREAM,PINS,STR_CHAN,TIM_CHAN,FREQ>
    where PINS: Pins<TIM,Ch<TIM_CHAN,false>>,
          STREAM: Stream,
          TIM:PwmExt+DmaCcrTimer<TIM_CHAN>,
          PINS::Channels:PwmPin,
          <PINS::Channels as PwmPin>::Duty:From<u16>,
          ChannelX<STR_CHAN>: Channel,
          CCR<TIM,TIM_CHAN>: PeriAddress + DMASet<STREAM, STR_CHAN, MemoryToPeripheral>,
          <CCR<TIM,TIM_CHAN> as PeriAddress>::MemSize:From<u16>+Word+'static+Copy
{
    type Error=();

    type Color=RGB8;

    /// stops the current stream overrides the buffer with new colors and restarts the transfer
    ///
    /// # Errors
    /// this function never returns an error
    ///
    /// # Panics
    /// if the buffer overflows this function will panic.
    /// the buffer must have at least a `len()` of `24*NUM_LEDS+8`
    fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: Iterator<Item = I>,
        I: Into<Self::Color> {
        self.disable_stream();
        //clear interrupts because else it doesnt work (presumably because of the fifoErrFlag, which gets set always)
        self.stream.clear_interrupts();
        let buf={
            let bits=iterator
                .flat_map(|c|{
                    let c:RGB8=c.into();
                    let (r,g,b)=c.into();
                    BitArray::<_,Msb0>::new([g,r,b])
                })
                .map(|b|if b{
                    self.duty_1
                }else{
                    self.duty_0
                });
            let wait=(0..=self.break_length).map(|_|0.into());
            let mut i=0;
            for d in wait.chain(bits){
                self.buf[i]=d.into();
                i+=1;
            }
            self.buf[i]=0.into();
            i+=1;
            &self.buf[0..i]
        };
        self.enable_stream(buf.as_ptr() as u32,buf.len() as u16);
        Ok(())
    }
}
