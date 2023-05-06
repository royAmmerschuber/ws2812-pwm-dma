use stm32f4xx_hal::pac;

/// internally used to enable/disable the timers DMA requests
pub trait DmaCcrTimer<const TIM_CHAN:u8>{
    fn enable_dma();
    fn disable_dma();
}

macro_rules! with_dma_int{
    ($TIM:ty, CH4)=>{
        impl<const TIM_CHAN:u8> DmaCcrTimer<TIM_CHAN> for $TIM{
            fn enable_dma(){
                let tim=unsafe{ &*<$TIM>::ptr() };
                match TIM_CHAN{
                    0=>{
                        tim.dier.modify(|_,w| w.cc1de().set_bit());
                    },
                    1=>{
                        tim.dier.modify(|_,w| w.cc2de().set_bit());
                    },
                    2=>{
                        tim.dier.modify(|_,w| w.cc3de().set_bit());
                    },
                    3=>{
                        tim.dier.modify(|_,w| w.cc4de().set_bit());
                    },
                    _=>unimplemented!()
                }
            }
            fn disable_dma(){
                let tim=unsafe{ &*<$TIM>::ptr() };
                match TIM_CHAN{
                    0=>{
                        tim.dier.modify(|_,w| w.cc1de().clear_bit());
                    },
                    1=>{
                        tim.dier.modify(|_,w| w.cc2de().clear_bit());
                    },
                    2=>{
                        tim.dier.modify(|_,w| w.cc3de().clear_bit());
                    },
                    3=>{
                        tim.dier.modify(|_,w| w.cc4de().clear_bit());
                    },
                    _=>unimplemented!()
                }
            }
        }
    };
    ($TIM:ty, CH2)=>{
        impl<const TIM_CHAN:u8> DmaCcrTimer<TIM_CHAN> for $TIM{
            fn enable_dma(){
                let tim=unsafe{ &*<$TIM>::ptr() };
                match TIM_CHAN{
                    0=>{
                        tim.dier.modify(|_,w| w.cc1de().set_bit());
                    },
                    1=>{
                        tim.dier.modify(|_,w| w.cc2de().set_bit());
                    },
                    _=>unimplemented!()
                }
            }
            fn disable_dma(){
                let tim=unsafe{ &*<$TIM>::ptr() };
                match TIM_CHAN{
                    0=>{
                        tim.dier.modify(|_,w| w.cc1de().clear_bit());
                    },
                    1=>{
                        tim.dier.modify(|_,w| w.cc2de().clear_bit());
                    },
                    _=>unimplemented!()
                }
            }
        }
    };
    ($TIM:ty, CH1)=>{
        impl<const TIM_CHAN:u8> DmaCcrTimer<TIM_CHAN> for $TIM{
            fn enable_dma(){
                let tim=unsafe{ &*<$TIM>::ptr() };
                match TIM_CHAN{
                    0=>{
                        tim.dier.modify(|_,w| w.cc1de().set_bit());
                    },
                    _=>unimplemented!()
                }
            }
            fn disable_dma(){
                let tim=unsafe{ &*<$TIM>::ptr() };
                match TIM_CHAN{
                    0=>{
                        tim.dier.modify(|_,w| w.cc1de().clear_bit());
                    },
                    _=>unimplemented!()
                }
            }
        }
    };
}
macro_rules! with_dma {
    ($($TIM:ty: $T:ident,)+) => {
        $(with_dma_int!{$TIM, $T})+
    };
}

// All F4xx parts have these timers.
//NOTE: doesnt support dma interrupts
with_dma!(
    pac::TIM1: CH4,
    pac::TIM5: CH4,
//     pac::TIM9: CH2,
//     pac::TIM11: CH1,
);

// All parts except for F410 add these timers.
#[cfg(not(feature = "stm32f410"))]
with_dma!(
    pac::TIM2: CH4,
    pac::TIM3: CH4,
    pac::TIM4: CH4,
    //NOTE: doesnt support dma interrupts
    // pac::TIM10: CH1,
);

// All parts except F401 and F411.
// #[cfg(not(any(feature = "stm32f401", feature = "stm32f411",)))]
// with_dma!(pac::TIM6: [Timer6, u16, m: tim6,],);

// All parts except F401, F410, F411.
#[cfg(not(any(feature = "stm32f401", feature = "stm32f410", feature = "stm32f411",)))]
with_dma!(
    // pac::TIM7: [Timer7, u16, m: tim7,],
    pac::TIM8: CH4,
    // pac::TIM12: CH2,
    // pac::TIM13: CH1,
    // pac::TIM14: CH1,
);
