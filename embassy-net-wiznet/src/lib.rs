#![no_std]
#![allow(async_fn_in_trait)]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

pub mod chip;
mod device;

use embassy_futures::select::{Either3, select3};
use embassy_net_driver_channel as ch;
use embassy_net_driver_channel::driver::LinkState;
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;

use crate::chip::{Chip, W5500};
pub use crate::device::InitError;
use crate::device::WiznetDevice;

// If you change this update the docs of State
const MTU: usize = 1514;

/// Type alias for the embassy-net driver.
pub type Device<'d> = embassy_net_driver_channel::Device<'d, MTU>;

/// Internal state for the embassy-net integration.
///
/// The two generic arguments `N_RX` and `N_TX` set the size of the receive and
/// send packet queue. With a the ethernet MTU of _1514_ this takes up `N_RX +
/// NTX * 1514` bytes. While setting these both to 1 is the minimum this might
/// hurt performance as a packet can not be received while processing another.
///
/// # Warning
/// On devices with a small amount of ram (think ~64k) watch out with the size
/// of there parameters. They will quickly use too much RAM.
pub struct State<const N_RX: usize, const N_TX: usize> {
    ch_state: ch::State<MTU, N_RX, N_TX>,
}

impl<const N_RX: usize, const N_TX: usize> State<N_RX, N_TX> {
    /// Create a new `State`.
    pub const fn new() -> Self {
        Self {
            ch_state: ch::State::new(),
        }
    }
}

/// PHY operating mode (PHYCFGR OPMDC bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PhyMode {
    /// 10BASE-T half duplex, no auto-negotiation.
    HalfDuplex10 = 0b000,
    /// 10BASE-T full duplex, no auto-negotiation.
    FullDuplex10 = 0b001,
    /// 100BASE-TX half duplex, no auto-negotiation.
    HalfDuplex100 = 0b010,
    /// 100BASE-TX full duplex, no auto-negotiation.
    FullDuplex100 = 0b011,
    /// 100BASE-TX half duplex, auto-negotiation on.
    HalfDuplex100Auto = 0b100,
    /// PHY powered down.
    PowerDown = 0b110,
    /// All capable, auto-negotiation on.
    AutoNegotiate = 0b111,
}

/// A PHYCFGR register value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PhyCfg(u8);

impl PhyCfg {
    /// The raw register value.
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Whether the link is up.
    pub const fn link_up(self) -> bool {
        self.0 & 0b001 != 0
    }

    /// Whether the link runs at 100 Mbps.
    pub const fn is_100mbps(self) -> bool {
        self.0 & 0b010 != 0
    }

    /// Whether the link is full duplex.
    pub const fn is_full_duplex(self) -> bool {
        self.0 & 0b100 != 0
    }
}

/// PHY reset, active low.
const PHY_CFG_RST: u8 = 1 << 7;
/// Take the mode from OPMDC, not PMODE.
const PHY_CFG_OPMD: u8 = 1 << 6;

/// Background runner for the driver.
///
/// You must call `.run()` in a background task for the driver to operate.
pub struct Runner<'d, C: Chip, SPI: SpiDevice, INT: Wait, RST: OutputPin> {
    mac: WiznetDevice<C, SPI>,
    ch: ch::Runner<'d, MTU>,
    int: INT,
    _reset: RST,
}

/// PHY configuration, W5500 only.
impl<'d, SPI: SpiDevice, INT: Wait, RST: OutputPin> Runner<'d, W5500, SPI, INT, RST> {
    /// Read the PHYCFGR register.
    pub async fn phy_cfg(&mut self) -> Result<PhyCfg, SPI::Error> {
        self.mac.read_phy_cfg().await.map(PhyCfg)
    }

    /// Force `mode` and pulse the PHY reset.
    pub async fn set_phy_mode(&mut self, mode: PhyMode) -> Result<PhyCfg, SPI::Error> {
        let cfg = PHY_CFG_OPMD | ((mode as u8) << 3);
        self.mac.write_phy_cfg(cfg).await?;
        // Ensure the reset is registered.
        Timer::after_millis(1).await;
        self.mac.write_phy_cfg(cfg | PHY_CFG_RST).await?;
        self.phy_cfg().await
    }
}

/// You must call this in a background task for the driver to operate.
impl<'d, C: Chip, SPI: SpiDevice, INT: Wait, RST: OutputPin> Runner<'d, C, SPI, INT, RST> {
    /// Run the driver.
    pub async fn run(mut self) -> ! {
        let (state_chan, mut rx_chan, mut tx_chan) = self.ch.split();
        let mut tick = Ticker::every(Duration::from_millis(500));
        loop {
            match select3(
                async {
                    self.int.wait_for_low().await.ok();
                    rx_chan.rx_buf().await
                },
                tx_chan.tx_buf(),
                tick.next(),
            )
            .await
            {
                Either3::First(p) => {
                    if let Ok(n) = self.mac.read_frame(p).await {
                        rx_chan.rx_done(n);
                    }
                }
                Either3::Second(p) => {
                    self.mac.write_frame(p).await.ok();
                    tx_chan.tx_done();
                }
                Either3::Third(()) => {
                    if self.mac.is_link_up().await {
                        state_chan.set_link_state(LinkState::Up);
                    } else {
                        state_chan.set_link_state(LinkState::Down);
                    }
                }
            }
        }
    }
}

/// Create a Wiznet ethernet chip driver for [`embassy-net`](https://crates.io/crates/embassy-net).
///
/// This returns two structs:
/// - a `Device` that you must pass to the `embassy-net` stack.
/// - a `Runner`. You must call `.run()` on it in a background task.
pub async fn new<'a, const N_RX: usize, const N_TX: usize, C: Chip, SPI: SpiDevice, INT: Wait, RST: OutputPin>(
    mac_addr: [u8; 6],
    state: &'a mut State<N_RX, N_TX>,
    spi_dev: SPI,
    int: INT,
    mut reset: RST,
) -> Result<(Device<'a>, Runner<'a, C, SPI, INT, RST>), InitError<SPI::Error>> {
    // Reset the chip.
    reset.set_low().ok();
    // Ensure the reset is registered.
    Timer::after_millis(1).await;
    reset.set_high().ok();

    // Wait for PLL lock. Some chips are slower than others.
    // Slowest is w5100s which is 100ms, so let's just wait that.
    Timer::after_millis(100).await;

    let mac = WiznetDevice::new(spi_dev, mac_addr).await?;

    let (runner, device) = ch::new(&mut state.ch_state, ch::driver::HardwareAddress::Ethernet(mac_addr));

    Ok((
        device,
        Runner {
            ch: runner,
            mac,
            int,
            _reset: reset,
        },
    ))
}
