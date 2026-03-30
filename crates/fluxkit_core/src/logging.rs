//! Internal logging shims for optional `defmt` / `log` support.

#[cfg(feature = "defmt")]
macro_rules! fluxkit_warn {
    ($($arg:tt)*) => {
        defmt::warn!($($arg)*)
    };
}

#[cfg(all(not(feature = "defmt"), feature = "log"))]
macro_rules! fluxkit_warn {
    ($($arg:tt)*) => {
        log::warn!($($arg)*)
    };
}

#[cfg(all(not(feature = "defmt"), not(feature = "log")))]
macro_rules! fluxkit_warn {
    ($($arg:tt)*) => {{}};
}
