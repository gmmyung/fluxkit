#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Workspace entry point for the `fluxkit` multi-crate library.
//!
//! The math primitives live in [`fluxkit_math`] and are re-exported here so
//! downstream users can depend on `fluxkit` as the umbrella crate.

pub use fluxkit_math::*;
