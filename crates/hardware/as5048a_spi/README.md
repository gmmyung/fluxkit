# as5048a-spi

AS5048A SPI absolute-encoder driver.

The driver owns the protocol details that are easy to get wrong on this
sensor:

- 16-bit framed SPI transactions with chip-select toggled per transfer
- even-parity command generation and response verification
- pipelined register reads using a second `NOP` transfer
- `EF` response handling
- decoded diagnostics and angle conversion helpers
- software zero-offset support for bring-up without OTP programming

The SPI peripheral must still be configured with the mode and timing
required by the AS5048A on the target platform.
