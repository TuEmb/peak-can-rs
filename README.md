# peak-can-rs

A Rust wrapper for PEAK-System's PCAN-Basic API, providing access to PEAK CAN interfaces.

## Features
- Supports sending and receiving CAN messages.
- Provides a safe Rust interface for working with PCAN devices.
- FFI bindings to the PCAN-Basic library.

## Requirements
- Windows OS
- PEAK-System PCAN drivers installed
- `PCANBasic.lib` and `PCANBasic.dll` available

## Installation

### 1. Install the PCAN-Basic Library
Download and install the [PCAN-Basic API](https://www.peak-system.com/PCAN-Basic.239.0.html) from PEAK-System.

### 2. Add `peak-can-rs` to Your Cargo Project

```toml
[dependencies]
peak-can = "0.1.0"
```

## Usage

### Example: Sending a CAN Message

```
cargo run --example receive
```

### Example: Receiving a CAN Message

```
cargo run --example receive
```

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributions
Contributions are welcome! Feel free to open an issue or a pull request.
Please read the contribution note first: https://github.com/TuEmb/peak-can-rs/wiki/Coding-Rules-&-Project-Constraints
