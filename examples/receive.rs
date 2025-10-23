use peak_can::bus::UsbBus;
use peak_can::socket::Baudrate;
use peak_can::socket::RecvCan;
use peak_can::socket::usb::UsbCanSocket;

fn main() {
    let usb_socket = match UsbCanSocket::open(UsbBus::USB1, Baudrate::Baud500K) {
        Ok(socket) => socket,
        Err(err) => {
            println!("{:?}", err);
            return;
        }
    };

    loop {
        let can_frame = usb_socket.recv();
        match can_frame {
            Ok((frame, timestamp)) => {
                println!("{:?}", frame);
                println!("{:?}", timestamp);
                println!("Timestamp - millis: {}, micros: {}", 
                         timestamp.millis, 
                         timestamp.micros,
                        );
            }
            Err(_) => {}
        }
    }
}
