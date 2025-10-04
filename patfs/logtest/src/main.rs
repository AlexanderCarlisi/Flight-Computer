use std::fs::{File, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};

/// push-float-to-vector
macro_rules! pftv {
    ($vector:expr, $f:expr) => {
        let bytes = $f.to_bits().to_be_bytes();
        for byte in bytes {
            $vector.push(byte);
        }
    };
}


/// Signaling the beginning of a Log, and subsequent Metadata -> flight states
const LOG_BEGIN_SEQUENCE: &str = "$$$$$$$$";

/// This allows the size of a state to be variable
/// which is nice for future updates where they may shrink or have more data
const LOG_STATE_SEQUENCE: &str = "&&&&&&&&";


#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct FlightState {
    dt: f32,
    ax: f32,
    ay: f32,
    az: f32,
    gx: f32,
    gy: f32,
    gz: f32,
    temperature: f32,
    pressure: f32,
    accelpitch: f32,
    accelroll: f32,
    altitude: f32,
    pitch: f32,
    roll: f32,
    yaw: f32,
    pidoutx: f32,
    pidouty: f32,
    servoangx: f32,
    servoangy: f32,
    descent_count: i16,
    parachute_deployed: bool,
    abort: bool,
    mode: u8
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct Metadata {
    epoch: i64, // File name = 'Log'+mh/dy/yr+':'+hr:mn:sc+'.pat'
}


// const SECTOR_SIZE: usize = 512;
/// Manually declare the size on the arduino, will be padded on different architectures
/// Deriving the struct as repr C only constructs it like it would in C, doesnt change the
/// padding, since that literally wouldnt make sense if you think about it.
///                      #f32, f32B
// const U8_FS_SIZE: u16 = (19 * 4) + (2 * 1) + (1 * 3);
// const FS_SIZE: usize = U8_FS_SIZE as usize;
// const U8_MD_SIZE: u16 = 64 + 16;


fn test_write(sd_mock: &mut File, logs: u16, states_per_log: usize) -> io::Result<()> {
    let mut buffer: Vec<u8> = Vec::new();
    for _ in 0..logs {
        // Each Log has metadata pertaining to it.
        // Information like begin sequence and state sequence are stored because
        // it's probably going to be easier to maintain in case things get changed
        // around.
        let metadata: Metadata = Metadata {
            epoch: 0,
        };

        for byte in LOG_BEGIN_SEQUENCE.as_bytes() {
            buffer.push(*byte);
        }
        
        for byte in metadata.epoch.to_be_bytes() {
            buffer.push(byte);
        }

        // Log example states to the 'file'
        for i in 0..states_per_log {
            let fs = FlightState {
                dt: 0.00234 * i as f32,
                ax: 1.0 * i as f32,
                ay: i as f32 * 1.0,
                az: i as f32 * 3.0,
                gx: 0.0,
                gy: i as f32 * 1.0,
                gz: i as f32 * 2.0,
                temperature: i as f32 * 3.0,
                pressure: i as f32 * 4.0,
                accelpitch: i as f32 * 0.34,
                accelroll: i as f32 * 34.345,
                altitude: i as f32 * 6.9602,
                pitch: i as f32 * 100.0,
                roll: i as f32 * 992.0,
                yaw: i as f32 * 345.0,
                pidoutx: i as f32 * 23.0,
                pidouty: i as f32 * 234.0,
                servoangx: i as f32 * 2345.0,
                servoangy: i as f32 * 264.0,
                descent_count: 4,
                parachute_deployed: true,
                abort: true,
                mode: 0x01
            };
            
            // What da heeel oh my gawd no wae ae ae aeeaaaa
            // Write the bytes directly to the vector because padding is a thing
            for byte in LOG_STATE_SEQUENCE.as_bytes() {
                buffer.push(*byte);
            }
            pftv!(buffer, fs.dt);
            pftv!(buffer, fs.ax);
            pftv!(buffer, fs.ay);
            pftv!(buffer, fs.az);
            pftv!(buffer, fs.gx);
            pftv!(buffer, fs.gy);
            pftv!(buffer, fs.gz);
            pftv!(buffer, fs.temperature);
            pftv!(buffer, fs.pressure);
            pftv!(buffer, fs.accelpitch);
            pftv!(buffer, fs.accelroll);
            pftv!(buffer, fs.altitude);
            pftv!(buffer, fs.pitch);
            pftv!(buffer, fs.roll);
            pftv!(buffer, fs.yaw);
            pftv!(buffer, fs.pidoutx);
            pftv!(buffer, fs.pidouty);
            pftv!(buffer, fs.servoangx);
            pftv!(buffer, fs.servoangy);
            for byte in fs.descent_count.to_be_bytes() {
                buffer.push(byte);
            }
            buffer.push(fs.parachute_deployed as u8);
            buffer.push(fs.abort as u8);
            buffer.push(fs.mode);
        }
    }

    // Flush full sector
    sd_mock.seek(SeekFrom::Start(std::mem::size_of::<Metadata>() as u64))?;
    sd_mock.write_all(&buffer)?;
    sd_mock.flush()?;
    Ok(())
}

fn test_read(sd_mock: &mut File) -> io::Result<()> {
    sd_mock.seek(SeekFrom::Start(0))?;
    let mut buffer: Vec<u8> = Vec::new();
    sd_mock.read_to_end(&mut buffer)?;
    let mut log_count = 0;
    let mut state_count = 0;
    
    let mut i = 0;
    while i < buffer.len() {
        if (i + 8) < buffer.len() { // Theres gotta be 8 characters for a sequence
            let charseq = std::str::from_utf8(&buffer[i..i+8]);
            if charseq.is_ok_and(|x| x == LOG_BEGIN_SEQUENCE) { // end, New Log
                println!("\nNEW LOG\n");
                i += 8;
                log_count += 1;
                continue;
                
            } else if charseq.is_ok_and(|x| x == LOG_STATE_SEQUENCE) { // end, New State
                println!("\nSTATE");
                i += 8;
                state_count += 1;
                continue;
            }
        }

        // Must be state data
        println!("{}", buffer[i]);
        i += 1;
    }
    println!("Log Count: {}", log_count);
    println!("State Count: {}", state_count);
    Ok(())
}

fn main() -> std::io::Result<()> {
    let mut sd_mock = OpenOptions::new()
        .write(true)
        .create(true)
        .read(true)
        .open("mock_sd.img")?; // SD Card image
    
    test_write(&mut sd_mock, 3, 21)?;
    test_read(&mut sd_mock)?;

    Ok(())
}
