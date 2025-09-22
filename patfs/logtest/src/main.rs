use std::fs::OpenOptions;
use std::io::{Read, Seek, SeekFrom, Write};
use std::mem;


#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct FlightState {
    ax: f32, ay: f32, az:f32,
    gx: f32, gy: f32, gz:f32,
    temperature: f32, pressure: f32, altitude: f32,
    pitch: f32, roll: f32, yaw: f32,
    pidoutx: f32, pidouty: f32, servoangx: f32, servoangy: f32,
    descent_count: i16, parachute_deployed: bool, abort: bool
}

const SECTOR_SIZE: usize = 512;
const FLIGHTSTATE_SIZE: usize = mem::size_of::<FlightState>();
const STATES_PER_SECTOR: usize = SECTOR_SIZE / mem::size_of::<FlightState>();


fn main() -> std::io::Result<()> {
    let mut sd_mock = OpenOptions::new()
        .write(true)
        .create(true)
        .read(true)
        .open("mock_sd.img")?; // SD Card image
    
    let mut sector = [0u8; SECTOR_SIZE];
    let mut state_count = 0;
    let mut sector_index = 0;
    
    for i in 0..20 {
        let fs = FlightState {
            ax: 1.0 * i as f32,
            ay: i as f32 * 1.0,
            az: i as f32 * 3.0,
            gx: 0.0,
            gy: i as f32 * 1.0,
            gz: i as f32 * 2.0,
            temperature: i as f32 * 3.0,
            pressure: i as f32 * 4.0,
            altitude: i as f32 * 6.9602,
            pitch: i as f32 * 100.0,
            roll: i as f32 * 992.0,
            yaw: i as f32 * 345.0,
            pidoutx: i as f32 * 23.0,
            pidouty: i as f32 * 234.0,
            servoangx: i as f32 * 2345.0,
            servoangy: i as f32 * 264.0,
            descent_count: i * 356,
            parachute_deployed: true,
            abort: true,
        };

        let fs_bytes: &[u8] = unsafe {
            std::slice::from_raw_parts(
                (&fs as *const FlightState) as *const u8,
                FLIGHTSTATE_SIZE 
            )
        };

        let start = state_count * mem::size_of::<FlightState>();
        sector[start..start + mem::size_of::<FlightState>()].copy_from_slice(fs_bytes);
        state_count += 1;

        // Flush full sector
        if state_count >= STATES_PER_SECTOR {
            sd_mock.seek(SeekFrom::Start((sector_index * SECTOR_SIZE) as u64))?;
            sd_mock.write_all(&sector)?;
            sd_mock.flush()?;

            println!("Written sector {}", sector_index);
            sector_index += 1;
            sector = [0u8; SECTOR_SIZE];
            state_count = 0;
        }
    }

    if state_count > 0 {
        sd_mock.seek(SeekFrom::Start((sector_index * SECTOR_SIZE) as u64))?;
        sd_mock.write_all(&sector)?;
        sd_mock.flush()?;
        println!("Written partial sector {}", sector_index);
    }
    
    sd_mock.seek(SeekFrom::Start(0))?;
    let mut sector = [0u8; SECTOR_SIZE];
    let mut sector_idx = 0;

    loop {
        let bytes_read = sd_mock.read(&mut sector)?;
        if bytes_read == 0 {
            break; // file end
        }

        for s in 0..STATES_PER_SECTOR {
            let start = s * FLIGHTSTATE_SIZE;
            if start >= bytes_read {
                break;
            }

            let mut state_bytes = [0u8; FLIGHTSTATE_SIZE];
            state_bytes.copy_from_slice(&sector[start..start + FLIGHTSTATE_SIZE]);
            let fs: FlightState = unsafe { std::ptr::read_unaligned(state_bytes.as_ptr() as *const _) };

            println!("Sector: {} {:#?}:", sector_idx,  fs);
        }
        sector_idx += 1;
    }

    Ok(())
}
