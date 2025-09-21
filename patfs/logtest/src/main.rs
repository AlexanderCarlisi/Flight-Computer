use std::fs::OpenOptions;
use std::io::{Read, Seek, SeekFrom, Write};
use std::mem;

const SECTOR_SIZE: usize = 512;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
struct FlightState {
    ax: f32, ay: f32, az:f32,
    gx: f32, gy: f32, gz:f32,
    temperature: f32, pressure: f32, altitude: f32,
    pitch: f32, roll: f32, yaw: f32,
    pidOutX: f32, pidOutY: f32, servoAngleX: f32, servoAngleY: f32,
    descent_count: i16, parachute_deployed: bool, abort: bool
}

fn main() -> std::io::Result<()> {
    let mut sd_mock = OpenOptions::new()
        .write(true)
        .create(true)
        .read(true)
        .open("mock_sd.img")?;
    let mut sector = [0u8; SECTOR_SIZE];

    let fs = FlightState {
        ax: 1.0, ay: 1.0, az: 3.0,
        gx: 0.0, gy: 1.0, gz: 2.0, temperature: 3f32,
        pressure: 4f32, altitude: 6.9602, pitch: 111100f32,
        roll: 9292.0, yaw: 345f32, pidOutX: 23f32, pidOutY: 234f32,
        servoAngleX: 2345f32, servoAngleY: 264f32,
        descent_count: 3456, parachute_deployed: true, abort: true
            
    };

    let fs_bytes = unsafe {
        std::slice::from_raw_parts(
            &fs as *const FlightState as *const u8, mem::size_of::<FlightState>()
        )
    };

    sector[..fs_bytes.len()].copy_from_slice(fs_bytes);
    sd_mock.seek(SeekFrom::Start(0))?;
    sd_mock.write_all(&sector)?;
    sd_mock.flush()?;

    println!("Sector Written Successfully");

    sd_mock.seek(SeekFrom::Start(0))?;
    let mut read_sector = [0u8; SECTOR_SIZE];
    sd_mock.read_exact(&mut read_sector)?;

    let read_fs: FlightState = unsafe {
        *(read_sector.as_ptr() as *const FlightState)
    };

    println!("Written Struct: {:?}", fs);
    println!("Read-back struct: {:?}", read_fs);

    Ok(())
}
