use clap::builder::IntoResettable;
/**
 * Modified fuser example "hello.rs"
 *
 */

// FUSER includes
use clap::{Arg, ArgAction, Command};
use fuser::{
    FileAttr, FileType, Filesystem, MountOption, ReplyAttr, ReplyData, ReplyDirectory, ReplyEntry,
    Request,
};
use libc::ENOENT;
use std::ffi::OsStr;
use std::time::{Duration, UNIX_EPOCH};
use chrono::{DateTime, Local, TimeZone};

// SD includes
use std::fs::OpenOptions;
use std::io::{Read, Seek, SeekFrom};

// FUSER Consts
const TTL: Duration = Duration::from_secs(1); // 1 second
const FS_NAME: &str = "PatFS";

// SD Consts
const BLOCK_SIZE: usize = 512; // BYTES
const META_DATA_SIZE: usize = 8; // BYTES

/// Signaling the beginning of a Log, and subsequent Metadata -> flight states
const LOG_BEGIN_SEQUENCE: &str = "$$$$$$$$";

/// This allows the size of a state to be variable
/// which is nice for future updates where they may shrink or have more data
const LOG_STATE_SEQUENCE: &str = "&&&&&&&&";

const SD_PATH: &str = "mock_sd.img";

struct LogFile {
    ino: u64,
    epoch: i64,
    filename: String,
    file_size: usize // metasize + all_states
}

impl LogFile {
    
    fn check_seq<'a>(slice: &'a [u8], sequence: &str) -> bool {
        let seq = str::from_utf8(&slice);
        return seq.is_ok_and(|x| x == sequence);
    }

    fn from_slice<'a>(ino: u64, slice: &'a [u8]) -> LogFile {
        let epoch: i64 = i64::from_be_bytes(slice[..8].try_into().unwrap());
        let dt: DateTime<Local> = Local.timestamp_opt(epoch, 0).unwrap();
        let filename: String = dt.format("%m/%d/%Y-%H:%M:%S").to_string();
        
        let mut i: usize = META_DATA_SIZE; // Skip metadata
        while i < slice.len() {
            if i+8 < slice.len() && LogFile::check_seq(&slice[i..i+8], LOG_BEGIN_SEQUENCE) {
                break;
            }
            i += 1;
        }

        return LogFile {
            ino,
            epoch,
            filename,
            file_size: i
        }
    }

    fn to_fileattr(&self) -> FileAttr {
        return FileAttr {
            ino: self.ino,
            size: self.file_size as u64,
            blocks: ((self.file_size + BLOCK_SIZE - 1) / BLOCK_SIZE) as u64,
            atime: UNIX_EPOCH,
            mtime: UNIX_EPOCH,
            ctime: UNIX_EPOCH,
            crtime: UNIX_EPOCH,
            kind: FileType::RegularFile,
            perm: 0x644,
            nlink: 1,
            uid: 501,
            gid: 20,
            rdev: 0,
            flags: 0,
            blksize: BLOCK_SIZE as u32
        };
    }

    fn get_logfiles() -> Option<Vec<LogFile>> {
        let mut logs: Vec<LogFile> = Vec::new();
        let mut card = OpenOptions::new()
            .write(false)
            .create(false)
            .read(true)
            .open(SD_PATH)
            .ok()?; // TODO: proper path?
        card.seek(SeekFrom::Start(0)).ok()?;
        let mut buffer: Vec<u8> = Vec::new();
        card.read_to_end(&mut buffer).ok()?;

        let mut i = 0;
        let mut ino = 2; // 1 = logs dir
        while i < buffer.len() {
            let log: LogFile =  LogFile::from_slice(ino, &buffer[i..]);
            i += log.file_size;
            logs.push(log);
            ino += 1;
        }
        
        return Some(logs);
    }
    
    fn get_fileattrs() -> Option<Vec<FileAttr>> {
        let logfiles: Vec<LogFile> = LogFile::get_logfiles().unwrap();
        let mut fileattrs: Vec<FileAttr> = Vec::new();
        for log in logfiles {
            fileattrs.push(log.to_fileattr());
        }
        return Some(fileattrs);
    }
}

/**
 * Linux Permission levels
 * u16: [0o755] : 0o octal format, each number corresponds to categories
 * 4 = read, 2 = write, 1 = execute
 *
 * [0oOWNER-GROUP-OTHERS]
 * OWNER = 4+2+1 = 7, read,write,execute
 * GROUP = 5, read,execute
 * OTHERS = 5, read,execute
 */
const LOGS_DIR_ATTR: FileAttr = FileAttr {
    ino: 1,
    size: 0,
    blocks: 0,
    atime: UNIX_EPOCH, // 1970-01-01 00:00:00
    mtime: UNIX_EPOCH,
    ctime: UNIX_EPOCH,
    crtime: UNIX_EPOCH,
    kind: FileType::Directory,
    perm: 0o755,
    nlink: 2,
    uid: 501,
    gid: 20,
    rdev: 0,
    flags: 0,
    blksize: 512,
};


struct PatFS;

impl Filesystem for PatFS {
    fn lookup(&mut self, _req: &Request, parent: u64, name: &OsStr, replyentry: ReplyEntry) {
        let logfiles = LogFile::get_logfiles().unwrap(); // TODO: error handle?
        for log in logfiles {
            if name.to_str() == Some(log.filename.as_str()) {
                replyentry.entry(&TTL, &log.to_fileattr(), 0);
                return;
            }
        }
        replyentry.error(ENOENT);
    }
    
    /** The FUSER docs specify a param that was removed _fh :) */
    fn getattr(&mut self, _req: &Request, ino: u64, replyattr: ReplyAttr) {
        if ino == 1 {
            replyattr.attr(&TTL, &LOGS_DIR_ATTR);
        } else {
            let logattrs = LogFile::get_fileattrs().unwrap(); // TODO: error handle?
            if ino-2 > (logattrs.len() - 1) as u64 {
                replyattr.error(ENOENT);
            } else {
                replyattr.attr(&TTL, &logattrs[(ino as usize)-2]);
            }
        }
    }

    fn read(
        &mut self,
        _req: &Request,
        ino: u64,
        _fh: u64,
        offset: i64,
        _size: u32,
        _flags: i32,
        _lock: Option<u64>,
        replydata: ReplyData,
    ) {
        let logfiles = LogFile::get_logfiles().unwrap();
        if ino >= 2 && ino-2 < logfiles.len() as u64 {
            let mut card = OpenOptions::new()
                .write(false)
                .create(false)
                .read(true)
                .open(SD_PATH)
                .unwrap(); // TODO: proper path?

            let mut seekto = 0;
            let mut logi = 0;
            while logi < ino-2 {
                seekto += logfiles[logi as usize].file_size;
                logi += 1;
            }
            seekto += offset as usize;
            card.seek(SeekFrom::Start(seekto as u64)).unwrap(); // TODO:
            let mut buffer: Vec<u8> = Vec::with_capacity(_size as usize);
            card.read_exact(&mut buffer).unwrap();
            replydata.data(&buffer);
        } else {
            replydata.error(ENOENT);
        }
    }

    fn readdir(
        &mut self,
        _req: &Request,
        ino: u64,
        _fh: u64,
        offset: i64,
        mut replydir: ReplyDirectory,
    ) {
        if ino != 1 {
            replydir.error(ENOENT);
            return;
        }

        let mut entries = vec![
            // INO, FILETYPE, NAME
            (1, FileType::Directory, ".".to_string()),
            (1, FileType::Directory, "..".to_string()),
        ];
        
        let logfiles = LogFile::get_logfiles().unwrap();
        for log in logfiles {
            entries.push((log.ino, FileType::RegularFile, log.filename));
        }

        for (i, entry) in entries.into_iter().enumerate().skip(offset as usize) {
            // i + 1 means the index of the next entry
            if replydir.add(entry.0, (i + 1) as i64, entry.1, entry.2) {
                break;
            }
        }
        replydir.ok();
    }
}

fn main() {
    let matches = Command::new("FuserPatFSMount")
        .version("0")
        .author("Alexander Carlisi")
        .arg(
            Arg::new("MOUNT_POINT")
                .required(true)
                .index(1)
                .help("Act as a client, and mount FUSE at given path"),
        )
        .arg(
            Arg::new("auto_unmount")
                .long("auto_unmount")
                .action(ArgAction::SetTrue)
                .help("Automatically unmount on process exit"),
        )
        .arg(
            Arg::new("allow-root")
                .long("allow-root")
                .action(ArgAction::SetTrue)
                .help("Allow root user to access filesystem"),
        )
        .get_matches(); // parse the CLI args
    
    // env_logger::init();
    let mountpoint = matches.get_one::<String>("MOUNT_POINT").unwrap();
    
    // Mount Options, get passed to the OS to deal with.
    let mut options = vec![MountOption::RO, MountOption::FSName(FS_NAME.to_string())];
    if matches.get_flag("auto_unmount") {
        options.push(MountOption::AutoUnmount);
    }
    if matches.get_flag("allow-root") {
        options.push(MountOption::AllowRoot);
    }
    
    fuser::mount2(PatFS, mountpoint, &options).unwrap(); // hangs until unmounted
}
