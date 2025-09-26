/**
 * Modified fuser example "hello.rs"
 *
 */

// FUSER includes
use clap::{Arg, ArgAction, Command, crate_version};
use fuser::{
    FileAttr, FileType, Filesystem, MountOption, ReplyAttr, ReplyData, ReplyDirectory, ReplyEntry,
    Request,
};
use libc::ENOENT;
use std::ffi::OsStr;
use std::time::{Duration, UNIX_EPOCH};
use chrono::{NaiveDateTime, Local, TimeZone};

// SD includes
use std::fs::OpenOptions;
use std::io::{Read, Seek, SeekFrom, Write};

// FUSER Consts
const TTL: Duration = Duration::from_secs(1); // 1 second
const FS_NAME: &str = "PatFS";

// SD Consts
const BLOCK_SIZE: usize = 512; // BYTES
const META_DATA_SIZE: usize = 26;


struct Log<'a> {
    stop_seq: &'a str, 
    ino: u64,
    size: usize,
    epoch: i64,
    filename: String
}
impl Log {
    fn from_slice(ino: u64, slice: [u8; usize]) -> Log {
        let stop_seq = slice[0..8];
        let epoch: i64 = slice[9..17];
        let size: usize = slice[18..20];
        let dt: DateTime<Local> = DateTime::from_timestamp_secs(log.epoch);
        let filename: String = dt.format("%m/%d/%Y %H:%M:%S").to_string();
        return Log {
            stop_seq,
            ino,
            size,
            epoch,
            filename
        }
    }

    fn to_fileattr(&self) -> FileAttr {
        return FileAttr {
            ino: self.ino,
            size: self.size,
            blocks: (self.size + BLOCK_SIZE - 1) / BLOCK_SIZE,
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
            blksize: BLOCK_SIZE
        };
    }
}

static LOGS: Vec<Log> = Vec::new();

fn populate_logs() {
    let mut card = OpenOptions::new()
        .write(false)
        .create(false)
        .read(true)
        .open("mock_sd.img")?; // TODO: proper path?
                               // 
    sd_mock.seek(SeekFrom::Start(0))?;
    let mut buffer: Vec<u8> = Vec::new();
    sd_mock.read_to_end(&mut buffer)?;
    let mut ino = 2 as u64; // 1 = logs dir
    let mut i = 0;
    while i < buffer.len() {
        if (i+8) < buffer.len() {
            let charseq = std::str::from_utf8(&buffer[i..i+8]);
            if charseq.is_ok_and(|x| x == LOG_BEGIN_SEQUENCE) {
                LOGS.push(Log::from_slice(ino, buffer[i..]));
                ino += 1;
                i += 8;
            }
        }
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


// TODO: ATTRIBUTES OF LOG FILE
// const HELLO_TXT_ATTR: FileAttr = FileAttr {
//     ino: 2,
//     size: 13,
//     blocks: 1,
//     atime: UNIX_EPOCH, // 1970-01-01 00:00:00
//     mtime: UNIX_EPOCH,
//     ctime: UNIX_EPOCH,
//     crtime: UNIX_EPOCH,
//     kind: FileType::RegularFile,
//     perm: 0o644, // [readwrite, read, read]
//     nlink: 1,
//     uid: 501,
//     gid: 20,
//     rdev: 0,
//     flags: 0,
//     blksize: 512,
// };


struct PatFS;

impl Filesystem for PatFS {
    fn lookup(&mut self, _req: &Request, parent: u64, name: &OsStr, replyentry: ReplyEntry) {
        let mut found: bool = false;
        for log in LOGS {
            if name.to_str() == log.filename {
                replyentry.entry(&TTL, log::to_fileattr(), 0);
                found = true;
                break;
            }
        }
        if !found {
            replyentry.error(ENOENT);
        }
    }

    fn getattr(&mut self, _req: &Request, ino: u64, _fh: Option<u64>, replyattr: ReplyAttr) {
        if ino == 1 {
            replyattr.attr(&TTL, &LOGS_DIR_ATTR);
        } else if ino > LOGS.len() - 1 {
            replyattr.error(ENOENT);
        } else {
            let log: Log = LOGS[ino-1];
            replyattr.attr(&TTL, &log::to_fileattr());
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
        if ino >= 2 && ino < LOGS.len() { // read_from_splice or something from SD, and just return that type shit
            // replydata.data(&HELLO_TXT_CONTENT.as_bytes()[offset as usize..]);
            let leading_logs: [Log; usize] = LOGS[..ino-1];
            let mut card = OpenOptions::new()
                .write(false)
                .create(false)
                .read(true)
                .open("mock_sd.img")?; // TODO: proper path?

            let mut seekto: u32 = 0;
            for log in leading_logs {
                seekto += log.size + META_DATA_SIZE;
            }
            let bytes = sd_mock.read_at(seekto+offset, _size)?;
            replydata.data(&bytes);
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

        let entries = vec![
            // INO, FILETYPE, NAME
            (1, FileType::Directory, "."),
            (1, FileType::Directory, ".."),
        ];
        
        LOGS.clear();
        populate_logs();
        for log in LOGS {
            entries.push((log.ino, FileType::RegularFile, String::as_str(log.filename)));
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
        .version(crate_version!())
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
    
    env_logger::init();
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
