//! Test protocol shared by the SPI link master and slave examples.

pub const FRAME_LEN: usize = 16;

const REQUEST_MAGIC: u8 = 0xa5;
const REPLY_MAGIC: u8 = 0x5a;

fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0x3c, |sum, byte| sum.rotate_left(1) ^ byte)
}

pub fn request(sequence: u8) -> [u8; FRAME_LEN] {
    let mut frame = [0; FRAME_LEN];
    frame[0] = REQUEST_MAGIC;
    frame[1] = sequence;
    for (index, byte) in frame[2..FRAME_LEN - 1].iter_mut().enumerate() {
        *byte = sequence.wrapping_add(index as u8).rotate_left((index % 8) as u32);
    }
    frame[FRAME_LEN - 1] = checksum(&frame[..FRAME_LEN - 1]);
    frame
}

pub fn reply(request: &[u8; FRAME_LEN]) -> Option<[u8; FRAME_LEN]> {
    if request[0] != REQUEST_MAGIC || request[FRAME_LEN - 1] != checksum(&request[..FRAME_LEN - 1]) {
        return None;
    }

    let mut frame = [0; FRAME_LEN];
    frame[0] = REPLY_MAGIC;
    frame[1] = request[1];
    for (reply, request) in frame[2..FRAME_LEN - 1].iter_mut().zip(&request[2..FRAME_LEN - 1]) {
        *reply = !request;
    }
    frame[FRAME_LEN - 1] = checksum(&frame[..FRAME_LEN - 1]);
    Some(frame)
}
