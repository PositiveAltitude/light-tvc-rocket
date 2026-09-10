//! Winbond W25N01GV external serial-NAND support.
//!
//! The device is NAND, not a byte-addressable SPI NOR part.  A program operation
//! writes one 2 KiB page and erasure is one 128 KiB block (64 pages).  Callers
//! must therefore use an append-only layout and must never overwrite a page.

use embedded_hal::spi::{Operation, SpiDevice};
use esp_idf_hal::delay::FreeRtos;

pub const PAGE_SIZE: usize = 2_048;
pub const SPARE_SIZE: usize = 64;
pub const PAGES_PER_BLOCK: u16 = 64;
pub const BLOCK_SIZE: usize = PAGE_SIZE * PAGES_PER_BLOCK as usize;
pub const BLOCK_COUNT: u16 = 1_024;
pub const TOTAL_SIZE: usize = BLOCK_SIZE * BLOCK_COUNT as usize;

const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_RESET: u8 = 0xff;
const CMD_READ_ID: u8 = 0x9f;
const CMD_GET_FEATURE: u8 = 0x0f;
const CMD_PAGE_READ: u8 = 0x13;
const CMD_READ_CACHE: u8 = 0x03;
const CMD_PROGRAM_LOAD: u8 = 0x02;
const CMD_PROGRAM_EXECUTE: u8 = 0x10;
const CMD_BLOCK_ERASE: u8 = 0xd8;
const REG_STATUS: u8 = 0xc0;

#[derive(Debug)]
pub enum Error<E> {
    Spi(E),
    Timeout,
    ProgramFailed,
    EraseFailed,
    EccUncorrectable,
    InvalidPage,
    InvalidBlock,
}

/// A minimal, synchronous W25N01GV driver.  It deliberately exposes NAND's
/// page/block geometry instead of pretending that this chip is a NOR flash.
pub struct W25N01GV<SPI> {
    spi: SPI,
}

impl<SPI> W25N01GV<SPI>
where
    SPI: SpiDevice<u8>,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    pub fn reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.write(&[CMD_RESET])?;
        self.wait_ready().map(|_| ())
    }

    /// Returns the three JEDEC bytes.  A W25N01GV normally reports EF AA 21.
    pub fn read_id(&mut self) -> Result<[u8; 3], Error<SPI::Error>> {
        // The first returned byte is a dummy; then come manufacturer, memory
        // type, and capacity (EF AA 21 for W25N01GV).
        let mut response = [0; 4];
        self.transaction(&[CMD_READ_ID], &mut response)?;
        Ok([response[1], response[2], response[3]])
    }

    /// Reads exactly one 2 KiB data page.  The 64-byte OOB/spare area is not
    /// returned, which prevents application data from damaging bad-block marks.
    pub fn read_page(
        &mut self,
        page: u16,
        data: &mut [u8; PAGE_SIZE],
    ) -> Result<(), Error<SPI::Error>> {
        self.validate_page(page)?;
        self.page_to_cache(page)?;
        self.read_cache(0, data)
    }

    /// Programs data at an erased page column. Each physical page accepts at
    /// most four program executions between erases; callers must account for
    /// that limit (the artifact store uses two on page zero and one elsewhere).
    pub fn program_page(&mut self, page: u16, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.program_page_at(page, 0, data)
    }

    pub fn program_page_at(
        &mut self,
        page: u16,
        column: u16,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        self.validate_page(page)?;
        if data.is_empty() || column as usize + data.len() > PAGE_SIZE {
            return Err(Error::InvalidPage);
        }
        self.write_enable()?;
        // PROGRAM LOAD and its payload must share one chip-select assertion.
        self.write_payload(&[CMD_PROGRAM_LOAD, (column >> 8) as u8, column as u8], data)?;
        self.write_enable()?;
        self.write(&[CMD_PROGRAM_EXECUTE, 0, (page >> 8) as u8, page as u8])?;
        let status = self.wait_ready()?;
        if status & 0x08 != 0 {
            Err(Error::ProgramFailed)
        } else {
            Ok(())
        }
    }

    /// Erases one 128 KiB block.  Erase destroys all 64 pages in the block.
    pub fn erase_block(&mut self, block: u16) -> Result<(), Error<SPI::Error>> {
        if block >= BLOCK_COUNT {
            return Err(Error::InvalidBlock);
        }
        let page = block * PAGES_PER_BLOCK;
        self.write_enable()?;
        self.write(&[CMD_BLOCK_ERASE, 0, (page >> 8) as u8, page as u8])?;
        let status = self.wait_ready()?;
        if status & 0x04 != 0 {
            Err(Error::EraseFailed)
        } else {
            Ok(())
        }
    }

    /// Checks the factory bad-block marker in the first page's spare area.
    pub fn is_bad_block(&mut self, block: u16) -> Result<bool, Error<SPI::Error>> {
        if block >= BLOCK_COUNT {
            return Err(Error::InvalidBlock);
        }
        // The factory marker lives in spare data, and an erased/unprogrammed
        // first page need not have meaningful ECC parity. Do not reject a
        // block-marker read merely because its main page has an ECC status.
        let page = block * PAGES_PER_BLOCK;
        self.write(&[CMD_PAGE_READ, 0, (page >> 8) as u8, page as u8])?;
        self.wait_ready()?;
        let mut marker = [0; 1];
        self.read_cache(PAGE_SIZE as u16, &mut marker)?;
        Ok(marker[0] != 0xff)
    }

    fn validate_page(&self, page: u16) -> Result<(), Error<SPI::Error>> {
        if page as usize >= BLOCK_COUNT as usize * PAGES_PER_BLOCK as usize {
            Err(Error::InvalidPage)
        } else {
            Ok(())
        }
    }

    fn write_enable(&mut self) -> Result<(), Error<SPI::Error>> {
        self.write(&[CMD_WRITE_ENABLE])
    }

    fn page_to_cache(&mut self, page: u16) -> Result<(), Error<SPI::Error>> {
        self.write(&[CMD_PAGE_READ, 0, (page >> 8) as u8, page as u8])?;
        let status = self.wait_ready()?;
        // ECCS=10 or 11 means an uncorrectable page.  ECCS=01 is corrected.
        if status & 0x30 >= 0x20 {
            Err(Error::EccUncorrectable)
        } else {
            Ok(())
        }
    }

    fn read_cache(&mut self, column: u16, data: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        self.transaction(
            &[CMD_READ_CACHE, (column >> 8) as u8, column as u8, 0],
            data,
        )
    }

    fn wait_ready(&mut self) -> Result<u8, Error<SPI::Error>> {
        // Datasheet maxima are milliseconds; this loop is bounded so a missing
        // or wedged chip cannot stall a flight-control task indefinitely.
        for _ in 0..10_000 {
            let status = self.feature(REG_STATUS)?;
            if status & 0x01 == 0 {
                return Ok(status);
            }
        }
        Err(Error::Timeout)
    }

    fn feature(&mut self, address: u8) -> Result<u8, Error<SPI::Error>> {
        let mut value = [0];
        self.transaction(&[CMD_GET_FEATURE, address], &mut value)?;
        Ok(value[0])
    }

    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.spi.write(bytes).map_err(Error::Spi)
    }

    fn transaction(&mut self, command: &[u8], read: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        self.spi
            .transaction(&mut [Operation::Write(command), Operation::Read(read)])
            .map_err(Error::Spi)
    }

    fn write_payload(&mut self, command: &[u8], payload: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.spi
            .transaction(&mut [Operation::Write(command), Operation::Write(payload)])
            .map_err(Error::Spi)
    }
}

const ARTIFACT_MAGIC: [u8; 4] = *b"LRAF";
const COMMIT_MAGIC: [u8; 4] = *b"DONE";
const ARTIFACT_VERSION: u8 = 1;
const METADATA_SECTOR_SIZE: usize = 512;
const COMMIT_COLUMN: u16 = METADATA_SECTOR_SIZE as u16;
const DATA_PAGES_PER_BLOCK: u16 = PAGES_PER_BLOCK - 1;
pub const ARTIFACT_BLOCK_CAPACITY: usize = DATA_PAGES_PER_BLOCK as usize * PAGE_SIZE;
const ARTIFACT_NAME_CAPACITY: usize = 240;

#[derive(Debug)]
pub enum ArtifactError<E> {
    Flash(Error<E>),
    InvalidName,
    NoSpace,
    NotFound,
    CorruptMetadata,
    Incomplete,
}

/// Metadata returned for the newest complete version of a named artifact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArtifactInfo {
    pub name: String,
    pub generation: u32,
    pub size: u64,
    pub blocks: u16,
}

#[derive(Clone, Debug)]
struct BlockRecord {
    physical_block: u16,
    name: String,
    generation: u32,
    logical_index: u16,
    size: u32,
    crc32: u32,
    is_final: bool,
    total_size: u64,
}

/// Versioned named artifacts on W25N01GV NAND.
///
/// Each physical block holds one sequential artifact segment. Page zero uses
/// one 512-byte program for its immutable header and a second for its commit
/// record. Pages 1..63 each receive one program operation. A version becomes
/// visible only when its final block is committed.
pub struct ArtifactStore<SPI> {
    flash: W25N01GV<SPI>,
}

impl<SPI> ArtifactStore<SPI>
where
    SPI: SpiDevice<u8>,
{
    pub fn new(flash: W25N01GV<SPI>) -> Self {
        Self { flash }
    }

    pub fn into_inner(self) -> W25N01GV<SPI> {
        self.flash
    }

    pub fn reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.flash.reset()
    }

    pub fn read_id(&mut self) -> Result<[u8; 3], Error<SPI::Error>> {
        self.flash.read_id()
    }

    /// Begins a new version without modifying older versions. Call `finish`
    /// to publish it; dropping an unfinished writer leaves an ignored version.
    pub fn begin(
        &mut self,
        name: &str,
    ) -> Result<ArtifactWriter<'_, SPI>, ArtifactError<SPI::Error>> {
        validate_name(name)?;
        let generation = self.next_generation(name)?;
        let block = self.allocate_block(name, generation, 0)?;
        Ok(ArtifactWriter {
            store: self,
            name: name.to_owned(),
            generation,
            logical_index: 0,
            block,
            next_page: 1,
            buffered: 0,
            page_buffer: [0xff; PAGE_SIZE],
            block_size: 0,
            block_crc: Crc32::new(),
            total_size: 0,
            finished: false,
        })
    }

    /// Finds the most recent complete version. The scan reads page zero of
    /// each non-bad block, as specified by the on-flash layout.
    pub fn find(&mut self, name: &str) -> Result<ArtifactInfo, ArtifactError<SPI::Error>> {
        validate_name(name)?;
        let mut records = self.scan_records()?;
        records.retain(|record| record.name == name);
        let final_record = records
            .iter()
            .filter(|record| record.is_final)
            .max_by_key(|record| record.generation)
            .ok_or(ArtifactError::NotFound)?;
        let generation = final_record.generation;
        let final_index = final_record.logical_index;
        for index in 0..=final_index {
            if !records
                .iter()
                .any(|record| record.generation == generation && record.logical_index == index)
            {
                return Err(ArtifactError::Incomplete);
            }
        }
        Ok(ArtifactInfo {
            name: name.to_owned(),
            generation,
            size: final_record.total_size,
            blocks: final_index + 1,
        })
    }

    /// Streams the newest complete artifact without allocating a copy of it.
    pub fn read(
        &mut self,
        name: &str,
        mut consumer: impl FnMut(&[u8]),
    ) -> Result<(), ArtifactError<SPI::Error>> {
        let info = self.find(name)?;
        let mut records = self.scan_records()?;
        records.retain(|record| record.name == name && record.generation == info.generation);
        let mut remaining = info.size as usize;
        let mut page = [0xff; PAGE_SIZE];
        for logical_index in 0..info.blocks {
            let record = records
                .iter()
                .find(|record| record.logical_index == logical_index)
                .ok_or(ArtifactError::Incomplete)?;
            let mut block_remaining = record.size as usize;
            let block_crc = record.crc32;
            let mut crc = Crc32::new();
            for page_offset in 1..PAGES_PER_BLOCK {
                if block_remaining == 0 {
                    break;
                }
                self.flash
                    .read_page(
                        record.physical_block * PAGES_PER_BLOCK + page_offset,
                        &mut page,
                    )
                    .map_err(ArtifactError::Flash)?;
                let length = block_remaining.min(PAGE_SIZE).min(remaining);
                crc.update(&page[..length]);
                consumer(&page[..length]);
                block_remaining -= length;
                remaining -= length;
            }
            if block_remaining != 0 || crc.finish() != block_crc {
                return Err(ArtifactError::CorruptMetadata);
            }
        }
        if remaining == 0 {
            Ok(())
        } else {
            Err(ArtifactError::Incomplete)
        }
    }

    /// Permanently erases every block belonging to every version of `name`.
    pub fn erase(&mut self, name: &str) -> Result<(), ArtifactError<SPI::Error>> {
        validate_name(name)?;
        for record in self
            .scan_records()?
            .into_iter()
            .filter(|record| record.name == name)
        {
            self.flash
                .erase_block(record.physical_block)
                .map_err(ArtifactError::Flash)?;
        }
        Ok(())
    }

    fn next_generation(&mut self, name: &str) -> Result<u32, ArtifactError<SPI::Error>> {
        Ok(self
            .scan_records()?
            .iter()
            .filter(|record| record.name == name)
            .map(|record| record.generation)
            .max()
            .unwrap_or(0)
            .wrapping_add(1))
    }

    fn allocate_block(
        &mut self,
        name: &str,
        generation: u32,
        logical_index: u16,
    ) -> Result<u16, ArtifactError<SPI::Error>> {
        let mut page = [0xff; PAGE_SIZE];
        for block in 0..BLOCK_COUNT {
            if block % 8 == 0 {
                FreeRtos::delay_ms(1);
            }
            if self
                .flash
                .is_bad_block(block)
                .map_err(ArtifactError::Flash)?
            {
                continue;
            }
            self.flash
                .read_page(block * PAGES_PER_BLOCK, &mut page)
                .map_err(ArtifactError::Flash)?;
            if page.iter().all(|byte| *byte == 0xff) {
                self.flash
                    .erase_block(block)
                    .map_err(ArtifactError::Flash)?;
                let mut header = [0xff; METADATA_SECTOR_SIZE];
                encode_header(&mut header, name, generation, logical_index)?;
                self.flash
                    .program_page_at(block * PAGES_PER_BLOCK, 0, &header)
                    .map_err(ArtifactError::Flash)?;
                return Ok(block);
            }
        }
        Err(ArtifactError::NoSpace)
    }

    fn commit_block(
        &mut self,
        block: u16,
        size: u32,
        crc32: u32,
        is_final: bool,
        total_size: u64,
    ) -> Result<(), ArtifactError<SPI::Error>> {
        let mut commit = [0xff; METADATA_SECTOR_SIZE];
        commit[..4].copy_from_slice(&COMMIT_MAGIC);
        commit[4..8].copy_from_slice(&size.to_le_bytes());
        commit[8..12].copy_from_slice(&crc32.to_le_bytes());
        commit[12] = if is_final { 0 } else { 0x7f };
        commit[16..24].copy_from_slice(&total_size.to_le_bytes());
        self.flash
            .program_page_at(block * PAGES_PER_BLOCK, COMMIT_COLUMN, &commit)
            .map_err(ArtifactError::Flash)
    }

    fn scan_records(&mut self) -> Result<Vec<BlockRecord>, ArtifactError<SPI::Error>> {
        let mut records = Vec::new();
        let mut page = [0xff; PAGE_SIZE];
        for block in 0..BLOCK_COUNT {
            if block % 8 == 0 {
                FreeRtos::delay_ms(1);
            }
            if self
                .flash
                .is_bad_block(block)
                .map_err(ArtifactError::Flash)?
            {
                continue;
            }
            self.flash
                .read_page(block * PAGES_PER_BLOCK, &mut page)
                .map_err(ArtifactError::Flash)?;
            let Some((name, generation, logical_index)) =
                decode_header(&page[..METADATA_SECTOR_SIZE])
            else {
                continue;
            };
            let Some((size, crc32, is_final, total_size)) =
                decode_commit(&page[METADATA_SECTOR_SIZE..])
            else {
                continue;
            };
            if size as usize > ARTIFACT_BLOCK_CAPACITY {
                continue;
            }
            records.push(BlockRecord {
                physical_block: block,
                name,
                generation,
                logical_index,
                size,
                crc32,
                is_final,
                total_size,
            });
        }
        Ok(records)
    }
}

/// An in-progress artifact writer. It owns a 2 KiB page buffer so callers may
/// stream arbitrary-sized writes while the NAND only receives safe page writes.
pub struct ArtifactWriter<'a, SPI>
where
    SPI: SpiDevice<u8>,
{
    store: &'a mut ArtifactStore<SPI>,
    name: String,
    generation: u32,
    logical_index: u16,
    block: u16,
    next_page: u16,
    buffered: usize,
    page_buffer: [u8; PAGE_SIZE],
    block_size: u32,
    block_crc: Crc32,
    total_size: u64,
    finished: bool,
}

impl<SPI> ArtifactWriter<'_, SPI>
where
    SPI: SpiDevice<u8>,
{
    pub fn write(&mut self, mut data: &[u8]) -> Result<(), ArtifactError<SPI::Error>> {
        if self.finished {
            return Err(ArtifactError::Incomplete);
        }
        while !data.is_empty() {
            if self.next_page == PAGES_PER_BLOCK && self.buffered == 0 {
                self.advance_block()?;
            }
            let take = (PAGE_SIZE - self.buffered).min(data.len());
            self.page_buffer[self.buffered..self.buffered + take].copy_from_slice(&data[..take]);
            self.buffered += take;
            data = &data[take..];
            if self.buffered == PAGE_SIZE {
                self.flush_page()?;
            }
        }
        Ok(())
    }

    /// Commits the final block and publishes this version atomically.
    pub fn finish(mut self) -> Result<ArtifactInfo, ArtifactError<SPI::Error>> {
        if self.buffered != 0 {
            self.flush_page()?;
        }
        self.store.commit_block(
            self.block,
            self.block_size,
            self.block_crc.finish(),
            true,
            self.total_size,
        )?;
        self.finished = true;
        Ok(ArtifactInfo {
            name: self.name.clone(),
            generation: self.generation,
            size: self.total_size,
            blocks: self.logical_index + 1,
        })
    }

    fn flush_page(&mut self) -> Result<(), ArtifactError<SPI::Error>> {
        let page = self.block * PAGES_PER_BLOCK + self.next_page;
        self.store
            .flash
            .program_page(page, &self.page_buffer[..self.buffered])
            .map_err(ArtifactError::Flash)?;
        self.block_crc.update(&self.page_buffer[..self.buffered]);
        self.block_size += self.buffered as u32;
        self.total_size += self.buffered as u64;
        self.next_page += 1;
        self.buffered = 0;
        self.page_buffer.fill(0xff);
        Ok(())
    }

    fn advance_block(&mut self) -> Result<(), ArtifactError<SPI::Error>> {
        self.store.commit_block(
            self.block,
            self.block_size,
            self.block_crc.finish(),
            false,
            0,
        )?;
        self.logical_index = self
            .logical_index
            .checked_add(1)
            .ok_or(ArtifactError::NoSpace)?;
        self.block = self
            .store
            .allocate_block(&self.name, self.generation, self.logical_index)?;
        self.next_page = 1;
        self.block_size = 0;
        self.block_crc = Crc32::new();
        Ok(())
    }
}

fn validate_name<E>(name: &str) -> Result<(), ArtifactError<E>> {
    if name.is_empty() || name.len() > ARTIFACT_NAME_CAPACITY || name.as_bytes().contains(&0) {
        Err(ArtifactError::InvalidName)
    } else {
        Ok(())
    }
}

fn encode_header<E>(
    out: &mut [u8; METADATA_SECTOR_SIZE],
    name: &str,
    generation: u32,
    logical_index: u16,
) -> Result<(), ArtifactError<E>> {
    validate_name(name)?;
    out[..4].copy_from_slice(&ARTIFACT_MAGIC);
    out[4] = ARTIFACT_VERSION;
    out[5] = name.len() as u8;
    out[8..12].copy_from_slice(&generation.to_le_bytes());
    out[12..14].copy_from_slice(&logical_index.to_le_bytes());
    out[16..16 + name.len()].copy_from_slice(name.as_bytes());
    Ok(())
}

fn decode_header(bytes: &[u8]) -> Option<(String, u32, u16)> {
    if bytes.len() < METADATA_SECTOR_SIZE
        || bytes[..4] != ARTIFACT_MAGIC
        || bytes[4] != ARTIFACT_VERSION
    {
        return None;
    }
    let name_len = bytes[5] as usize;
    if name_len == 0 || name_len > ARTIFACT_NAME_CAPACITY {
        return None;
    }
    let name = core::str::from_utf8(&bytes[16..16 + name_len])
        .ok()?
        .to_owned();
    Some((
        name,
        u32::from_le_bytes(bytes[8..12].try_into().ok()?),
        u16::from_le_bytes(bytes[12..14].try_into().ok()?),
    ))
}

fn decode_commit(bytes: &[u8]) -> Option<(u32, u32, bool, u64)> {
    if bytes.len() < METADATA_SECTOR_SIZE
        || bytes[..4] != COMMIT_MAGIC
        || (bytes[12] != 0 && bytes[12] != 0x7f)
    {
        return None;
    }
    Some((
        u32::from_le_bytes(bytes[4..8].try_into().ok()?),
        u32::from_le_bytes(bytes[8..12].try_into().ok()?),
        bytes[12] == 0,
        u64::from_le_bytes(bytes[16..24].try_into().ok()?),
    ))
}

#[derive(Clone, Copy)]
struct Crc32(u32);

impl Crc32 {
    const fn new() -> Self {
        Self(!0)
    }
    fn update(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.0 ^= *byte as u32;
            for _ in 0..8 {
                self.0 = if self.0 & 1 != 0 {
                    (self.0 >> 1) ^ 0xedb8_8320
                } else {
                    self.0 >> 1
                };
            }
        }
    }
    const fn finish(self) -> u32 {
        !self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn metadata_round_trip() {
        let mut header = [0xff; METADATA_SECTOR_SIZE];
        encode_header::<()>(&mut header, "flight-log", 17, 3).unwrap();
        assert_eq!(
            decode_header(&header),
            Some(("flight-log".to_owned(), 17, 3))
        );
    }

    #[test]
    fn crc32_matches_standard_vector() {
        let mut crc = Crc32::new();
        crc.update(b"123456789");
        assert_eq!(crc.finish(), 0xcbf4_3926);
    }
}
