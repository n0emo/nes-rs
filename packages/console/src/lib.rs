pub const RESET_VECTOR: u16 = 0xfffc;

#[derive(Clone)]
pub struct Console {
    pub cpu: Cpu,
    pub ram: Box<[u8; 2usize.pow(15)]>,
    pub rom: Box<[u8; 2usize.pow(15)]>,
}

#[derive(Debug)]
pub struct WriteError;

#[derive(Debug)]
pub struct ReadError;

pub trait Memory {
    fn read(&self, address: u16) -> Result<u8, ReadError> {
        let _ = address;
        Err(ReadError)
    }

    fn read_word(&self, address: u16) -> Result<Word, ReadError> {
        let low = self.read(address)?;
        let high = self.read(address + 1)?;
        Ok(Word { low, high })
    }

    fn write(&mut self, address: u16, value: u8) -> Result<(), WriteError> {
        let _ = (address, value);
        Err(WriteError)
    }

    fn write_word(&mut self, address: u16, value: Word) -> Result<(), WriteError> {
        self.write(address, value.low)?;
        self.write(address + 1, value.high)?;
        Ok(())
    }
}

impl Console {
    pub fn new() -> Self {
        Self {
            cpu: Cpu::default(),
            ram: Box::new([0; 2usize.pow(15)]),
            rom: Box::new([0; 2usize.pow(15)]),
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct Cpu {
    /// program counter
    pub pc: u16,

    /// accumulator
    pub a: u8,

    /// X register
    pub x: u8,

    /// Y register
    pub y: u8,

    /// stack pointer
    pub sp: u8,

    /// Status Register
    pub sr: StatusRegister,

    pub command_queue: VecDeque<CpuCommand>,

    pub operand: Word,

    pub value: Word,
}

impl Cpu {
    /// Perform init sequence
    pub fn init(&mut self, mem: &dyn Memory) {
        *self = Default::default();
        let pc = mem
            .read_word(RESET_VECTOR)
            .expect("Error reading reset vector");
        self.pc = pc.get();
        self.push_command(CpuCommand::FetchInstrution);
    }

    /// Execute single CPU cycle
    ///
    /// Note: this function does not execute the whole instructions as single
    /// one can take several cycles
    pub fn cycle(&mut self, mem: &mut dyn Memory) -> Result<(), ExecuteError> {
        let cmd = self.pop_command().expect("No command in the command queue");
        self.execute_command(mem, cmd)
    }

    fn execute_command(
        &mut self,
        mem: &mut dyn Memory,
        command: CpuCommand,
    ) -> Result<(), ExecuteError> {
        match command {
            CpuCommand::FetchInstrution => {
                if let Err(e) = self.fetch_instruction(mem) {
                    self.command_queue.clear();
                    self.push_command(CpuCommand::FetchInstrution);
                    return Err(e);
                } else {
                    self.push_command(CpuCommand::FetchInstrution);
                }
            }
            CpuCommand::ReadA => {
                log::debug!("ReadA: read ${:02x} from A", self.a);
                self.value = Word {
                    low: self.a,
                    high: 0,
                };
            }
            CpuCommand::WriteA => {
                log::debug!("WriteA: write ${:02x} to A", self.value.low);
                self.a = self.value.low;
            }
            CpuCommand::WritePc => {
                log::debug!("WritePc: write ${:04x} to pc", self.value.get());
                self.pc = self.operand.get();
            }
            CpuCommand::LoadAddressAbsolute => {
                self.operand = mem.read_word(self.pc)?;
                self.pc += 2;
                log::debug!(
                    "LoadAddressAbsolute: loaded address ${:04x} to operand",
                    self.operand.get()
                );
            }
            CpuCommand::ReadAtAddress => {
                self.value = Word {
                    low: mem.read(self.operand.get())?,
                    high: 0,
                };
                log::debug!(
                    "ReadAtAddress: read ${:02x} from ${:04x}",
                    self.value.low,
                    self.operand.get()
                );
            }
            CpuCommand::WriteAtAddress => {
                mem.write(self.operand.get(), self.value.low)?;
                log::debug!(
                    "WriteAtAddress: write ${:02x} at address ${:04x}",
                    self.value.low,
                    self.operand.get()
                );
            }
            CpuCommand::ReadImmediate => {
                self.operand = Word {
                    low: mem.read(self.pc)?,
                    high: 0,
                };
                self.pc += 1;
                self.value = Word {
                    low: self.operand.low,
                    high: 0,
                };
                log::debug!("ReadImmediate: load ${:02x} from operand", self.value.low);
            }
        }

        Ok(())
    }

    fn fetch_instruction(&mut self, mem: &mut dyn Memory) -> Result<(), ExecuteError> {
        let opcode = mem.read(self.pc)?;
        self.pc += 1;
        let instruction = INSTRUCTION_CODE_TABLE[opcode as usize];
        let addr_mode = ADDRESS_MODE_TABLE[opcode as usize];

        log::debug!(
            "FetchInstruction: Loaded instruction {instruction:?} with address mode {addr_mode:?}"
        );

        match instruction {
            InstructionCode::LDA => {
                self.push_read_command(addr_mode)?;
                self.push_command(CpuCommand::WriteA);
            }
            InstructionCode::STA => {
                self.push_command(CpuCommand::ReadA);
                self.push_write_command(addr_mode)?;
            }
            InstructionCode::JMP => {
                self.push_read_command(addr_mode)?;
                self.push_command(CpuCommand::WritePc);
            }
            _ => {
                self.pc += addr_mode.size() as u16;
                self.push_command(CpuCommand::FetchInstrution);
                return Err(ExecuteError::InstructionNotImplemented(instruction));
            }
        }

        Ok(())
    }

    fn push_command(&mut self, command: CpuCommand) {
        self.command_queue.push_back(command);
    }

    fn pop_command(&mut self) -> Option<CpuCommand> {
        self.command_queue.pop_front()
    }

    fn push_read_command(&mut self, mode: AddressMode) -> Result<(), ExecuteError> {
        match mode {
            AddressMode::None => {}
            AddressMode::Acc => self.push_command(CpuCommand::ReadA),
            AddressMode::Abs => {
                self.push_command(CpuCommand::LoadAddressAbsolute);
                self.push_command(CpuCommand::ReadAtAddress);
            }
            AddressMode::AbsX => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::AbsY => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Imm => self.push_command(CpuCommand::ReadImmediate),
            AddressMode::Impl => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Ind => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::IndX => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::IndY => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Rel => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Zpg => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::ZpgX => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::ZpgY => return Err(ExecuteError::AddressModeNotImplemented(mode)),
        };

        Ok(())
    }

    fn push_write_command(&mut self, mode: AddressMode) -> Result<(), ExecuteError> {
        match mode {
            AddressMode::None => {}
            AddressMode::Acc => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Abs => {
                self.push_command(CpuCommand::LoadAddressAbsolute);
                self.push_command(CpuCommand::WriteAtAddress);
            }
            AddressMode::AbsX => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::AbsY => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Imm => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Impl => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Ind => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::IndX => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::IndY => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Rel => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::Zpg => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::ZpgX => return Err(ExecuteError::AddressModeNotImplemented(mode)),
            AddressMode::ZpgY => return Err(ExecuteError::AddressModeNotImplemented(mode)),
        }

        Ok(())
    }
}

#[derive(Clone, Debug)]
pub enum CpuCommand {
    FetchInstrution,
    ReadA,
    WriteA,
    WritePc,
    LoadAddressAbsolute,
    ReadAtAddress,
    WriteAtAddress,
    ReadImmediate,
}

#[derive(Debug)]
pub enum ExecuteError {
    ReadError,
    WriteError,
    InstructionNotImplemented(InstructionCode),
    AddressModeNotImplemented(AddressMode),
}

impl From<ReadError> for ExecuteError {
    fn from(_value: ReadError) -> Self {
        Self::ReadError
    }
}

impl From<WriteError> for ExecuteError {
    fn from(_value: WriteError) -> Self {
        Self::WriteError
    }
}

#[derive(Clone, Copy, Debug)]
pub enum InstructionCode {
    // Legal codes
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,

    // Illegal codes
    ALR,
    ANC,
    ANE,
    ARR,
    DCP,
    ISC,
    JAM,
    LAS,
    LAX,
    LXA,
    RLA,
    RRA,
    SAX,
    SBX,
    SHA,
    SHX,
    SHY,
    SLO,
    SRE,
    TAS,
    USBC,
}

#[derive(Clone, Copy, Debug)]
pub enum AddressMode {
    None,
    Acc,
    Abs,
    AbsX,
    AbsY,
    Imm,
    Impl,
    Ind,
    IndX,
    IndY,
    Rel,
    Zpg,
    ZpgX,
    ZpgY,
}

impl AddressMode {
    pub fn size(self) -> u8 {
        match self {
            Self::None | Self::Acc | Self::Impl => 0,
            Self::Imm | Self::Rel | Self::Zpg | Self::ZpgX | Self::ZpgY => 1,
            Self::Abs | Self::AbsX | Self::Ind | Self::IndX | Self::IndY | Self::AbsY => 2,
        }
    }
}

mod tables {
    use crate::AddressMode::{self, *};

    use super::InstructionCode::{self, *};

    #[rustfmt::skip]
    pub static INSTRUCTION_CODE_TABLE: [InstructionCode; 256] = [
        BRK, ORA, JAM, SLO, NOP, ORA, ASL, SLO, PHP, ORA, ASL,  ANC, NOP, ORA, ASL, SLO,
        BPL, ORA, JAM, SLO, NOP, ORA, ASL, SLO, CLC, ORA, NOP,  SLO, NOP, ORA, ASL, SLO,
        JSR, AND, JAM, RLA, BIT, AND, ROL, RLA, PLP, AND, ROL,  ANC, BIT, AND, ROL, RLA,
        BMI, AND, JAM, RLA, NOP, AND, ROL, RLA, SEC, AND, NOP,  RLA, NOP, AND, ROL, RLA,
        RTI, EOR, JAM, SRE, NOP, EOR, LSR, SRE, PHA, EOR, LSR,  ALR, JMP, EOR, LSR, SRE,
        BVC, EOR, JAM, SRE, NOP, EOR, LSR, SRE, CLI, EOR, NOP,  SRE, NOP, EOR, LSR, SRE,
        RTS, ADC, JAM, RRA, NOP, ADC, ROR, RRA, PLA, ADC, ROR,  ARR, JMP, ADC, ROR, RRA,
        BVS, ADC, JAM, RRA, NOP, ADC, ROR, RRA, SEI, ADC, NOP,  RRA, NOP, ADC, ROR, RRA,
        NOP, STA, NOP, SAX, STY, STA, STX, SAX, DEY, NOP, TXA,  ANE, STY, STA, STX, SAX,
        BCC, STA, JAM, SHA, STY, STA, STX, SAX, TYA, STA, TXS,  TAS, SHY, STA, SHX, SHA,
        LDY, LDA, LDX, LAX, LDY, LDA, LDX, LAX, TAY, LDA, TAX,  LXA, LDY, LDA, LDX, LAX,
        BCS, LDA, JAM, LAX, LDY, LDA, LDX, LAX, CLV, LDA, TSX,  LAS, LDY, LDA, LDX, LAX,
        CPY, CMP, NOP, DCP, CPY, CMP, DEC, DCP, INY, CMP, DEX,  SBX, CPY, CMP, DEC, DCP,
        BNE, CMP, JAM, DCP, NOP, CMP, DEC, DCP, CLD, CMP, NOP,  DCP, NOP, CMP, DEC, DCP,
        CPX, SBC, NOP, ISC, CPX, SBC, INC, ISC, INX, SBC, NOP, USBC, CPX, SBC, INC, ISC,
        BEQ, SBC, JAM, ISC, NOP, SBC, INC, ISC, SED, SBC, NOP,  ISC, NOP, SBC, INC, ISC,
    ];

    #[rustfmt::skip]
    pub static ADDRESS_MODE_TABLE: [AddressMode; 256] = [
        Impl, IndX, None, IndX, Zpg,  Zpg,  Zpg,  Zpg,  Impl, Imm,  Acc,  Imm,  Abs,  Abs,  Abs,  Abs,
        Rel,  IndY, None, IndY, ZpgX, ZpgX, ZpgX, ZpgX, Impl, AbsY, Impl, AbsY, AbsX, AbsX, AbsX, AbsX,
        Abs,  IndX, None, IndX, Zpg,  Zpg,  Zpg,  Zpg,  Impl, Imm,  Acc,  Imm,  Abs,  Abs,  Abs,  Abs,
        Rel,  IndY, None, IndY, ZpgX, ZpgX, ZpgX, ZpgX, Impl, AbsY, Impl, AbsY, AbsX, AbsX, AbsX, AbsX,
        Impl, IndX, None, IndX, Zpg,  Zpg,  Zpg,  Zpg,  Impl, Imm,  Acc,  Imm,  Abs,  Abs,  Abs,  Abs,
        Rel,  IndY, None, IndY, ZpgX, ZpgX, ZpgX, ZpgX, Impl, AbsY, Impl, AbsY, AbsX, AbsX, AbsX, AbsX,
        Impl, IndX, None, IndX, Zpg,  Zpg,  Zpg,  Zpg,  Impl, Imm,  Acc,  Imm,  Ind,  Abs,  Abs,  Abs,
        Rel,  IndY, None, IndY, ZpgX, ZpgX, ZpgX, ZpgX, Impl, AbsY, Impl, AbsY, AbsX, AbsX, AbsX, AbsX,
        Imm,  IndX, Imm,  IndX, Zpg,  Zpg,  Zpg,  Zpg,  Impl, Imm,  Impl, Imm,  Abs,  Abs,  Abs,  Abs,
        Rel,  IndY, None, IndY, ZpgX, ZpgX, ZpgY, ZpgY, Impl, AbsY, Impl, AbsY, AbsX, AbsX, AbsY, AbsY,
        Imm,  IndX, Imm,  IndX, Zpg,  Zpg,  Zpg,  Zpg,  Impl, Imm,  Impl, Imm,  Abs,  Abs,  Abs,  Abs,
        Rel,  IndY, None, IndY, ZpgX, ZpgX, ZpgY, ZpgY, Impl, AbsY, Impl, AbsY, AbsX, AbsX, AbsY, AbsY,
        Imm,  IndX, Imm,  IndX, Zpg,  Zpg,  Zpg,  Zpg,  Impl, Imm,  Impl, Imm,  Abs,  Abs,  Abs,  Abs,
        Rel,  IndY, None, IndY, ZpgX, ZpgX, ZpgX, ZpgX, Impl, AbsY, Impl, AbsY, AbsX, AbsX, AbsX, AbsX,
        Imm,  IndX, Imm,  IndX, Zpg,  Zpg,  Zpg,  Zpg,  Impl, Imm,  Impl, Imm,  Abs,  Abs,  Abs,  Abs,
        Rel,  IndY, None, IndY, ZpgX, ZpgX, ZpgX, ZpgX, Impl, AbsY, Impl, AbsY, AbsX, AbsX, AbsX, AbsX,
    ];
}

use std::collections::VecDeque;

pub use tables::ADDRESS_MODE_TABLE;
pub use tables::INSTRUCTION_CODE_TABLE;

pub trait AddressResolver {
    fn address(&self) -> Result<Word, ReadError> {
        Err(ReadError)
    }

    fn load(&self) -> Result<u8, ReadError> {
        Err(ReadError)
    }

    fn store(&mut self, val: u8) -> Result<(), WriteError> {
        let _ = val;
        Err(WriteError)
    }
}

pub struct NoneAddressResolver;

impl AddressResolver for NoneAddressResolver {}

pub struct AccumulatorAddressResolver<'a> {
    pub a: &'a mut u8,
}

impl<'a> AddressResolver for AccumulatorAddressResolver<'a> {
    fn load(&self) -> Result<u8, ReadError> {
        Ok(*self.a)
    }

    fn store(&mut self, val: u8) -> Result<(), WriteError> {
        *self.a = val;
        Ok(())
    }
}

pub struct AbsoluteAddressResolver<'a> {
    pub operand: Word,
    pub mem: &'a mut dyn Memory,
}

impl<'a> AddressResolver for AbsoluteAddressResolver<'a> {
    fn address(&self) -> Result<Word, ReadError> {
        Ok(self.operand)
    }

    fn load(&self) -> Result<u8, ReadError> {
        self.mem.read(self.operand.get())
    }

    fn store(&mut self, val: u8) -> Result<(), WriteError> {
        self.mem.write(self.operand.get(), val)
    }
}

// TODO:
//  AbsX,
//  AbsY,
//  Imm,
//  Impl,
//  Ind,
//  IndX,
//  IndY,
//  Rel,
//  Zpg,
//  ZpgX,
//  ZpgY,

#[derive(Clone, Copy, Default, Debug)]
pub struct Word {
    /// Least significant byte of the word
    pub low: u8,

    /// Most significant byte of the word
    pub high: u8,
}

impl Word {
    #[inline]
    pub const fn new(value: u16) -> Self {
        Self {
            low: (value & 0x00FF) as u8,
            high: (value & 0xFF00 >> 8) as u8,
        }
    }

    pub fn get(self) -> u16 {
        (self.low as u16) | ((self.high as u16) << 8)
    }

    pub fn add(self, val: u8) -> Self {
        let address = self.get();
        Self::new(address + val as u16)
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub struct StatusRegister(pub u8);

impl StatusRegister {
    pub fn new(value: u8) -> Self {
        Self(value)
    }

    pub fn get(self) -> u8 {
        self.0
    }

    /// Negative flag
    pub fn n(self) -> bool {
        self.get() & 0b_1000_0000 != 0
    }

    /// Overflow flag
    pub fn v(self) -> bool {
        self.get() & 0b_0100_0000 != 0
    }

    /// Ignored flag
    pub fn ignored(self) -> bool {
        self.get() & 0b_0010_0000 != 0
    }

    /// Break flag
    pub fn b(self) -> bool {
        self.get() & 0b_0001_0000 != 0
    }

    /// Decimal flag
    pub fn d(self) -> bool {
        self.get() & 0b_0000_1000 != 0
    }

    // Interrupt flag (IRQ disable)
    pub fn i(self) -> bool {
        self.get() & 0b_0000_0100 != 0
    }

    // Zero flag
    pub fn z(self) -> bool {
        self.get() & 0b_0000_0010 != 0
    }

    // Carry flag
    pub fn c(self) -> bool {
        self.get() & 0b_0000_0001 != 0
    }
}
