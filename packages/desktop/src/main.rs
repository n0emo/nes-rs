#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::{sync::Arc, thread, time::Duration};

use eframe::{
    egui::{self, mutex::RwLock, Color32, Rect, Sense, Vec2}, run_native, App, CreationContext, NativeOptions
};
use nes_console::{Cpu, Memory, ReadError, WriteError};

const PALETTE: [Color32; 24] = [
    Color32::BLACK,
    Color32::DARK_GRAY,
    Color32::GRAY,
    Color32::LIGHT_GRAY,
    Color32::WHITE,
    Color32::BROWN,
    Color32::DARK_RED,
    Color32::RED,
    Color32::LIGHT_RED,
    Color32::CYAN,
    Color32::MAGENTA,
    Color32::YELLOW,
    Color32::ORANGE,
    Color32::LIGHT_YELLOW,
    Color32::KHAKI,
    Color32::DARK_GREEN,
    Color32::GREEN,
    Color32::LIGHT_GREEN,
    Color32::DARK_BLUE,
    Color32::BLUE,
    Color32::LIGHT_BLUE,
    Color32::PURPLE,
    Color32::GOLD,
    Color32::PLACEHOLDER,
];

const RAM_ADDRESS: u16 = 0x0000;
const RAM_END: u16 = 0x3999;
const SCREEN_ADDRESS: u16 = 0x4000;
const SCREEN_END: u16 = 0x4999;
const KEY_ADDRESS: u16 = 0x5000;
const RANDOM_ADDRESS: u16 = 0x5001;
const ROM_ADDRESS: u16 = 0x8000;
const ROM_END: u16 = 0xffff;

fn main() -> eframe::Result {
    env_logger::builder().init();
    let options = NativeOptions::default();
    run_native(
        "NES Emulator",
        options,
        Box::new(|cc| Ok(Box::new(NesApp::new(cc)))),
    )
}

struct NesApp {
    cpu: Arc<RwLock<Cpu>>,
    mem: Arc<RwLock<SimpleMem>>,
    _join: thread::JoinHandle<()>,
}

struct SimpleMem {
    ram: Box<[u8]>,
    rom: Box<[u8]>,
    screen: Box<[u8]>,
    last_key_register: u8,
    random_register: u8,
    ctx: egui::Context,
}

impl SimpleMem {
    pub fn new(rom: Vec<u8>, ctx: egui::Context) -> Self {
        assert_eq!(rom.len(), 2usize.pow(15), "Rom must be exactly 32KB size");
        Self {
            ram: vec![0; 2usize.pow(14)].into_boxed_slice(),
            rom: rom.into_boxed_slice(),
            screen: vec![0; 2usize.pow(12)].into_boxed_slice(),
            last_key_register: 0,
            random_register: 0,
            ctx,
        }
    }
}

impl Memory for SimpleMem {
    fn read(&self, address: u16) -> Result<u8, ReadError> {
        match address {
            RAM_ADDRESS..=RAM_END => Ok(self.ram[(address - RAM_ADDRESS) as usize]),
            ROM_ADDRESS..=ROM_END => Ok(self.rom[(address - ROM_ADDRESS) as usize]),
            SCREEN_ADDRESS..=SCREEN_END => Ok(self.screen[(address - SCREEN_ADDRESS) as usize]),
            KEY_ADDRESS => Ok(self.last_key_register),
            RANDOM_ADDRESS => Ok(self.random_register),
            _ => Err(ReadError),
        }
    }

    fn write(&mut self, address: u16, value: u8) -> Result<(), WriteError> {
        match address {
            RAM_ADDRESS..=RAM_END => {
                self.ram[(address - RAM_ADDRESS) as usize] = value;
                Ok(())
            }
            SCREEN_ADDRESS..SCREEN_END => {
                self.screen[(address - SCREEN_ADDRESS) as usize] = value;
                self.ctx.request_repaint();
                Ok(())
            }
            _ => Err(WriteError),
        }
    }
}

impl NesApp {
    pub fn new(cc: &CreationContext) -> Self {
        let rom = std::env::args()
            .nth(1)
            .expect("You must provide path to rom");
        let rom = std::fs::read(rom).expect("Error reading rom");
        let mem = Arc::new(RwLock::new(SimpleMem::new(rom, cc.egui_ctx.clone())));
        let cpu = Arc::new(RwLock::new(Cpu::default()));
        cpu.write().init(&*mem.read());

        let cpu_copy = cpu.clone();
        let mem_copy = mem.clone();
        let join = thread::spawn(move || {
            let cpu = cpu_copy;
            let mem = mem_copy;
            loop {
                if let Err(e) = cpu.write().cycle(&mut *mem.write()) {
                    log::error!("Error executing instructio: {e:?}");
                }

                thread::sleep(Duration::from_micros(1));
            }
        });

        Self { cpu, mem, _join: join }
    }
}

impl App for NesApp {
    fn update(&mut self, ctx: &eframe::egui::Context, _frame: &mut eframe::Frame) {

        egui::Window::new("Screen").resizable(true).show(ctx, |ui| {
            let space = ui.available_size();
            let (res, painter) = ui.allocate_painter(space, Sense::empty());
            let size = f32::min(res.rect.size().x, res.rect.size().y) / 64.0;
            let mut x = 0.0;
            let mut y = 0.0;
            for byte in &self.mem.read().screen {
                let color = PALETTE.get(*byte as usize).cloned().unwrap_or_default();
                let pos = res.rect.left_top() + Vec2::new(x * size, y * size);
                let rect = Rect {
                    min: pos,
                    max: pos + Vec2::new(size, size),
                };
                painter.rect_filled(rect, 0, color);

                x += 1.0;
                if x >= 64.0 {
                    x = 0.0;
                    y += 1.0;
                }
            }
        });

        egui::Window::new("Cpu state").show(ctx, |ui| {
            let Cpu {
                pc,
                a,
                x,
                y,
                sp,
                sr,
                ..
            } = self.cpu.read().clone();
            ui.heading("6502 registers");
            ui.monospace(format!("A=${a:02x} X=${x:02x} Y=${y:02x}"));
            ui.monospace(format!("SP=${sp:02x} PC=${pc:02x}"));
            ui.monospace(format!("NV-BDIZC"));
            ui.monospace(format!("{:08b}", sr.get()));
        });
    }
}
