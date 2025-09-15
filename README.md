# Custom-1-Channel-EEG (PCB + DSP)

A custom single-channel EEG device developed from PCB design to real-time brainwave visualization.  
Features a low-noise analog front-end with filtering, STM32 DSP (biquads, Welch’s method), and verified alpha detection (~6 dB SNR, ~5× alpha power).

---

## Demo
▶️ Watch the demo video here: [YouTube Link](https://youtu.be/_DtyJymNVPs)  

**Report (PDF):** [Custom 1-Channel EEG Report](docs/Custom-1-Channel-EEG-Device-From-PCB-to-Brainwaves.pdf)

---

## Repository Layout
- `hardware/` – Gerber files, drill files, BOM, LTspice simulations
- `firmware/` – STM32 (CMSIS-DSP) source code and project files
- `docs/` – report, schematics, images

---

## Quick Start
1. Build firmware with STM32CubeIDE (or compatible toolchain).  
2. Order PCB using Gerber files from `/hardware/`.  
3. Source components using the BOM in `/hardware/`.  
4. Connect electrodes: **O2** (active), **mastoid** (reference), **Fz/forehead** (DRL).  
5. Stream band powers over USART and visualize with a serial plotter.  

---

## License
- **Code**: MIT License  
- **Hardware**: CERN OHL-W License (see LICENSE files)  
