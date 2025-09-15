# Custom-1-Channel-EEG(PCB+DSP)
A custom single-channel EEG device from PCB design to real-time brainwave visualization. Features analog front-end filtering, STM32 DSP (biquads, Welch’s), and verified alpha detection (~6 dB SNR, 5× alpha power). Demo: https://youtu.be/_DtyJymNVPs

**Report (PDF):** docs/Custom-1-Channel-EEG-Report.pdf

## Repo layout
- `hardware/` – KiCad, LTspice
- `firmware/` – STM32 (CMSIS-DSP)
- `matlab/` – design/validation scripts
- `results/` – plots, screenshots
- `docs/` – report, images

## Quick start
- Build firmware with STM32CubeIDE (or your toolchain).
- Place electrodes: O2(active), mastoid(ref), Fz/forehead(DRL).
- Stream band powers over USART; visualize with serial plotter.

## License
Code: MIT. Hardware: CERN OHL-W (see LICENSE files).
