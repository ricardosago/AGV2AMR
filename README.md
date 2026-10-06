# AGV2AMR

AGV2AMR is a hardware-software project focused on the evolution of an Automated Guided Vehicle (AGV) into an Autonomous Mobile Robot (AMR).

The repository brings together embedded firmware, control algorithms, communication interfaces, sensor integration, and high-level software developed throughout the project.

## Project Structure

```text
AGV2AMR/
├── AGV/
│   └── codigos/
│       ├── MatLab/     # Previous modeling, analysis, and control code
│       ├── Python/     # Python tools developed for the original AGV
│       └── STM32/      # Embedded firmware developed for the original AGV
│
├── Arduino/            # Arduino-based development and hardware testing
├── STM32/              # Current STM32 firmware and embedded control
├── README.md
└── .gitignore
