# Electrical CAD Library

This folder is the import library for the 2026 electrical-board layout in Fusion 360.  All downloaded models are STEP (`.step`) files because no manufacturer-provided Fusion 360 (`.f3d`) files were available for these parts. Fusion imports these files directly; save imported components into a Fusion design as needed.

## Folder layout

```text
cad/electrical/
├── 01_control-system/     # Required FRC control-system components
├── 02_motor-controllers/  # Controllers and controller/motor assemblies
├── 03_power/              # Optional/current FRC power hardware
└── README.md
```

## Downloaded models

| Model | File | Why it is here | Primary source |
|---|---|---|---|
| NI roboRIO 2.0 | `01_control-system/NI_roboRIO_2.0.step` | Standard FRC controller | AndyMark `am-3000a` CAD download |
| REV Power Distribution Hub, REV-11-1850 | `01_control-system/REV_PDH_REV-11-1850.step` | Standard FRC distribution | REV product CAD download |
| Vivid Hosting VH-109 radio | `01_control-system/Vivid_Hosting_VH-109.step` | 2026 FRC radio | Vivid Hosting CAD download |
| navX2-MXP, Studica 71042 | `01_control-system/71042-navX2-mxp.stp` | Connected through roboRIO MXP SPI by this project | Existing source model in the working tree; verify against the physical unit before drilling |
| Robot Signal Light, 855PB | `01_control-system/Robot_Signal_Light_855PB.step` | Required FRC signal light | AndyMark `am-3583` CAD download |
| REV SPARK MAX, REV-11-2158 | `02_motor-controllers/REV_SPARK_MAX_REV-11-2158.step` | Used by the codebase | REV product CAD download |
| REV NEO Vortex + SPARK Flex, 8 mm shaft | `02_motor-controllers/REV_NEO_Vortex_and_SPARK_Flex_8mm_shaft.step` | Used by the codebase where a Flex is docked to a Vortex | REV product CAD download |
| REV Radio Power Module, REV-11-1856 | `03_power/REV_Radio_Power_Module_REV-11-1856.step` | Legal/common FRC radio-power option; only use if installed on this robot | REV product CAD download |
| MK ES17-12 12 V battery | `03_power/MK_ES17-12_12V_battery.step` | Standard FRC battery envelope | Public FRC team CAD repository (`zenmanenergy/FRC9214`) |

## Project-specific evidence

The code identifies the following electrical hardware and quantities, suitable for starting a board layout:

| Hardware | Code evidence | Layout quantity |
|---|---|---:|
| SPARK MAX | Four swerve-turn controllers; intake roller; climber; shooter feeder | 7 |
| SPARK Flex | Four swerve-drive controllers; intake extension; two shooter flywheels | 7 |
| navX2-MXP | `DriveConstants` says it is connected through MXP SPI; local STEP model present | 1 |
| navX3-CAN | `DriveConstants` says it is connected on CAN (ID 0) | 1 |
| PhotonVision cameras | Two enabled camera names, but no physical camera/coprocessor SKU | 2 |
| Climber proximity switch | DIO 1, but no manufacturer/part number | 1 |

The combined Vortex/Flex model accurately represents a docked Flex/Vortex package. Confirm before using it for every Flex: the source code identifies Flex controllers but does not prove the mechanical mounting arrangement or motor SKU for every one.

## Deliberately unresolved items

These components need a part number or a physically confirmed installation before a model can be added without creating a misleading electrical layout:

| Component | Current status | What is needed |
|---|---|---|
| 120 A main breaker (Eaton/Cooper Bussmann 185120F) | Required FRC hardware; public STEP source was not available without a third-party account | Use the actual breaker part number/physical unit for a verified model |
| Anderson SB50 | Required battery quick-disconnect; manufacturer locks CAD behind login | Provide the exact connector or download its vendor model manually |
| navX3-CAN | Studica advertises a STEP model, but its CAD download site is access-blocked from this environment | Download `71043` from Studica once access is available |
| Cameras, coprocessor, proximity sensor | Code/docs are intentionally generic and do not state a SKU | Provide exact make/model and mounting configuration |
| Individual PDH breakers, fuses, WAGO terminals, wire and cable | Included in or routed to the PDH and not normally placed as separate board CAD components | Add only if the electrical CAD needs harness-level clearance checks |

## Sources and file integrity

Files were downloaded from the manufacturers' published CAD links and validated as ISO-10303 STEP data. The following SHA-256 checksums make it easy to detect local corruption:

```text
NI_roboRIO_2.0.step                                  6e19ed3e825c8c521f57ec510eda860556ac422f134dc584b8342ca4d94be791
71042-navX2-mxp.stp                                  e401e0fa5945da2a238af8cb065dd163ae65264a5810a517f85d168888ccd58b
REV_PDH_REV-11-1850.step                             bb14d12b33e8b21bb49f85166f9fb55e358d3044f85580321a291e3a7b9a1fa9
Vivid_Hosting_VH-109.step                            6abe89119794aecd6b1bc33c1f67fedadb808f791462987719902850999ce97e
Robot_Signal_Light_855PB.step                        6dc6d4c850e6cf5798d3b2082ffc15527ad5cdd0869b641439cf62905ee074ca
REV_SPARK_MAX_REV-11-2158.step                       b8b374fc3dbb3bfa07ed10c9bdbc2ee407591a35bbe42035ddda87a0dcfa7e4d
REV_NEO_Vortex_and_SPARK_Flex_8mm_shaft.step         f8a9c317ab5ecb0e807af9b0e4f5066063baf8b05ff7381e07260f9429aa180b
REV_Radio_Power_Module_REV-11-1856.step              5d8b07cea09e5a83b5250f22ba40d414bd9571cdd738f01a022e419a2af01e73
MK_ES17-12_12V_battery.step                          38e988f0ef6f469e6e65271c9ddda1b4d742f47606885565f098c630ae4bde80
```

## Fusion 360 use

1. In the target Fusion design, choose **Insert > Insert CAD** and select the required STEP file.
2. Create one component per physical controller; do not copy a single component body if you need a useful bill of materials.
3. Start with the PDH, roboRIO, radio, RSL, and battery envelope; then place the fourteen motor controllers based on wire-routing and service access.
4. Verify clearances against the physical hardware before drilling. CAD does not prove connector reach, bend radius, or breaker/fuse access.
