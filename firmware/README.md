# firmware

Onboard software for the biomimetic remora robot. Runs on an STM32H743VITx under
FreeRTOS and covers the whole control stack: behaviour selection, motion planning,
low-level servo control, sensor acquisition and the learned neck controller.

## Target

| | |
|---|---|
| MCU | STM32H743VITx |
| RTOS | FreeRTOS (CMSIS-RTOS v2) |
| Toolchain | GNU Arm Embedded |
| CubeMX project | `remorarl43.ioc` |
| X-CUBE-AI | 9.0.0-19802 |

## Structure

```
Core/Src, Core/Inc         application code
X-CUBE-AI/App/             network code generated from models/attachment.tflite
.ai/                       X-CUBE-AI intermediate description of the generated network
CMakeLists.txt             build description
STM32H743VITX_*.ld         linker scripts
remorarl43.ioc             STM32CubeMX project
```

The application code is organised as a control hierarchy:

```
fish_controller.c        behaviour layer — swimming, attachment, hitchhiking,
                         detachment, sliding
motion_planner.c         motion planning; converts speed and attitude commands into
                         actuator setpoints
cpg_controller.c         central pattern generators driving the pectoral fins and tail
speed_convert_layer_v2.c maps speed commands onto CPG amplitudes and frequencies
motor.c, pid.c           servo actuation and closed-loop position control
IMU.c                    head and trunk orientation feedback
atk_ms53l1m*.c           time-of-flight ranging modules on the head
p8864_sensor.c           ranging-array acquisition
p8864_plane_solver.c,    surface plane estimation from the ranging array, with
karman_filter_1dim.c     filtering
easyMatrix.c             small dense linear algebra used by the plane solver
para_service.c           runtime parameter service
wireless_model.c         radio command interface
log.c                    serial logging
network_app.c            learned neck controller (see below)
```

`Core/Src/freertos.c` creates the RTOS tasks and shows how they interact.

## Neck controller

During attachment the neck pitch angle is set by a learned controller instead of a
fixed law. The network is a Keras MLP (6 inputs: disc pitch angle, head height and
their first and second time derivatives; 256–128–1) exported to TensorFlow Lite and
converted to C by X-CUBE-AI. `network_app.c` wraps the generated interface: it
normalises the sensor inputs, runs inference and returns the neck pitch angle. The
control task in `Core/Src/freertos.c` acquires the ranging data, differentiates it and
gates the network output to the neck servo.

Other neck motions and the adhesive disc are driven by calibrated actuation rather than
by the network.

To regenerate the network code from the model:

```
stedgeai generate --target stm32h7 --name remora_nn_6t1 \
  -m models/attachment.tflite --compression none \
  --allocate-inputs --allocate-outputs
```

## Build

`CMakeLists.txt` is generated from the STM32CubeMX template and selects the
`arm-none-eabi-*` toolchain directly, so a plain CMake configure works once the
dependencies below are in place:

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

This produces `build/remorarl43.elf`, `.hex` and `.bin`.

## Dependencies not vendored here

STM32 HAL/CMSIS, FreeRTOS and the X-CUBE-AI runtime are not redistributed here. Open
`remorarl43.ioc` in STM32CubeMX with the X-CUBE-AI expansion package to regenerate them.

The plane solver in `p8864_plane_solver.c` uses the small dense linear-algebra library in
`Libs/Lapack` (`svd`, `inv`, `mul` and the `declareFunctions.h` umbrella header), which is
also not redistributed here. `CMakeLists.txt` puts `Libs/Lapack/src/LinearAlgebra`,
`Libs/Lapack/src/Lapack/Include` and the `qpOASES` headers on the include path; without
them the firmware does not compile.
