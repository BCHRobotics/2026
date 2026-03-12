# Climber Flow Diagram

This document describes what happens in code when the driver presses the PS5 `L1` button.

## Trigger

In `RobotContainer.configureBindings()`, the PS5 `L1` button is assigned to `climbTrigger`.

`climbTrigger.onTrue(new ClimbCommand(robotDrive, climber, this::getSelectedClimbStartPose));`

That means a `ClimbCommand` is scheduled once when the button changes from not-pressed to pressed.

## Does The Button Need To Be Held?

No.

Because the binding uses `onTrue(...)` instead of `whileTrue(...)`, the command starts on the press event and then continues running on its own until:

- it reaches the `FINISHED` phase, or
- it is interrupted by some other command that requires the `Drivetrain` or `Climber` subsystem.

Holding the button does not make the command continue any differently. One press is enough to start the full sequence.

## Selected Start Pose

The command gets its start pose from the SmartDashboard chooser:

- `getSelectedClimbStartPose()` returns the currently selected climb pose.
- If nothing is selected, it defaults to `ClimbConstants.kBlueLeftStartPose`.

## Sequence Overview

When the command is scheduled, it runs through these phases:

1. `ALIGNING_TO_START`
2. `EXTENDING_AT_START`
3. `APPROACHING_CLIMB`
4. `CLIMBING`
5. `FINISHED`

## Detailed Flow

### 1. Command initialization

When `ClimbCommand.initialize()` runs:

- it reads the selected climb start pose from the chooser
- it reads the robot's current pose from `drivetrain.getPose()`
- it computes `targetPose` by moving forward from `startPose` by `ClimbConstants.kDriveForwardDistanceMeters`
- it sets the phase to `ALIGNING_TO_START`
- it stops the climber motor
- it resets the PID controllers
- it sets the drive PID targets to `startPose`
- it publishes status to SmartDashboard

### 2. ALIGNING_TO_START

In `execute()`, if the phase is `ALIGNING_TO_START`:

- the robot drives toward `startPose` using `driveToPose(startPose)`
- that helper uses the current fused drivetrain pose and PID controllers to generate chassis speeds
- the phase does not change until `isAtPose(currentPose, startPose)` is true

When the robot reaches `startPose`:

- the drivetrain is stopped
- if the climber is already extended, the command skips to `APPROACHING_CLIMB`
- otherwise it goes to `EXTENDING_AT_START`

### 3. EXTENDING_AT_START

If the phase is `EXTENDING_AT_START`:

- the drivetrain stays stopped
- the climber calls `extendClimber()` until `isExtendLimitReached()` becomes true

`isExtendLimitReached()` is true when either:

- the forward limit switch is pressed, or
- the climber current spike logic says the extend stop has been reached

Once extended:

- the climber motor is stopped
- the phase changes to `APPROACHING_CLIMB`
- the drive PID target is changed to `targetPose`

### 4. APPROACHING_CLIMB

If the phase is `APPROACHING_CLIMB`:

- the robot drives toward `targetPose` using `driveToPose(targetPose)`
- this continues until `isAtPose(currentPose, targetPose)` is true

When `targetPose` is reached:

- the drivetrain is stopped
- the phase changes to `CLIMBING`

### 5. CLIMBING

If the phase is `CLIMBING`:

- the drivetrain stays stopped
- the climber calls `retractClimber()` until `isRetractLimitReached()` becomes true

`isRetractLimitReached()` is true when either:

- the reverse limit switch is pressed, or
- the climber current spike logic says the retract stop has been reached

Once retracted:

- the climber motor is stopped
- the phase changes to `FINISHED`

### 6. Command end

When the command ends, `end(boolean interrupted)`:

- stops the drivetrain
- stops the climber motor
- sets `ClimbCommand/Running` to false on SmartDashboard
- records whether the command was completed or interrupted

## Flow Diagram

```mermaid
flowchart TD
    A[Driver presses PS5 L1] --> B[onTrue schedules ClimbCommand]
    B --> C[initialize]
    C --> D[Read selected startPose]
    D --> E[Read current drivetrain pose]
    E --> F[Compute targetPose from startPose]
    F --> G[Set phase to ALIGNING_TO_START]

    G --> H{At startPose?}
    H -- No --> I[Drive to startPose]
    I --> H
    H -- Yes --> J{Already extended?}

    J -- No --> K[EXTENDING_AT_START]
    K --> L{Extend limit reached?}
    L -- No --> M[Extend climber]
    M --> L
    L -- Yes --> N[Stop climber]
    N --> O[Set target to targetPose]

    J -- Yes --> O
    O --> P[APPROACHING_CLIMB]
    P --> Q{At targetPose?}
    Q -- No --> R[Drive to targetPose]
    R --> Q
    Q -- Yes --> S[CLIMBING]

    S --> T{Retract limit reached?}
    T -- No --> U[Retract climber]
    U --> T
    T -- Yes --> V[Stop climber]
    V --> W[FINISHED]
```

## Important Behavioral Note

The sequence is not tied to the button being held.

The important distinction is:

- `onTrue(...)` means start once on press
- `whileTrue(...)` would mean run only while held

This code uses `onTrue(...)`, so one press starts the full climb sequence.