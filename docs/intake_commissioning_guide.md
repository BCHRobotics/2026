# Ball Intake Commissioning Guide

This guide is for  commissioning the ball intake subsystem on the robot.

The intake has two separate functions:

1. The extend mechanism moves the intake in and out.
2. The roller mechanism pulls game pieces in.

The current code does three important things:

1. It calibrates the extend mechanism at the start of autonomous by retracting until it sees a current spike.
2. It toggles the extend mechanism with the PS5 cross button.
3. It toggles the rollers with the PS5 circle button.

## What The Code Currently Does

### Controls

On a PS5 driver controller:

- Cross: toggle intake extension between home and the configured extended position.
- Circle: toggle intake rollers on and off.
- Triangle: zero drivetrain heading.
- Square: vision face-point command.

On an Xbox driver controller, the equivalent intake buttons are:

- A: toggle intake extension.
- B: toggle intake rollers.

### Extend Calibration Logic

When calibration runs, the robot:

1. Drives the extend motor in the retract direction.
2. Watches extend motor current.
3. Declares success when current reaches the configured threshold.
4. Sets the current encoder position to zero.
5. Uses that zero as the retracted home position.

If the current spike is not detected before timeout, calibration fails.

### Extend Position Logic

After calibration, the extend mechanism uses motor encoder position control.

- Retracted target: `0.0`
- Extended target: `25.0`
- Position tolerance: `1.0`

These numbers are in the current configured encoder units. Right now the conversion factor is `1.0`, so this is effectively raw motor rotations unless updated later.

## Important Constants To Know

These live in `Constants.BallIntakeConstants`.

| Constant | Current Value | Meaning |
|---|---:|---|
| `kExtendMotorCanId` | 4 | Extend motor controller CAN ID |
| `kRunMotorCanId` | 6 | Roller motor controller CAN ID |
| `kExtendCurrentLimit` | 40 | Extend motor current limit |
| `kRunCurrentLimit` | 40 | Roller motor current limit |
| `kExtendedPosition` | 25.0 | Target position for intake extension |
| `kRetractedPosition` | 0.0 | Home position after calibration |
| `kExtendPositionTolerance` | 1.0 | Allowed error for in-position check |
| `kExtendPositionP` | 0.08 | Extend position proportional gain |
| `kCalibrateSpeed` | -0.2 | Retract speed during homing |
| `kCalibrateCurrentThreshold` | 15.0 A | Current spike threshold for home detect |
| `kCalibrateTimeoutSeconds` | 3.0 s | Max time allowed for homing |
| `kRunSpeed` | 0.1 | Roller intake speed |

## CAN ID Reference

This table lists the CAN IDs currently defined in code.

| CAN ID | Device | Motor Controller | Subsystem |
|---:|---|---|---|
| 1 | Shooter feeder motor | SPARK MAX | Shooter |
| 2 | Shooter motor 2 | SPARK Flex | Shooter |
| 3 | Shooter motor 1 | SPARK Flex | Shooter |
| 4 | Ball intake extend motor | SPARK Flex | Ball Intake |
| 6 | Ball intake roller motor | SPARK MAX | Ball Intake |
| 10 | Rear left drive motor | SPARK Flex | Drivetrain |
| 11 | Rear left turning motor | SPARK MAX | Drivetrain |
| 12 | Front left drive motor | SPARK Flex | Drivetrain |
| 13 | Front left turning motor | SPARK MAX | Drivetrain |
| 14 | Front right drive motor | SPARK Flex | Drivetrain |
| 15 | Front right turning motor | SPARK MAX | Drivetrain |
| 16 | Rear right drive motor | SPARK Flex | Drivetrain |
| 17 | Rear right turning motor | SPARK MAX | Drivetrain |

IDs not listed above are not currently assigned in `Constants.java`.

## SmartDashboard Values To Watch

During commissioning, watch these values:

| Dashboard Key | What It Means |
|---|---|
| `BallIntake/CalibrateState` | `IDLE`, `CALIBRATING`, `CALIBRATED`, or `FAILED` |
| `BallIntake/ExtendCurrent` | Current draw of the extend motor |
| `BallIntake/ExtendPosition` | Current encoder position |
| `BallIntake/TargetExtendPosition` | Commanded position target |
| `BallIntake/ExtendEnabled` | Whether the extend toggle is currently on |
| `BallIntake/RunEnabled` | Whether the roller toggle is currently on |
| `BallIntake/AtTarget` | Whether extend position is within tolerance |
| `BallIntake/RunCurrent` | Current draw of the roller motor |

## Safety Before You Start

Do not skip this section.

1. Put the robot on blocks or otherwise make sure the intake cannot hit the floor, frame, wiring, or students.
2. Remove game pieces before the first tests.
3. Verify the mechanism can fully retract into its hard stop without damaging anything.
4. Verify wiring polarity, CAN IDs, and motor/controller assignment before enabling.
5. Keep one student on the Driver Station and one student watching the mechanism.
6. Be ready to disable the robot immediately if the intake moves in the wrong direction or binds.

## Pre-Commissioning Mechanical Checks

Before enabling the robot:

1. Confirm the extend motor really retracts toward the physical home stop when commanded negative.
2. Confirm the home stop is solid and repeatable.
3. Confirm the mechanism has enough compliance that brief contact at homing will not cause damage.
4. Confirm the roller spins in the desired intake direction when commanded positive.
5. Check chain, belt, gears, and fasteners.
6. Verify the intake can reach the intended extended position without mechanical interference.

## Commissioning Procedure

### Phase 1: Power-On Inspection

1. Deploy code.
2. Open SmartDashboard or Shuffleboard.
3. Add the BallIntake values listed above.
4. Enable in Disabled mode only long enough to confirm values are updating.

Expected result:

- `BallIntake/CalibrateState` should begin in `IDLE`.
- `BallIntake/RunEnabled` should be `false`.
- `BallIntake/ExtendEnabled` should be `false`.

### Phase 2: Verify Calibration Behavior

The current code starts intake calibration at the beginning of autonomous command execution.

1. Put the robot in a safe test setup where the intake can retract to its hard stop.
2. Select any autonomous routine.
3. Enable autonomous.
4. Watch the mechanism and the BallIntake dashboard values.

Expected result for a successful calibration:

1. Intake retracts slowly.
2. Extend motor current rises when the mechanism reaches the hard stop.
3. `BallIntake/CalibrateState` changes to `CALIBRATED`.
4. `BallIntake/ExtendPosition` resets to about `0.0`.
5. The selected autonomous command continues only after calibration succeeds.

Expected result for a failed calibration:

1. Intake retracts for at most about `3.0` seconds.
2. `BallIntake/CalibrateState` changes to `FAILED`.
3. Autonomous is skipped.

If the mechanism drives away from the home stop instead of toward it, disable immediately. That means the extend direction or inversion is wrong for the homing logic.

### Phase 3: Verify Roller Toggle

1. Enter teleop.
2. Press circle once.
3. Observe roller motion.
4. Press circle again.

Expected result:

1. First press turns rollers on.
2. `BallIntake/RunEnabled` becomes `true`.
3. `BallIntake/RunCurrent` rises above idle.
4. Second press turns rollers off.
5. `BallIntake/RunEnabled` becomes `false`.

If the roller spins the wrong direction, fix `kRunMotorInverted` or `kRunSpeed` sign.

### Phase 4: Verify Extend Toggle

Only do this after successful calibration.

1. Enter teleop after a successful auto calibration test, or otherwise ensure the subsystem has been calibrated in the current boot session.
2. Press cross once.
3. Watch the intake extend.
4. Press cross again.
5. Watch the intake retract to home.

Expected result:

1. First press commands the intake to the extended target.
2. `BallIntake/ExtendEnabled` becomes `true`.
3. `BallIntake/TargetExtendPosition` becomes `25.0`.
4. `BallIntake/AtTarget` becomes `true` once motion settles.
5. Second press commands the intake back to `0.0`.
6. `BallIntake/ExtendEnabled` becomes `false`.

If cross does nothing, the subsystem is probably not calibrated. The current code ignores extend-position requests until calibration succeeds.

## Tuning Instructions

### 1. Tune Home Detection Current

Goal: detect the hard stop reliably without waiting too long or slamming hard.

Procedure:

1. Run autonomous calibration several times.
2. Record the normal retract current before contact.
3. Record the current spike when the hard stop is reached.
4. Set `kCalibrateCurrentThreshold` above normal running current but below damaging stall current.

Signs the threshold is too low:

- Calibration finishes early.
- Zero point is inconsistent.
- Intake does not fully retract before being declared calibrated.

Signs the threshold is too high:

- Calibration times out.
- Mechanism pushes too hard into the stop.
- Current stays high too long.

### 2. Tune Extended Position

Goal: reach the real deployed intake position.

Procedure:

1. Start from a successful calibration.
2. Press cross to extend.
3. Observe the actual physical position.
4. Increase `kExtendedPosition` if it does not extend far enough.
5. Decrease `kExtendedPosition` if it over-extends or binds.
6. Repeat until the intake lands at the desired deployed position.

Important note:

The code currently uses `kExtendPositionConversionFactor = 1.0`. That means `kExtendedPosition` is probably not yet in inches, degrees, or meters. If you want physically meaningful tuning, replace the conversion factor with one based on your gearbox and mechanism geometry.

### 3. Tune Position PID

Goal: move to target cleanly without oscillation.

Procedure:

1. Start with only `P` gain.
2. Increase `kExtendPositionP` until the intake reaches target firmly.
3. If it oscillates, reduce `P`.
4. Add a small `D` only if needed to reduce overshoot.
5. Leave `I` at zero unless you have a demonstrated steady-state error that cannot be fixed mechanically or with `P`.

Signs `P` is too low:

- Slow response.
- Does not quite reach target.

Signs `P` is too high:

- Buzzing or hunting around the setpoint.
- Visible overshoot and repeated correction.

## Common Failure Cases

### Intake moves the wrong way during calibration

Likely causes:

- Extend motor inversion is wrong.
- Retract speed sign is wrong for the mechanism.
- Motor wiring or gearbox direction assumption is reversed.

### Calibration always fails

Likely causes:

- Current threshold too high.
- No real hard stop contact.
- Mechanism binds before reaching the intended home stop.
- Current limiting is too aggressive.

### Intake calibrates but extend target is wrong every time

Likely causes:

- Hard stop is not repeatable.
- Encoder slips or resets unexpectedly.
- `kExtendedPosition` is not tuned.
- Conversion factor does not match the mechanism.

### Cross button does nothing in teleop

Likely causes:

- The subsystem has not calibrated successfully in this boot session.
- Autonomous calibration was skipped or failed.

### Rollers run backward

Likely causes:

- Roller inversion is wrong.
- Intake speed sign is opposite of the mechanism direction.

## Student Sign-Off Checklist

Mark each item when verified.

- [ ] Extend motor retracts toward the hard stop during calibration.
- [ ] Calibration succeeds repeatedly without timing out.
- [ ] Extend encoder resets to about `0.0` at home.
- [ ] Cross toggles intake extension out and back.
- [ ] Circle toggles rollers on and off.
- [ ] Roller direction is correct for intake.
- [ ] Intake reaches the intended deployed position.
- [ ] Intake returns cleanly to home.
- [ ] No rubbing, binding, or hard impacts observed.
- [ ] Final constants recorded in code after testing.

## Recommended Next Improvement

The current implementation calibrates the intake at autonomous start. For pit testing and teleop bring-up, it would be more practical to also add a manual calibration button or run calibration during robot startup or disabled init. That would let students verify the intake without entering autonomous each time.