# Sim Rotation Not Reflected in AdvantageKit Pose

## Symptom
In simulation, translating with the left joystick works correctly (pose X/Y updates in AdvantageScope), but rotating with the right joystick does not update the robot's heading in the logged pose.

## Root Cause: `toChassisSpeeds` can't recover omega from desired states

`simulationPeriodic()` recovers `omegaRadiansPerSecond` by calling:
```java
var chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates);
simYaw += chassisSpeeds.omegaRadiansPerSecond * 0.02;
```

`toChassisSpeeds()` inverts the swerve kinematics math: it works backwards from each module's **speed + angle** to reconstruct chassis vx/vy/omega. This only gives a correct omega if the module *angles* are already pointing in the tangential (rotational) direction (roughly ±45° for a square chassis).

In simulation, the turning motors don't actually reach their setpoints — `simulationUpdate()` in `MAXSwerveModule` does move the sim encoder toward the desired angle, but there is significant lag. So even when a pure rotation is commanded:

- `desiredStates` has the correct tangential angles
- The **actual encoder positions** (used by `getState()`, not `getDesiredState()`) lag behind
- However, `simulationPeriodic()` uses `getDesiredState()` not `getState()` — so the angles *should* be correct

**The real problem** is that `toChassisSpeeds()` uses the module **speed** component of each `SwerveModuleState`. For a pure rotation, `desaturateWheelSpeeds()` in `setModuleStates()` scales down speeds if any exceed `maxSpeedNormal`. More critically, `SwerveModuleState.optimize()` inside `setDesiredState()` may **flip the speed to negative** and rotate the angle by 180° to avoid spinning the turning motor more than 90°. The `m_desiredState` stored is the **pre-optimization** state (see `MAXSwerveModule.java` line 116 — `m_desiredState = desiredState` is set *before* optimization is applied to `correctedDesiredState`). This means the angle passed to `toChassisSpeeds()` is the raw commanded angle, not the optimized one actually sent to the motor — causing a mismatch that can produce a near-zero or wrong omega.

## Secondary Issue: Execution Order
WPILib calls `simulationPeriodic()` **before** commands execute in a given cycle. So `simulationPeriodic()` always reads desired states set by the *previous* command cycle — a one-loop delay. For translation this is acceptable; for rotation it compounds the angle mismatch above.

## Fix Options

### Option A (Recommended): Track `ChassisSpeeds` directly
Instead of reconstructing omega from module states, store the last commanded `ChassisSpeeds` directly in `drive()` and use it in `simulationPeriodic()`:

```java
// Add field:
private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

// At end of drive():
lastCommandedSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

// In simulationPeriodic(), replace toChassisSpeeds(...) call with:
var chassisSpeeds = lastCommandedSpeeds;
```

This avoids the kinematics round-trip entirely and is the most reliable approach.

### Option B: Use `driveRobotRelative` / `setChassisSpeeds` to cache speeds
Same idea as Option A but intercept at `setChassisSpeeds()`.

### Option C: Pass the `ChassisSpeeds` to `setModuleStates` and cache it there
Would require refactoring `setModuleStates` to accept and store the source `ChassisSpeeds`.

## Files to Change
- [`src/main/java/frc/robot/subsystems/Drivetrain.java`](src/main/java/frc/robot/subsystems/Drivetrain.java)
  - Add `lastCommandedSpeeds` field
  - Cache speeds in `drive()` (and `setChassisSpeeds()` for autonomous)
  - Use cached speeds in `simulationPeriodic()` instead of `toChassisSpeeds(desiredStates)`
