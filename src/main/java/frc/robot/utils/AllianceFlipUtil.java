package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.NavigationConstants;

/**
 * AllianceFlipUtil — ONE place that knows how to mirror field coordinates
 * between the red and blue alliances.
 *
 * ============================================================================
 * WHY THIS CLASS EXISTS (read me, future students!)
 * ============================================================================
 * The field is symmetric: the red side is the blue side rotated 180 degrees
 * around the center of the field (a "point reflection"). Before this class
 * existed, different files each hand-rolled their own mirroring math, and at
 * least one of them was wrong:
 *
 *   WRONG:   red_rotation = 180 - blue_rotation
 *   RIGHT:   red_rotation = blue_rotation + 180   (mod 360)
 *
 * Why is "180 - x" wrong? Try a 45-degree heading: 180 - 45 = 135, but the
 * correct answer is 45 + 180 = 225. Those point in completely different
 * directions! The two formulas only agree when the heading is 0 or 180
 * degrees — which is exactly why the bug hid so long: our climb poses are
 * all straight up/down the field.
 *
 * RULES FOR THE TEAM:
 *   1. Never write "fieldLength - x" or "180 - heading" in a command or
 *      subsystem. Call these methods instead.
 *   2. Author all poses in the BLUE frame, then flip them with flip()/apply().
 *   3. flip(flip(x)) must always equal x — the unit tests prove it.
 *
 * Pattern borrowed from Team 6328 (Mechanical Advantage), adapted to this
 * robot's Constants layout.
 * ============================================================================
 */
public final class AllianceFlipUtil {

  private AllianceFlipUtil() {
    // Utility class: no instances.
  }

  // ---- Primitive flips (ALWAYS flip, no alliance check) ----

  /** Mirrors an X coordinate across the field's center line. */
  public static double flipX(double x) {
    return NavigationConstants.kFieldLength - x;
  }

  /** Mirrors a Y coordinate across the field's center line. */
  public static double flipY(double y) {
    return NavigationConstants.kFieldWidth - y;
  }

  /** Mirrors a field translation to the other alliance's frame. */
  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  /**
   * Mirrors a heading. THE FIX: rotate by 180 degrees (pi radians), NOT
   * "180 minus the angle". Rotating the whole field 180 degrees adds 180 to
   * every heading — it does not reflect it.
   */
  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.fromDegrees(180.0));
  }

  /** Mirrors a full pose (position AND heading) to the other alliance's frame. */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  // ---- Alliance-aware applies (flip only when running on red) ----

  /**
   * @return true when the DriverStation currently reports the red alliance.
   *     Defaults to blue (no flip) when no alliance is known, e.g. at the bench.
   */
  public static boolean shouldFlip() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /** Mirrors an X coordinate only when on the red alliance. */
  public static double applyX(double x) {
    return shouldFlip() ? flipX(x) : x;
  }

  /** Mirrors a Y coordinate only when on the red alliance. */
  public static double applyY(double y) {
    return shouldFlip() ? flipY(y) : y;
  }

  /** Mirrors a translation only when on the red alliance. */
  public static Translation2d apply(Translation2d translation) {
    return shouldFlip() ? flip(translation) : translation;
  }

  /** Mirrors a heading only when on the red alliance. */
  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? flip(rotation) : rotation;
  }

  /** Mirrors a pose only when on the red alliance (uses the live alliance). */
  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? flip(pose) : pose;
  }

  /**
   * Mirrors a pose for a SPECIFIC alliance instead of the live one.
   *
   * Used by commands whose contract is "these coordinates are blue-frame;
   * convert them for whichever alliance this instance targets" — e.g.
   * GoToPositionRelativeCommand, which captures the alliance at construction
   * time so an auto routine can't be surprised by a mid-match alliance change.
   */
  public static Pose2d apply(Pose2d pose, DriverStation.Alliance alliance) {
    return alliance == DriverStation.Alliance.Red ? flip(pose) : pose;
  }
}
