package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.NavigationConstants;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for AllianceFlipUtil — the ONE place that mirrors field
 * coordinates between alliances.
 *
 * WHY TEST THIS? Because the bug these tests prevent already shipped once:
 * a command used "180 - heading" instead of "heading + 180", and the error
 * was invisible for every straight-line pose the team actually used. These
 * tests fail loudly on that class of mistake, for ALL headings.
 */
public class AllianceFlipUtilTest {

  /** Tolerance for comparing angles in degrees (floating-point wiggle room). */
  private static final double kAngleToleranceDeg = 1e-9;
  /** Tolerance for comparing positions in meters. */
  private static final double kPositionToleranceM = 1e-9;

  // ------------------------------------------------------------------
  // THE core property: flipping twice returns exactly what you started
  // with, for every heading. The old "180 - heading" math fails this at
  // e.g. 45 degrees: flip(45) = 135, then flip(135) = 45... wait, that one
  // passes by luck; try 30 degrees: 180-30=150 -> 180-150=30. Hmm! The old
  // formula is an involution too. The REAL proof it's wrong is the next
  // test — it checks the flip against ground truth, not just self-consistency.
  // ------------------------------------------------------------------
  @Test
  public void flipTwiceReturnsOriginal_forManyHeadings() {
    for (double heading = -179; heading <= 180; heading += 7) {
      Pose2d original = new Pose2d(3.3, 4.4, Rotation2d.fromDegrees(heading));
      Pose2d roundTrip = AllianceFlipUtil.flip(AllianceFlipUtil.flip(original));
      // Compare with tolerance: two mirror operations go through trig, so
      // results can differ from the original in the last few bits.
      assertTrue(
          Math.abs(original.getX() - roundTrip.getX()) < kPositionToleranceM
              && Math.abs(original.getY() - roundTrip.getY()) < kPositionToleranceM
              && Math.abs(wrapped(original.getRotation().minus(roundTrip.getRotation()).getDegrees()))
                  < 1e-6,
          "flip(flip(pose)) should return the original pose at heading " + heading);
    }
  }

  /**
   * Ground-truth check against hand-computed answers. This is the test that
   * catches the old bug: for a 45-degree heading, the correct red-frame value
   * is 225 degrees ("45 + 180"), NOT 135 ("180 - 45").
   */
  @Test
  public void rotationMirroringMatchesGroundTruth() {
    double length = NavigationConstants.kFieldLength;
    double width = NavigationConstants.kFieldWidth;

    // Diagonal heading: where "180 - x" and "x + 180" disagree.
    Pose2d diagonal =
        new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(45.0));
    Pose2d flippedDiagonal = AllianceFlipUtil.flip(diagonal);
    assertEquals(length - 2.0, flippedDiagonal.getX(), kPositionToleranceM);
    assertEquals(width - 2.0, flippedDiagonal.getY(), kPositionToleranceM);
    assertEquals(
        225.0, wrapped(flippedDiagonal.getRotation().getDegrees()), kAngleToleranceDeg,
        "A 45-degree heading must become 225 (= 45 + 180), not 135 (= 180 - 45)");

    // Straight-up-the-field heading (the case that used to hide the bug).
    Pose2d straightUp = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0));
    assertEquals(
        270.0, wrapped(AllianceFlipUtil.flip(straightUp).getRotation().getDegrees()), kAngleToleranceDeg);

    // Heading pointing along +X flips to point along -X.
    Pose2d east = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(0.0));
    assertEquals(
        180.0, wrapped(AllianceFlipUtil.flip(east).getRotation().getDegrees()), kAngleToleranceDeg);
  }

  /**
   * Symmetry sanity check: mirrored positions must be symmetric about the
   * field center, i.e. distance from center is preserved for X and Y.
   */
  @Test
  public void translationMirroringPreservesDistanceFromCenter() {
    Translation2d p = new Translation2d(12.9, 6.1);
    Translation2d flipped = AllianceFlipUtil.flip(p);
    assertEquals(
        NavigationConstants.kFieldLength - p.getX(), flipped.getX(), kPositionToleranceM);
    assertEquals(
        NavigationConstants.kFieldWidth - p.getY(), flipped.getY(), kPositionToleranceM);
  }

  /** Corner symmetry: flipping each corner must land on the opposite corner. */
  @Test
  public void cornersMapToOppositeCorners() {
    double L = NavigationConstants.kFieldLength;
    double W = NavigationConstants.kFieldWidth;

    Translation2d[][] cornerPairs = {
      {new Translation2d(0, 0), new Translation2d(L, W)},
      {new Translation2d(L, 0), new Translation2d(0, W)},
      {new Translation2d(L, W), new Translation2d(0, 0)},
      {new Translation2d(0, W), new Translation2d(L, 0)}
    };

    for (Translation2d[] pair : cornerPairs) {
      Translation2d result = AllianceFlipUtil.flip(pair[0]);
      assertEquals(pair[1].getX(), result.getX(), kPositionToleranceM);
      assertEquals(pair[1].getY(), result.getY(), kPositionToleranceM);
    }
  }

  /**
   * The actual climb start poses must round-trip through the mirror — if
   * someone edits a blue pose or the util, red stays consistent automatically.
   * This directly guards Constants.ClimbConstants' generated red poses.
   */
  @Test
  public void climbStartPosesRoundTrip() {
    assertTrue(flipEquals(ClimbPoseHolder.blueLeft, ClimbPoseHolder.redLeft));
    assertTrue(flipEquals(ClimbPoseHolder.blueRight, ClimbPoseHolder.redRight));
  }

  /** Helper: compare two poses within tolerance (Pose2d.equals is exact). */
  private boolean flipEquals(Pose2d blue, Pose2d red) {
    return Math.abs(blue.getX() - (NavigationConstants.kFieldLength - red.getX()))
            < kPositionToleranceM
        && Math.abs(blue.getY() - (NavigationConstants.kFieldWidth - red.getY()))
            < kPositionToleranceM
        && Math.abs(wrapped(blue.getRotation().minus(red.getRotation()).getDegrees()) - 180.0)
            < kAngleToleranceDeg;
  }

  /** Wraps any angle into [0, 360). */
  private static double wrapped(double degrees) {
    double d = degrees % 360.0;
    return d < 0 ? d + 360.0 : d;
  }

  /** Indirection so the test can read ClimbConstants without dragging in hardware statics. */
  private static class ClimbPoseHolder {
    static final Pose2d blueLeft = frc.robot.Constants.ClimbConstants.kBlueLeftStartPose;
    static final Pose2d blueRight = frc.robot.Constants.ClimbConstants.kBlueRightStartPose;
    static final Pose2d redLeft = frc.robot.Constants.ClimbConstants.kRedLeftStartPose;
    static final Pose2d redRight = frc.robot.Constants.ClimbConstants.kRedRightStartPose;
  }
}
