package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

public final class DriverStationState {
  private static final double kHubActiveLeadTimeSeconds = 5.0;

  private static boolean hubActive;
  private static boolean hubActiveFiveSecondsEarly;

  private DriverStationState() {}

  public static boolean isHubActive() {
    return hubActive;
  }

  public static boolean isHubActiveFiveSecondsEarly() {
    return hubActiveFiveSecondsEarly;
  }

  public static boolean refreshHubActive() {
    hubActive = computeHubActive(0.0);
    hubActiveFiveSecondsEarly = computeHubActive(kHubActiveLeadTimeSeconds);
    return hubActive;
  }

  private static boolean computeHubActive(double leadTimeSeconds) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }

    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime() - leadTimeSeconds;
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      return true;
    } else if (matchTime > 105) {
      return shift1Active;
    } else if (matchTime > 80) {
      return !shift1Active;
    } else if (matchTime > 55) {
      return shift1Active;
    } else if (matchTime > 30) {
      return !shift1Active;
    } else {
      return true;
    }
  }
}