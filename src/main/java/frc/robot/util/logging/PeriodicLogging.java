package frc.robot.util.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.periodic.PeriodicBase;

public class PeriodicLogging extends PeriodicBase {
  private boolean startActive = false;

  private final Field2d m_field = new Field2d();

  public static final int autonomousDuration = 20;
  public static final int transitionDuration = 10;
  public static final int periodDuration = 25;
  public static final int endgameDuration = 30;
  public static final int teleopDuration = transitionDuration + 4 * periodDuration + endgameDuration;

  public PeriodicLogging() {
    SmartDashboard.putData("Field", m_field);
  }

  public String getZone() {
    return switch (ShotControl.getInstance().getZone()) {
      case ALLIANCE_ZONE -> "AZ";
      case NEUTRAL_ZONE -> "NZ";
      case ENEMY_ZONE -> "EZ";
    };
  }

  public void updateStartingShift(double matchTime, String gameData) {
    if (matchTime > teleopDuration - 3 && matchTime < teleopDuration && !gameData.isEmpty()) {
      if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'B') {
        startActive = gameData.charAt(0) != AllianceManager.chooseFromAlliance('B', 'R');
      }
    }
  }

  public boolean isActive(double matchTime) {
    // active in autonomous
    if (DriverStation.isAutonomous()) {
      return true;
    }
    // active in transition period
    if (matchTime > teleopDuration - transitionDuration) {
      return true;
    }
    // alternate teleop activity
    int period = 4 - (int) ((matchTime - endgameDuration) / periodDuration);
    if (period % 2 == 1) {
      return startActive;
    } else {
      return !startActive;
    }
  }

  public double getRemainingPeriodTime(double matchTime) {
    if (DriverStation.isAutonomous())
      return matchTime;
    if (matchTime < endgameDuration)
      return matchTime;
    return (matchTime - endgameDuration) % periodDuration;
  }

  public boolean canShoot(double matchTime) {
    Pose2d turretPose = RobotOdometry.instance.getPose("Main").plus(TurretConstants.turretTransform2d);
    boolean inAllianceZone = AllianceManager
        .chooseFromAlliance(FieldConstants.blueAllianceZone, FieldConstants.redAllianceZone)
        .poseSatisfies(turretPose);
    if (!inAllianceZone)
      return false;
    if (AllianceManager.chooseFromAlliance(1, 2) == 1) {
      double timeOfFlight = ShotControl.AZInterpolator.getTimeOfFlight(
          FieldConstants.hubPositionBlue.getTranslation().getDistance(turretPose.getTranslation()));
      System.out.println(matchTime - timeOfFlight);
      return isActive(matchTime - timeOfFlight);
    } else {
      double timeOfFlight = ShotControl.AZInterpolator.getTimeOfFlight(
          FieldConstants.hubPositionRed.getTranslation().getDistance(turretPose.getTranslation()));
      System.out.println(matchTime - timeOfFlight);
      return isActive(matchTime - timeOfFlight);
    }
  }

  @Override
  public void periodic() {
    double matchTime = DriverStation.getMatchTime();
    Logger.recordOutput("Dashboard/MatchTime", matchTime);
    Logger.recordOutput("Dashboard/IsActivePeriod", isActive(matchTime));
    Logger.recordOutput("Dashboard/IsSafeToShoot", canShoot(matchTime));
    Logger.recordOutput("Dashboard/RemainingPeriodTime", getRemainingPeriodTime(matchTime));

    String gameData = DriverStation.getGameSpecificMessage();
    updateStartingShift(matchTime, gameData);
    Logger.recordOutput("Dashboard/GameSpecificMessage", gameData);
    Logger.recordOutput("Dashboard/Zone", getZone());
    Logger.recordOutput("Dashboard/RobotType", RobotConstants.RobotInformation.robot.getName());
    m_field.setRobotPose(RobotOdometry.instance.getPose("Main"));
  }
}
