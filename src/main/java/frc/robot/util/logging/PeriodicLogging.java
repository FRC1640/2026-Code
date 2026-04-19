package frc.robot.util.logging;

import org.littletonrobotics.junction.Logger;

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

  public boolean active;
  public boolean initial;
  private String alliance;
  private final Field2d m_field = new Field2d();
  public PeriodicLogging() {
    active = false;
    alliance = AllianceManager.chooseFromAlliance("B", "R");
    SmartDashboard.putData("Field", m_field);
  }

  public String getZone() {
    double x = RobotOdometry.instance.getPose("Main").getX();
    if (x > FieldConstants.hubPositionBlue.getX() && x < FieldConstants.hubPositionRed.getX()) {
      return "NZ";
    }
    if (alliance.equals("R") && x > FieldConstants.hubPositionRed.getX()
        || alliance.equals("B") && x < FieldConstants.hubPositionBlue.getX()) {
      return "AZ";
    } else {
      return "EZ";
    }
  }

  public boolean getActive(double matchTime) {
    String gameData = DriverStation.getGameSpecificMessage();
    if (DriverStation.isAutonomous()) {
      active = false;
    } else if (137 < matchTime && matchTime < 140) {
      if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'B') {
        initial = gameData.charAt(0) != alliance.charAt(0);
      } else {
        initial = false;
      }
      active = false;
    } else if (130 < matchTime && matchTime < 137) {
      active = false;
    } else {
      int period = (int) ((matchTime - 30) / 25);
      if (period % 2 == 1) {
        active = initial;
      } else {
        active = !initial;
      }
    }
    return active;
  }

  public double getRemainingPeriodTime() {
    return (DriverStation.getMatchTime() - 30) % 25;
  }

  public boolean canShoot(double matchTime) {
    if (AllianceManager.chooseFromAlliance(1, 2) == 1) {
      double timeOfFlight = ShotControl.AZInterpolator
          .getTimeOfFlight(FieldConstants.hubPositionBlue.getTranslation().getDistance(RobotOdometry.instance
              .getPose("Main").plus(TurretConstants.turretTransform2d).getTranslation()));
      return getActive(DriverStation.getMatchTime() - timeOfFlight);
    } else {
      double timeOfFlight = ShotControl.AZInterpolator
          .getTimeOfFlight(FieldConstants.hubPositionRed.getTranslation().getDistance(RobotOdometry.instance
              .getPose("Main").plus(TurretConstants.turretTransform2d).getTranslation()));
      return getActive(DriverStation.getMatchTime() - timeOfFlight);
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Dashboard/IsActivePeriod", getActive(DriverStation.getMatchTime()));
    Logger.recordOutput("Dashboard/RemainingPeriodTime", getRemainingPeriodTime());
    Logger.recordOutput("Dashboard/MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("Dashboard/GameSpecificMessage", DriverStation.getGameSpecificMessage());
    Logger.recordOutput("Dashboard/Zone", getZone());
    Logger.recordOutput("Dashboard/RobotType", RobotConstants.RobotInformation.robot.getName());
    Logger.recordOutput("Dashboard/IsSafeToShoot", canShoot(DriverStation.getMatchTime()));
    m_field.setRobotPose(RobotOdometry.instance.getPose("Main"));
  }
}
