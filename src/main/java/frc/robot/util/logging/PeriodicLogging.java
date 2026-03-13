package frc.robot.util.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.periodic.PeriodicBase;

public class PeriodicLogging extends PeriodicBase {

  public boolean active;
  public boolean initial = false;
  private String alliance;
  private final Field2d m_field = new Field2d();
  public PeriodicLogging() {
    active = false;
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

  public boolean getActive() {

    if (DriverStation.isAutonomous()) {
      active = true;
    } else if (130 < DriverStation.getMatchTime() && DriverStation.getMatchTime() < 140) {
      active = true;
    } else if (30 < DriverStation.getMatchTime() && DriverStation.getMatchTime() < 130) {
      int period = (int) ((DriverStation.getMatchTime() - 30) / 25);
      if (period % 2 == 1) {
        active = initial;
      } else {
        active = !initial;
      }
    } else {
      active = true;
    }
    return active;
  }

  public double getRemainingPeriodTime() {
    if (DriverStation.isAutonomous() || DriverStation.getMatchTime() < 30) {
      return DriverStation.getMatchTime();
    } else {
      return (DriverStation.getMatchTime() - 30) % 25;
    }
  }

  @Override
  public void periodic() {
    String gameData = DriverStation.getGameSpecificMessage();
    alliance = AllianceManager.chooseFromAlliance("B", "R");
    if (137 < DriverStation.getMatchTime() && DriverStation.getMatchTime() < 140 && !gameData.isEmpty()) {
      if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'B') {
        initial = gameData.charAt(0) != alliance.charAt(0);
      } else {
        initial = false;
      }
    }
    Logger.recordOutput("Dashboard/IsActivePeriod", getActive());
    Logger.recordOutput("Dashboard/InitialPeriod", initial);
    Logger.recordOutput("Dashboard/RemainingPeriodTime", getRemainingPeriodTime());
    Logger.recordOutput("Dashboard/MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("Dashboard/GameSpecificMessage", DriverStation.getGameSpecificMessage());
    Logger.recordOutput("Dashboard/Zone", getZone());
    Logger.recordOutput("Dashboard/RobotType", RobotConstants.RobotInformation.robot.getName());
    m_field.setRobotPose(RobotOdometry.instance.getPose("Main"));
  }
}
