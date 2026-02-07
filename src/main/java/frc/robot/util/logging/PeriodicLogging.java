package frc.robot.util.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.periodic.PeriodicBase;

public class PeriodicLogging extends PeriodicBase {

  public boolean active;
  public boolean initial;
  private String alliance;

  public PeriodicLogging() {
    active = false;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      alliance = "R";
    } else {
      alliance = "B";
    }
  }

  public String getZone() {
    double x = RobotOdometry.instance.getPose("Main").getX();
    if (x > FieldConstants.hubPositionBlue.getX() && x < FieldConstants.hubPositionRed.getX()) {
      return "NZ";
    }
    if (DriverStation.getAlliance().get() == Alliance.Red && x > FieldConstants.hubPositionRed.getX()
        || DriverStation.getAlliance().get() == Alliance.Blue && x < FieldConstants.hubPositionBlue.getX()) {
      return "AZ";
    } else {
      return "EZ";
    }
  }
  public boolean getActive() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (DriverStation.isAutonomous()) {
      active = false;
    } else if (137 < DriverStation.getMatchTime() && DriverStation.getMatchTime() < 140) {
      if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'B') {
        initial = gameData.charAt(0) != alliance.charAt(0);
      } else {
        initial = false;
      }
      active = false;
    } else if (130 < DriverStation.getMatchTime() && DriverStation.getMatchTime() < 137) {
      active = false;
    } else {
      int period = (int) ((DriverStation.getMatchTime() - 30) / 25);
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
  @Override
  public void periodic() {
    Logger.recordOutput("Dashboard/IsActivePeriod", getActive());
    Logger.recordOutput("Dashboard/RemainingPeriodTime", getRemainingPeriodTime());
    Logger.recordOutput("Dashboard/MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("Dashboard/GameSpecificMessage", DriverStation.getGameSpecificMessage());
    Logger.recordOutput("Dashboard/Zone", getZone());
    Logger.recordOutput("Dashboard/RobotType", RobotConstants.RobotInformation.robot.getName());
  }
}
