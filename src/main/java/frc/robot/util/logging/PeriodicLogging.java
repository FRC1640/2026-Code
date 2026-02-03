package frc.robot.util.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.periodic.PeriodicBase;

public class PeriodicLogging extends PeriodicBase {

  public boolean active;
  public boolean initial;
  private String gameData;
  private String alliance;

  public PeriodicLogging() {
    active = false;
    gameData = DriverStation.getGameSpecificMessage();
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      alliance = "R";
    } else {
      alliance = "B";
    }
  }

  public boolean getActive(){
    if (DriverStation.isAutonomous()) {
      active = false;
    } else if (137 < DriverStation.getMatchTime() && DriverStation.getMatchTime() < 140) {
      if (gameData.charAt(0) == 'R' || gameData.charAt(0) == 'B') {
        initial = gameData.charAt(0) != alliance.charAt(0);
      }
      else{
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
  @Override
  public void periodic() {
    Logger.recordOutput("Dashboard/IsActivePeriod", getActive());
  }
}
