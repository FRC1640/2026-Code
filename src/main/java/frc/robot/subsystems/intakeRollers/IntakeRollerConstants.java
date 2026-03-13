package frc.robot.subsystems.intakeRollers;

import frc.robot.util.robotswitcher.SwitchableCANID;

public class IntakeRollerConstants {
  public static final int canID = SwitchableCANID.of(16).get();
  public static final double gearRatio = 1;
  public static final double intakeVoltage = 12;
  public static final double reverseIntakeVoltage = -10;
  public static final double intakeCurrentLimitAmps = Double.POSITIVE_INFINITY; // TODO: change

}
