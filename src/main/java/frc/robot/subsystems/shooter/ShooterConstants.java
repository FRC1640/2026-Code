package frc.robot.subsystems.shooter;

import frc.robot.util.robotswitcher.SwitchableCANID;

public class ShooterConstants {

  public static final int canId = SwitchableCANID.of(10).get();
  public static final int followerCanId = SwitchableCANID.of(11).get();
  public static final double jamCurrentAmps = Double.POSITIVE_INFINITY;
  public static final double setpointVelocityToleranceRPM = 100;

  public static final double spinupBoostVoltage = 10;

  public static final double percentageRequiredToAdjustSpinup = 0.8;
}
