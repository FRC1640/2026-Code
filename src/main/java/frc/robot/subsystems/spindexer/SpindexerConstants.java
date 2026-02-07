package frc.robot.subsystems.spindexer;

import frc.robot.util.robotswitcher.SwitchableCANID;

public class SpindexerConstants {

  public static int indexerSparkCanId = SwitchableCANID.of(14).get();; // TODO: set id and inverted and set to actual
  // values
  public static boolean indexerSparkInverted = false;

  public static double indexerGearRatioSim = 1;
}
