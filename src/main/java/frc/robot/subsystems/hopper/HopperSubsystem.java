package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  private HopperIO io;
  private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }
}
