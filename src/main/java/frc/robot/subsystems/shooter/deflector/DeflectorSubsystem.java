package frc.robot.subsystems.shooter.deflector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeflectorSubsystem extends SubsystemBase {
  private DeflectorIO io;
  private DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  public DeflectorSubsystem(DeflectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deflector", inputs);
  }
}
