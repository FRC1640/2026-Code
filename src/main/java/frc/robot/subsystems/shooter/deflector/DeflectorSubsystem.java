package frc.robot.subsystems.shooter.deflector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
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

  /*
   * Commands
   */
  public Command runHoodToAngle(DoubleSupplier angle) {
    return run(() -> io.setDeflectorAngle(angle.getAsDouble()));
  }

}
