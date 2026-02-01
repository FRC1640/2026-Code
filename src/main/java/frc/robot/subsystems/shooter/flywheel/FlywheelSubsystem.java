package frc.robot.subsystems.shooter.flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }


  /*
   * Commands
   */
  public Command runFlywheelSpeed(DoubleSupplier speed) {
    return run(() -> io.setFlywheelSpeed(speed.getAsDouble())).finallyDo(() -> io.setFlywheelSpeed(0));
  }
}
