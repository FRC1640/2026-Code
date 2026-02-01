package frc.robot.subsystems.shooter.flywheel;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

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

  public Command runFlywheelSpeed(Supplier<TurretSetpoint> setpoint) {
    return run(() -> io.setFlywheelSpeed(setpoint.get()));
  }
}
