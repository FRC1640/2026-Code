package frc.robot.subsystems.hopper;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  private HopperIO io;
  private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }

  private void stop() {
    io.setHopperVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  /*
   * Commands
   */
  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setHopperVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

}
