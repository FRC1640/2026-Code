package frc.robot.subsystems.hopper;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motorDashboard.DashboardInterface;

public class HopperSubsystem extends SubsystemBase implements DashboardInterface {
  private HopperIO io;
  private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setHopperVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  private void stop() {
    io.setHopperVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier joystickValue) {
    return runVoltageCommand(() -> joystickValue.getAsDouble()*-8);
  }

  @Override
  public String getName() {
    return "HopperSubsystem";
  }
}
