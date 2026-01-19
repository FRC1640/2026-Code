package frc.robot.subsystems.frank;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  public Command setPositionCommand(double pos) {
    return run(() -> io.setMotorPosition(pos)).finallyDo(() -> io.setMotorVoltage(0));
  }

  public Command setVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setMotorVoltage(voltage.getAsDouble())).finallyDo(() -> io.setMotorVoltage(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("FrankArm", inputs);
  }
}
