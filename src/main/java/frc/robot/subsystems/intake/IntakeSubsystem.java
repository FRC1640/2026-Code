package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  private void stop() {
    io.setMotorVoltage(0, inputs);
  }

  private void rollerStop() {
    io.setRollerMotorVoltage(0, inputs);
  }

  public Command setIntakePositionCommand(double pos) {
    return run(() -> io.setMotorPosition(pos, inputs)).finallyDo(this::stop);
  }

  public Command setRollerVoltageCommand(double voltage) {
    return run(() -> io.setRollerMotorVoltage(voltage, inputs)).finallyDo(this::rollerStop);
  }

  public Command setRollerVelocityCommand(double velocity) {
    return run(() -> io.setRollerVelocity(velocity, inputs)).finallyDo(this::rollerStop);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
