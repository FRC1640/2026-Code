package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;

public class IntakeSubsystem extends SubsystemBase {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static SubsystemInfo info = Subsystems.intakeSubsystem;

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public Command setIntakePositionCommand(double pos) {
    return run(() -> io.setIntakePosition(pos, inputs)).finallyDo(this::stop);
  }

  public Command setRollerVelocityCommand(double velocity) {
    return run(() -> io.setRollerVelocity(velocity, inputs)).finallyDo(this::rollerStop);
  }

  public Command setRollerVoltageCommand(double voltage) {
    return run(() -> io.setRollerVoltage(voltage, inputs)).finallyDo(this::rollerStop);
  }

  private void stop() {
    io.setIntakeVoltage(0, inputs);
  }

  private void rollerStop() {
    io.setRollerVoltage(0, inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // custom formatting
  public static IntakeIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new IntakeIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new IntakeIOReal();
      case SIM -> new IntakeIOSim();
      case REPLAY -> new IntakeIO() {};
    };
  } // spotless formatting
}
