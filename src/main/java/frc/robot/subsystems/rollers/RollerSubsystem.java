package frc.robot.subsystems.rollers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class RollerSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.rollerSubsystem;

  private RollerIO io;
  private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public RollerSubsystem(RollerIO io) {
    super(info);
    this.io = io;
  }

  private void stop() {
    io.setVoltage(0, inputs);
  }

  public Command setVelocityCommand(double velocity) {
    return run(() -> io.setVelocity(velocity, inputs)).finallyDo(this::stop);
  }

  public Command setVoltageCommand(double voltage) {
    return run(() -> io.setVoltage(voltage, inputs)).finallyDo(this::stop);
  }

  public Command runCommand() {
    return setVoltageCommand(RollerConstants.runVoltage);
  }

  public Command stopCommand() {
    return setVoltageCommand(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static RollerIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new RollerIO() {
      };
    return switch (Robot.getMode()) {
      case REAL -> new RollerIOReal();
      case SIM -> new RollerIOSim();
      case REPLAY -> new RollerIO() {
      };
    };
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return run(() -> {
      io.setVoltage(leftJoystickValue.getAsDouble() * -8, inputs);
    }).finallyDo(this::stop);
  }
}
