package frc.robot.subsystems.hopper;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class HopperSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.hopperSubsystem;

  private HopperIO io;
  private HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public HopperSubsystem(HopperIO io) {
    super();
    this.io = io;
    setName(info.getName());
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  private void stop() {
    io.setVoltage(0);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command reverseVoltageCommand(double volts) {
    return run(() -> io.setVoltage(-Math.abs(volts))).finallyDo(this::stop);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -8);
  }

  // custom formatting
  public static HopperIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) return new HopperIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new HopperIOReal();
      case SIM -> new HopperIOSim();
      case REPLAY -> new HopperIO() {};
    };
  } // spotless formatting
}
