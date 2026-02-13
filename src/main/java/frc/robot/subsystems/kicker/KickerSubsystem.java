package frc.robot.subsystems.kicker;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class KickerSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.kickerSubsystem;

  private KickerIO io;
  private KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public KickerSubsystem(KickerIO io) {
    super(info);
    this.io = io;
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

  private double getVelocity() {
    return inputs.encoderVelocity;
  }

  public Command reverseVoltageCommand(double volts) {
    return run(() -> io.setVoltage(-Math.abs(volts))).finallyDo(this::stop);
  }

  public boolean overMaxVelocity(){
    return getVelocity() >= KickerConstants.maxVelocity;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public Command runCommand(){
    return runVoltageCommand(()-> KickerConstants.runVoltage);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -8);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static KickerIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) return new KickerIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new KickerIOReal();
      case SIM -> new KickerIOSim();
      case REPLAY -> new KickerIO() {};
    };
  } // spotless formatting
}
