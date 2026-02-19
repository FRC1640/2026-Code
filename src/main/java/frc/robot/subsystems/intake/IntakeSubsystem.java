package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class IntakeSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotConstants.RobotTypes.intakeSubsystem;

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    super(info);
    this.io = io;
  }

  public Command setPositionCommand(double pos) {
    return run(() -> io.setPosition(pos)).finallyDo(this::stop);
  }

  public Command setPositionCommand(Supplier<Double> pos) {
    return run(() -> io.setPosition(pos.get())).finallyDo(this::stop);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command intakeDownCommand() {
    return setPositionCommand(IntakeConstants.downPosition);
  }

  public Command intakeUpCommand() {
    return setPositionCommand(IntakeConstants.upPosition);
  }

  public Command oscillateIntakeCommand(double pos, double amp, double freq) {
    Timer t  = new Timer();
    return new InstantCommand(() -> {t.start();}).andThen(setPositionCommand(() -> pos + amp*Math.sin(t.get()*freq))).finallyDo(()->{t.stop(); t.reset();});
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return run(() -> {
      io.setVoltage(leftJoystickValue.getAsDouble() * -8);
    }).finallyDo(this::stop);
  }

  private void stop() {
    io.setVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static IntakeIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new IntakeIO() {
      };
    return switch (Robot.getMode()) {
      case REAL -> new IntakeIOReal();
      case SIM -> new IntakeIOSim();
      case REPLAY -> new IntakeIO() {
      };
    };
  }
}
