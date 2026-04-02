package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.ShotControl;
import frc.robot.util.limits.ExponentialMovingAverage;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;
import frc.robot.constants.RobotConstants.RobotTypes;

public class ShooterSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotTypes.shooterSubsystem;

  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ExponentialMovingAverage currentEMA;

  private SysIdRoutine sysIdRoutine;

  private double testVelocityRPM = 240;
  private static final double minTestVelocityRPM = 0;
  private static final double maxTestVelocityRPM = 600;

  public ShooterSubsystem(ShooterIO io) {
    super(info);
    this.io = io;

    currentEMA = new ExponentialMovingAverage(2.0, 10.0,
        () -> Math.max(inputs.leaderMotorCurrent, inputs.followerMotorCurrent), "ShooterCurrent");

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(8), Seconds.of(15),
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        // Use the signed voltage value so reverse phases produce negative voltages.
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, this)); // TODO: maybe
    // change
    // this?
  }

  public Command shootCommand() {
    return runVelocityRPMCommand(() -> ShotControl.getInstance().getSetpoint().shooterVelocityRPM());
  }

  public Command runVelocityRPMCommand(DoubleSupplier speed) {
    return run(() -> io.setVelocityRadPerSec(speed.getAsDouble() * 2 * Math.PI / 60)).finallyDo(this::stop);
  }

  public Command runVelocityRadPerSecCommand(DoubleSupplier speed) {
    return run(() -> io.setVelocityRadPerSec(speed.getAsDouble())).finallyDo(this::stop);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -8);
  }

  public void setTestVelocity(double velocityRPM) {
    this.testVelocityRPM = MathUtil.clamp(velocityRPM, minTestVelocityRPM, maxTestVelocityRPM);
  }

  public void incrementTestVelocity(double velocityDeltaRPM) {
    setTestVelocity(testVelocityRPM + velocityDeltaRPM);
  }

  public double getTestVelocity() {
    return testVelocityRPM;
  }

  public boolean isJamDetected() {
    return currentEMA.get() > ShooterConstants.jamCurrentAmps;
  }

  public boolean isAtSetpoint() {
    double currentRPM = (inputs.leaderVelocityRPM + inputs.followerVelocityRPM) * 0.5;
    double setpointRPM = ShotControl.getInstance().getSetpoint().shooterVelocityRPM();
    return Math.abs(currentRPM - setpointRPM) < ShooterConstants.setpointVelocityToleranceRPM;
  }

  public boolean isAtTestSetpoint() {
    double currentRPM = (inputs.leaderVelocityRPM + inputs.followerVelocityRPM) * 0.5;
    return Math.abs(currentRPM - testVelocityRPM) < ShooterConstants.setpointVelocityToleranceRPM;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Subsystems/Shooter/currentEMA", currentEMA.get());
    Logger.recordOutput("Subsystems/Shooter/jamDetected", isJamDetected());
    Logger.recordOutput("Subsystems/Shooter/isAtSetpoint", isAtSetpoint());
    Logger.recordOutput("Subsystems/Shooter/testVelocityRPM", testVelocityRPM);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static ShooterIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new ShooterIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new ShooterIOReal();
      case SIM -> new ShooterIOSim();
      case REPLAY -> new ShooterIO() {};
    };
  } // spotless formatting
}
