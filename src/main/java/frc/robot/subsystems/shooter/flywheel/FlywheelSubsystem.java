package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.util.limits.ExponentialMovingAverage;

public class FlywheelSubsystem extends SubsystemBase {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  SysIdRoutine sysIdRoutine;

  private ExponentialMovingAverage currentEMA;
  private boolean jamDetected = false;


  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(8), Seconds.of(15),
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.magnitude()), null, this)); // TODO: maybe
    // change
    // this?
    
    currentEMA = new ExponentialMovingAverage(
        2.0,
        10.0,
        () -> Math.average(inputs.motorCurrent, inputs.motorFollowerCurrent),
        "FlywheelCurrent");
  }

  public boolean isJamDetected() {
    return jamDetected;
  }

  public void clearJamDetected() {
    jamDetected = false;
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
