package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.turret.TurretConstants.turretAngleLimits;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.ShotControl.TurretSetpoint;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class TurretSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM

  public static final SubsystemInfo info = RobotTypes.turretSubsystem;

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final SysIdRoutine sysIdRoutine;
  int correctionMultiplier = 0;

  public TurretSubsystem(TurretIO io) {
    super(info);
    this.io = io;

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Seconds).of(0.5), Volts.of(4), Seconds.of(20),
            state -> Logger.recordOutput("Turret/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, this));
  }

  public Command trackCommand() {
    return run(this::track).finallyDo(this::stop);
  }

  public Command setAngleCommand(DoubleSupplier angle) {
    return run(() -> io.setTurretState(angle.getAsDouble(), 0));
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
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

  private void track() {
    ChassisSpeeds odometrySpeeds = RobotOdometry.instance.getVelocity("Main");
    if (Math.hypot(odometrySpeeds.vxMetersPerSecond,
        odometrySpeeds.vyMetersPerSecond) > TurretConstants.trackingLinearVelocityThreshold
        || odometrySpeeds.omegaRadiansPerSecond > TurretConstants.trackingRotationalVelocityThreshold) {
      Logger.recordOutput("Subsystems/Turret/odometryProhibition", true);
      io.setTurretState(inputs.angleRadians, 0);
      return;
    }

    Logger.recordOutput("Subsystems/Turret/odometryProhibition", false);
    TurretSetpoint setpoint = ShotControl.getInstance().getSetpoint();
    double finalAngle = 0;
    // limit angle setpoint
    if (turretAngleLimits.inRange(setpoint.turretAngleRad())) {
      finalAngle = setpoint.turretAngleRad();
      Logger.recordOutput("Turret/inTargetRange", true);
      correctionMultiplier = 0;
    } else {
      finalAngle = turretAngleLimits.clampPosition(setpoint.turretAngleRad());
      Logger.recordOutput("Turret/inTargetRange", false);
      if (setpoint.turretAngleRad() > turretAngleLimits.high) {
        correctionMultiplier = 1;
      } else {
        correctionMultiplier = -1;
      }
    }
    io.setTurretState(finalAngle, setpoint.turretOmegaRadPerSec());
  }

  private void stop() {
    io.setVoltage(0);
  }

  public Rotation2d getAngle() {
    return new Rotation2d(inputs.angleRadians);
  }

  public int getMultiplierDrive() {
    return correctionMultiplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Shot/turretDirection", RobotOdometry.instance.getPose("Main")
        .plus(new Transform2d(new Translation2d(1, new Rotation2d(inputs.angleRadians)), new Rotation2d())));
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
    public static TurretIO getIOByMode() {
        if (!RobotConstants.RobotInformation.robot.isEnabled(info)) {
            return new TurretIO() {
            };
        }
        return switch (Robot.getMode()) {
            case REAL ->
                new TurretIOReal();
            case SIM ->
                new TurretIOSim();
            case REPLAY ->
                new TurretIO() {
                };
        };
    } // spotless formatting

}
