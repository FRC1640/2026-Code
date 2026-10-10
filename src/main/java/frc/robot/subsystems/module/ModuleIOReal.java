package frc.robot.subsystems.module;

import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.odometry.SparkOdometryThread;
import frc.robot.sensors.resolvers.ResolverPWM;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

import static frc.robot.subsystems.drive.DriveConstants.useSteerVelocitySetpoint;

public class ModuleIOReal implements ModuleIO {
  private double driveVelocitySetpoint = 0;
  private double steerVelocitySetpoint = 0;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;
  private final Queue<Double> driveVelocityQueue;

  private final RelativeEncoder driveEncoder;
  private final ResolverPWM steeringEncoder;
  // private final ResolverVoltage steeringEncoder;

  private final SparkFlex driveSpark;
  private final SparkMax steerSpark;

  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFF;
  private final PIDController steerPID;
  private final SimpleMotorFeedforward steerFF;

  private Rotation2d lastSteerSetpoint = new Rotation2d();
  private int lastSteerSetpointCounter = 0;

  private final String logPath;

  public ModuleIOReal(ModuleInfo id) {
    drivePID = RobotPIDConstants.constructPID(RobotPIDConstants.drivePid, "drivePID" + id.id.toString());
    driveFF = RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.driveFF, "driveFF" + id.id.toString());
    steerPID = RobotPIDConstants.constructPID(RobotPIDConstants.steerPid, "steerPID" + id.id.toString());
    steerFF = RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.steerFF, "steerFF" + id.id.toString());
    driveSpark = SparkConfigurer.configSparkFlex(id.driveID, SparkConstants.driveConfig);
    steerSpark = SparkConfigurer.configSparkMax(id.steerID, SparkConstants.steerConfig);
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark,
        () -> driveSpark.getEncoder().getPosition());

    driveEncoder = driveSpark.getEncoder();
    steeringEncoder = new ResolverPWM(id.resolverChannel, id.angleOffset);
    // steeringEncoder =
    // new ResolverVoltage(
    // id.resolverChannel,
    // DriveConstants.initalSlope,
    // DriveConstants.finalSlope,
    // 180.0,
    // 90.0,
    // id.angleOffset);
    driveVelocityQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark,
        () -> driveEncoder.getVelocity());

    turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(steerSpark,
        () -> steeringEncoder.getDegrees() % 360);

    logPath = "Subsystems/Module/" + id.id.toString();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.steerConnected = true;
    inputs.drivePositionMeters = -(driveEncoder.getPosition() / DriveConstants.driveGearRatio)
        * DriveConstants.wheelRadius * 2 * Math.PI;
    inputs.driveVelocityMetersPerSecond = -((driveEncoder.getVelocity() / DriveConstants.driveGearRatio) / 60) * 2
        * Math.PI * DriveConstants.wheelRadius;
    inputs.driveAppliedVoltage = driveSpark.getAppliedOutput() * driveSpark.getBusVoltage();
    inputs.driveCurrentAmps = driveSpark.getOutputCurrent();
    inputs.driveTempCelsius = driveSpark.getMotorTemperature();
    inputs.steerAppliedVoltage = steerSpark.getAppliedOutput() * steerSpark.getBusVoltage();

    inputs.steerCurrentAmps = steerSpark.getOutputCurrent();
    inputs.steerRadPerSec = steerSpark.getEncoder().getVelocity() * Math.PI * 2 / 60
        / DriveConstants.steerGearRatio;
    inputs.steerTempCelsius = steerSpark.getMotorTemperature();
    // inputs.steerEncoderRawValue = steeringEncoder.getFrequency();
    inputs.steerEncoderRelative = (360
        - (steerSpark.getEncoder().getPosition() / DriveConstants.steerGearRatio * 360)) % 360;
    inputs.steerAngleDegrees = (steeringEncoder.getDegrees()) % 360;

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryDrivePositionsMeters = drivePositionQueue.stream().mapToDouble(
        (Double value) -> -(value / DriveConstants.driveGearRatio) * DriveConstants.wheelRadius * 2 * Math.PI)
        .toArray();

    inputs.odometryTurnPositions = turnPositionQueue.stream().map((Double value) -> Rotation2d.fromDegrees(value))
        .toArray(Rotation2d[]::new);

    inputs.driveVelocities = driveVelocityQueue.stream()
        .mapToDouble((Double value) -> -(value / DriveConstants.driveGearRatio) / 60
            * DriveConstants.wheelRadius * 2 * Math.PI)
        .toArray();

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
    driveVelocityQueue.clear();

    // inputs.rawEncoderValue = steeringEncoder.getRawValue();

    lastSteerSetpointCounter = Math.max(0, lastSteerSetpointCounter - 1);
  }

  @Override
  public void setDriveVelocity(double velocity, ModuleIOInputs inputs) {
    double pidSpeed = driveFF.calculate(velocity);
    pidSpeed += drivePID.calculate(inputs.driveVelocityMetersPerSecond, velocity);
    setDriveVoltage(pidSpeed);
    driveVelocitySetpoint = velocity;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveSpark.setVoltage(voltage);
  }

  @Override
  public void setSteerPosition(Rotation2d angle, ModuleIOInputs inputs) {
    Rotation2d delta = angle.minus(Rotation2d.fromDegrees(inputs.steerAngleDegrees));
    double sin = Math.sin(delta.getRadians());
    double pidVoltage = steerPID.calculate(sin, 0) * 6;

    Logger.recordOutput(logPath + "/errorRadians", (Math.abs(inputs.steerAngleDegrees - angle.getDegrees()) % 180) * Math.PI / 180);

    // calculate setpoint velocity using last input
    double setpointDerivative;
    if (!isLastSteerSetpointValid() || !useSteerVelocitySetpoint)
      setpointDerivative = 0; // do not set velocity if estimate is stale
    else { // if last setpoint is fresh (last robot loop), approximate derivative of
        // setpoint
      setpointDerivative = angle.minus(lastSteerSetpoint).getRadians() / 0.02;
    } // clamp velocity setpoint
    Logger.recordOutput(logPath + "/steerSetpointDerivative", setpointDerivative, Units.RadiansPerSecond);

    double angularVelocityRadPerSec;
    // force velocity setpoint to zero if setpoint discontinuous
    if (MathUtil.isNear(0, setpointDerivative, DriveConstants.steerSetpointContinuityDeltaRadPerSec))
      angularVelocityRadPerSec = MathUtil.clamp(setpointDerivative, -DriveConstants.maxSteerRateRadiansPerSecond,
          DriveConstants.maxSteerRateRadiansPerSecond);
    else
      angularVelocityRadPerSec = 0;
    angularVelocityRadPerSec = Math.abs(angularVelocityRadPerSec) * Math.max(0, Math.signum(pidVoltage));

    steerVelocitySetpoint = angularVelocityRadPerSec;
    // update last setpoint
    updateLastSteerSetpoint(angle);

    // compute output voltage
    double ffVoltage = steerFF.calculate(angularVelocityRadPerSec);
    Logger.recordOutput(logPath + "/steerPIDVoltage", pidVoltage);
    Logger.recordOutput(logPath + "/steerFFVoltage", ffVoltage);
    setSteerVoltage(ffVoltage + pidVoltage);
  }

  @Override
  public void setSteerVoltage(double voltage) {
    steerSpark.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public double driveVelocitySetpoint() {
    return driveVelocitySetpoint;
  }

  @Override
  public double steerVelocitySetpoint() {
    return steerVelocitySetpoint;
  }

  private void updateLastSteerSetpoint(Rotation2d lastSteerSetpoint) {
    this.lastSteerSetpoint = lastSteerSetpoint;
    lastSteerSetpointCounter = 2;
  }

  private boolean isLastSteerSetpointValid() {
    return lastSteerSetpointCounter > 0;
  }
}
