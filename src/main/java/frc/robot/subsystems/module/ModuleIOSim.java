package frc.robot.subsystems.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.drive.DriveConstants;

public class ModuleIOSim implements ModuleIO {
  private double driveVelocitySetpoint = 0;
  private double steerVelocitySetpoint = 0;
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFF;
  private final PIDController steerPID;
  private final SimpleMotorFeedforward steerFF;

  private Rotation2d lastSteerSetpoint = new Rotation2d();
  private int lastSteerSetpointCounter = 0;

  public ModuleIOSim(ModuleInfo id) {
    drivePID = RobotPIDConstants.constructPID(RobotPIDConstants.drivePid, "drivePID" + id.id.toString());
    driveFF = RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.driveFF, "driveFF" + id.id.toString());
    steerPID = RobotPIDConstants.constructPID(RobotPIDConstants.steerPid, "steerPID" + id.id.toString());
    steerFF = RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.steerFF, "steerFF" + id.id.toString());
    DCMotor driveGearbox = DCMotor.getNeoVortex(1);
    DCMotor turnGearbox = DCMotor.getNeo550(1);
    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(driveGearbox, 0.00019125, DriveConstants.driveGearRatio),
        driveGearbox);
    turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(turnGearbox, 0.002174375, DriveConstants.steerGearRatio),
        turnGearbox);
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
    driveAppliedVolts = voltage;
  }

  @Override
  public void setSteerPosition(Rotation2d angle, ModuleIOInputs inputs) {
    Rotation2d delta = angle.minus(Rotation2d.fromDegrees(inputs.steerAngleDegrees));
    double sin = Math.sin(delta.getRadians());

    // calculate setpoint velocity using last input
    double angularVelocityRadPerSec;
    if (!isLastSteerSetpointValid())
      angularVelocityRadPerSec = 0; // do not set velocity if estimate is stale
    else { // if last setpoint is fresh (last robot loop), approximate derivative of setpoint
      angularVelocityRadPerSec = angle.minus(lastSteerSetpoint).getRadians() / 0.02;
    } // clamp velocity setpoint
    angularVelocityRadPerSec = MathUtil.clamp(angularVelocityRadPerSec,
        -DriveConstants.maxSteerRateRadiansPerSecond, DriveConstants.maxSteerRateRadiansPerSecond);
    steerVelocitySetpoint = angularVelocityRadPerSec;
    // update last setpoint
    updateLastSteerSetpoint(angle);

    // compute output voltage
    double voltage = steerFF.calculate(angularVelocityRadPerSec) + steerPID.calculate(sin, 0) * 12;
    setSteerVoltage(voltage);
  }

  @Override
  public void setSteerVoltage(double voltage) {
    turnAppliedVolts = -MathUtil.clamp(voltage, -12, 12);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    driveSim.setInputVoltage(-driveAppliedVolts);
    turnSim.setInputVoltage(turnAppliedVolts);
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionMeters = driveSim.getAngularPositionRad() * DriveConstants.wheelRadius;
    inputs.driveVelocityMetersPerSecond = driveSim.getAngularVelocityRadPerSec() * DriveConstants.wheelRadius;

    inputs.driveAppliedVoltage = driveAppliedVolts;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

    inputs.steerConnected = true;
    inputs.steerAngleDegrees += (turnSim.getAngularVelocityRPM() * 360 / 60) * 0.02;
    inputs.steerRadPerSec = turnSim.getAngularVelocityRPM() * 2 * Math.PI / 60;
    inputs.steerAppliedVoltage = turnAppliedVolts;
    inputs.steerCurrentAmps = turnSim.getCurrentDrawAmps();

    inputs.odometryTimestamps = new double[]{Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsMeters = new double[]{inputs.drivePositionMeters};
    inputs.odometryTurnPositions = new Rotation2d[]{Rotation2d.fromDegrees(inputs.steerAngleDegrees)};
    inputs.driveVelocities = new double[]{inputs.driveVelocityMetersPerSecond};    

    lastSteerSetpointCounter = Math.max(0, lastSteerSetpointCounter - 1);
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
