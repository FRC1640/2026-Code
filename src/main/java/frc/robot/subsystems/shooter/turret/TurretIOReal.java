package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class TurretIOReal implements TurretIO {
  private SparkMax turretMotor;
  private SparkAnalogSensor turretEncoder;
  private PIDController turretController;

  public TurretIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(TurretConstants.canId, true);
    turretMotor = SparkConfigurer.configSparkMax(config);
    turretEncoder = turretMotor.getAnalog();
    turretController = RobotPIDConstants.constructPID(RobotPIDConstants.toyTurret);
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    double clampedAngle = TurretConstants.turretAngleLimits.clampPosition(angle);
    setTurretVoltage(turretController.calculate(getTurretPosition(), clampedAngle));
  }

  @Override
  public void setTurretVoltage(double voltage) {
    turretMotor.setVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    Logger.recordOutput("Subsystems/Turret/AnalogVoltage", turretEncoder.getVoltage());
    inputs.turretAngle = getTurretPosition();
    inputs.turretAngularVelocity = getTurretVelocity();
    inputs.turretMotorCurrent = turretMotor.getOutputCurrent();
    inputs.turretMotorVoltage = turretMotor.getBusVoltage() * turretMotor.getAppliedOutput();
    inputs.turretMotorTemperature = turretMotor.getMotorTemperature();
  }

  private double getTurretPosition() {
    return (turretEncoder.getPosition() - TurretConstants.potLowerVoltage)
        / (TurretConstants.potUpperVoltage - TurretConstants.potLowerVoltage) * 2 * Math.PI - Math.PI;
  }

  private double getTurretVelocity() {
    return turretEncoder.getVelocity() / (TurretConstants.potUpperVoltage - TurretConstants.potLowerVoltage) * 2
        * Math.PI;
  }
}
