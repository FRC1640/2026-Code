package frc.robot.subsystems.shooter.deflector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class DeflectorIOReal implements DeflectorIO {
  private SparkMax deflectorMotor;
  private AbsoluteEncoder deflectorEncoder;
  private SparkClosedLoopController deflectorController;

  public DeflectorIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(DeflectorConstants.canId, false);
    deflectorMotor = SparkConfigurer.configSparkMax(config);

    deflectorController = deflectorMotor.getClosedLoopController();
  }

  @Override
  public void setDeflectorAngle(double angle) {
    deflectorController.setSetpoint(angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0); // TODO: max motion
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs){
    inputs.deflectorAngle = deflectorEncoder.getPosition() * 2 * Math.PI; // TODO: same assumption as in TurretIOReal.java
    inputs.deflectorMotorTemperature = deflectorMotor.getMotorTemperature();
    inputs.deflectorMotorCurrent = deflectorMotor.getOutputCurrent();
  }
}
