package frc.robot.subsystems.shooter.flywheel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class FlywheelIOReal implements FlywheelIO {
  private SparkFlex flywheelMotor;
  private RelativeEncoder flywheelEncoder;
  private SparkClosedLoopController flywheelController;
  private SparkFlex flywheelMotorFollower;
  private RelativeEncoder flywheelEncoderFollower;

  public FlywheelIOReal() {
    SparkConfiguration config = SparkConstants.getFlywheelFlex(FlywheelConstants.canId, false);
    config.getInnerConfig().closedLoop.pid(0.0001, 0, 0, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    flywheelMotor = SparkConfigurer.configSparkFlex(config);
    SparkConfiguration followerConfig = SparkConstants.getFlywheelFlex(FlywheelConstants.followerCanId, false,flywheelMotor);
    flywheelMotorFollower = SparkConfigurer.configSparkFlex(followerConfig);
    flywheelController = flywheelMotor.getClosedLoopController();
    flywheelEncoder = flywheelMotor.getEncoder();
    flywheelEncoderFollower = flywheelMotorFollower.getEncoder();

  }

  @Override
  public void setFlywheelSpeed(double speed) {
    flywheelController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, 0.0); // TODO:
    // max
    // motion
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelSpeed = flywheelEncoder.getVelocity();
    inputs.flywheelMotorTemperature = flywheelMotor.getMotorTemperature();
    inputs.flywheelMotorCurrent = flywheelMotor.getOutputCurrent();
    inputs.flywheelFollowerSpeed = flywheelEncoderFollower.getVelocity();
    inputs.flywheelMotorFollowerTemperature = flywheelMotorFollower.getMotorTemperature();
    inputs.flywheelMotorFollowerCurrent = flywheelMotorFollower.getOutputCurrent();
  }
}
