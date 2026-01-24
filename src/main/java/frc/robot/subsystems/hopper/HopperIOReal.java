package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;

import frc.robot.constants.SparkConstants;
import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfigurer;

public class HopperIOReal implements HopperIO {
  private SparkMax hopperMotor;

  public HopperIOReal() {
    hopperMotor = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(HopperConstants.canId, false));
  }

  @Override
  public void setHopperVoltage(double voltage) {
    hopperMotor.setVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.hopperMotorCurrent = hopperMotor.getOutputCurrent();
    inputs.hopperMotorVoltage = hopperMotor.getAppliedOutput() * hopperMotor.getBusVoltage();
    inputs.hopperMotorTemperature = hopperMotor.getMotorTemperature();
  }
}
