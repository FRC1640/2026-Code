package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class FlywheelIOSim implements FlywheelIO {
  private DCMotorSim flywheelMotor;
  private PIDController velocityController;

  public FlywheelIOSim() {
    DCMotor gearboxSim = DCMotor.getNEO(1);
    flywheelMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearboxSim, 0.0002, 1), gearboxSim);
    velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.flywheelVelocityPidSim);
  }

  @Override
  public void setFlywheelSpeed(double speedRadPerSec) {
    double outputVolts = velocityController.calculate(flywheelMotor.getAngularVelocityRadPerSec(), speedRadPerSec);
    flywheelMotor.setInputVoltage(VoltageLim.clampVoltage(outputVolts));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    flywheelMotor.update(0.02);

    inputs.flywheelSpeed = flywheelMotor.getAngularVelocityRadPerSec();
    inputs.flywheelFollowerSpeed = flywheelMotor.getAngularVelocityRadPerSec();
    inputs.flywheelMotorCurrent = flywheelMotor.getCurrentDrawAmps();
    inputs.flywheelMotorFollowerCurrent = flywheelMotor.getCurrentDrawAmps();
    inputs.flywheelMotorVoltage = flywheelMotor.getInputVoltage();
    inputs.flywheelMotorFollowerVoltage = flywheelMotor.getInputVoltage();
    inputs.flywheelMotorTemperature = 0;
    inputs.flywheelMotorFollowerTemperature = 0;
  }
}
