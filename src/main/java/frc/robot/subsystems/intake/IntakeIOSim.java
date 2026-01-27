package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.limits.VoltageLim;
import frc.robot.constants.RobotPIDConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeMotor;
  private PIDController intakePID = RobotPIDConstants.constructPID(RobotPIDConstants.intakeSim);

  public IntakeIOSim() {
    DCMotor simGearbox = DCMotor.getNEO(1);
    intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(simGearbox, 0.00019125, IntakeConstants.gearRatio), simGearbox);
  }
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotor.update(0.02);

    // TODO: unit conversions
    inputs.motorTemperature = 0;
    inputs.motorCurrent = intakeMotor.getCurrentDrawAmps();
    inputs.motorVoltage = intakeMotor.getInputVoltage();
    inputs.encoderPosition = intakeMotor.getAngularPositionRad();
    inputs.encoderVelocity = intakeMotor.getAngularVelocityRadPerSec();
  }

  @Override
  public void setMotorVoltage(double voltage, IntakeIOInputs inputs) {
    intakeMotor.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setMotorPosition(double pos, IntakeIOInputs inputs) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    intakeMotor.setInputVoltage(
        VoltageLim.applyLimits(inputs.encoderPosition, intakePID.calculate(inputs.encoderPosition, pos),
            IntakeConstants.intakeLowerLimit, IntakeConstants.intakeUpperLimit));
  }
}
