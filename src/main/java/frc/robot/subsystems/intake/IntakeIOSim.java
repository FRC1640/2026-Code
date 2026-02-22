package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim m_motor;
  private final PIDController m_positionController = RobotPIDConstants.constructPID(RobotPIDConstants.intakeSim);
  Mechanism2d mech;
  // the mechanism root node
  MechanismRoot2d root;
  MechanismLigament2d intakeLigament;

  public IntakeIOSim() {
    DCMotor gearbox = DCMotor.getNEO(1);
    m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.00019125, IntakeConstants.gearRatio),
        gearbox);
    m_motor.setAngle(IntakeConstants.upPosition);
    mech = new Mechanism2d(3, 3);
    root = mech.getRoot("intake", 1.5, 0);
    intakeLigament = root.append(new MechanismLigament2d("intake", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData("IntakeMech", mech);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    m_motor.update(0.02);

    // TODO: unit conversions
    inputs.motorTemperatureCelsius = 0; // degrees celsius
    inputs.motorCurrent = m_motor.getCurrentDrawAmps(); // amps
    inputs.motorVoltage = m_motor.getInputVoltage(); // volts
    inputs.encoderPositionRadians = m_motor.getAngularPositionRad(); // radians
    inputs.encoderVelocityRadiansPerSecond = m_motor.getAngularVelocityRadPerSec(); // rad/s
    intakeLigament.setAngle(Units.radiansToDegrees(m_motor.getAngularPositionRad()));
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void setPosition(double pos) {
    Logger.recordOutput("Subsystems/Intake/Setpoint", pos);
    setVoltage(IntakeConstants.positionLimits.clampOutput(m_motor.getAngularPositionRad(),
        VoltageLim.clampVoltage(m_positionController.calculate(m_motor.getAngularPositionRad(), pos))));
  }
}
