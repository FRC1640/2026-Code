package frc.robot.subsystems.climber;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class ClimberIOSim implements ClimberIO {

    private final SparkFlexSim m_motorSim;
    private final SparkAbsoluteEncoder m_encoder;
    private final SparkAbsoluteEncoderSim m_encoderSim;
    private final SparkFlex m_motor;
    private final SparkClosedLoopController m_positionController;

    public ClimberIOSim() {
        DCMotor motorGearboxSim = DCMotor.getNeoVortex(1);
        m_motor = SparkConfigurer.configSparkFlex(ClimberConstants.canId, SparkConstants.climberConfig);

        m_motorSim = new SparkFlexSim(m_motor, motorGearboxSim);
        m_encoder = m_motor.getAbsoluteEncoder();
        m_encoderSim = new SparkAbsoluteEncoderSim(m_motor);
        m_positionController = m_motor.getClosedLoopController();
    }

    @Override
    public void setPosition(double position) {
        m_positionController.setSetpoint(position, null, null);

    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        m_motorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                        0),
                RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                0.02);
        inputs.encoderVelocity = m_encoder.getVelocity();
        inputs.motorCurrent = m_motor.getOutputCurrent();
        inputs.motorTemperature = m_motor.getMotorTemperature();
        inputs.motorVoltage = m_motor.getBusVoltage();
    }
}
