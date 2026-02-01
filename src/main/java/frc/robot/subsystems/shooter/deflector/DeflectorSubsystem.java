package frc.robot.subsystems.shooter.deflector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motorDashboard.DashboardInterface;

public class DeflectorSubsystem extends SubsystemBase implements DashboardInterface {
  private DeflectorIO io;
  private DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  public DeflectorSubsystem(DeflectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deflector", inputs);
  }

  public Command runVoltageCommand(DoubleSupplier voltage){
    return run(()-> io.setDeflectorMotorVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  private void stop() {
    io.setDeflectorMotorVoltage(0);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier joystickValue) {
    return runVoltageCommand(()-> joystickValue.getAsDouble()*-8);
  }

  @Override
  public String getName() {
    return "Deflector Subsystem";
  }
}
