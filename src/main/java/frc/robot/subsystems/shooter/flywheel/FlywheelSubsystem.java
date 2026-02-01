package frc.robot.subsystems.shooter.flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motorDashboard.DashboardInterface;

public class FlywheelSubsystem extends SubsystemBase implements DashboardInterface {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
  }

  public Command setFlywheelSpeed(DoubleSupplier speed){
    return run(()-> io.setFlywheelSpeed(speed.getAsDouble())).finallyDo(this::stop);
  }

  public void stop(){
    io.setFlywheelSpeed(0);
  }
  public void stopVoltage(){
    io.setFlywheelVoltage(0);
  }
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public Command runVoltageCommand(DoubleSupplier voltage){
    return run(()-> io.setFlywheelVoltage(voltage.getAsDouble())).finallyDo(this::stopVoltage);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier joystickValue) {
    return runVoltageCommand(()-> joystickValue.getAsDouble()*-8);
  }

  @Override
  public String getName() {
    return "Flywheel Subsystem";
  }
}
