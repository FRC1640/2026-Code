package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motorDashboard.DashboardInterface;

public class IndexerSubsystem extends SubsystemBase implements DashboardInterface {
  IndexerIO io;
  IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public IndexerSubsystem(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setIndexerMotorVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  private void stop() {
    io.setIndexerMotorVoltage(0);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier joystickValue) {
    return runVoltageCommand(()-> joystickValue.getAsDouble()*-8);
  }

  @Override
  public String getName() {
    return "Indexer Subsystem";
  }
}
