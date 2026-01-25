package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
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
}
