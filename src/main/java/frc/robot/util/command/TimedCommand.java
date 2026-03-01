package frc.robot.util.command;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TimedCommand extends Command {
  Timer t;
  Consumer<Double> toRun;

  public TimedCommand(Consumer<Double> toRun, Subsystem... requirements) {
    super();
    this.toRun = toRun;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    t = new Timer();
    t.start();
  }

  @Override
  public void execute() {
    toRun.accept(t.get());
  }

  @Override
  public void end(boolean interrupted) {
    t.stop();
    t.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
