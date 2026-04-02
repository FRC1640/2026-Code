package frc.robot.util.sysid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class CreateSysIdCommand {
  /**
   * @param quasistaticCommand
   * @param dynamicCommand
   * @param commandName
   * @param startNext
   *            starts next phase of routine when this is triggered
   * @param cancel
   *            pauses sysid routine when this is triggered
   * @param stopMotors
   * @return complete sysid command
   */
  public static Command createCommand(Function<SysIdRoutine.Direction, Command> quasistaticCommand,
      Function<SysIdRoutine.Direction, Command> dynamicCommand, String commandName, BooleanSupplier startNext,
      BooleanSupplier cancel, Runnable stopMotors) {
    // Each phase should run until the operator presses startNext (A).
    // If cancel (B) is pressed at any time the whole routine should stop.
    // Implementation:
    // - Each phase is a ParallelDeadlineGroup with a WaitUntilCommand(startNext)
    // so the phase runs until A is pressed.
    // - After each phase we run InstantCommand(stopMotors) and a short wait
    // to ensure motors are stopped before the next phase.
    // - The entire sequence is wrapped with .until(cancel) so B aborts the
    // routine immediately. Finally, ensure motors are stopped when the
    // command ends.

    // Run each phase for a fixed duration (2 seconds) instead of waiting for a
    // button press to advance.
    Command forwardPhase = new ParallelDeadlineGroup(new WaitUntilCommand(startNext),
        quasistaticCommand.apply(SysIdRoutine.Direction.kForward));
    forwardPhase.setName(commandName + " Quasistatic Forward");

    Command reversePhase = new ParallelDeadlineGroup(new WaitUntilCommand(startNext),
        quasistaticCommand.apply(SysIdRoutine.Direction.kReverse));
    reversePhase.setName(commandName + " Quasistatic Reverse");

    Command dynamicForwardPhase = new ParallelDeadlineGroup(new WaitUntilCommand(startNext),
        dynamicCommand.apply(SysIdRoutine.Direction.kForward));
    dynamicForwardPhase.setName(commandName + " Dynamic Forward");

    Command dynamicReversePhase = new ParallelDeadlineGroup(new WaitUntilCommand(startNext),
        dynamicCommand.apply(SysIdRoutine.Direction.kReverse));
    dynamicReversePhase.setName(commandName + " Dynamic Reverse");

    Command sysIdCommand = new SequentialCommandGroup(forwardPhase, new WaitCommand(0.5), reversePhase,
        new WaitCommand(0.5), dynamicForwardPhase, new WaitCommand(0.5), dynamicReversePhase).until(cancel)
            .finallyDo(stopMotors);
    sysIdCommand.setName(commandName);
    return sysIdCommand;
  }
}
