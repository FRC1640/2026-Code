package frc.robot.util.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class AutonBuilder {

  private static AutonBuilder instance;

  public record Auton(Command command, Path firstPath, RobotCommands robotCommands) {
    public Auton(Command command, Path firstPath, RobotCommands robotCommands) {
      this.robotCommands = robotCommands;
      this.firstPath = firstPath;
      this.command = Commands.sequence(robotCommands.setSteerPositionCommand(firstPath != null ? firstPath.getInitialModuleDirection() : Rotation2d.fromDegrees(180)), command);
      
    }
  }

  public final HashMap<String, Auton> autons = new HashMap<String, Auton>();

  public AutonBuilder(RobotCommands robotCommands, FollowPath.Builder pathBuilder) {
    AutonBuilder.instance = this;

    // custom format

    /*--------
    | AUTONS |
    --------*/

    // None
    autons.put("None", new Auton(Commands.none(), null, robotCommands));

    // Example: Preload -> Near Hub -> Shoot for 8 seconds -> Outpost
    autons.put("Example", new Auton(
        Commands.sequence(
            // robotCommands.setSteerPositionCommand(new
            // Path("e1").getInitialModuleDirection()),
            pathBuilder.build(new Path("e1")),
            Commands.deadline(
                new WaitCommand(8),
                robotCommands.autoShootCommand()),
            pathBuilder.build(new Path("e2"))),
        new Path("e1"), robotCommands));

    // Leave: Leave Starting Line
    autons.put("Leave", new Auton(
        Commands.sequence(
            // robotCommands.setSteerPositionCommand(new
            // Path("l1").getInitialModuleDirection()),
            pathBuilder.build(new Path("l1"))),
        new Path("l1"), robotCommands));

    autons.put("Double Sweep Outpost",
        new Auton(
            Commands.sequence(
                pathBuilder.build(new Path("outpost sweep 2")),
                Commands.deadline(robotCommands.waitForShotCommand(true),
                    pathBuilder.build(new Path("outpost sweep 3"))),
                Commands.parallel(
                    robotCommands.autoShootCommand().withTimeout(6),
                    Commands.sequence(new WaitCommand(0.75), robotCommands.autoOscillateCommand()))),
            new Path("outpost sweep 2"), robotCommands));
    autons.put("Double Sweep Depot",
        new Auton(
            Commands.sequence(
                Commands.deadline(robotCommands.waitForShotCommand(true),
                    pathBuilder.build(new Path("double sweep depot 1"))),
                Commands.parallel(
                    robotCommands.autoShootCommand().withTimeout(6),
                    Commands.sequence(new WaitCommand(0.75), robotCommands.autoOscillateCommand())),
                Commands.deadline(robotCommands.waitForShotCommand(true),
                    pathBuilder.build(new Path("double sweep depot 2"))),
                Commands.parallel(
                    robotCommands.autoShootCommand().withTimeout(6),
                    Commands.sequence(new WaitCommand(0.75), robotCommands.autoOscillateCommand()))),
            new Path("double sweep depot 1"), robotCommands));

    // TODO: add autons here!!!! MAKE SURE YOU PRESERVE THE HOOD AND PROPER SHOOTERIDLE USE.

    // spotless format

  }

  public static AutonBuilder getInstance() {
    return instance;
  }
}
