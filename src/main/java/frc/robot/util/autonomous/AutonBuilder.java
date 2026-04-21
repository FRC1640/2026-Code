package frc.robot.util.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class AutonBuilder {

    private static AutonBuilder instance;

    public record Auton(Command command, RobotCommands robotCommands) {
        public Auton(Command command, RobotCommands robotCommands) {
            this.robotCommands = robotCommands;
            this.command = // Commands.sequence(
                    // robotCommands.setSteerPositionCommand(
                    // firstPath != null ? firstPath.getInitialModuleDirection() :
                    // Rotation2d.fromDegrees(180)),
                    (command).finallyDo(AutonBuilder.getInstance().autoEndCallback);

        }
    }

    public final HashMap<String, Auton> autons = new HashMap<String, Auton>();

    public final Runnable autoEndCallback;

    public AutonBuilder(RobotCommands robotCommands, FollowPath.Builder pathBuilder, Runnable autoEndCallback) {
        AutonBuilder.instance = this;
        this.autoEndCallback = autoEndCallback;

        // custom format

        /*--------
        | AUTONS |
        --------*/

        // None
        autons.put("None", new Auton(Commands.none(), robotCommands));

        // custom format
        /*
         * autons.put("OTrench 1Sweep Depot",
         * new Auton(Commands.sequence(
         * pathBuilder.build(new Path("dt1sde1"))), robotCommands));
         */ // spotless format

        // custom format
        /*
         * autons.put("OTrench 1Sweep Outpost",
         * new Auton(Commands.sequence(
         * pathBuilder.build(new Path("ot1sou1"))), robotCommands));
         */ // spotless format

        autons.put("Center Outpost Depot", new Auton(Commands.sequence(pathBuilder.build(new Path("ceoude1")),
                new WaitCommand(2),
                new InstantCommand(
                        () -> CommandScheduler.getInstance().schedule(robotCommands.autoOscillateCommand(65, 0))),
                pathBuilder.build(new Path("ceoude2"))), robotCommands));

        autons.put("OTrench 2Sweep OBump",
                new Auton(
                        Commands.sequence(pathBuilder.build(new Path("ot2sob1")).finallyDo(
                                () -> CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
                        robotCommands));

        autons.put("DTrench 2Sweep DBump",
                new Auton(
                        Commands.sequence(pathBuilder.build(new Path("dt2sdb1")).finallyDo(
                                () -> CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
                        robotCommands));

        // TODO: add autons here!!!! MAKE SURE YOU PRESERVE THE HOOD AND PROPER
        // SHOOTERIDLE USE.

        // spotless format

    }

    public static AutonBuilder getInstance() {
        return instance;
    }
}
