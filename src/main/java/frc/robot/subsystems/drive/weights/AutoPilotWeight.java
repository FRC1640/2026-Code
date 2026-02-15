package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoPilotWeight implements DriveWeight {

    Supplier<APTarget> target;
    Supplier<DriveSubsystem> driveSubsystemSupplier;
    Supplier<Pose2d> robotPose;
    Autopilot.APResult out;
    PIDController controller;

    public AutoPilotWeight(Supplier<APTarget> target, Supplier<Pose2d> robotPose,
            Supplier<DriveSubsystem> driveSubsystem) {
        this.target = target;
        this.driveSubsystemSupplier = driveSubsystem;
        this.robotPose = robotPose;
        this.controller = RobotPIDConstants.constructPID(RobotPIDConstants.autopilotTurnPID);
        this.controller.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    @AutoLogOutput(key = "test")
    public ChassisSpeeds getSpeeds() {
        if (!RobotConstants.AutopilotConstants.kAutopilot.atTarget(robotPose.get(), target.get())) {
            out = RobotConstants.AutopilotConstants.kAutopilot.calculate(robotPose.get(),
                    driveSubsystemSupplier.get().getChassisSpeeds(), target.get());
            return new ChassisSpeeds(out.vx(), out.vy(), RadiansPerSecond.of(controller.calculate(robotPose.get().getRotation().getRadians(), out.targetAngle().getRadians())));

        }
        return new ChassisSpeeds();
    }
    @Override
    public double getWeight() {
      return 5;
    }

}
