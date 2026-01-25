package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AutoAlignConfig {
  public static final double maxDistanceFromTarget = 0.3;
  public static final PathConstraints coralStationPathConstraints = new PathConstraints(3, 3.5, Math.PI + 0.75,
      4 * Math.PI);
  public static final PathConstraints coralStationPathConstraintsSlow = new PathConstraints(2, 3, Math.PI + 0.75,
      4 * Math.PI);
  public static final PathConstraints pathConstraints = new PathConstraints(2, 3, Math.PI, 4 * Math.PI);

  // local align
  public static final Constraints localAlignPpidConstraints = new Constraints(2, 2.5);
  public static final double profiledDistThreshold = 0.4;
}
