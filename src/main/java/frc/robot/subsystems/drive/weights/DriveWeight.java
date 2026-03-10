package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;

public interface DriveWeight {

  public ChassisSpeeds getSpeeds();

  public default Vector<N3> getWeight() {
    return VecBuilder.fill(1, 1, 1);
  }

  public default String getName() {
    return "DriveWeight" + toString();
  }

  public default void setWeight(double weight) {
  }

  public default boolean cancelCondition() {
    return false;
  }

  public default boolean isEnabled() {
    return true;
  }

  public default void onStart() {
  }

  public default void onFinish() {
  }
}
