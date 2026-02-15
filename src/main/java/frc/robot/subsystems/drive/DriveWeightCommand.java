package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.weights.DriveWeight;

public class DriveWeightCommand {
  static ArrayList<DriveWeight> persistentWeights = new ArrayList<>();

  static ArrayList<DriveWeight> weights = new ArrayList<>();

  public static Command create(DriveSubsystem driveSubsystem, BooleanSupplier limitSpeeds) {
    Command c = driveSubsystem.runVelocityCommand(() -> getAllSpeeds(), limitSpeeds);
    return c;
  }

  public static void addWeight(DriveWeight weight) {
    if (!weights.contains(weight)) {
      weights.add(weight);
    }
  }

  public static void removeWeight(DriveWeight weight) {
    if (weights.contains(weight)) {
      weights.remove(weight);
    }
  }

  public static void addPersistentWeight(DriveWeight weight) {
    if (!persistentWeights.contains(weight)) {
      persistentWeights.add(weight);
    }
  }

  public static void removePersistentWeight(DriveWeight weight) {
    if (persistentWeights.contains(weight)) {
      persistentWeights.remove(weight);
    }
  }

  public static void removeAllWeights() {
    weights.clear();
  }

  private static ChassisSpeeds getAllSpeeds() {
    ChassisSpeeds speeds = new ChassisSpeeds();
    // remove weights with a true cancel condition
    Iterator<DriveWeight> iterator = weights.iterator();
    while (iterator.hasNext()) {
      DriveWeight weight = iterator.next();
      if (weight.cancelCondition()) {
        iterator.remove();
      }
    }
    double totalVx = 0;
    double totalVy = 0;
    double totalOmega = 0;

    double weightSumVx = 0;
    double weightSumVy = 0;
    double weightSumOmega = 0;
    // iterate over remaining weights and add speeds
    for (DriveWeight driveWeight : weights) {
      ChassisSpeeds outputSpeeds = driveWeight.getSpeeds();
      Vector<N3> weight = driveWeight.getWeight();
      totalVx += outputSpeeds.vxMetersPerSecond * weight.get(0);
      totalVy += outputSpeeds.vyMetersPerSecond * weight.get(1);
      totalOmega += outputSpeeds.omegaRadiansPerSecond * weight.get(2);

      weightSumVx += weight.get(0);
      weightSumVy += weight.get(1);
      weightSumOmega += weight.get(2);
    }
    for (DriveWeight driveWeight : persistentWeights) {
      ChassisSpeeds outputSpeeds = driveWeight.getSpeeds();
      Vector<N3> weight = driveWeight.getWeight();
      totalVx += outputSpeeds.vxMetersPerSecond * weight.get(0);
      totalVy += outputSpeeds.vyMetersPerSecond * weight.get(1);
      totalOmega += outputSpeeds.omegaRadiansPerSecond * weight.get(2);

      weightSumVx += weight.get(0);
      weightSumVy += weight.get(1);
      weightSumOmega += weight.get(2);

      Logger.recordOutput("Weight/outputSpeeds", outputSpeeds);
      Logger.recordOutput("Weight/weight", weight);
    }
    Logger.recordOutput("Weight/totalVx", totalVx);
    Logger.recordOutput("Weight/totalVy", totalVy);
    Logger.recordOutput("Weight/totalOmega", totalOmega);
    Logger.recordOutput("Weight/weightSumVx", weightSumVx);
    Logger.recordOutput("Weight/weightSumVy", weightSumVy);
    Logger.recordOutput("Weight/weightSumOmega", weightSumOmega);

    for (int i = 0; i < persistentWeights.size(); i++) {
      Logger.recordOutput("Weight/persistent/" + i, persistentWeights.get(i).toString());
    }
    for (int i = 0; i < weights.size(); i++) {
      Logger.recordOutput("Weight/regular/" + i, weights.get(i).toString());
    }

    speeds = new ChassisSpeeds(totalVx / weightSumVx, totalVy / weightSumVy, totalOmega / weightSumOmega);

    return decreaseSpeeds(speeds);
  }

  public static int getWeightsSize() {
    return weights.size() + persistentWeights.size();
  }

  public static ChassisSpeeds decreaseSpeeds(ChassisSpeeds speeds) {
    double max = Math.max(
        Math.hypot(speeds.vxMetersPerSecond / DriveConstants.maxSpeed,
            speeds.vyMetersPerSecond / DriveConstants.maxSpeed),
        speeds.omegaRadiansPerSecond / DriveConstants.maxOmega);
    if (max > 1) {
      return speeds.times(1 / max);
      // System.out.println(speeds);
    }

    return speeds;
  }

  public static Trigger createWeightTrigger(DriveWeight weight, BooleanSupplier condition) {
    new Trigger(() -> weight.cancelCondition()).onTrue(new InstantCommand(() -> removeWeight(weight)));
    return new Trigger(condition).onTrue(new InstantCommand(() -> {
      addWeight(weight);
      weight.onStart();
    })).onFalse(new InstantCommand(() -> {
      removeWeight(weight);
      weight.onFinish();
    }));
  }

  /*
   * public static void createWeightTrigger(DriveWeight weight, BooleanSupplier
   * condition, BooleanSupplier startRequirement) { new Trigger(() ->
   * weight.cancelCondition()) .onTrue(new InstantCommand(() ->
   * removeWeight(weight))); new Trigger(() -> condition.getAsBoolean() &&
   * startRequirement.getAsBoolean()) .onTrue(new InstantCommand(() ->
   * {addWeight(weight); weight.onStart();})); new Trigger(condition) .onFalse(new
   * InstantCommand(() -> removeWeight(weight))); }
   */

  public static boolean checkWeight(DriveWeight weight) {
    return weights.contains(weight) || persistentWeights.contains(weight);
  }
}
