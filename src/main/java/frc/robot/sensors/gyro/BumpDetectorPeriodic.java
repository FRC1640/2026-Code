package frc.robot.sensors.gyro;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.periodic.PeriodicBase;

public class BumpDetectorPeriodic extends PeriodicBase {
  private double[] angle;
  private Gyro gyro;
  private final double cutoff;
  private final int len;

  public BumpDetectorPeriodic(Gyro gyro, int len, double cutoff) {
    this.gyro = gyro;
    angle = new double[len];
    this.len = len;
    this.cutoff = cutoff;

  }

  @Override
  public void periodic() {
    periodic(gyro.getPitch().getRadians(),gyro.getRoll().getRadians());
  }
  
  public void periodic(double pitch, double roll) {
    for (int i = len - 1; i > 0; i--) {
      angle[i] = angle[i - 1];
    }
    // custom formating
    Translation3d pitchVector = new Translation3d(
        Math.abs(Math.cos(pitch)), 
        0,
        Math.abs(Math.sin(pitch)));
    Translation3d rollVector = new Translation3d(
        0,
        Math.abs(Math.cos(roll)),
        Math.abs(Math.sin(roll)));
    // spotless formating
    angle[0] = Math.abs(Math.acos((pitchVector.cross(rollVector)).dot(new Vector<>(new SimpleMatrix(3,1,true,0,0,1)))));
  }

  public boolean get() {
    double max = 0;
    for (int i = 0; i < len; i++) {
      max = (angle[i] > max) ? angle[i] : max;
    }
    return (max > cutoff);
  }

  private double getDouble() {
    double max = 0;
    for (int i = 0; i < len; i++) {
      max = (angle[i] > max) ? angle[i] : max;
    }
    return (max);
  }

  public void test() {
    double[] testArray = {0,Math.PI/6,Math.PI/3,Math.PI/2,2*Math.PI/3,5*Math.PI/6,Math.PI,0,0,0};
    for (double i: testArray){
        System.out.println("\n"+i+":    ");
        for (double j: testArray){
            periodic(i,j);
            //System.out.print(j+": "+getDouble()+"    ");
            System.out.print(String.format("%.2f",j)+": "+String.format("%.2f",getDouble())+"      ");
        }
    }
  }

}
