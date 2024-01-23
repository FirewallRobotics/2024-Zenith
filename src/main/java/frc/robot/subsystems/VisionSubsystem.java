package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

  public NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public NetworkTable apriltag = inst.getTable("Apriltag");
  public NetworkTableEntry ntTagCenterX = apriltag.getEntry("Center_x");
  public NetworkTableEntry ntTagCenterY = apriltag.getEntry("Center_y");
  public NetworkTableEntry ntTagID = apriltag.getEntry("ID");
  public NetworkTableEntry ntPose = apriltag.getEntry("Pose");

  public double decelerationDistance = Constants.VisionConstants.kDecelerationDistance;

  public VisionSubsystem() {}

  @Override
  public void periodic() {

    double tagCenterX = ntTagCenterX.getDouble(0);
    double tagCenterY = ntTagCenterY.getDouble(0);
  }

  private double VisionAdjust(double difference, double targetRange) {
    // Distance from cone based on width
    if (Math.abs(difference) <= targetRange) // Close enough
    {
      return 0.0;
    } else if (difference < 0) // Too far
    {
      return 1 * DecelerationSpeed(difference, targetRange);
    } else // Too close
    {
      return -1 * DecelerationSpeed(difference, targetRange);
    }
  }

  private double DecelerationSpeed(double positionDifference, double targetRange) {
    double distanceFromTarget = Math.abs(positionDifference) - targetRange;
    double speed = (distanceFromTarget / decelerationDistance) * 9.0 / 10.0 + 0.1;

    if (speed < 1) // Max value for speed is 1
    {
      return speed;
    } else {
      return 1.0;
    }
  }
}