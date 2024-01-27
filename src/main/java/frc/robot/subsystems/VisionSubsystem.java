package frc.robot.subsystems;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Set;

import javax.xml.crypto.dsig.keyinfo.KeyInfo;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();  
  private NetworkTable aprilTagsTable = inst.getTable("AprilTagsTable");

  private double decelerationDistance = Constants.VisionConstants.kDecelerationDistance;

  private Set<String> tags; 

  public VisionSubsystem() {}

  @Override
  public void periodic() {

    tags = aprilTagsTable.getKeys();
    
  }

  private void printAllTags(){
    String tagStr = "";

    for(String tag : tags){
      tagStr += tag + ", ";
    }

    System.out.println(tagStr);
  }

  private void testTag(String tagName){
    System.out.println("We have " + tagName + ": " + tags.contains(tagName));
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