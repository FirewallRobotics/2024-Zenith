package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import org.json.JSONArray;
import org.json.JSONObject;

public class VisionSubsystem extends SubsystemBase {

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable aprilTagsTable = inst.getTable("PiDetector");

  NetworkTable Limetable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = Limetable.getEntry("tx");
  NetworkTableEntry ty = Limetable.getEntry("ty");
  NetworkTableEntry ta = Limetable.getEntry("ta");

  private static NetworkTable Unicorntable = inst.getTable("UnicornHatRIO");
  static final StringPublisher dblPub = Unicorntable.getStringTopic("ToUnicornStatus").publish();

  private final String[] speakerTags = {"4", "7"};
  private final String speakerDistanceToTagName = "Dist";
  private final String speakerCenterName = "Center";
  private final String speakerRotationName = "XYZ";

  private final String[] ampTags = {"9", "1"};
  private final String ampCenterName = "Center";
  public final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  public VisionSubsystem() {
    int[] validIDs = {3, 4};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
  }

  @Override
  public void periodic() {
    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public boolean checkForSpeakerTag() {
    for (String tagNum : speakerTags) {
      if (aprilTagsTable.containsKey(tagNum)) {
        return true;
      }
    }

    return false;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagDistanceToTag() {
    String tag = findSpeakerTagInView();

    if (tag != null) {
      JSONObject jsonObj =
          new JSONObject(
              aprilTagsTable.getEntry(tag).getString("{\"" + speakerDistanceToTagName + "\": 0}"));

      return jsonObj.getFloat(speakerDistanceToTagName);
    }

    return 0;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagCenterX() {
    String tag = findSpeakerTagInView();

    if (tag != null) {
      JSONObject jsonObj =
          new JSONObject(
              aprilTagsTable.getEntry(tag).getString("{\"" + speakerCenterName + "\": [0,0]}"));

      JSONArray centerArray = jsonObj.getJSONArray(speakerCenterName);

      return centerArray.getFloat(0);
    }

    return 0;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagRotationZ() {
    String tag = findSpeakerTagInView();

    if (tag != null) {
      JSONObject jsonObj =
          new JSONObject(
              aprilTagsTable.getEntry(tag).getString("{\"" + speakerRotationName + "\": [0,0,0]}"));

      JSONArray rotationArray = jsonObj.getJSONArray(speakerRotationName);

      return rotationArray.getFloat(2);
    }

    return 0;
  }

  private String findSpeakerTagInView() {
    for (String tagNum : speakerTags) {
      if (aprilTagsTable.containsKey(tagNum)) {
        return tagNum;
      }
    }

    return null;
  }

  // Communication For The Unicornhat
  public static void UnicornNotify(String status) {
    dblPub.set(status);
  }

  public float getAmpTagCenterX() {
    String tag = findAmpTagInView();

    if (tag != null) {
      JSONObject jsonObj =
          new JSONObject(
              aprilTagsTable.getEntry(tag).getString("{\"" + ampCenterName + "\": [0,0]}"));

      JSONArray centerArray = jsonObj.getJSONArray(ampCenterName);

      return centerArray.getFloat(0);
    }

    return 0;
  }

  public float getAmpTagCenterY() {
    String tag = findAmpTagInView();

    if (tag != null) {
      JSONObject jsonObj =
          new JSONObject(
              aprilTagsTable.getEntry(tag).getString("{\"" + ampCenterName + "\": [0,0]}"));

      JSONArray centerArray = jsonObj.getJSONArray(ampCenterName);

      return centerArray.getFloat(1);
    }

    return 0;
  }

  private String findAmpTagInView() {
    for (String tagNum : ampTags) {
      if (aprilTagsTable.containsKey(tagNum)) {
        return tagNum;
      }
    }

    return null;
  }
}
