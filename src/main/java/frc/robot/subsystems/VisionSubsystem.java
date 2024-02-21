package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Set;
import org.json.JSONArray;
import org.json.JSONObject;

public class VisionSubsystem extends SubsystemBase {

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable aprilTagsTable = inst.getTable("PiDetector");

  private NetworkTable ringTable = inst.getTable("RingFinder");

  private double decelerationDistance = Constants.VisionConstants.kDecelerationDistance;
  private double[] declareRingPosNeeded = Constants.VisionConstants.kCenterOfScreen;

  private Set<String> tags;

  private final String[] speakerTags = {"6", "5"};
  private final String speakerDistanceToTagName = "Dist";
  private final String speakerCenterName = "Center";
  private final String speakerRotationName = "XYZ";

  private final String ringKey = "FoundRing";

  private final String[] ampTags = {"9", "1"};
  private final String ampCenterName = "Center";

  public VisionSubsystem() {}

  @Override
  public void periodic() {

    tags = aprilTagsTable.getKeys();

    if (checkForSpeakerTag()) {
      System.out.println("Distance to Tag -> " + getSpeakerTagDistanceToTag());
      System.out.println("CenterX -> " + getSpeakerTagCenterX());
      System.out.println("RotationZ -> " + getSpeakerTagRotationZ());
    } else {
      System.out.print("No Speaker Tag - Current Tags: ");
      printAllTags();
    }
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

  private void printAllTags() {
    String tagStr = "Tags: ";

    for (String tag : tags) {
      tagStr += tag + ", ";
    }

    System.out.println(tagStr);
  }

  private void testTag(String tagName) {
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

  public float getPixelX() {
    Boolean orangeCheckBoolean = OrangeCheckInView();

    if (orangeCheckBoolean) {
      JSONObject jsonObj = new JSONObject(ringTable.getEntry(ringKey).getString("{\"X\": 0}"));

      return jsonObj.getFloat("X");
    }

    return -1;
  }

  public float getPixelY() {
    Boolean orangeCheckBoolean = OrangeCheckInView();

    if (orangeCheckBoolean) {
      JSONObject jsonObj = new JSONObject(ringTable.getEntry(ringKey).getString("{\"Y\": 0}"));

      return jsonObj.getFloat("Y");
    }

    return -1;
  }

  private boolean OrangeCheckInView() {

    if (ringTable.containsKey(ringKey)) {
      return true;
    }
    return false;
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
