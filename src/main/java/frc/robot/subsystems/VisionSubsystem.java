package frc.robot.subsystems;

import edu.wpi.first.networktables.FloatArrayEntry;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.FloatEntry;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Set;
import org.json.JSONArray;
import org.json.JSONObject;

public class VisionSubsystem extends SubsystemBase {

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable aprilTagsTable = inst.getTable("PiDetector");

  private NetworkTable ringTable = inst.getTable("RingFinder");

  private static NetworkTable Unicorntable = inst.getTable("UnicornHatRIO");
  static final StringPublisher dblPub = Unicorntable.getStringTopic("ToUnicornStatus").publish();

  private double decelerationDistance = Constants.VisionConstants.kDecelerationDistance;
  private double[] declareRingPosNeeded = Constants.VisionConstants.kCenterOfScreen;

  private Set<String> tags;

  private final String[] speakerTags = {"4", "7"};
  private final String speakerDistanceToTagName = "Dist";
  private final String speakerCenterName = "Center";
  private final String speakerRotationName = "XYZ";

  private final String ringKey = "Rings";

  private final String[] ampTags = {"9", "1"};
  private final String ampCenterName = "Center";

  public VisionSubsystem() {}

  @Override
  public void periodic() {

    tags = aprilTagsTable.getKeys();

    if (checkForSpeakerTag()) {
      System.out.println("CurrSpeakerTag -> " + findSpeakerTagInView());
      System.out.println("Distance to Tag -> " + getSpeakerTagDistanceToTag());
      System.out.println("CenterX -> " + getSpeakerTagCenterX());
      System.out.println("RotationZ -> " + getSpeakerTagRotationZ());
    } else {
      // System.out.print("No Speaker Tag - Current Tags: ");
      // printAllTags();
    }
  }

  public boolean checkForSpeakerTag() {
    for (String tagNum : speakerTags) {      
      if (aprilTagsTable.containsKey(tagNum + "-" + speakerCenterName)) {
        return true;
      }
    }

    return false;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagDistanceToTag() {
    String tagNum = findSpeakerTagInView();

    if (tagNum != null) {
      FloatTopic floatTopic = aprilTagsTable.getFloatTopic(tagNum + "-" + speakerDistanceToTagName);
      FloatEntry floatEntry = floatTopic.getEntry(0);
      return floatEntry.get();
    }

    return 0;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagCenterX() {
    String tagNum = findSpeakerTagInView();

    if (tagNum != null) {
      FloatArrayTopic arrayTopic = aprilTagsTable.getFloatArrayTopic(tagNum + "-" + speakerCenterName);
      FloatArrayEntry arrayEntry = arrayTopic.getEntry(new float[] {0,0});
      float[] array = arrayEntry.get();
      return array[0];
    }

    return 0;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagRotationZ() {
    String tagNum = findSpeakerTagInView();

    if (tagNum != null) {
      FloatArrayTopic arrayTopic = aprilTagsTable.getFloatArrayTopic(tagNum + "-" + speakerRotationName);
      FloatArrayEntry arrayEntry = arrayTopic.getEntry(new float[] {0,0,0});
      float[] array = arrayEntry.get();
      return array[2];
    }

    return 0;
  }

  private String findSpeakerTagInView() {
    for (String tagNum : speakerTags) {
      if (aprilTagsTable.containsKey(tagNum + "-" + speakerCenterName)) {
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

  // Communication For The Unicornhat
  public static void UnicornNotify(String status) {
    dblPub.set(status);
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
    boolean orangeCheckBoolean = OrangeCheckInView();

    if (orangeCheckBoolean) {
      FloatArrayTopic arrayTopic = aprilTagsTable.getFloatArrayTopic(ringKey);
      FloatArrayEntry arrayEntry = arrayTopic.getEntry(new float[] {0,0});
      float[] array = arrayEntry.get();
      return array[0];
    }

    return -1;
  }

  public float getPixelY() {
    boolean orangeCheckBoolean = OrangeCheckInView();

    if (orangeCheckBoolean) {
      FloatArrayTopic arrayTopic = aprilTagsTable.getFloatArrayTopic(ringKey);
      FloatArrayEntry arrayEntry = arrayTopic.getEntry(new float[] {0,0});
      float[] array = arrayEntry.get();
      return array[1];
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
    String tagNum = findAmpTagInView();

    if (tagNum != null) {
      FloatArrayTopic arrayTopic = aprilTagsTable.getFloatArrayTopic(tagNum + "-" + ampCenterName);
      FloatArrayEntry arrayEntry = arrayTopic.getEntry(new float[] {0,0});
      float[] array = arrayEntry.get();
      return array[0];
    }

    return 0;
  }

  public float getAmpTagCenterY() {
    String tagNum = findAmpTagInView();

    if (tagNum != null) {
      FloatArrayTopic arrayTopic = aprilTagsTable.getFloatArrayTopic(tagNum + "-" + ampCenterName);
      FloatArrayEntry arrayEntry = arrayTopic.getEntry(new float[] {0,0});
      float[] array = arrayEntry.get();
      return array[1];
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
