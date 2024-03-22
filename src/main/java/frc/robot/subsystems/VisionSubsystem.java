package frc.robot.subsystems;

import edu.wpi.first.networktables.FloatArrayEntry;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.FloatEntry;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.reflect.Array;
import java.util.Set;
import org.json.JSONArray;
import org.json.JSONObject;

public class VisionSubsystem extends SubsystemBase {

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable aprilTagsTable = inst.getTable("PiDetector");

  //place for future growth?
  FloatTopic Dist1 = aprilTagsTable.getFloatTopic("1-Dist");
  FloatTopic Dist2 = aprilTagsTable.getFloatTopic("2-Dist");
  FloatTopic Dist3 = aprilTagsTable.getFloatTopic("3-Dist");
  FloatTopic Dist4 = aprilTagsTable.getFloatTopic("4-Dist");
  FloatTopic Dist5 = aprilTagsTable.getFloatTopic("5-Dist");
  FloatTopic Dist6 = aprilTagsTable.getFloatTopic("6-Dist");
  FloatTopic Dist7 = aprilTagsTable.getFloatTopic("7-Dist");
  FloatTopic Dist8 = aprilTagsTable.getFloatTopic("8-Dist");
  FloatTopic Dist9 = aprilTagsTable.getFloatTopic("9-Dist");
  FloatTopic Dist10 = aprilTagsTable.getFloatTopic("10-Dist");
  FloatTopic Dist11 = aprilTagsTable.getFloatTopic("11-Dist");
  FloatTopic Dist12 = aprilTagsTable.getFloatTopic("12-Dist");
  FloatTopic Dist13 = aprilTagsTable.getFloatTopic("13-Dist");
  FloatTopic Dist14 = aprilTagsTable.getFloatTopic("14-Dist");
  FloatTopic Dist15 = aprilTagsTable.getFloatTopic("15-Dist");
  FloatTopic Dist16 = aprilTagsTable.getFloatTopic("16-Dist");

  FloatTopic TimeSeen1 = aprilTagsTable.getFloatTopic("1-TimeSeen");
  FloatTopic TimeSeen2 = aprilTagsTable.getFloatTopic("2-TimeSeen");
  FloatTopic TimeSeen3 = aprilTagsTable.getFloatTopic("3-TimeSeen");
  FloatTopic TimeSeen4 = aprilTagsTable.getFloatTopic("4-TimeSeen");
  FloatTopic TimeSeen5 = aprilTagsTable.getFloatTopic("5-TimeSeen");
  FloatTopic TimeSeen6 = aprilTagsTable.getFloatTopic("6-TimeSeen");
  FloatTopic TimeSeen7 = aprilTagsTable.getFloatTopic("7-TimeSeen");
  FloatTopic TimeSeen8 = aprilTagsTable.getFloatTopic("8-TimeSeen");
  FloatTopic TimeSeen9 = aprilTagsTable.getFloatTopic("9-TimeSeen");
  FloatTopic TimeSeen10 = aprilTagsTable.getFloatTopic("10-TimeSeen");
  FloatTopic TimeSeen11 = aprilTagsTable.getFloatTopic("11-TimeSeen");
  FloatTopic TimeSeen12 = aprilTagsTable.getFloatTopic("12-TimeSeen");
  FloatTopic TimeSeen13 = aprilTagsTable.getFloatTopic("13-TimeSeen");
  FloatTopic TimeSeen14 = aprilTagsTable.getFloatTopic("14-TimeSeen");
  FloatTopic TimeSeen15 = aprilTagsTable.getFloatTopic("15-TimeSeen");
  FloatTopic TimeSeen16 = aprilTagsTable.getFloatTopic("16-TimeSeen");

  FloatArrayTopic Center1 = aprilTagsTable.getFloatArrayTopic("1-Center");
  FloatArrayTopic Center2 = aprilTagsTable.getFloatArrayTopic("2-Center");
  FloatArrayTopic Center3 = aprilTagsTable.getFloatArrayTopic("3-Center");
  FloatArrayTopic Center4 = aprilTagsTable.getFloatArrayTopic("4-Center");
  FloatArrayTopic Center5 = aprilTagsTable.getFloatArrayTopic("5-Center");
  FloatArrayTopic Center6 = aprilTagsTable.getFloatArrayTopic("6-Center");
  FloatArrayTopic Center7 = aprilTagsTable.getFloatArrayTopic("7-Center");
  FloatArrayTopic Center8 = aprilTagsTable.getFloatArrayTopic("8-Center");
  FloatArrayTopic Center9 = aprilTagsTable.getFloatArrayTopic("9-Center");
  FloatArrayTopic Center10 = aprilTagsTable.getFloatArrayTopic("10-Center");
  FloatArrayTopic Center11 = aprilTagsTable.getFloatArrayTopic("11-Center");
  FloatArrayTopic Center12 = aprilTagsTable.getFloatArrayTopic("12-Center");
  FloatArrayTopic Center13 = aprilTagsTable.getFloatArrayTopic("13-Center");
  FloatArrayTopic Center14 = aprilTagsTable.getFloatArrayTopic("14-Center");
  FloatArrayTopic Center15 = aprilTagsTable.getFloatArrayTopic("15-Center");
  FloatArrayTopic Center16 = aprilTagsTable.getFloatArrayTopic("16-Center");

  FloatArrayTopic XYZ1 = aprilTagsTable.getFloatArrayTopic("1-XYZ");
  FloatArrayTopic XYZ2 = aprilTagsTable.getFloatArrayTopic("2-XYZ");
  FloatArrayTopic XYZ3 = aprilTagsTable.getFloatArrayTopic("3-XYZ");
  FloatArrayTopic XYZ4 = aprilTagsTable.getFloatArrayTopic("4-XYZ");
  FloatArrayTopic XYZ5 = aprilTagsTable.getFloatArrayTopic("5-XYZ");
  FloatArrayTopic XYZ6 = aprilTagsTable.getFloatArrayTopic("6-XYZ");
  FloatArrayTopic XYZ7 = aprilTagsTable.getFloatArrayTopic("7-XYZ");
  FloatArrayTopic XYZ8 = aprilTagsTable.getFloatArrayTopic("8-XYZ");
  FloatArrayTopic XYZ9 = aprilTagsTable.getFloatArrayTopic("9-XYZ");
  FloatArrayTopic XYZ10 = aprilTagsTable.getFloatArrayTopic("10-XYZ");
  FloatArrayTopic XYZ11 = aprilTagsTable.getFloatArrayTopic("11-XYZ");
  FloatArrayTopic XYZ12 = aprilTagsTable.getFloatArrayTopic("12-XYZ");
  FloatArrayTopic XYZ13 = aprilTagsTable.getFloatArrayTopic("13-XYZ");
  FloatArrayTopic XYZ14 = aprilTagsTable.getFloatArrayTopic("14-XYZ");
  FloatArrayTopic XYZ15 = aprilTagsTable.getFloatArrayTopic("15-XYZ");
  FloatArrayTopic XYZ16 = aprilTagsTable.getFloatArrayTopic("16-XYZ");

  private NetworkTable ringTable = inst.getTable("RingFinder");

  private static NetworkTable Unicorntable = inst.getTable("UnicornHatRIO");
  static final StringPublisher dblPub = Unicorntable.getStringTopic("ToUnicornStatus").publish();

  private double decelerationDistance = Constants.VisionConstants.kDecelerationDistance;
  private double[] declareRingPosNeeded = Constants.VisionConstants.kCenterOfScreen;

  private Set<String> tags;

  //Actually extracting data
  final GenericSubscriber SubDist4 = Dist4.genericSubscribe(PubSubOption.keepDuplicates(false));
  final GenericSubscriber SubDist7 = Dist7.genericSubscribe(PubSubOption.keepDuplicates(false));
  final GenericSubscriber SubCenter4 = Center4.genericSubscribe(PubSubOption.keepDuplicates(false));
  final GenericSubscriber SubCenter7 = Center7.genericSubscribe(PubSubOption.keepDuplicates(false));
  final GenericSubscriber SubXYZ4 = XYZ4.genericSubscribe(PubSubOption.keepDuplicates(false));
  final GenericSubscriber SubXYZ7 = XYZ7.genericSubscribe(PubSubOption.keepDuplicates(false));

  final GenericSubscriber SubCenter9 = Center9.genericSubscribe(PubSubOption.keepDuplicates(false));
  final GenericSubscriber SubCenter1 = Center1.genericSubscribe(PubSubOption.keepDuplicates(false));

  private final String[] speakerTags = {"4", "7"};
  private final String speakerDistanceToTagName = "Dist";
  private final String speakerCenterName = "Center";
  private final String speakerRotationName = "XYZ";

  private final String ringKey = "Rings";

  private final String[] ampTags = {"9", "1"};
  private final String ampCenterName = "Center";

  public VisionSubsystem() {

  }

  @Override
  public void periodic() {

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
