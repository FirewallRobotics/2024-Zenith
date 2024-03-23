package frc.robot.subsystems;

import edu.wpi.first.networktables.FloatArrayEntry;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.FloatEntry;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Time;
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
  NetworkTableEntry Dist1 = aprilTagsTable.getEntry("1-Dist");
  NetworkTableEntry Dist2 = aprilTagsTable.getEntry("2-Dist");
  NetworkTableEntry Dist3 = aprilTagsTable.getEntry("3-Dist");
  NetworkTableEntry Dist4 = aprilTagsTable.getEntry("4-Dist");
  NetworkTableEntry Dist5 = aprilTagsTable.getEntry("5-Dist");
  NetworkTableEntry Dist6 = aprilTagsTable.getEntry("6-Dist");
  NetworkTableEntry Dist7 = aprilTagsTable.getEntry("7-Dist");
  NetworkTableEntry Dist8 = aprilTagsTable.getEntry("8-Dist");
  NetworkTableEntry Dist9 = aprilTagsTable.getEntry("9-Dist");
  NetworkTableEntry Dist10 = aprilTagsTable.getEntry("10-Dist");
  NetworkTableEntry Dist11 = aprilTagsTable.getEntry("11-Dist");
  NetworkTableEntry Dist12 = aprilTagsTable.getEntry("12-Dist");
  NetworkTableEntry Dist13 = aprilTagsTable.getEntry("13-Dist");
  NetworkTableEntry Dist14 = aprilTagsTable.getEntry("14-Dist");
  NetworkTableEntry Dist15 = aprilTagsTable.getEntry("15-Dist");
  NetworkTableEntry Dist16 = aprilTagsTable.getEntry("16-Dist");

  NetworkTableEntry TimeSeen1 = aprilTagsTable.getEntry("1-TimeSeen");
  NetworkTableEntry TimeSeen2 = aprilTagsTable.getEntry("2-TimeSeen");
  NetworkTableEntry TimeSeen3 = aprilTagsTable.getEntry("3-TimeSeen");
  NetworkTableEntry TimeSeen4 = aprilTagsTable.getEntry("4-TimeSeen");
  NetworkTableEntry TimeSeen5 = aprilTagsTable.getEntry("5-TimeSeen");
  NetworkTableEntry TimeSeen6 = aprilTagsTable.getEntry("6-TimeSeen");
  NetworkTableEntry TimeSeen7 = aprilTagsTable.getEntry("7-TimeSeen");
  NetworkTableEntry TimeSeen8 = aprilTagsTable.getEntry("8-TimeSeen");
  NetworkTableEntry TimeSeen9 = aprilTagsTable.getEntry("9-TimeSeen");
  NetworkTableEntry TimeSeen10 = aprilTagsTable.getEntry("10-TimeSeen");
  NetworkTableEntry TimeSeen11 = aprilTagsTable.getEntry("11-TimeSeen");
  NetworkTableEntry TimeSeen12 = aprilTagsTable.getEntry("12-TimeSeen");
  NetworkTableEntry TimeSeen13 = aprilTagsTable.getEntry("13-TimeSeen");
  NetworkTableEntry TimeSeen14 = aprilTagsTable.getEntry("14-TimeSeen");
  NetworkTableEntry TimeSeen15 = aprilTagsTable.getEntry ("15-TimeSeen");
  NetworkTableEntry TimeSeen16 = aprilTagsTable.getEntry("16-TimeSeen");

  NetworkTableEntry Center1 = aprilTagsTable.getEntry ("1-Center");
  NetworkTableEntry Center2 = aprilTagsTable.getEntry ("2-Center");
  NetworkTableEntry Center3 = aprilTagsTable.getEntry ("3-Center");
  NetworkTableEntry Center4 = aprilTagsTable.getEntry ("4-Center");
  NetworkTableEntry Center5 = aprilTagsTable.getEntry ("5-Center");
  NetworkTableEntry Center6 = aprilTagsTable.getEntry ("6-Center");
  NetworkTableEntry Center7 = aprilTagsTable.getEntry ("7-Center");
  NetworkTableEntry Center8 = aprilTagsTable.getEntry ("8-Center");
  NetworkTableEntry Center9 = aprilTagsTable.getEntry ("9-Center");
  NetworkTableEntry Center10 = aprilTagsTable.getEntry ("10-Center");
  NetworkTableEntry Center11 = aprilTagsTable.getEntry ("11-Center");
  NetworkTableEntry Center12 = aprilTagsTable.getEntry ("12-Center");
  NetworkTableEntry Center13 = aprilTagsTable.getEntry ("13-Center");
  NetworkTableEntry Center14 = aprilTagsTable.getEntry ("14-Center");
  NetworkTableEntry Center15 = aprilTagsTable.getEntry ("15-Center");
  NetworkTableEntry Center16 = aprilTagsTable.getEntry ("16-Center");

  NetworkTableEntry XYZ1 = aprilTagsTable.getEntry ("1-XYZ");
  NetworkTableEntry XYZ2 = aprilTagsTable.getEntry ("2-XYZ");
  NetworkTableEntry XYZ3 = aprilTagsTable.getEntry ("3-XYZ");
  NetworkTableEntry XYZ4 = aprilTagsTable.getEntry ("4-XYZ");
  NetworkTableEntry XYZ5 = aprilTagsTable.getEntry ("5-XYZ");
  NetworkTableEntry XYZ6 = aprilTagsTable.getEntry ("6-XYZ");
  NetworkTableEntry XYZ7 = aprilTagsTable.getEntry ("7-XYZ");
  NetworkTableEntry XYZ8 = aprilTagsTable.getEntry ("8-XYZ");
  NetworkTableEntry XYZ9 = aprilTagsTable.getEntry ("9-XYZ");
  NetworkTableEntry XYZ10 = aprilTagsTable.getEntry ("10-XYZ");
  NetworkTableEntry XYZ11 = aprilTagsTable.getEntry ("11-XYZ");
  NetworkTableEntry XYZ12 = aprilTagsTable.getEntry("12-XYZ");
  NetworkTableEntry XYZ13 = aprilTagsTable.getEntry ("13-XYZ");
  NetworkTableEntry XYZ14 = aprilTagsTable.getEntry ("14-XYZ");
  NetworkTableEntry XYZ15 = aprilTagsTable.getEntry ("15-XYZ");
  NetworkTableEntry XYZ16 = aprilTagsTable.getEntry ("16-XYZ");

  private NetworkTable ringTable = inst.getTable("RingFinder");
  private NetworkTableEntry ring = ringTable.getEntry("Rings");

  private static NetworkTable Unicorntable = inst.getTable("UnicornHatRIO");
  static final StringPublisher dblPub = Unicorntable.getStringTopic("ToUnicornStatus").publish();

  private double decelerationDistance = Constants.VisionConstants.kDecelerationDistance;
  private double[] declareRingPosNeeded = Constants.VisionConstants.kCenterOfScreen;

  private Set<String> tags;

  private float[] defaultfloat = {1};

  //Actually extracting data
  private float SubDist4;
  private float SubDist7;
  private float[] SubCenter4;
  private float[] SubCenter7;
  private float[] SubXYZ4;
  private float[] SubXYZ7;

  private float[] SubCenter9;
  private float[] SubCenter1;

  private float Time1;
  private float Time9;
  private float Time4;
  private float Time7;

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

    SubDist4 = Dist4.getFloat(0);
    SubDist7 = Dist7.getFloat(0);
    SubCenter4 = Center4.getFloatArray(defaultfloat);
    SubCenter7 = Center7.getFloatArray(defaultfloat);
    SubXYZ4 = XYZ4.getFloatArray(defaultfloat);
    SubXYZ7 = XYZ7.getFloatArray(defaultfloat);;

    SubCenter9 = Center9.getFloatArray(defaultfloat);
    SubCenter1 = Center1.getFloatArray(defaultfloat);

    Time1 = TimeSeen1.getFloat(0);
    Time9 = TimeSeen9.getFloat(0);
    Time4 = TimeSeen4.getFloat(0);
    Time7 = TimeSeen7.getFloat(0);

    if(NetworkTablesJNI.now() + 10 >= Time1 && !tags.contains("1")){
      tags.add("1");
    }else if((NetworkTablesJNI.now() + 10 >= Time1) == false){
      tags.remove("1");
    }

    if(NetworkTablesJNI.now() + 10 >= Time9 && !tags.contains("9")){
      tags.add("9");
    }else if((NetworkTablesJNI.now() + 10 >= Time9) == false){
      tags.remove("9");
    }

    if(NetworkTablesJNI.now() + 10 >= Time4 && !tags.contains("4")){
      tags.add("4");
    }else if((NetworkTablesJNI.now() + 10 >= Time4) == false){
      tags.remove("4");
    }

    if(NetworkTablesJNI.now() + 10 >= Time7 && !tags.contains("7")){
      tags.add("7");
    }else if((NetworkTablesJNI.now() + 10 >= Time7) == false){
      tags.remove("7");
    }

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
      if (tags.contains(tagNum)) {
        return true;
      }
    }

    return false;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagDistanceToTag() {
    String tagNum = findSpeakerTagInView();

    if(tagNum.equals("4")){
      return SubDist4;
    }

    if(tagNum.equals("7")){
      return SubDist7;
    }

    return 0;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagCenterX() {
    String tagNum = findSpeakerTagInView();

    if(tagNum.equals("4")){
      return SubCenter4[0];
    }

    if(tagNum.equals("7")){
      return SubCenter7[0];
    }

    return 0;
  }

  /** Must have checkForSpeakerTag return true before executing this method */
  public float getSpeakerTagRotationZ() {
    String tagNum = findSpeakerTagInView();

    if(tagNum.equals("4")){
      return SubXYZ4[2];
    }

    if(tagNum.equals("7")){
      return SubXYZ7[2];
    }

    return 0;
  }

  private String findSpeakerTagInView() {
    for (String tagNum : speakerTags) {
      if (tags.contains("tagNum")) {
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
