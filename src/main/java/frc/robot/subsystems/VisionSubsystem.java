package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private static NetworkTable Unicorntable = inst.getTable("UnicornHatRIO");
  static final StringPublisher dblPub = Unicorntable.getStringTopic("ToUnicornStatus").publish();

  public VisionSubsystem() {}
  // Communication For The Unicornhat
  public static void UnicornNotify(String status) {
    dblPub.set(status);
  }
}
