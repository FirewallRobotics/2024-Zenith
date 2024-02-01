package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.hal.AddressableLEDJNI;
// import edu.wpi.first.hal.simulation.AddressableLEDDataJNI;
// import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  // All the steps to initialization can be found in the WPILib Documentation on AddressableLEDs

  // Set up variables for LEDs
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  public LEDSubsystem() {

    // Assign PWM port from RoboRIO, previous code used port 1
    m_led = new AddressableLED(1);

    // Assign length (number of LEDs being changed)
    m_ledBuffer = new AddressableLEDBuffer(27);

    // Transfers length from m_ledbuffer to m_led
    m_led.setLength(m_ledBuffer.getLength());

    // Sets LED output data
    m_led.setData(m_ledBuffer);

    // Writes the LED output continuously
    m_led.start();
  }

  private void SetLights(int rgb_red, int rgb_green, int rgb_blue) {

    // Sets all LEDs to RGB values
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, rgb_red, rgb_green, rgb_blue);
    }

    // Applies the m_ledbuffer data to the m_led object
    m_led.setData(m_ledBuffer);
  }
}
