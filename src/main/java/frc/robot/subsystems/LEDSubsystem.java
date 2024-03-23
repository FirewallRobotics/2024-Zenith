package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.hal.AddressableLEDJNI;
// import edu.wpi.first.hal.simulation.AddressableLEDDataJNI;
// import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

  // All the steps to initialization can be found in the WPILib Documentation on AddressableLEDs

  // Set up variables for LEDs
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  private int timer;
  private int scroll_offset;

  public LEDSubsystem() {

    // Assign PWM port from RoboRIO, previous code used port 1
    m_led = new AddressableLED(LEDConstants.kLEDPort);

    // Assign length (number of LEDs being changed)
    m_ledBuffer = new AddressableLEDBuffer(27);

    // Transfers length from m_ledbuffer to m_led
    m_led.setLength(m_ledBuffer.getLength());

    // Sets LED output data
    m_led.setData(m_ledBuffer);

    // Writes the LED output continuously
    m_led.start();
    timer = 0;
    scroll_offset = 7;
  }

  void SetLights(int rgb_red, int rgb_green, int rgb_blue) {

    // Sets all LEDs to RGB values
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, rgb_red, rgb_green, rgb_blue);
    }

    // Applies the m_ledbuffer data to the m_led object
    m_led.setData(m_ledBuffer);
  }

  public void SetPurple() {

    // Sets all LEDs to RGB values
    SetLights(150, 26, 192);
  }

  public void SetOrange() {
    SetLights(240, 79, 5);
  }

  // public void LightScroll() {

  //   // Meant to have scrolling lights for asthetic; run as default command

  //   int rgb_blue;
  //   int rgb_green;
  //   int rgb_red;

  //   if (RobotBase.isReal()) {
  //     // Robot RGB (need to be darker)
  //     rgb_red = 5;
  //     rgb_green = 0;
  //     rgb_blue = 5;
  //   } else {
  //     // Simulator RGB (need to be brighter)
  //     rgb_red = 50;
  //     rgb_green = 0;
  //     rgb_blue = 50;
  //   }

  //   // NOTE: For the lightscroll effect, the rgb values start 3x smaller because I multiply it by
  //   // either 1, 2, or 3 to change brightness

  //   // Timer increments every scheduler run and scroll offset decrements every 10 iterations
  //   timer++;

  //   if (timer == 10) {
  //     timer = 0;
  //     scroll_offset--;
  //   }

  //   // Scroll offset loop (starts at 6 and ends at 1, looping back at 0)
  //   if (scroll_offset == 0) scroll_offset = 6;

  //   // Turns on half of the lights and turns off other half based on the offset
  //   // Brightness is found by taking the remainder of (Scroll offset + LED position) / 6, and
  //   // multiplying by that value
  //   // only if it is less than 4. This means brightness is only ever multiplied by 1, 2, or 3
  //   for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //     if (((scroll_offset + i) % 6) < 4) {
  //       m_ledBuffer.setRGB(
  //           i,
  //           rgb_red * ((scroll_offset + i) % 6),
  //           rgb_green * (scroll_offset + i),
  //           rgb_blue * ((scroll_offset + i) % 6));
  //     } else {
  //       m_ledBuffer.setRGB(i, 0, 0, 0);
  //       // Turns off light
  //     }
  //   }

  //   // Applies the m_ledbuffer data to the m_led object
  //   m_led.setData(m_ledBuffer);
  // }

  // public void RunDefaultLED(int LEDprox) {
  //   if (LEDprox == 0) {
  //     LightScroll();
  //   }
  // }
}
