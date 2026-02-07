package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Patterns.Aurora;
import frc.robot.subsystems.Patterns.Dots;
import frc.robot.subsystems.Patterns.Fire;
import frc.robot.subsystems.Patterns.Particles;
import frc.robot.subsystems.Patterns.Police;

public class LEDs extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  private AddressableLEDBufferView left;
  private AddressableLEDBufferView right;
  private AddressableLEDBufferView topleft;
  private AddressableLEDBufferView topright;
  private AddressableLEDBufferView top;
  private AddressableLEDBufferView whole;
  private ArrayList<AddressableLEDBufferView> segmentsUnifiedTop;
  private ArrayList<AddressableLEDBufferView> segmentsSplitTop;

  private boolean breathing = false;

  public LEDPattern oldPattern;
  public LEDPattern currentPattern = LEDConstants.PATTERN_YELLOW;

  private int cyclesWhileBlinking = 0;
  private boolean blinking;

  private List<Integer> heat;

  public LEDs() {
    m_led = new AddressableLED(LEDConstants.LED_PORT);
    m_buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
    m_led.setLength(LEDConstants.LED_COUNT);
    m_led.start();
    // 288 led total
    this.left = m_buffer.createView(0, 104);
    this.topleft = m_buffer.createView(105, 142);
    this.topright = m_buffer.createView(143, 180).reversed();
    this.right = m_buffer.createView(181, 286).reversed();

    this.top = m_buffer.createView(105, 180);

    this.whole = m_buffer.createView(0, 286);

    segmentsUnifiedTop = new ArrayList<>();
    segmentsUnifiedTop.add(left);
    segmentsUnifiedTop.add(top);
    segmentsUnifiedTop.add(right);

    segmentsSplitTop = new ArrayList<>();
    segmentsSplitTop.add(left);
    segmentsSplitTop.add(topleft);
    segmentsSplitTop.add(topright);
    segmentsSplitTop.add(right);

    heat = new ArrayList<>(Collections.nCopies(right.getLength(), 0));

    runPattern(LEDConstants.PATTERN_YELLOW);
    updatePattern();
  }

  @Override
  public void periodic() {

    if (this.currentPattern == LEDConstants.PATTERN_FIRE) {
      heat = Fire.fire(left, 50, 60, left.getLength(), heat);
      heat = Fire.fire(right, 50, 60, right.getLength(), heat);
    } else if (this.currentPattern == LEDConstants.PATTERN_POLICE) {
      Police.runPolice(segmentsUnifiedTop, m_buffer);
    } else if (this.currentPattern == LEDConstants.PATTERN_DOTS) {
      Dots.runDots(segmentsUnifiedTop);
    } else if (this.currentPattern == LEDConstants.PATTERN_AURORA) {
      Aurora.runAurora(this.whole);
    } else if (this.currentPattern == LEDConstants.PATTERN_PARTICLES) {
      Particles.runParticles(segmentsUnifiedTop);
    } else {
      updatePattern();
    }

    m_led.setData(m_buffer);
  }

  public void updatePattern() {
    if (this.currentPattern != this.oldPattern
        || blinking
        || breathing
        || currentPattern.equals(LEDConstants.PATTERN_RAINBOW_SCROLLING)
        || currentPattern.equals(LEDConstants.PATTERN_SCROLL)) {
      this.runPattern(currentPattern);
    }
  }

  public void resetAnimation() {
    this.runPattern(currentPattern);
  }

  public void runPattern(LEDPattern pattern) {
    if (breathing) {
      pattern = pattern.breathe(LEDConstants.BREATHE_LOOP_TIME);
    }
    if (blinking) {
      pattern = pattern.blink(
          Time.ofBaseUnits(LEDConstants.BLINK_ON_LENGTH, Seconds),
          Time.ofBaseUnits(LEDConstants.BLINK_OFF_LENGTH, Seconds));
      cyclesWhileBlinking++;
      if (cyclesWhileBlinking > LEDConstants.BLINK_CYCLES) {
        blinking = false;
        cyclesWhileBlinking = 0;
      }
    }

    if (pattern == LEDConstants.PATTERN_RAINBOW_SCROLLING) {

      pattern.applyTo(this.whole);

    } else {

      pattern.applyTo(this.left);
      pattern.applyTo(this.right);

      pattern.applyTo(this.topleft);
      pattern.applyTo(this.topright);
    }
    this.oldPattern = this.currentPattern;
  }

  public Command blinkLEDs() {
    return this.runOnce(() -> this.blink());
  }

  public Command setPatternCommand(LEDPattern pattern) {
    return new InstantCommand(
        () -> {
          this.currentPattern = pattern;
          this.updatePattern(); // Ensure immediate update
        })
        .ignoringDisable(true);
  }

  public Command setBreatheCommand(boolean b) {
    return new InstantCommand(
        () -> {
          this.breathing = b;
          this.updatePattern(); // Ensure immediate update
        })
        .ignoringDisable(true);
  }

  public void blink() {
    System.out.println("Blinking LEDS");
    blinking = true;
  }
}
