package frc.robot.subsystems.Patterns;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import frc.robot.Constants.LEDConstants;

public class Dots {

  private static double[] dots = new double[] {(-1), (-1), (-1)};
  private static boolean[] dotsDecreasing = new boolean[] {false, false, true};

  public static void runDots(ArrayList<AddressableLEDBufferView> segmentsUnifiedTop) {
    for (int i = 0; i < segmentsUnifiedTop.size(); i++) {

      if (dots[i] == -1) {
        dots[i] = (1.0 * segmentsUnifiedTop.get(i).getLength()) / (LEDConstants.DOT_FREQUENCY_CYCLES);
      }
      AddressableLEDBufferView segment = segmentsUnifiedTop.get(i);
      int length = segment.getLength();
      if (dotsDecreasing[i]) {
        dots[i] -= length / LEDConstants.DOT_FREQUENCY_CYCLES;
      } else {
        dots[i] += length / LEDConstants.DOT_FREQUENCY_CYCLES;
      }
      if (dots[i] <= 0 || dots[i] >= length - 1) {
        dotsDecreasing[i] = !dotsDecreasing[i];
        if (dots[i] <= 0) {
          dots[i] = 0;
        } else {
          dots[i] = length - 1;
        }
      }
      LEDConstants.PATTERN_DOTS_BACKGROUND.applyTo(segment);
      segment.setRGB((int) dots[i], 255, 255, 255);
      for (int j = 0; j < LEDConstants.DOTS_TRAIL_LENGTH; j++) {
        int brightness = 255 - (j * (255 / LEDConstants.DOTS_TRAIL_LENGTH));
        if (dotsDecreasing[i] && dots[i] + j < length) {
          segment.setRGB(
              (int) dots[i] + j,
              (int) LEDConstants.DOT_COLOR.red * brightness,
              (int) LEDConstants.DOT_COLOR.blue * brightness,
              (int) LEDConstants.DOT_COLOR.green * brightness);
        }
        if (!dotsDecreasing[i] && dots[i] - j > 0) {
          brightness = 255 - (j * (255 / LEDConstants.DOTS_TRAIL_LENGTH));
          segment.setRGB(
              (int) dots[i] - j,
              (int) LEDConstants.DOT_COLOR.red * brightness,
              (int) LEDConstants.DOT_COLOR.blue * brightness,
              (int) LEDConstants.DOT_COLOR.green * brightness);
        }
      }
    }
  }
}
