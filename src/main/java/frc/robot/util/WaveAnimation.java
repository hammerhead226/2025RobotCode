package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color8Bit;

public class WaveAnimation {
  static final double[] waveOklab1 =
      OklabUtil.linearsRBGToOklab(
          OklabUtil.sRBGtoLinearRGB(new double[] {0 / 255.0, 0 / 255.0, 226 / 255.0}));
  static final double[] waveOklab2 =
      OklabUtil.linearsRBGToOklab(
          OklabUtil.sRBGtoLinearRGB(new double[] {90 / 255.0, 226 / 255.0, 200 / 255.0}));
  static final int waveLength = 20;
  static final double wavePeriod = 2.0;

  public static Color8Bit[] getColors(int size, double t) {
    Color8Bit[] result = new Color8Bit[size];
    double[][] waveColors = new double[waveLength][3];

    for (int i = 0; i < waveLength; i++) {
      double x =
          (Math.sin(2 * Math.PI * (i / (double) waveLength + (t % wavePeriod) / wavePeriod)) + 1.0)
              / 2.0;
      waveColors[i] =
          new double[] {
            waveOklab1[0] * x + waveOklab2[0] * (1 - x),
            waveOklab1[1] * x + waveOklab2[1] * (1 - x),
            waveOklab1[2] * x + waveOklab2[2] * (1 - x)
          };

      waveColors[i] = OklabUtil.linearRGBtosRGB(OklabUtil.oklabToLinearRGB(waveColors[i]));
    }

    for (int i = 0; i < size; i++) {
      result[i] =
          new Color8Bit(
              (int) (waveColors[i % waveLength][0] * 255),
              (int) (waveColors[i % waveLength][1] * 255),
              (int) (waveColors[i % waveLength][2] * 255));
    }

    return result;
  }
}
