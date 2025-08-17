package frc.robot.util;

public class OklabUtil {
  public static double[] sRBGtoLinearRGB(double[] sRGB) {
    if (sRGB.length != 3) {
      return new double[3];
    }
    return new double[] {
      componentsRGBToLinearRGB(sRGB[0]),
      componentsRGBToLinearRGB(sRGB[1]),
      componentsRGBToLinearRGB(sRGB[2]),
    };
  }

  public static double[] linearRGBtosRGB(double[] sRGB) {
    if (sRGB.length != 3) {
      return new double[3];
    }
    return new double[] {
      componentLinearRGBTosRGB(sRGB[0]),
      componentLinearRGBTosRGB(sRGB[1]),
      componentLinearRGBTosRGB(sRGB[2]),
    };
  }

  public static double[] linearsRBGToOklab(double[] lRGB) {
    if (lRGB.length != 3) {
      return new double[3];
    }

    double[] oklab = {
      0.4122214708f * lRGB[0] + 0.5363325363f * lRGB[1] + 0.0514459929f * lRGB[2],
      0.2119034982f * lRGB[0] + 0.6806995451f * lRGB[1] + 0.1073969566f * lRGB[2],
      0.0883024619f * lRGB[0] + 0.2817188376f * lRGB[1] + 0.6299787005f * lRGB[2]
    };

    oklab[0] = Math.pow(oklab[0], 1.0 / 3.0);
    oklab[1] = Math.pow(oklab[1], 1.0 / 3.0);
    oklab[2] = Math.pow(oklab[2], 1.0 / 3.0);

    return new double[] {
      0.2104542553f * oklab[0] + 0.7936177850f * oklab[1] - 0.0040720468f * oklab[2],
      1.9779984951f * oklab[0] - 2.4285922050f * oklab[1] + 0.4505937099f * oklab[2],
      0.0259040371f * oklab[0] + 0.7827717662f * oklab[1] - 0.8086757660f * oklab[2],
    };
  }

  public static double[] oklabToLinearRGB(double[] oklab) {
    if (oklab.length != 3) {
      return new double[3];
    }

    double[] lRGB = {
      oklab[0] + 0.3963377774f * oklab[1] + 0.2158037573f * oklab[2],
      oklab[0] - 0.1055613458f * oklab[1] - 0.0638541728f * oklab[2],
      oklab[0] - 0.0894841775f * oklab[1] - 1.2914855480f * oklab[2]
    };

    lRGB[0] = Math.pow(lRGB[0], 3.0);
    lRGB[1] = Math.pow(lRGB[1], 3.0);
    lRGB[2] = Math.pow(lRGB[2], 3.0);

    return new double[] {
      +4.0767416621f * lRGB[0] - 3.3077115913f * lRGB[1] + 0.2309699292f * lRGB[2],
      -1.2684380046f * lRGB[0] + 2.6097574011f * lRGB[1] - 0.3413193965f * lRGB[2],
      -0.0041960863f * lRGB[0] - 0.7034186147f * lRGB[1] + 1.7076147010f * lRGB[2],
    };
  }

  public static double componentsRGBToLinearRGB(double x) {
    if (x >= 0.0031308) {
      return (1.055) * Math.pow(x, (1.0 / 2.4)) - 0.055;
    }
    return 12.92 * x;
  }

  public static double componentLinearRGBTosRGB(double x) {
    if (x >= 0.04045) {
      return Math.pow(((x + 0.055) / (1 + 0.055)), 2.4);
    }
    return x / 12.92;
  }
}
