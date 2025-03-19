package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants;

public class ReefPositionsUtil {
  public static final String ASCII_ART_BLUE_REEF =
      """
      K  J
    L      I
  A          H
  B          G
    C      F
      D  E
  """;

  public static final String ASCII_ART_RED_REEF =
      """
      E  D
    F      C
  G          B
  H          A
    I      L
      J  K
  """;

  /*
   * index:
   * 0, 1, 2, 3, ..., 10, 11
   * key:
   * A, B, C, D, ..., K, L
   */
  public static final Pose2d[] BLUE_REEF_POSE2DS = getAllReefPoses(false);
  public static final Pose2d[] RED_REEF_POSE2DS = getAllReefPoses(true);

  public static Pose2d[] getAllReefPoses(boolean isRed) {
    Pose2d[] result = new Pose2d[12];
    for (int i = 0; i < 12; i++) {
      // account for FieldConstants ordering
      int index = -i + (isRed ? 7 : 1);

      // take postive mod to stay in range
      index = index % 12;
      if (index < 0) {
        index += 12;
      }

      result[i] =
          FieldConstants.Reef.branchPositions
              .get(index)
              .get(FieldConstants.ReefHeight.L1)
              .toPose2d();
    }
    return result;
  }

  public static final int DIGITS = 4;
  public static final String ROUND_FORM = "%." + DIGITS + "f";

  public static void printOffsetPoses() {
    System.out.println(ASCII_ART_BLUE_REEF);
    System.out.println("x, y, angle");
    for (int i = 0; i < 12; i++) {
      Pose2d offsetPose =
          DriveCommands.rotateAndNudge(
              BLUE_REEF_POSE2DS[i],
              new Translation2d(
                  SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET,
                  // SubsystemConstants.LEFT_RIGHT_BRANCH_OFFSET),
                  SubsystemConstants.APPROACH_OFFSET_LEFT_RIGHT_OFFSET),
              Rotation2d.kZero);

      System.out.println((char) ('A' + i) + " -");
      System.out.println(String.format(ROUND_FORM, offsetPose.getX()));
      System.out.println(String.format(ROUND_FORM, offsetPose.getY()));
      System.out.println(String.format(ROUND_FORM, offsetPose.getRotation().getDegrees()));
    }
  }
}
