package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;

public class Bounds {

  double robotWidth;
  double centerOffset;

  public Bounds() {

    double robotWidth = 0.9;
    double centerOffset = 1; // change number
  }

  public boolean isInBounds(Pose2d currentPose) {
    double leftEdge = FieldConstants.fieldWidth - robotWidth;
    double rightEdge = robotWidth;

    double bottomEdge =
        (!AllianceFlipUtil.shouldFlip())
            ? robotWidth
            : FieldConstants.fieldLength / 2 + centerOffset + robotWidth;
    double topEdge =
        (!AllianceFlipUtil.shouldFlip())
            ? FieldConstants.fieldLength / 2 - centerOffset - robotWidth
            : FieldConstants.fieldLength - robotWidth;
    double driveX = currentPose.getX();
    double driveY = currentPose.getY();

    return (driveX <= topEdge && driveX >= bottomEdge)
        && (driveY <= leftEdge && driveY >= rightEdge);
  }

  public boolean rectangleBounds(
      Pose2d bottomLeftCorner, double Length, double Width, Pose2d currentPose) {
    // left corner is the lower x and y value corner so the corner closest to (0,0)

    double xMax =
        (!AllianceFlipUtil.shouldFlip())
            ? bottomLeftCorner.getX() + Width
            : bottomLeftCorner.getX() + Width + FieldConstants.fieldLength / 2;

    double yMax =
        (!AllianceFlipUtil.shouldFlip())
            ? bottomLeftCorner.getY() + Width
            : bottomLeftCorner.getY() + Length + FieldConstants.fieldLength / 2;

    double xMin =
        (!AllianceFlipUtil.shouldFlip())
            ? bottomLeftCorner.getX()
            : bottomLeftCorner.getX() + FieldConstants.fieldLength / 2;

    double yMin =
        (!AllianceFlipUtil.shouldFlip())
            ? bottomLeftCorner.getY()
            : bottomLeftCorner.getY() + FieldConstants.fieldLength / 2;

    double driveX = currentPose.getX();
    double driveY = currentPose.getY();

    return (driveX <= xMax && driveX >= xMin) && (driveY <= yMax && driveY >= yMin);
  }

  public boolean circleBounds(Translation2d circlePose, double circleRadius, Pose2d currentPose) {
    circlePose =
        (!AllianceFlipUtil.shouldFlip())
            ? circlePose
            : circlePose.plus(new Translation2d(0, 13.522325));

    Translation2d robotPose = currentPose.getTranslation();
    double distance = robotPose.getDistance(circlePose);
    return (distance > circleRadius + robotWidth);
  }
}
