package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AlignToReefAuto extends Command {
  private final Drive drive;
  private final LED led;

  Command pathCommand;

  public AlignToReefAuto(Drive drive, LED led) {
    this.drive = drive;
    this.led = led;

    addRequirements(drive, led);
  }

  @Override
  public void initialize() {
    // led.setState(SubsystemConstants.LED_STATE.ALIGNING);
    Pose2d targetPose = getNearestReefSide();
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drive.getPose(), targetPose);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            new PathConstraints(3.5, 2.7, 100, 180), // these numbers from last year's code
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0.5, targetPose.getRotation()));
    path.preventFlipping = true;

    pathCommand = AutoBuilder.followPath(path);
    pathCommand.initialize();
  }

  public Pose2d getNearestReefSide() {
    Translation2d start = FieldConstants.Reef.center;
    Translation2d end = drive.getPose().getTranslation();
    Translation2d v = end.minus(start);
    Rotation2d angle = new Rotation2d(v.getX(), v.getY());

    // https://www.desmos.com/calculator/44dd9koglh

    // negate since branchPositions is CW not CCW
    // +6/12 since branchPositions starts at branch B not the +x axis
    double rawRotations = angle.getRotations();
    double adjustedRotations = -rawRotations + (7.0 / 12.0);

    // % 1 to just get the fractional part of the rotation
    // multiply by 12 before flooring so [0,1) maps to 0,1,2...10,11 evenly
    double fractionalRotation = adjustedRotations % 1;
    if (fractionalRotation < 0) {
      fractionalRotation++;
    }
    int index = (int) Math.floor(fractionalRotation * 12);

    Logger.recordOutput("align to reef target index", index);

    // System.out.println("align to reef target index " + index);
    // System.out.println("list size " + FieldConstants.Reef.branchPositions.size());

    Pose2d result =
        FieldConstants.Reef.branchPositions.get(index).get(FieldConstants.ReefHeight.L1).toPose2d();

    // flip rotation
    Rotation2d rotation2d = result.getRotation().rotateBy(new Rotation2d(Math.PI));

    // back up target position (so it doesn't clip)
    // x is nearer/farther, y is sideways
    Translation2d offsetFromBranch = new Translation2d(-0.7, 0);
    offsetFromBranch = offsetFromBranch.rotateBy(rotation2d);
    Translation2d translation2d = result.getTranslation().plus(offsetFromBranch);

    result = new Pose2d(translation2d, rotation2d);

    Logger.recordOutput("align to reef target Pose2d", result);
    return result;
  }

  @Override
  public void execute() {
    // we could switch to PID as we get closer to the wall? for instance
    // if both CANRanges measure a short distance
    led.setState(LED_STATE.FLASHING_YELLOW);
    pathCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    // or any other way we can measure "close enough" to desired position
    return false;
  }
}
