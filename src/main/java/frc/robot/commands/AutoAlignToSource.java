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

public class AutoAlignToSource extends Command {
  private final Drive drive;
  private final LED led;

  Command pathCommand;

  public AutoAlignToSource(Drive drive, LED led) {
    this.drive = drive;
    this.led = led;

    drive.getPose().getTranslation();

    addRequirements(drive, led);
  }

  @Override
  public void initialize() {
    // led.setState(SubsystemConstants.LED_STATE.ALIGNING);

    // Translation2d targetTranslation2d = drive.getPose() > FieldConstants.HEIGHT / 2.0 ?
    // FieldConstants.Sources.Lower.Position : FieldConstants.Sources.Higher.Position;
    // Rotation2d targetRotation2d = new Rotation2d(
    //   targetTranslation2d.getX()-drive.getPose().getX(),
    //   targetTranslation2d.getY()-drive.getPose().getY()
    //   );

    // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //   drive.getPose(),
    //   new Pose2d(targetTranslation2d, targetRotation2d)
    // );

    Pose2d targetPose = getNearestSourceSide();

    // FieldConstants.Sources.Lower.Position : FieldConstants.Sources.Higher.Position;
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

  private Pose2d getNearestSourceSide() {
    Pose2d result =
        drive.getPose().getY() > FieldConstants.fieldWidth / 2.0
            ? FieldConstants.CoralStation.leftCenterFace
            : FieldConstants.CoralStation.rightCenterFace;

    // flip rotation
    Rotation2d rotation2d = result.getRotation(); // .rotateBy(new Rotation2d(Math.PI));

    // back up target position (so it doesn't clip)
    // x is nearer/farther, y is sideways
    Translation2d offsetFromBranch = new Translation2d(0.7, 0);
    offsetFromBranch = offsetFromBranch.rotateBy(rotation2d);
    Translation2d translation2d = result.getTranslation().plus(offsetFromBranch);

    result = new Pose2d(translation2d, rotation2d);

    Logger.recordOutput("align to reef target Pose2d", result);
    return result;
  }

  @Override
  public void execute() {
    led.setState(LED_STATE.FLASHING_BLUE);
    pathCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
