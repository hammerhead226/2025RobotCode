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
// import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AlignToProcessor extends Command {
  // Elevator elevator;
  Drive drive;
  LED led;
  Command pathCommand;

  public AlignToProcessor(/*Elevator elevator,*/ Drive drive, LED led) {
    this.drive = drive;
    this.led = led;
    // this.elevator = elevator;
    addRequirements(/*elevator,*/ drive, led);
  }

  @Override
  public void initialize() {
    Pose2d targetPose = FieldConstants.Processor.centerFace;

    // flip rotation
    Rotation2d rotation2d = targetPose.getRotation().rotateBy(new Rotation2d(Math.PI));

    // back up target position (so it doesn't clip)
    // x is nearer/farther, y is sideways
    Translation2d offsetFromBranch =
        new Translation2d(
            -0.7,
            0); // TODO: This appears to be overwritten in DriveCommands.java/isn't affecting the
    // targetPose of pathCommand.
    offsetFromBranch = offsetFromBranch.rotateBy(rotation2d);
    Translation2d translation2d = targetPose.getTranslation().plus(offsetFromBranch);

    targetPose = new Pose2d(translation2d, rotation2d);
    Logger.recordOutput("align to reef target Pose2d", targetPose);

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

  @Override
  public void execute() {
    led.setState(LED_STATE.FLASHING_YELLOW);
    pathCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
