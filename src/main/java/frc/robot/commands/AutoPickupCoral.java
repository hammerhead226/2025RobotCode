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
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.ObjectDetection;
import java.util.List;

public class AutoPickupCoral extends Command {
  private final Drive drive;
  private final LED led;
  private final ObjectDetection objectDetection;
  // private final CoralIntake coralIntake;

  Command pathCommand;

  public AutoPickupCoral(
      /*CoralIntake coralIntake,*/ ObjectDetection objectDetection, Drive drive, LED led) {
    // this.coralIntake = coralIntake;
    this.objectDetection = objectDetection;
    this.drive = drive;
    this.led = led;

    drive.getPose().getTranslation();
  }

  @Override
  public void initialize() {
    // led.setState(SubsystemConstants.LED_STATE.ALIGNING);

    // TODO once coralIntake is done change this to be legit
    // coralIntake.runFlywheel();

    Translation2d targetTranslation2d =
        objectDetection.getNotePositionRobotRelative().plus(drive.getPose().getTranslation());

    Rotation2d targetRotation2d =
        new Rotation2d(
            targetTranslation2d.getX() - drive.getPose().getX(),
            targetTranslation2d.getY() - drive.getPose().getY());

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            drive.getPose(), new Pose2d(targetTranslation2d, targetRotation2d));

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            new PathConstraints(3.5, 2.7, 100, 180), // these numbers from last year's code
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0.5, targetRotation2d));
    path.preventFlipping = true;

    pathCommand = AutoBuilder.followPath(path);
    pathCommand.initialize();
  }

  @Override
  public void execute() {
    led.setState(LED_STATE.FLASHING_WHITE);
    pathCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;

    // TODO in coralIntake, implment hasCoral
    // return coralIntake.hasCoral();

    // see last year's code for comparing the currentAmps to a threshold
    /*
    if (feedInputs.currentAmps
    > 13) { // TODo add additional check to filter out false positives
    // } else if (feedInputs.currentAmps > 10000) {
    Logger.recordOutput("see note val", "current");
    lastNoteState = NoteState.CURRENT;
    return NoteState.CURRENT;
     */
  }
}
