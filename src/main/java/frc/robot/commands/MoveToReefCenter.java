// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToReefCenter extends Command {
  /** Creates a new MoveToReefCenter. */
  Drive drive;

  Command pathCommand;
  boolean pointsTooClose;
  boolean isPathFinished;

  public MoveToReefCenter(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPathFinished = false;
    Pose2d atPose =
        DriveCommands.rotateAndNudge(
            new Pose2d(
                (drive.getNearestCenterLeft().getX() + drive.getNearestCenterRight().getX()) / 2.,
                (drive.getNearestCenterLeft().getY() + drive.getNearestCenterRight().getY()) / 2.,
                drive.getNearestCenterLeft().getRotation()),
            new Translation2d(SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET, -0.025),
            Rotation2d.kZero);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drive.getPose(), atPose);
    Logger.recordOutput("Debug OTF Paths/Reef Align", atPose);
    List<RotationTarget> holomorphicRotations =
        Arrays.asList(new RotationTarget(0.5, atPose.getRotation().plus(Rotation2d.kCW_90deg)));

    pointsTooClose = drive.getPose().getTranslation().getDistance(atPose.getTranslation()) <= 0.01;
    if (!pointsTooClose) {
      PathPlannerPath path =
          new PathPlannerPath(
              waypoints,
              holomorphicRotations,
              new ArrayList<>(),
              new ArrayList<>(),
              new ArrayList<>(),
              new PathConstraints(
                  2.5,
                  2.5,
                  Math.toRadians(100),
                  Math.toRadians(180)), // these numbers from last year's code
              null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be null for on-the-fly paths.
              new GoalEndState(0, atPose.getRotation().rotateBy(Rotation2d.fromDegrees(-90))),
              false);
      path.preventFlipping = true;
      pathCommand = AutoBuilder.followPath(path);
      pathCommand.initialize();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!pointsTooClose) {
      pathCommand.execute();
      isPathFinished = pathCommand.isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!pointsTooClose) {
      pathCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isPathFinished || pointsTooClose;
  }
}
