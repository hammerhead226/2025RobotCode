// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApproachReef extends Command {
  private final Drive drive;
  private final SuperStructure superStructure;
  private final boolean isRight;
  Command pathCommand;
  BooleanSupplier continuePath;
  /** Creates a new ApproachReef. */
  public ApproachReef(
      Drive drive, SuperStructure superStructure, boolean isRight, BooleanSupplier continuePath) {
    this.drive = drive;
    this.superStructure = superStructure;
    this.isRight = isRight;
    this.continuePath = continuePath;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d reefPose = isRight ? drive.getNearestCenterRight() : drive.getNearestCenterLeft();

    Pose2d awayPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(
                SubsystemConstants.NEAR_FAR_AWAY_REEF_OFFSET,
                SubsystemConstants.LEFT_RIGHT_BRANCH_OFFSET),
            Rotation2d.kZero);

    Pose2d atPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(
                SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET,
                SubsystemConstants.LEFT_RIGHT_BRANCH_OFFSET),
            Rotation2d.kZero);

    // Terminates the align command if robot is rotated 120 degrees or more from the destination
    // pose
    double angleToTargetPoseDegrees =
        Units.radiansToDegrees(
            MathUtil.angleModulus(drive.getRotation().minus(awayPose.getRotation()).getRadians()));
    if (angleToTargetPoseDegrees > 60 && angleToTargetPoseDegrees < 120) {
      end(true);
    }

    ChassisSpeeds fieldRelChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    double chassisSpeedSingular =
        Math.hypot(
            fieldRelChassisSpeeds.vxMetersPerSecond, fieldRelChassisSpeeds.vyMetersPerSecond);

    Pose2d currentPoseFacingVelocity;
    if (chassisSpeedSingular >= 0.2) {
      currentPoseFacingVelocity =
          new Pose2d(
              drive.getPose().getTranslation(),
              new Rotation2d(
                  fieldRelChassisSpeeds.vxMetersPerSecond,
                  fieldRelChassisSpeeds.vyMetersPerSecond));
    } else {
      Pose2d nextPose = drive.isNearReef() ? atPose : awayPose;
      Translation2d v = nextPose.getTranslation().minus(drive.getPose().getTranslation());
      currentPoseFacingVelocity =
          new Pose2d(drive.getPose().getTranslation(), new Rotation2d(v.getX(), v.getY()));
    }

    // targetPose =
    //     new Pose2d(
    //         targetPose.getTranslation(),
    //         targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(-90)));
    // new Pose2d(
    //     drive.getNearestSide().getTranslation().minus(drive.getOffset()),
    //     drive.getNearestSide().getRotation());
    // new Pose2d(nearestSide.getTranslation().minus(offset), nearestSide.getRotation())

    List<Waypoint> waypoints;
    if (drive.isNearReef()) {
      waypoints = PathPlannerPath.waypointsFromPoses(currentPoseFacingVelocity, atPose);
    } else {
      waypoints = PathPlannerPath.waypointsFromPoses(currentPoseFacingVelocity, awayPose, atPose);
    }

    List<RotationTarget> holomorphicRotations =
        Arrays.asList(
            new RotationTarget(1.0, awayPose.getRotation().plus(Rotation2d.kCW_90deg)),
            new RotationTarget(1.9, atPose.getRotation().plus(Rotation2d.kCW_90deg)));

    List<EventMarker> eventMarkers = new ArrayList<>();
    eventMarkers.add(
        new EventMarker("test trigger", 0.1, superStructure.getSuperStructureCommand()));
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            holomorphicRotations,
            new ArrayList<>(),
            new ArrayList<>(),
            eventMarkers,
            new PathConstraints(1.5, 1, 100, 180), // these numbers from last year's code
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0, atPose.getRotation().rotateBy(Rotation2d.fromDegrees(-90))),
            false);
    path.preventFlipping = true;

    pathCommand = AutoBuilder.followPath(path);
    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      superStructure.nextState();
      pathCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished() || !continuePath.getAsBoolean();
  }
}
