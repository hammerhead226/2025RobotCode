// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Branch;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ApproachBranch extends Command {
  private final Drive drive;
  private final LED led;
  private final Supplier<Branch> branchSupplier;
  private boolean pointsTooClose;

  Command pathCommand;
  BooleanSupplier continuePath;

  Pose2d atPose;
  Pose2d awayPose;

  boolean isPathFinished;
  boolean skipPath;
  /** Creates a new ApproachBranch. */
  public ApproachBranch(
      Drive drive,
      LED led,
      SuperStructure superStructure,
      Supplier<Branch> branchSupplier,
      BooleanSupplier continuePath) {
    this.drive = drive;
    this.led = led;
    this.branchSupplier = branchSupplier;
    this.continuePath = continuePath;
    addRequirements(drive, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPathFinished = false;
    skipPath = false;
    led.setState(LED_STATE.FLASHING_RED);

    boolean isRight = branchSupplier.get().getPipe() % 2 == 1;
    Pose2d reefPose = Drive.transformPerAlliance(branchSupplier.get().getPose().toPose2d());

    double sideOffset =
        isRight
            ? SubsystemConstants.CORRECTION_RIGHT_BRANCH_OFFSET.get()
            : SubsystemConstants.CORRECTION_LEFT_BRANCH_OFFSET.get();
    awayPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(SubsystemConstants.NEAR_FAR_AWAY_REEF_OFFSET, sideOffset),
            // SubsystemConstants.APPROACH_OFFSET_LEFT_RIGHT_OFFSET),
            Rotation2d.kPi);
    atPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET, sideOffset),
            // SubsystemConstants.APPROACH_OFFSET_LEFT_RIGHT_OFFSET),
            Rotation2d.kPi);

    ChassisSpeeds fieldRelChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    double chassisLinearSpeedSingular =
        Math.hypot(
            fieldRelChassisSpeeds.vxMetersPerSecond, fieldRelChassisSpeeds.vyMetersPerSecond);

    Pose2d currentPoseFacingVelocity;
    if (chassisLinearSpeedSingular >= 0.2) {
      currentPoseFacingVelocity =
          new Pose2d(
              drive.getPose().getTranslation(),
              new Rotation2d(
                  fieldRelChassisSpeeds.vxMetersPerSecond,
                  fieldRelChassisSpeeds.vyMetersPerSecond));
    } else {
      Translation2d v = awayPose.getTranslation().minus(drive.getPose().getTranslation());
      currentPoseFacingVelocity =
          new Pose2d(drive.getPose().getTranslation(), new Rotation2d(v.getX(), v.getY()));
    }

    List<Waypoint> waypoints;
    List<RotationTarget> holomorphicRotations;
    List<EventMarker> eventMarkers = new ArrayList<>();
    PathConstraints pathConstraints;

    Rotation2d rotatedVelocity =
        currentPoseFacingVelocity
            .getRotation()
            .rotateBy(Rotation2d.kZero.minus(atPose.getRotation()));

    Logger.recordOutput("rotated velocity", rotatedVelocity.getDegrees());

    List<ConstraintsZone> constraintsZones = new ArrayList<>();
    if (drive.shouldEndPath()) {
      skipPath = true;
    } else {
      if (!drive.isNearReef()) {
        waypoints = PathPlannerPath.waypointsFromPoses(currentPoseFacingVelocity, awayPose, atPose);
        holomorphicRotations =
            Arrays.asList(
                new RotationTarget(1.0, awayPose.getRotation().plus(Rotation2d.kCW_90deg)),
                new RotationTarget(1.7, atPose.getRotation().plus(Rotation2d.kCW_90deg)));
        if (atPose
                    .getRotation()
                    .minus(drive.getRotation().minus(Rotation2d.kCCW_90deg))
                    .getDegrees()
                <= 45
            // && Math.abs(atPoseRobotRelative.getY()) <= 0.8
            && Math.abs(rotatedVelocity.getDegrees()) < 80) {
          pathConstraints = new PathConstraints(2, 2.5, Math.toRadians(200), Math.toRadians(300));
        } else {
          pathConstraints = new PathConstraints(1.9, 2.2, Math.toRadians(150), Math.toRadians(250));
          // pathConstraints = new PathConstraints(0.1, 2, 150, 250);
        }
        constraintsZones.add(
            new ConstraintsZone(
                0.9, 2, new PathConstraints(1, 1.5, Math.toRadians(180), Math.toRadians(200))));
      } else {
        pathConstraints = new PathConstraints(1, 1.5, Math.toRadians(180), Math.toRadians(200));
        waypoints = PathPlannerPath.waypointsFromPoses(currentPoseFacingVelocity, atPose);
        holomorphicRotations =
            Arrays.asList(new RotationTarget(0.7, atPose.getRotation().plus(Rotation2d.kCW_90deg)));
      }
      Logger.recordOutput("Debug OTF Paths/Reef Align", atPose);

      pointsTooClose =
          drive.getPose().getTranslation().getDistance(atPose.getTranslation()) <= 0.01;
      // isPathFinished = drive.shouldEndPath();

      if (!pointsTooClose) {
        PathPlannerPath path =
            new PathPlannerPath(
                waypoints,
                holomorphicRotations,
                new ArrayList<>(),
                constraintsZones,
                eventMarkers,
                pathConstraints, // these numbers from last year's code
                new IdealStartingState(
                    chassisLinearSpeedSingular,
                    drive.getRotation()), // The ideal starting state, this is only relevant for
                // pre-planned paths, so
                // can
                // be null for on-the-fly paths.
                new GoalEndState(0, atPose.getRotation().rotateBy(Rotation2d.fromDegrees(-90))),
                false);
        path.preventFlipping = true;

        pathCommand = AutoBuilder.followPath(path);

        pathCommand.initialize();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!pointsTooClose && !skipPath) {
      isPathFinished = pathCommand.isFinished();
      pathCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    if (!pointsTooClose && !skipPath) {
      pathCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !continuePath.getAsBoolean() || pointsTooClose || isPathFinished || skipPath;
  }
}
