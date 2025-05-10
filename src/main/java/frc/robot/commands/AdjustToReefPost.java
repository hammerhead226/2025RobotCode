// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoral.ScoralArm;
import frc.robot.util.SlewRateLimiter;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** This is a test for java doc. Open in /docs/frc/robot/commands/AdjustToReefPost.html */
public class AdjustToReefPost extends Command {
  /** Creates a new AdjustToReefPost. */
  enum AlignState {
    BRANCH_SENSOR,
    ODOMETRY,
    ODOMETRY_SENSOR_FUSED,
    DONE,
    CANCEL
  }

  enum WiggleState {
    RIGHT,
    LEFT,
    DONE
  }

  Drive drive;
  ScoralArm scoralArm;
  SuperStructure superStructure;

  boolean isRight;
  Pose2d offsetPose;
  Pose2d odometryTargetPose;

  AlignState alignState;
  WiggleState wiggleState;

  double reefSensorDistance;
  double branchSensorDistance;
  double angleToGoal;

  PIDController sensorForwardPID = new PIDController(0.1, 0, 0);
  ProfiledPIDController odometryForwardPID =
      new ProfiledPIDController(3, 1, 0.5, new TrapezoidProfile.Constraints(3, 4.5));
  ProfiledPIDController odometrySidePID =
      new ProfiledPIDController(3, 1, 0.5, new TrapezoidProfile.Constraints(3, 4.5));
  ProfiledPIDController rotationPID =
      new ProfiledPIDController(2.9, 0., 0.2, new TrapezoidProfile.Constraints(200, 300));

  SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(0.8);
  SlewRateLimiter sidewaysSlewRateLimiter = new SlewRateLimiter(0.8);
  SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(0.8);

  BooleanSupplier triggerPressed;
  ChassisSpeeds chassisSpeeds;

  Pose2d pidEndPose = new Pose2d();

  boolean isAligned;

  private final double angleTolerance = 2;

  public AdjustToReefPost(
      Drive drive,
      ScoralArm scoralArm,
      SuperStructure supersStructure,
      boolean isRight,
      BooleanSupplier triggerPressed) {
    this.drive = drive;
    this.scoralArm = scoralArm;
    this.superStructure = supersStructure;
    this.isRight = isRight;
    this.triggerPressed = triggerPressed;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.isReefAutoAlignDone = false;

    alignState =
        scoralArm.isCANRangeConnected()
                && superStructure.getCurrentState() != SuperStructureState.L2
            ? AlignState.ODOMETRY_SENSOR_FUSED
            // ? AlignState.ODOMETRY
            : AlignState.ODOMETRY;
    // : AlignState.ODOMETRY_SENSOR_FUSED;

    wiggleState = WiggleState.RIGHT;

    isAligned = false;

    sensorForwardPID.setTolerance(0.5);
    odometryForwardPID.setTolerance(Units.inchesToMeters(0.5));
    odometrySidePID.setTolerance(Units.inchesToMeters(0.5));
    rotationPID.setTolerance(0.5);

    Pose2d reefPose = isRight ? drive.getNearestCenterRight() : drive.getNearestCenterLeft();
    double sideOffset =
        isRight
            ? SubsystemConstants.CORRECTION_RIGHT_BRANCH_OFFSET.get()
            : SubsystemConstants.CORRECTION_LEFT_BRANCH_OFFSET.get();

    offsetPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(
                SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET,
                SubsystemConstants.ADJUST_OFFSET_LEFT_RIGHT_OFFSET),
            Rotation2d.kZero);

    odometryTargetPose =
        DriveCommands.rotateAndNudge(
            reefPose,
            new Translation2d(SubsystemConstants.NEAR_FAR_AT_REEF_OFFSET, sideOffset),
            Rotation2d.kZero);

    odometryForwardPID.reset(drive.getPose().getX());
    odometrySidePID.reset(drive.getPose().getY());
    rotationPID.reset(drive.getRotation().getDegrees());

    rotationPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Aligning State", alignState);
    Logger.recordOutput("Wiggle State", wiggleState);

    reefSensorDistance = drive.getCANRangeDistanceInches();
    branchSensorDistance = scoralArm.getCANRangeDistance();
    angleToGoal =
        drive.getRotation().getDegrees()
            - offsetPose.getRotation().plus(Rotation2d.kCW_90deg).getDegrees();

    if (!drive.isSlowMode()) {
      forwardSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      sidewaysSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
      rotationSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
    } else {
      forwardSlewRateLimiter.changeRateLimit(2);
      sidewaysSlewRateLimiter.changeRateLimit(2);
      rotationSlewRateLimiter.changeRateLimit(14);
    }

    double odometryForwardEffort = 0;
    double odometrySideEffort = 0;
    double branchSensorForwardEffort = 0;
    double reefSensorSideEffort =
        drive.isCANRangeConnected()
            ? -MathUtil.clamp(
                sensorForwardPID.calculate(drive.getCANRangeDistanceInches(), 12.5), -1, 1)
            : 0;
    // double reefSensorSideEffort = 0;
    double rotationEffort =
        Units.degreesToRadians(
            rotationPID.calculate(
                drive.getPose().getRotation().getDegrees(),
                offsetPose.getRotation().plus(Rotation2d.kCW_90deg).getDegrees()));
    boolean branchSensorConditions =
        superStructure.getCurrentState() == SuperStructureState.L4
            ? (reefSensorDistance <= 14
                    && (branchSensorDistance >= 9 && branchSensorDistance <= 14)
                    && Math.abs(angleToGoal) <= angleTolerance)
                && scoralArm.getCANRangeDistanceStd() <= 0.08
            : (reefSensorDistance <= 14
                    && (branchSensorDistance >= 17 && branchSensorDistance <= 22)
                    && Math.abs(angleToGoal) <= angleTolerance)
                && scoralArm.getCANRangeDistanceStd() <= 0.08;

    double distanceFromOdometryTargetPose =
        drive.getPose().getTranslation().getDistance(odometryTargetPose.getTranslation());
    if (alignState == AlignState.ODOMETRY_SENSOR_FUSED) {
      isAligned =
          branchSensorConditions && distanceFromOdometryTargetPose <= Units.inchesToMeters(6);
      odometryForwardEffort =
          odometryForwardPID.calculate(drive.getPose().getX(), odometryTargetPose.getX());
      odometrySideEffort =
          odometrySidePID.calculate(drive.getPose().getY(), odometryTargetPose.getY());
      branchSensorForwardEffort = 0;

      if (distanceFromOdometryTargetPose <= Units.inchesToMeters(1.5)) {
        pidEndPose = drive.getPose();
        alignState = AlignState.BRANCH_SENSOR;
      }
    } else if (alignState == AlignState.ODOMETRY) {
      isAligned =
          (distanceFromOdometryTargetPose <= Units.inchesToMeters(1) && Math.abs(angleToGoal) <= 2);
      odometryForwardEffort =
          odometryForwardPID.calculate(drive.getPose().getX(), odometryTargetPose.getX());
      odometrySideEffort =
          odometrySidePID.calculate(drive.getPose().getY(), odometryTargetPose.getY());
      branchSensorForwardEffort = 0;
    } else {
      odometryForwardEffort = 0;
      odometrySideEffort = 0;

      isAligned = branchSensorConditions;

      double distanceFromEndPose =
          Math.abs(pidEndPose.getTranslation().getDistance(drive.getPose().getTranslation()));
      switch (wiggleState) {
        case RIGHT:
          branchSensorForwardEffort = 0.31;
          if (distanceFromEndPose > Units.inchesToMeters(2)) {
            wiggleState = WiggleState.LEFT;
            pidEndPose = drive.getPose();
          }
          break;
        case LEFT:
          branchSensorForwardEffort = -0.31;
          if (distanceFromEndPose > Units.inchesToMeters(4)) {
            wiggleState = WiggleState.DONE;
          }
          break;
        case DONE:
          alignState = AlignState.ODOMETRY;
          break;
      }
    }

    // Logger.recordOutput("Reef Aligning/pidEndPose", pidEndPose);
    // Logger.recordOutput("Reef Aligning/offsetPose", offsetPose);
    Logger.recordOutput("Reef Aligning/odometryTargetPose", odometryTargetPose);
    Logger.recordOutput("Reef Aligning/odometryForwardEffort", odometryForwardEffort);
    Logger.recordOutput("Reef Aligning/odometrySideEffort", odometrySideEffort);
    Logger.recordOutput("Reef Aligning/branchSensorForwardEffort", branchSensorForwardEffort);
    Logger.recordOutput("Reef Aligning/reefSensorSideEffort", reefSensorSideEffort);
    Logger.recordOutput(
        "Reef Aligning/distance from offset",
        drive.getPose().getTranslation().getDistance(offsetPose.getTranslation()));

    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            odometryForwardEffort, odometrySideEffort, rotationEffort, drive.getRotation());

    drive.runVelocity(
        new ChassisSpeeds(
                forwardSlewRateLimiter.calculate(chassisSpeeds.vxMetersPerSecond),
                sidewaysSlewRateLimiter.calculate(chassisSpeeds.vyMetersPerSecond),
                chassisSpeeds.omegaRadiansPerSecond)
            .plus(new ChassisSpeeds(branchSensorForwardEffort, reefSensorSideEffort, 0)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    alignState = AlignState.CANCEL;
    if (isAligned) {
      alignState = AlignState.DONE;
      drive.isReefAutoAlignDone = true;
      superStructure.nextState();
    }
    Logger.recordOutput("Aligning State", alignState);
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !triggerPressed.getAsBoolean() || isAligned;
  }
}
