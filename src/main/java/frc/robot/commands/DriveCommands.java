// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.*;
import frc.robot.constants.SubsystemConstants.SuperStructureState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.util.SlewRateLimiter;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.05;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static double rotationErrorDegrees = 0;

  private static double wantedRotationVelocityRadsPerSec = 0;

  private static double rotationAssistEffort = 0;

  private static Pose2d previousTargetPose;
  private static Pose2d targetPose;

  private boolean FieldRelative = true;

  public static Pose2d getTargetPose() {
    return targetPose;
  }

  // private static ProfiledPIDController goof = new ProfiledPIDController(1.5, 0,
  // 0, )

  // profiled controllers
  static SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(0.8);
  static SlewRateLimiter sidewaysSlewRateLimiter = new SlewRateLimiter(0.8);
  static SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(0.8);

  static ProfiledPIDController rotationPID =
      new ProfiledPIDController(2.9, 0., 0.2, new TrapezoidProfile.Constraints(200, 300));
  // new ProfiledPIDController(0, 0., 0, new TrapezoidProfile.Constraints(70,
  // 120));

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      SuperStructure superStructure,
      LED led,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldRelativeSupplier) {
    // BooleanSupplier sourceAlignSupplier,
    // BooleanSupplier processorAlignSupplier,
    // BooleanSupplier anchorAlignSupplier) {
    return Commands.run(
        () -> {
          rotationPID.setTolerance(0.1);
          rotationPID.enableContinuousInput(-180, 180);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);
          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());

          double forwardSpeed = speeds.vxMetersPerSecond;

          double sidewaysSpeed = speeds.vyMetersPerSecond;

          double rotationSpeed = speeds.omegaRadiansPerSecond;

          double speedDebuff = 0.75;
          targetPose = null;

          if (2 == 1) {
            if (superStructure.getWantedState() == SuperStructureState.SOURCE) {
              targetPose = drive.getNearestSource();
              targetPose =
                  new Pose2d(
                      new Translation2d(0, 0),
                      targetPose.getRotation().plus(Rotation2d.fromDegrees(-90)));
              // Logger.recordOutput("Debug Driver Alignment/drive targetPose name", "source");
            } else if (superStructure.getWantedState() == SuperStructureState.PROCESSOR) {
              targetPose = Drive.transformPerAlliance(FieldConstants.Processor.centerFace);
              targetPose =
                  rotateAndNudge(targetPose, new Translation2d(-0.5, 0), new Rotation2d(Math.PI));
              targetPose =
                  new Pose2d(
                      targetPose.getTranslation(),
                      targetPose.getRotation().plus(Rotation2d.fromDegrees(-90)));
              // Logger.recordOutput("Debug Driver Alignment/drive targetPose name", "processor");
            }
          } else {
            // Logger.recordOutput("Debug Driver Alignment/drive targetPose name", "none");
          }

          if (targetPose != null && !targetPose.equals(previousTargetPose)) {
            previousTargetPose = targetPose;

            rotationPID.reset(drive.getRotation().getDegrees());
          }

          Logger.recordOutput("Debug Driver Alignment/drive targetPose", targetPose);

          if (targetPose != null) {

            // hacky code so that it always rotates the shorter direction
            double driveDegrees = drive.getPose().getRotation().getDegrees() % 360;
            if (driveDegrees < 0) {
              driveDegrees += 360;
            }
            double targetDegrees = targetPose.getRotation().getDegrees() % 360;
            if (targetDegrees < 0) {
              targetDegrees += 360;
            }
            if (driveDegrees - targetDegrees > 180) {
              driveDegrees -= 360;
            } else if (targetDegrees - driveDegrees > 180) {
              targetDegrees -= 360;
            }

            rotationErrorDegrees = driveDegrees - targetDegrees;

            Logger.recordOutput("drive commands drive rotation", drive.getRotation().getDegrees());
            Logger.recordOutput(
                "drive commands target rotation", targetPose.getRotation().getDegrees());

            double rotationPIDResult =
                Math.toRadians(
                    rotationPID.calculate(
                        drive.getRotation().getDegrees(), targetPose.getRotation().getDegrees()));
            Logger.recordOutput("rotationPIDResult", rotationPIDResult);
            wantedRotationVelocityRadsPerSec =
                MathUtil.clamp(
                    rotationPIDResult,
                    -drive.getMaxAngularSpeedRadPerSec(),
                    drive.getMaxAngularSpeedRadPerSec());

            rotationAssistEffort =
                superStructure.getWantedState() == SuperStructureState.SOURCE
                        || superStructure.getWantedState() == SuperStructureState.PROCESSOR
                    ? (wantedRotationVelocityRadsPerSec - rotationSpeed) * speedDebuff
                    : 0;

          } else {
            wantedRotationVelocityRadsPerSec = rotationSpeed;
            rotationAssistEffort = 0;
          }

          Logger.recordOutput("Debug Driver Alignment/target pose", targetPose);

          Logger.recordOutput(
              "Debug Driver Alignment/Rotation Profile Position deg",
              rotationPID.getSetpoint().position);

          Logger.recordOutput(
              "Debug Driver Alignment/Rotation Profile Velocity rad/s",
              Math.toRadians(rotationPID.getSetpoint().velocity));

          Logger.recordOutput(
              "Debug Driver Alignment/Wanted Rotation Velocity rad/s",
              wantedRotationVelocityRadsPerSec);

          Logger.recordOutput(
              "Debug Driver Alignment/Rotation Assist Effort", rotationAssistEffort);
          Logger.recordOutput("Debug Driver Alignment/Is Slow Mode?", drive.isSlowMode());

          if (!drive.isSlowMode()) {
            forwardSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
            sidewaysSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
            rotationSlewRateLimiter.changeRateLimit(Integer.MAX_VALUE);
          } else {
            forwardSlewRateLimiter.changeRateLimit(2);
            sidewaysSlewRateLimiter.changeRateLimit(2);
            rotationSlewRateLimiter.changeRateLimit(14);
          }

          double totalInputSpeed = Math.hypot(forwardSpeed, sidewaysSpeed);
          double scale =
              totalInputSpeed > drive.getMaxLinearSpeedMetersPerSec()
                  ? drive.getMaxLinearSpeedMetersPerSec() / totalInputSpeed
                  : 1;

            ChassisSpeeds inputSpeeds = fieldRelativeSupplier.getAsBoolean()
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardSpeed, sidewaysSpeed, rotationSpeed, isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation())
            : ChassisSpeeds.fromRobotRelativeSpeeds(
              forwardSpeed, sidewaysSpeed, rotationSpeed, Rotation2d.kZero);
              
          // isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation());

          ChassisSpeeds assistSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  new ChassisSpeeds(0, 0, rotationAssistEffort), drive.getRotation());

          ChassisSpeeds finalInputSpeed = inputSpeeds.plus(assistSpeeds).times(scale);

          drive.runVelocity(
              new ChassisSpeeds(
                  forwardSlewRateLimiter.calculate(finalInputSpeed.vxMetersPerSecond),
                  sidewaysSlewRateLimiter.calculate(finalInputSpeed.vyMetersPerSecond),
                  rotationSlewRateLimiter.calculate(finalInputSpeed.omegaRadiansPerSecond)));
        },
        drive);
  }

  /*
   * translation:
   * +x is forward relative to the robot's new rotation
   * +y is left relative to the robot's new rotation
   */
  public static Pose2d rotateAndNudge(Pose2d pose, Translation2d translation, Rotation2d rotation) {
    Rotation2d rotation2d = pose.getRotation().rotateBy(rotation);
    Translation2d translation2d = pose.getTranslation().plus(translation.rotateBy(rotation2d));
    return new Pose2d(translation2d, rotation2d);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds
                      .fromFieldRelativeSpeeds(
                          speeds,
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation()));
              double rotationSpeed;
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
