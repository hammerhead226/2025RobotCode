// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SimConstants.Mode;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.scoral.DistanceSensorIO;
import frc.robot.subsystems.scoral.DistanceSensorIOInputsAutoLogged;
import frc.robot.subsystems.vision.ObjectDetection;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 100 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontRight.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  // CHANGE THESE WHEN SEASON STARTS
  private static final double OBJECT_BUFFER_SIZE_SECONDS = 1.0;
  private static final double ROBOT_MASS_KG = 62;
  private static final double ROBOT_MOI = 4.4;
  private static final double WHEEL_COF = 1.2;
  // private final SwerveSetpointGenerator setpointGenerator;
  // private SwerveSetpoint previouSetpoint;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              Units.inchesToMeters(1.625),
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;

  private final DistanceSensorIO distanceIO;
  private final DistanceSensorIOInputsAutoLogged distanceInputs =
      new DistanceSensorIOInputsAutoLogged();

  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();

  public static double chassisSpeedMetersPerSec;
  public static double speedX;
  public static double speedY;
  public static double rotationVelocityDegsPerSec;

  private final TimeInterpolatableBuffer<Pose2d> gamePieceBuffer =
      TimeInterpolatableBuffer.createBuffer(OBJECT_BUFFER_SIZE_SECONDS);

  // private Pose2d nearestSide = new Pose2d();
  private Pose2d lastReefFieldPose;
  public boolean slowMode = false;
  public boolean isReefAutoAlignDone = false;
  public boolean isBargeAutoAlignDone = false;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      DistanceSensorIO distanceIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    this.distanceIO = distanceIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.5, 0.0, 0.0), new PIDConstants(5.5, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Swerve Setpoint Generator
    // setpointGenerator = new SwerveSetpointGenerator(PP_CONFIG, getMaxAngularSpeedRadPerSec());
    // ChassisSpeeds currentSpeeds =
    //     getChassisSpeeds(); // Method to get current robot-relative chassis speeds
    // SwerveModuleState[] currentStates =
    //     getModuleStates(); // Method to get the current swerve module states
    // previouSetpoint =
    //     new SwerveSetpoint(
    //         currentSpeeds, currentStates, DriveFeedforwards.zeros(PP_CONFIG.numModules));

    // Configure SysId
    speedX = 0;
    speedY = 0;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public Pose2d getPoseAtTimeStamp(double seconds) {

    return poseEstimator.sampleAt(seconds).orElse(new Pose2d());
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    distanceIO.updateInputs(distanceInputs);

    Logger.processInputs("Drive/Distance Sensor", distanceInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Swerve/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      speedX = getChassisSpeeds().vxMetersPerSecond;
      speedY = getChassisSpeeds().vyMetersPerSecond;
      chassisSpeedMetersPerSec = Math.sqrt(speedX * speedX + speedY * speedY);
      rotationVelocityDegsPerSec = Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && SimConstants.currentMode != Mode.SIM);
    // setNearestReefSide();
    Logger.recordOutput("Swerve/overallChassisSpeed", chassisSpeedMetersPerSec);

    Logger.recordOutput("should pid align", shouldEndPath());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // previouSetpoint =
    //     setpointGenerator.generateSetpoint(
    //         previouSetpoint, speeds, SubsystemConstants.LOOP_PERIOD_SECONDS);
    // SwerveModuleState[] setpointStates = previouSetpoint.moduleStates();
    // Calculate module setpoints
    speeds = ChassisSpeeds.discretize(speeds, SubsystemConstants.LOOP_PERIOD_SECONDS);

    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Swerve/SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // this.speedX = speeds.vxMetersPerSecond;
    // this.speedY = speeds.vyMetersPerSecond;
    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Swerve/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "Swerve/SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }
  // public double getTimeStamp(){

  // return poseEstimator.
  // }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // Pose2d test = new Pose2d(new Translation2d(3, 5),
    // Rotation2d.fromDegrees(-60));
    // poseEstimator.addVisionMeasurement(
    // test, timestampSeconds, visionMeasurementStdDevs);
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    Logger.recordOutput(
        "Debug Vision/recieved pose in addVisionMeasurement", visionRobotPoseMeters);
  }

  public void addObjectMeasurement(
      Translation2d objectRobotRelativePoseMeters, double timeStampSeconds) {
    Pose2d robotPoseAtTimeStamp = poseEstimator.sampleAt(timeStampSeconds).orElse(new Pose2d());

    gamePieceBuffer.addSample(
        timeStampSeconds,
        ObjectDetection.calculateNotePositionFieldRelative(
            robotPoseAtTimeStamp, objectRobotRelativePoseMeters));
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
  }

  public double getMaxLinearSpeedMetersPerSec(Elevator elevator) {
    double baseSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1;

    // should be based off of constant setpoint, but adjust as needed
    if (elevator.getElevatorPosition()
        >= SubsystemConstants.ElevatorConstants.L3_SETPOINT_INCHES - 2.0) {
      return baseSpeed * 0.5;
    } else {
      return baseSpeed;
    }
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return (getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS) * 0.6;
  }

  public void enableSlowMode(boolean enabled) {
    slowMode = enabled;
    // linearSpeedMultiplier = enabled ? 0.3 : 1;
    // angularSpeedMultiplier = enabled ? 0.1 : 1;
  }

  public boolean isSlowMode() {
    return slowMode;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  // private class toPoseEstimatorConsumer implements VisionConsumer {
  //   @Override
  //   public void accept(
  //       Pose2d visionRobotPoseMeters,
  //       double timestampSeconds,
  //       Matrix<N3, N1> visionMeasurementStdDevs) {
  //     Logger.recordOutput("visionRobotPoseMetersTHREE", visionRobotPoseMeters); // THREE

  //     poseEstimator.addVisionMeasurement(
  //         visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  //     // Logger.recordOutput("visionRobotPoseMetersTHREE", visionRobotPoseMeters); //
  //     // THREE
  //   }
  // }

  // public VisionConsumer getToPoseEstimatorConsumer() {
  //   return new toPoseEstimatorConsumer();
  // }

  // private class rawGyroRotationSupplier implements Supplier<Rotation2d> {
  //   @Override
  //   public Rotation2d get() {
  //     return rawGyroRotation;
  //   }
  // }

  // public rawGyroRotationSupplier getRawGyroRotationSupplier() {
  //   return new rawGyroRotationSupplier();
  // }

  // public Pose2d getNearestSide() {
  // return nearestSide;
  // }

  public Pose2d getNearestCenter() {
    int index = getNearestParition(6);
    Logger.recordOutput("Debug Driver Alignment/align to reef center target index", index);
    lastReefFieldPose = transformPerAlliance(FieldConstants.Reef.centerFaces[index]);
    return lastReefFieldPose;
  }

  public Pose2d getNearestCenterLeft() {
    int index = getNearestParition(6) * 2 + 1;
    // if (DriverStation.getAlliance().isPresent()
    //   && DriverStation.getAlliance().get() == Alliance.Red) {
    //   index += 1;
    // }
    Logger.recordOutput("Debug Driver Alignment/align to reef center left target index", index);
    return passBranchFieldPose(index);
  }

  public Pose2d getNearestCenterRight() {
    int index = getNearestParition(6) * 2;
    // if (!(DriverStation.getAlliance().isPresent()
    //   && DriverStation.getAlliance().get() == Alliance.Red)) {
    //   index += 1;
    // }
    Logger.recordOutput("Debug Driver Alignment/align to reef center left target index", index);
    return passBranchFieldPose(index);
  }

  public Pose2d getNearestSide() {
    int index = getNearestParition(12);
    Logger.recordOutput("Debug Driver Alignment/align to reef target index", index);
    return passBranchFieldPose(index);
  }

  private Pose2d passBranchFieldPose(int index) {
    lastReefFieldPose =
        transformPerAlliance(
            FieldConstants.Reef.branchPositions
                .get(index)
                .get(FieldConstants.ReefHeight.L1)
                .toPose2d());
    return lastReefFieldPose;
  }

  public Pose2d getLastReefFieldPose() {
    return lastReefFieldPose;
  }

  public int getNearestParition(int partitions) {
    Translation2d start = transformPerAlliance(FieldConstants.Reef.center);
    Translation2d end = getPose().getTranslation();
    Translation2d v = end.minus(start);
    Rotation2d angle = new Rotation2d(v.getX(), v.getY());

    // https://www.desmos.com/calculator/44dd9koglh

    // negate since branchPositions is CW not CCW
    // +7/12 since branchPositions starts at branch B not the +x axis
    double rawRotations = angle.getRotations();
    double adjustedRotations = -rawRotations;

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      adjustedRotations += (7.0 / 12.0);
    } else {
      adjustedRotations += (1.0 / 12.0);
    }

    // % 1 to just get the fractional part of the rotation
    // multiply by 12 before flooring so [0,1) maps to
    // 0,1,2...partitions-2,partitions-1 evenly
    double fractionalRotation = adjustedRotations % 1;
    if (fractionalRotation < 0) {
      fractionalRotation++;
    }
    int index = (int) Math.floor(fractionalRotation * partitions);

    return index;
  }
  
  public Pose2d getNearestSource() {
    if (getPose()
            .getTranslation()
            .getDistance(
                transformPerAlliance(FieldConstants.CoralStation.leftCenterFace.getTranslation()))
        < getPose()
            .getTranslation()
            .getDistance(
                transformPerAlliance(
                    FieldConstants.CoralStation.rightCenterFace.getTranslation()))) {
      return transformPerAlliance(FieldConstants.CoralStation.leftCenterFace);

    } else {
      return transformPerAlliance(FieldConstants.CoralStation.rightCenterFace);
    }
  }

  public boolean isAtReefSide() {
    return DriveCommands.getTargetPose() != null
        && getPose().getTranslation().getDistance(DriveCommands.getTargetPose().getTranslation())
            <= 1;
  }

  public boolean isNearReef() {
    // for reference from reef wall to reef wall is about 65 inches or 1.65 meters
    // return getPose().getTranslation().getDistance(FieldConstants.Reef.center) <= 1.7;
    return getPose()
            .getTranslation()
            .getDistance(AllianceFlipUtil.apply(FieldConstants.Reef.center))
        <= 1.7;
  }

  public boolean shouldRunReefCommand() {
    return getPose()
            .getTranslation()
            .getDistance(AllianceFlipUtil.apply(FieldConstants.Reef.center))
        <= 2;
    // <= 0.5;
  }

  public boolean shouldEndPath() {
    // return false;
    return getPose()
            .getTranslation()
            .getDistance(AllianceFlipUtil.apply(FieldConstants.Reef.center))
        <= 1.3;
  }

  public boolean isAtReefRotation() {
    return DriveCommands.getTargetPose() != null
        && DriveCommands.getTargetPose().getRotation().minus(getRotation()).getDegrees() < 10;
  }

  // takes in a Pose2d from blue alliance's perspective and flips it if we are on
  // red allience
  public static Pose2d transformPerAlliance(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      return pose;
    }
    return new Pose2d(
        transformPerAlliance(pose.getTranslation()), transformPerAlliance(pose.getRotation()));
  }

  // takes in a Pose2d from blue alliance's perspective and flips it if we are on
  // red allience
  public static Translation2d transformPerAlliance(Translation2d translation) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      return translation;
    }
    return translation.rotateAround(
        new Translation2d(FieldConstants.fieldLength / 2, FieldConstants.fieldWidth / 2),
        Rotation2d.kPi);
  }

  public static Rotation2d transformPerAlliance(Rotation2d rotation) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      return rotation;
    }
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public double getCANRangeDistanceInches() {
    return distanceInputs.distanceInches;
  }

  public boolean isCANRangeConnected() {
    return distanceInputs.connected;
  }
}
