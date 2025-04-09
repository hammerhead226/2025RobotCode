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

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.TunerConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ReefPositionsUtil;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (SimConstants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        // new PowerDistribution(1, ModuleType.kRev);
        // CameraServer.startAutomaticCapture();
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/logs"));

        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    SignalLogger.enableAutoLogging(false);

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.getScoralRollers().stop();
    robotContainer.getClimber().setArmCurrent(robotContainer.getClimber().getArmPositionDegs());
    robotContainer.getClimber().setArmGoal(robotContainer.getClimber().getArmPositionDegs());
    robotContainer.getScoralArm().setArmCurrent(robotContainer.getScoralArm().getArmPositionDegs());
    robotContainer.getScoralArm().setArmGoal(robotContainer.getScoralArm().getArmPositionDegs());
    robotContainer.getScoralArm().setConstraints(150, 300);
    robotContainer
        .getElevator()
        .setElevatorCurrent(robotContainer.getElevator().getElevatorPosition());
    robotContainer
        .getElevator()
        .setElevatorGoal(robotContainer.getElevator().getElevatorPosition());

    robotContainer.getLED().setState(LED_STATE.FIRE);
    LimelightHelpers.setLimelightNTDouble("limelight-reef", "throttle_set", 50);
    LimelightHelpers.SetIMUMode("limelight-reef", 1);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    LimelightHelpers.setLimelightNTDouble("limelight-reef", "throttle_set", 50);
  }

  @Override
  public void robotInit() {
    ReefPositionsUtil.printOffsetPoses();
    switch (SimConstants.currentMode) {
      case REAL:
        UsbCamera cam = CameraServer.startAutomaticCapture();

        cam.setResolution(640, 480);
        break;

      case SIM:
        // Set up the camera server
        // UsbCamera cam = CameraServer.startAutomaticCapture();
        // cam.setResolution(640, 480);
        break;

      case REPLAY:
        // Set up the camera server
        // UsbCamera cam = CameraServer.startAutomaticCapture();
        // cam.setResolution(640, 480);
        break;
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    LimelightHelpers.setLimelightNTDouble("limelight-reef", "throttle_set", 1);
    LimelightHelpers.SetIMUMode("limelight-reef", 1);

    robotContainer.getScoralArm().setArmCurrent(robotContainer.getScoralArm().getArmPositionDegs());
    robotContainer.getScoralArm().setArmGoal(robotContainer.getScoralArm().getArmPositionDegs());

    robotContainer
        .getElevator()
        .setElevatorCurrent(robotContainer.getElevator().getElevatorPosition());
    robotContainer
        .getElevator()
        .setElevatorGoal(robotContainer.getElevator().getElevatorPosition());

    robotContainer.getClimber().setArmCurrent(robotContainer.getClimber().getArmPositionDegs());
    robotContainer.getClimber().setArmGoal(robotContainer.getClimber().getArmPositionDegs());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Logger.recordOutput(
        "Debug Super Structure/Wanted State", robotContainer.getSuperStructure().getWantedState());
    Logger.recordOutput(
        "Debug Super Structure/Current State",
        robotContainer.getSuperStructure().getCurrentState());
    Logger.recordOutput(
        "Debug Super Structure/At State Goals", robotContainer.getSuperStructure().atGoals());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    LimelightHelpers.setLimelightNTDouble("limelight-reef", "throttle_set", 1);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    LimelightHelpers.SetIMUMode("limelight-reef", 1);

    robotContainer.getClimber().setArmCurrent(robotContainer.getClimber().getArmPositionDegs());
    robotContainer.getClimber().setArmGoal(robotContainer.getClimber().getArmPositionDegs());
    robotContainer.getScoralArm().setArmCurrent(robotContainer.getScoralArm().getArmPositionDegs());
    robotContainer.getScoralArm().setArmGoal(robotContainer.getScoralArm().getArmPositionDegs());
    robotContainer.getScoralArm().setConstraints(150, 300);

    robotContainer
        .getElevator()
        .setElevatorCurrent(robotContainer.getElevator().getElevatorPosition());
    robotContainer
        .getElevator()
        .setElevatorGoal(robotContainer.getElevator().getElevatorPosition());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    Logger.recordOutput(
        "Debug Super Structure/Wanted State", robotContainer.getSuperStructure().getWantedState());
    Logger.recordOutput(
        "Debug Super Structure/Current State",
        robotContainer.getSuperStructure().getCurrentState());
    Logger.recordOutput(
        "Debug Super Structure/At State Goals", robotContainer.getSuperStructure().atGoals());

    Logger.recordOutput("algaeMode", robotContainer.getSuperStructure().getAlgaeMode());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
