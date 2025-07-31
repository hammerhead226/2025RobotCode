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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.LED_STATE;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.tabletCommands;
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

  private NetworkTable table;
  private NetworkTableEntry a4Button, a3Button, a2Button, a1Button;
  private NetworkTableEntry b4Button, b3Button, b2Button, b1Button;
  private NetworkTableEntry c4Button, c3Button, c2Button, c1Button;
  private NetworkTableEntry d4Button, d3Button, d2Button, d1Button;
  private NetworkTableEntry e4Button, e3Button, e2Button, e1Button;
  private NetworkTableEntry f4Button, f3Button, f2Button, f1Button;
  private NetworkTableEntry g4Button, g3Button, g2Button, g1Button;
  private NetworkTableEntry h4Button, h3Button, h2Button, h1Button;
  private NetworkTableEntry i4Button, i3Button, i2Button, i1Button;
  private NetworkTableEntry j4Button, j3Button, j2Button, j1Button;
  private NetworkTableEntry k4Button, k3Button, k2Button, k1Button;
  private NetworkTableEntry l4Button, l3Button, l2Button, l1Button;
  private NetworkTableEntry Alg1, Alg2, Alg3, Alg4, Alg5, Alg6;
  private NetworkTableEntry leftCoralStation, rightCoralStation;
  private NetworkTableEntry algaeInBarge, proccesor;

  tabletCommands tablet = new tabletCommands();

  @Override
  public void robotInit() {
    ReefPositionsUtil.printOffsetPoses();
    NetworkTableInstance.getDefault().startClient4("tabletUI-sim");
    NetworkTableInstance.getDefault().startDSClient();

    table = NetworkTableInstance.getDefault().getTable("Reef");
    a4Button = table.getEntry("A4");
    a3Button = table.getEntry("A3");
    a2Button = table.getEntry("A2");
    a1Button = table.getEntry("A1");
    b4Button = table.getEntry("B4");
    b3Button = table.getEntry("B3");
    b2Button = table.getEntry("B2");
    b1Button = table.getEntry("B1");
    c4Button = table.getEntry("C4");
    c3Button = table.getEntry("C3");
    c2Button = table.getEntry("C2");
    c1Button = table.getEntry("C1");
    d4Button = table.getEntry("D4");
    d3Button = table.getEntry("D3");
    d2Button = table.getEntry("D2");
    d1Button = table.getEntry("D1");
    e4Button = table.getEntry("E4");
    e3Button = table.getEntry("E3");
    e2Button = table.getEntry("E2");
    e1Button = table.getEntry("E1");
    f4Button = table.getEntry("F4");
    f3Button = table.getEntry("F3");
    f2Button = table.getEntry("F2");
    f1Button = table.getEntry("F1");
    g4Button = table.getEntry("G4");
    g3Button = table.getEntry("G3");
    g2Button = table.getEntry("G2");
    g1Button = table.getEntry("G1");
    h4Button = table.getEntry("H4");
    h3Button = table.getEntry("H3");
    h2Button = table.getEntry("H2");
    h1Button = table.getEntry("H1");
    i4Button = table.getEntry("I4");
    i3Button = table.getEntry("I3");
    i2Button = table.getEntry("I2");
    i1Button = table.getEntry("I1");
    j4Button = table.getEntry("J4");
    j3Button = table.getEntry("J3");
    j2Button = table.getEntry("J2");
    j1Button = table.getEntry("J1");
    k4Button = table.getEntry("K4");
    k3Button = table.getEntry("K3");
    k2Button = table.getEntry("K2");
    k1Button = table.getEntry("K1");
    l4Button = table.getEntry("L4");
    l3Button = table.getEntry("L3");
    l2Button = table.getEntry("L2");
    l1Button = table.getEntry("L1");
    Alg1 = table.getEntry("Alg1");
    Alg2 = table.getEntry("Alg2");
    Alg3 = table.getEntry("Alg3");
    Alg4 = table.getEntry("Alg4");
    Alg5 = table.getEntry("Alg5");
    Alg6 = table.getEntry("Alg6");
    leftCoralStation = table.getEntry("leftCoralStation");
    rightCoralStation = table.getEntry("rightCoralStation");
    algaeInBarge = table.getEntry("algaeInBarge");
    proccesor = table.getEntry("proccesor");

    // UsbCamera cam = CameraServer.startAutomaticCapture();

    // cam.setResolution(640, 480);
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
  public void testPeriodic() {
    if (a4Button.getBoolean(false)) {
      System.out.println("A4 button pressed!");
      tablet.setA4Val(true);
      a4Button.setBoolean(false);
    }
    if (a3Button.getBoolean(false)) {
      System.out.println("A3 button pressed!");
      tablet.setA3Val(true);
      a3Button.setBoolean(false);
    }
    if (a2Button.getBoolean(false)) {
      System.out.println("A2 button pressed!");
      tablet.setA2Val(true);
      a2Button.setBoolean(false);
    }
    if (a1Button.getBoolean(false)) {
      System.out.println("A1 button pressed!");
      tablet.setA1Val(true);
      a1Button.setBoolean(false);
    }

    if (b4Button.getBoolean(false)) {
      System.out.println("B4 button pressed!");
      tablet.setB4Val(true);
      b4Button.setBoolean(false);
    }
    if (b3Button.getBoolean(false)) {
      System.out.println("B3 button pressed!");
      tablet.setB3Val(true);
      b3Button.setBoolean(false);
    }
    if (b2Button.getBoolean(false)) {
      System.out.println("B2 button pressed!");
      tablet.setB2Val(true);
      b2Button.setBoolean(false);
    }
    if (b1Button.getBoolean(false)) {
      System.out.println("B1 button pressed!");
      tablet.setB1Val(true);
      b1Button.setBoolean(false);
    }

    if (c4Button.getBoolean(false)) {
      System.out.println("C4 button pressed!");
      tablet.setC4Val(true);
      c4Button.setBoolean(false);
    }
    if (c3Button.getBoolean(false)) {
      System.out.println("C3 button pressed!");
      tablet.setC3Val(true);
      c3Button.setBoolean(false);
    }
    if (c2Button.getBoolean(false)) {
      System.out.println("C2 button pressed!");
      tablet.setC2Val(true);
      c2Button.setBoolean(false);
    }
    if (c1Button.getBoolean(false)) {
      System.out.println("C1 button pressed!");
      tablet.setC1Val(true);
      c1Button.setBoolean(false);
    }

    if (d4Button.getBoolean(false)) {
      System.out.println("D4 button pressed!");
      tablet.setD4Val(true);
      d4Button.setBoolean(false);
    }
    if (d3Button.getBoolean(false)) {
      System.out.println("D3 button pressed!");
      tablet.setD3Val(true);
      d3Button.setBoolean(false);
    }
    if (d2Button.getBoolean(false)) {
      System.out.println("D2 button pressed!");
      tablet.setD2Val(true);
      d2Button.setBoolean(false);
    }
    if (d1Button.getBoolean(false)) {
      System.out.println("D1 button pressed!");
      tablet.setD1Val(true);
      d1Button.setBoolean(false);
    }

    if (e4Button.getBoolean(false)) {
      System.out.println("E4 button pressed!");
      tablet.setE4Val(true);
      e4Button.setBoolean(false);
    }
    if (e3Button.getBoolean(false)) {
      System.out.println("E3 button pressed!");
      tablet.setE3Val(true);
      e3Button.setBoolean(false);
    }
    if (e2Button.getBoolean(false)) {
      System.out.println("E2 button pressed!");
      tablet.setE2Val(true);
      e2Button.setBoolean(false);
    }
    if (e1Button.getBoolean(false)) {
      System.out.println("E1 button pressed!");
      tablet.setE1Val(true);
      e1Button.setBoolean(false);
    }

    if (f4Button.getBoolean(false)) {
      System.out.println("F4 button pressed!");
      tablet.setF4Val(true);
      f4Button.setBoolean(false);
    }
    if (f3Button.getBoolean(false)) {
      System.out.println("F3 button pressed!");
      tablet.setF3Val(true);
      f3Button.setBoolean(false);
    }
    if (f2Button.getBoolean(false)) {
      System.out.println("F2 button pressed!");
      tablet.setF2Val(true);
      f2Button.setBoolean(false);
    }
    if (f1Button.getBoolean(false)) {
      System.out.println("F1 button pressed!");
      tablet.setF1Val(true);
      f1Button.setBoolean(false);
    }

    if (g4Button.getBoolean(false)) {
      System.out.println("G4 button pressed!");
      tablet.setG4Val(true);
      g4Button.setBoolean(false);
    }
    if (g3Button.getBoolean(false)) {
      System.out.println("G3 button pressed!");
      tablet.setG3Val(true);
      g3Button.setBoolean(false);
    }
    if (g2Button.getBoolean(false)) {
      System.out.println("G2 button pressed!");
      tablet.setG2Val(true);
      g2Button.setBoolean(false);
    }
    if (g1Button.getBoolean(false)) {
      System.out.println("G1 button pressed!");
      tablet.setG1Val(true);
      g1Button.setBoolean(false);
    }

    if (h4Button.getBoolean(false)) {
      System.out.println("H4 button pressed!");
      tablet.setH4Val(true);
      h4Button.setBoolean(false);
    }
    if (h3Button.getBoolean(false)) {
      System.out.println("H3 button pressed!");
      tablet.setH3Val(true);
      h3Button.setBoolean(false);
    }
    if (h2Button.getBoolean(false)) {
      System.out.println("H2 button pressed!");
      tablet.setH2Val(true);
      h2Button.setBoolean(false);
    }
    if (h1Button.getBoolean(false)) {
      System.out.println("H1 button pressed!");
      tablet.setH1Val(true);
      h1Button.setBoolean(false);
    }

    if (i4Button.getBoolean(false)) {
      System.out.println("I4 button pressed!");
      tablet.setI4Val(true);
      i4Button.setBoolean(false);
    }
    if (i3Button.getBoolean(false)) {
      System.out.println("I3 button pressed!");
      tablet.setI3Val(true);
      i3Button.setBoolean(false);
    }
    if (i2Button.getBoolean(false)) {
      System.out.println("I2 button pressed!");
      tablet.setI2Val(true);
      i2Button.setBoolean(false);
    }
    if (i1Button.getBoolean(false)) {
      System.out.println("I1 button pressed!");
      tablet.setI1Val(true);
      i1Button.setBoolean(false);
    }

    if (j4Button.getBoolean(false)) {
      System.out.println("J4 button pressed!");
      tablet.setJ4Val(true);
      j4Button.setBoolean(false);
    }
    if (j3Button.getBoolean(false)) {
      System.out.println("J3 button pressed!");
      tablet.setJ3Val(true);
      j3Button.setBoolean(false);
    }
    if (j2Button.getBoolean(false)) {
      System.out.println("J2 button pressed!");
      tablet.setJ2Val(true);
      j2Button.setBoolean(false);
    }
    if (j1Button.getBoolean(false)) {
      System.out.println("J1 button pressed!");
      tablet.setJ1Val(true);
      j1Button.setBoolean(false);
    }

    if (k4Button.getBoolean(false)) {
      System.out.println("K4 button pressed!");
      tablet.setK4Val(true);
      k4Button.setBoolean(false);
    }
    if (k3Button.getBoolean(false)) {
      System.out.println("K3 button pressed!");
      tablet.setK3Val(true);
      k3Button.setBoolean(false);
    }
    if (k2Button.getBoolean(false)) {
      System.out.println("K2 button pressed!");
      tablet.setK2Val(true);
      k2Button.setBoolean(false);
    }
    if (k1Button.getBoolean(false)) {
      System.out.println("K1 button pressed!");
      tablet.setK1Val(true);
      k1Button.setBoolean(false);
    }

    if (l4Button.getBoolean(false)) {
      System.out.println("L4 button pressed!");
      tablet.setL4Val(true);
      l4Button.setBoolean(false);
    }
    if (l3Button.getBoolean(false)) {
      System.out.println("L3 button pressed!");
      tablet.setL3Val(true);
      l3Button.setBoolean(false);
    }
    if (l2Button.getBoolean(false)) {
      System.out.println("L2 button pressed!");
      tablet.setL2Val(true);
      l2Button.setBoolean(false);
    }
    if (l1Button.getBoolean(false)) {
      System.out.println("L1 button pressed!");
      tablet.setL1Val(true);
      l1Button.setBoolean(false);
    }

    if (Alg1.getBoolean(false)) {
      System.out.println("Alg1 button pressed!");
      Alg1.setBoolean(false);
    }
    if (Alg2.getBoolean(false)) {
      System.out.println("Alg2 button pressed!");
      Alg2.setBoolean(false);
    }
    if (Alg3.getBoolean(false)) {
      System.out.println("Alg3 button pressed!");
      Alg3.setBoolean(false);
    }
    if (Alg4.getBoolean(false)) {
      System.out.println("Alg4 button pressed!");
      Alg4.setBoolean(false);
    }
    if (Alg5.getBoolean(false)) {
      System.out.println("Alg5 button pressed!");
      Alg5.setBoolean(false);
    }
    if (Alg6.getBoolean(false)) {
      System.out.println("Alg6 button pressed!");
      Alg6.setBoolean(false);
    }

    if (leftCoralStation.getBoolean(false)) {
      System.out.println("Left Coral Station button pressed!");
      tablet.setLCSVal(true);
      leftCoralStation.setBoolean(false);
    }
    if (rightCoralStation.getBoolean(false)) {
      System.out.println("Right Coral Station button pressed!");
      tablet.setRCSVal(true);
      rightCoralStation.setBoolean(false);
    }
    if (algaeInBarge.getBoolean(false)) {
      System.out.println("Algae In Barge button pressed!");
      tablet.setBrgVal(true);
      algaeInBarge.setBoolean(false);
    }
    if (proccesor.getBoolean(false)) {
      System.out.println("Processor button pressed!");
      tablet.setPrcsrVal(true);
      proccesor.setBoolean(false);
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (a4Button.getBoolean(false)) {
      System.out.println("A4 button pressed!");
      tablet.setA4Val(true);
      a4Button.setBoolean(false);
    }
    if (a3Button.getBoolean(false)) {
      System.out.println("A3 button pressed!");
      tablet.setA3Val(true);
      a3Button.setBoolean(false);
    }
    if (a2Button.getBoolean(false)) {
      System.out.println("A2 button pressed!");
      tablet.setA2Val(true);
      a2Button.setBoolean(false);
    }
    if (a1Button.getBoolean(false)) {
      System.out.println("A1 button pressed!");
      tablet.setA1Val(true);
      a1Button.setBoolean(false);
    }

    if (b4Button.getBoolean(false)) {
      System.out.println("B4 button pressed!");
      tablet.setB4Val(true);
      b4Button.setBoolean(false);
    }
    if (b3Button.getBoolean(false)) {
      System.out.println("B3 button pressed!");
      tablet.setB3Val(true);
      b3Button.setBoolean(false);
    }
    if (b2Button.getBoolean(false)) {
      System.out.println("B2 button pressed!");
      tablet.setB2Val(true);
      b2Button.setBoolean(false);
    }
    if (b1Button.getBoolean(false)) {
      System.out.println("B1 button pressed!");
      tablet.setB1Val(true);
      b1Button.setBoolean(false);
    }

    if (c4Button.getBoolean(false)) {
      System.out.println("C4 button pressed!");
      tablet.setC4Val(true);
      c4Button.setBoolean(false);
    }
    if (c3Button.getBoolean(false)) {
      System.out.println("C3 button pressed!");
      tablet.setC3Val(true);
      c3Button.setBoolean(false);
    }
    if (c2Button.getBoolean(false)) {
      System.out.println("C2 button pressed!");
      tablet.setC2Val(true);
      c2Button.setBoolean(false);
    }
    if (c1Button.getBoolean(false)) {
      System.out.println("C1 button pressed!");
      tablet.setC1Val(true);
      c1Button.setBoolean(false);
    }

    if (d4Button.getBoolean(false)) {
      System.out.println("D4 button pressed!");
      tablet.setD4Val(true);
      d4Button.setBoolean(false);
    }
    if (d3Button.getBoolean(false)) {
      System.out.println("D3 button pressed!");
      tablet.setD3Val(true);
      d3Button.setBoolean(false);
    }
    if (d2Button.getBoolean(false)) {
      System.out.println("D2 button pressed!");
      tablet.setD2Val(true);
      d2Button.setBoolean(false);
    }
    if (d1Button.getBoolean(false)) {
      System.out.println("D1 button pressed!");
      tablet.setD1Val(true);
      d1Button.setBoolean(false);
    }

    if (e4Button.getBoolean(false)) {
      System.out.println("E4 button pressed!");
      tablet.setE4Val(true);
      e4Button.setBoolean(false);
    }
    if (e3Button.getBoolean(false)) {
      System.out.println("E3 button pressed!");
      tablet.setE3Val(true);
      e3Button.setBoolean(false);
    }
    if (e2Button.getBoolean(false)) {
      System.out.println("E2 button pressed!");
      tablet.setE2Val(true);
      e2Button.setBoolean(false);
    }
    if (e1Button.getBoolean(false)) {
      System.out.println("E1 button pressed!");
      tablet.setE1Val(true);
      e1Button.setBoolean(false);
    }

    if (f4Button.getBoolean(false)) {
      System.out.println("F4 button pressed!");
      tablet.setF4Val(true);
      f4Button.setBoolean(false);
    }
    if (f3Button.getBoolean(false)) {
      System.out.println("F3 button pressed!");
      tablet.setF3Val(true);
      f3Button.setBoolean(false);
    }
    if (f2Button.getBoolean(false)) {
      System.out.println("F2 button pressed!");
      tablet.setF2Val(true);
      f2Button.setBoolean(false);
    }
    if (f1Button.getBoolean(false)) {
      System.out.println("F1 button pressed!");
      tablet.setF1Val(true);
      f1Button.setBoolean(false);
    }

    if (g4Button.getBoolean(false)) {
      System.out.println("G4 button pressed!");
      tablet.setG4Val(true);
      g4Button.setBoolean(false);
    }
    if (g3Button.getBoolean(false)) {
      System.out.println("G3 button pressed!");
      tablet.setG3Val(true);
      g3Button.setBoolean(false);
    }
    if (g2Button.getBoolean(false)) {
      System.out.println("G2 button pressed!");
      tablet.setG2Val(true);
      g2Button.setBoolean(false);
    }
    if (g1Button.getBoolean(false)) {
      System.out.println("G1 button pressed!");
      tablet.setG1Val(true);
      g1Button.setBoolean(false);
    }

    if (h4Button.getBoolean(false)) {
      System.out.println("H4 button pressed!");
      tablet.setH4Val(true);
      h4Button.setBoolean(false);
    }
    if (h3Button.getBoolean(false)) {
      System.out.println("H3 button pressed!");
      tablet.setH3Val(true);
      h3Button.setBoolean(false);
    }
    if (h2Button.getBoolean(false)) {
      System.out.println("H2 button pressed!");
      tablet.setH2Val(true);
      h2Button.setBoolean(false);
    }
    if (h1Button.getBoolean(false)) {
      System.out.println("H1 button pressed!");
      tablet.setH1Val(true);
      h1Button.setBoolean(false);
    }

    if (i4Button.getBoolean(false)) {
      System.out.println("I4 button pressed!");
      tablet.setI4Val(true);
      i4Button.setBoolean(false);
    }
    if (i3Button.getBoolean(false)) {
      System.out.println("I3 button pressed!");
      tablet.setI3Val(true);
      i3Button.setBoolean(false);
    }
    if (i2Button.getBoolean(false)) {
      System.out.println("I2 button pressed!");
      tablet.setI2Val(true);
      i2Button.setBoolean(false);
    }
    if (i1Button.getBoolean(false)) {
      System.out.println("I1 button pressed!");
      tablet.setI1Val(true);
      i1Button.setBoolean(false);
    }

    if (j4Button.getBoolean(false)) {
      System.out.println("J4 button pressed!");
      tablet.setJ4Val(true);
      j4Button.setBoolean(false);
    }
    if (j3Button.getBoolean(false)) {
      System.out.println("J3 button pressed!");
      tablet.setJ3Val(true);
      j3Button.setBoolean(false);
    }
    if (j2Button.getBoolean(false)) {
      System.out.println("J2 button pressed!");
      tablet.setJ2Val(true);
      j2Button.setBoolean(false);
    }
    if (j1Button.getBoolean(false)) {
      System.out.println("J1 button pressed!");
      tablet.setJ1Val(true);
      j1Button.setBoolean(false);
    }

    if (k4Button.getBoolean(false)) {
      System.out.println("K4 button pressed!");
      tablet.setK4Val(true);
      k4Button.setBoolean(false);
    }
    if (k3Button.getBoolean(false)) {
      System.out.println("K3 button pressed!");
      tablet.setK3Val(true);
      k3Button.setBoolean(false);
    }
    if (k2Button.getBoolean(false)) {
      System.out.println("K2 button pressed!");
      tablet.setK2Val(true);
      k2Button.setBoolean(false);
    }
    if (k1Button.getBoolean(false)) {
      System.out.println("K1 button pressed!");
      tablet.setK1Val(true);
      k1Button.setBoolean(false);
    }

    if (l4Button.getBoolean(false)) {
      System.out.println("L4 button pressed!");
      tablet.setL4Val(true);
      l4Button.setBoolean(false);
    }
    if (l3Button.getBoolean(false)) {
      System.out.println("L3 button pressed!");
      tablet.setL3Val(true);
      l3Button.setBoolean(false);
    }
    if (l2Button.getBoolean(false)) {
      System.out.println("L2 button pressed!");
      tablet.setL2Val(true);
      l2Button.setBoolean(false);
    }
    if (l1Button.getBoolean(false)) {
      System.out.println("L1 button pressed!");
      tablet.setL1Val(true);
      l1Button.setBoolean(false);
    }

    if (Alg1.getBoolean(false)) {
      System.out.println("Alg1 button pressed!");
      Alg1.setBoolean(false);
    }
    if (Alg2.getBoolean(false)) {
      System.out.println("Alg2 button pressed!");
      Alg2.setBoolean(false);
    }
    if (Alg3.getBoolean(false)) {
      System.out.println("Alg3 button pressed!");
      Alg3.setBoolean(false);
    }
    if (Alg4.getBoolean(false)) {
      System.out.println("Alg4 button pressed!");
      Alg4.setBoolean(false);
    }
    if (Alg5.getBoolean(false)) {
      System.out.println("Alg5 button pressed!");
      Alg5.setBoolean(false);
    }
    if (Alg6.getBoolean(false)) {
      System.out.println("Alg6 button pressed!");
      Alg6.setBoolean(false);
    }

    if (leftCoralStation.getBoolean(false)) {
      System.out.println("Left Coral Station button pressed!");
      tablet.setLCSVal(true);
      leftCoralStation.setBoolean(false);
    }
    if (rightCoralStation.getBoolean(false)) {
      System.out.println("Right Coral Station button pressed!");
      tablet.setRCSVal(true);
      rightCoralStation.setBoolean(false);
    }
    if (algaeInBarge.getBoolean(false)) {
      System.out.println("Algae In Barge button pressed!");
      tablet.setBrgVal(true);
      algaeInBarge.setBoolean(false);
    }
    if (proccesor.getBoolean(false)) {
      System.out.println("Processor button pressed!");
      tablet.setPrcsrVal(true);
      proccesor.setBoolean(false);
    }
  }
}
