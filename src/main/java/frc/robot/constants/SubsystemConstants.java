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

// does not include swerve constants

package frc.robot.constants;

public final class SubsystemConstants {
  public static boolean coralStuckMode = false;
  public static final String CANIVORE_ID_STRING = "CAN Bus 2";
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final boolean tuningMode = true;

  public static final double NEAR_FAR_AT_REEF_OFFSET = -0.47;
  public static final double NEAR_FAR_AWAY_REEF_OFFSET = -0.9;
  public static final double LEFT_RIGHT_BRANCH_OFFSET = -0.024;

  public static class IntakeConstants {
    public static final double CURRENT_LIMIT = 40.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;
  }

  public static final class ShooterConstants {}

  public static class ElevatorConstants {
    public static final double CURRENT_LIMIT = 40.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double STOW_SETPOINT_INCH = -0.02;

    public static final double DEFAULT_THRESHOLD = 0.1;

    public static final double ELEVATOR_GEAR_RATIO = 12.;

    public static final double PROCESSOR_SETPOINT_INCHES = 4;

    public static final double CANRAGE_ZERO_SETPOINT_INCHES = 2;

    // TODO:: NEED TO FIND
    public static final double L1_SETPOINT_INCHES = 0;
    public static final double L2_SETPOINT_INCHES = 7.5;
    public static final double L3_SETPOINT_INCHES = 14.5;
    public static final double L4_SETPOINT_INCHES = 27.4;
    public static final double BARGE_SETPOINT = 27.5;
  }

  public static final class ClimberConstants {
    public static final double CURRENT_LIMIT = 35.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double DEFAULT_THRESHOLD = 0.1;
    public static final double ARM_GEAR_RATIO = 1;

    public static final double STOW_SETPOINT_DEG = 108;
    public static final double DEPLOY_SETPOINT_DEG = -20;
  }

  public static class ScoralArmConstants {
    public static final double CURRENT_LIMIT = 50.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;

    public static final double DEFAULT_THRESHOLD = 1;
    public static final double ARM_GEAR_RATIO = (25.0 / 1) / (14.0 / 32);

    public static final double STOW_SETPOINT_DEG = 96.8;
    public static final double LOW_CORAL_SCORING_SETPOINT_DEG = 76.5;
    public static final double L4_CORAL_SCORING_SETPOINT_DEG = 64;
    public static final double BARGE_SETPOINT_DEG = 85;
    public static final double PROCESSOR_SETPOINT_DEG = 20;

    
    public static final double DEFAULT_MAX_VELOCITY_DEG_PER_SEC = 150; // was at 90
    public static final double DEFAULT_MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 300; // was at 190
  }

  public static class LEDConstants {
    public static final int NUMBER_LEDS = 57 + 24 + 13;
  }

  public static enum LED_STATE {
    BLUE,
    RED,
    YELLOW,
    GREEN,
    GREY,
    PURPLE,
    PAPAYA_ORANGE,
    FLASHING_GREEN,
    FLASHING_RED,
    FLASHING_BLUE,
    FIRE,
    OFF
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum CoralState {
    DEFAULT,
    NO_CORAL,
    SENSOR,
    CURRENT
  }

  public static enum AlgaeState {
    DEFAULT,
    NO_ALGAE,
    CURRENT,
  }

  public static enum ElevatorState {
    ZERO,
    STOW,
    L1,
    L2,
    L3,
    L4,
    SOURCE,
    PROCESSOR
  }

  public static enum ScoralFlywheelState {
    ZERO,
    INTAKING_CORAL,
    INTAKING_ALGAE,
    SCORING_CORAL,
    SCORING_ALGAE
  }

  public static enum ClimberState {
    STOW,
    STAGE_ONE,
    STAGE_TWO,
    HANG
  }

  public static enum AlignState {
    ALIGNING,
    IN_POSITION
  }

  public static enum DriveState {
    FULLSPEED,
    SLOW
  }

  public static enum SuperStructureState {
    // NONE,
    STOW,
    STOW_ALGAE,
    SOURCE,
    PROCESSOR,
    BARGE_EXTEND,
    ALGAE_SCORE,
    L1,
    L2,
    L3,
    L4,
    SCORING_CORAL,
    INTAKE_ALGAE
  }

  public static final double CORAL_DIST = 4.0; // 1300; // CHANGE THIS
}
