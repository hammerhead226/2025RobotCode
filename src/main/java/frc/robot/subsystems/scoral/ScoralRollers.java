package frc.robot.subsystems.scoral;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.constants.SubsystemConstants.CoralState;
import frc.robot.subsystems.commoniolayers.FlywheelIO;
import frc.robot.subsystems.commoniolayers.FlywheelIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ScoralRollers extends SubsystemBase {
  private final FlywheelIO rollers;
  private final DistanceSensorIO sensor;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final DistanceSensorIOInputsAutoLogged sInputs = new DistanceSensorIOInputsAutoLogged();
  private SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;
  private AlgaeState lastAlgaeState;

  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 1);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 1);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 1);

  private CoralState lastCoralState;

  /** Creates a new Flywheel. */
  public ScoralRollers(
      FlywheelIO rollers,
      DistanceSensorIO sensor,
      CoralState lastCoralState,
      AlgaeState lastAlgaeState) {
    this.rollers = rollers;
    this.sensor = sensor;
    this.lastCoralState = lastCoralState;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (SimConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        rollers.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.1);
        rollers.configurePID(0, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));

    updateTunableNumbers();
  }

  @Override
  public void periodic() {
    rollers.updateInputs(inputs);
    sensor.updateInputs(sInputs);
    Logger.processInputs("Scoral Rollers", inputs);
    Logger.processInputs("Scoral Rollers/CANrange", sInputs);
    Logger.recordOutput("Debug Scoral Rollers/running command", getCurrentCommand() != null);
    Logger.recordOutput(
        "Debug Scoral Rollers/running command name",
        getCurrentCommand() == null ? "" : getCurrentCommand().getName());

    updateTunableNumbers();
  }

  /** Run open loop at the specified voltage. */
  int runVoltsCount = 1;

  public void runVolts(double volts) {
    Logger.recordOutput("Debug Rollers/runVoltsCount", runVoltsCount++);
    rollers.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    rollers.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Debug Rollers/SetpointRPM", velocityRPM);
  }

  public Command runVoltsCommmand(double volts) {
    // Elastic.sendNotification(
    //     new Notification(
    //         NotificationLevel.INFO, "Notice", "Flywheel is being run at " + volts + " volts."));
    return new InstantCommand(() -> runVolts(volts), this);
  }

  public Command runVelocityCommand(double velocityRPM) {

    return new InstantCommand(() -> runVelocity(velocityRPM), this).withTimeout(5);
  }

  public Command flywheelStop() {
    return new InstantCommand(() -> stop(), this);
  }

  /** Stops the flywheel. */
  public void stop() {
    rollers.stop();
  }

  public Command stopCommand() {

    return new InstantCommand(() -> stop(), this);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public AlgaeState seesAlgae() {
    Logger.recordOutput("Debug Rollers/see algae val", "default");
    if (inputs.leaderStatorCurrentAmps > 9) {
      Logger.recordOutput("Debug Rollers/see algae val", "current");
      lastAlgaeState = AlgaeState.CURRENT;
      return AlgaeState.CURRENT;

    } else {
      Logger.recordOutput("Debug Rollers/see algae val", "no algae");
      lastAlgaeState = AlgaeState.NO_ALGAE;
      return AlgaeState.NO_ALGAE;
    }
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Debug Rollers/Velocity RPM")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  int count = 0;

  public CoralState seesCoral() {
    Logger.recordOutput("Debug Rollers/see coral called", count++);
    Logger.recordOutput("Debug Rollers/see coral val", "default");

    if ((sInputs.distanceInches < SubsystemConstants.CORAL_DIST)) {
      Logger.recordOutput("Debug Rollers/see coral val", "sensor");
      lastCoralState = CoralState.SENSOR;
      return CoralState.SENSOR;

    } else if (inputs.leaderStatorCurrentAmps > 30) {
      Logger.recordOutput("Debug Rollers/see coral val", "current");
      lastCoralState = CoralState.CURRENT;
      return CoralState.CURRENT;

    } else {
      Logger.recordOutput("Debug Rollers/see coral val", "no coral");
      lastCoralState = CoralState.NO_CORAL;
      return CoralState.NO_CORAL;
    }
  }

  public CoralState getLastCoralState() {
    return lastCoralState;
  }

  public double getDistance() {
    return sInputs.distanceInches;
  }

  public void zero() {
    rollers.stop();
  }

  private void updateTunableNumbers() {
    if (kV.hasChanged(hashCode()) || kA.hasChanged(hashCode()) || kS.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
    }
  }
}
