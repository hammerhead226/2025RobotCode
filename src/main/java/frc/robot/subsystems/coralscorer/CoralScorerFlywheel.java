package frc.robot.subsystems.coralscorer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants.AlgaeState;
import frc.robot.subsystems.commoniolayers.FlywheelIO;
import frc.robot.subsystems.commoniolayers.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.coralIntake.flywheels.CoralIntakeSensorIOInputsAutoLogged;
import frc.robot.subsystems.newalgaeintake.FeederIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralScorerFlywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final CoralIntakeSensorIOInputsAutoLogged sInputs =
      new CoralIntakeSensorIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private AlgaeState lastAlgaeState;
  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public CoralScorerFlywheel(FlywheelIO io, AlgaeState lastAlgaeState) {
    this.io = io;
    this.lastAlgaeState = lastAlgaeState;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (SimConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        io.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        io.configurePID(0.0, 0.0, 0.0);
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
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  public Command runVoltsCommmand(double volts) {

    return new InstantCommand(() -> runVolts(volts), this);
  }

  public Command runVelocityCommand(double velocityRPM) {

    return new InstantCommand(() -> runVelocity(velocityRPM), this);
  }

  public Command flywheelStop() {
    return new InstantCommand(() -> stop(), this);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public AlgaeState seesAlgae() {
    Logger.recordOutput("see note val", "default");
    if (feedInputs.currentAmps > 13) {
      Logger.recordOutput("see note val", "current");
      lastAlgaeState = AlgaeState.CURRENT;
      return AlgaeState.CURRENT;

    } else {
      Logger.recordOutput("see note val", "no note");
      lastAlgaeState = AlgaeState.NO_ALGAE;
      return AlgaeState.NO_ALGAE;
    }
  }

  public AlgaeState getLastCoralState() {
    return lastAlgaeState;
  }
}
