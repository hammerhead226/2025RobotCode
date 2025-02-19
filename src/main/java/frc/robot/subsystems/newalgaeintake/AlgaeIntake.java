package frc.robot.subsystems.newalgaeintake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.LoggedTunableNumber;

public class AlgaeIntake extends SubsystemBase {
  SimpleMotorFeedforward feederFFModel;

  private final AlgaeIntakeFeederIO feeder;
  private final AlgaeIntakeArmIO arm;

  private static final LoggedTunableNumber feederkP = new LoggedTunableNumber("feederkP");
  private static final LoggedTunableNumber feederkI = new LoggedTunableNumber("feederkI");
  private static final LoggedTunableNumber feederkD = new LoggedTunableNumber("feederkD");
  private static LoggedTunableNumber armkP = new LoggedTunableNumber("armkP");
  private static LoggedTunableNumber armkG = new LoggedTunableNumber("armkG");
  private static LoggedTunableNumber armkV = new LoggedTunableNumber("armkV");

  public AlgaeIntake(AlgaeIntakeFeederIO feeder, AlgaeIntakeArmIO arm) {
    switch (SimConstants.currentMode) {
      case REAL:
        feederFFModel = new SimpleMotorFeedforward(0, 0, 0);

        feederkP.initDefault(0.23); // make constant
        feederkI.initDefault(5); // make constant
        feederkD.initDefault(0);

        armkG.initDefault(0.29);
        armkV.initDefault(1);
        armkP.initDefault(1.123);
        break;
      case REPLAY:
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);

        armkG.initDefault(0.29);
        armkV.initDefault(1);
        armkP.initDefault(1.123);
        break;
      case SIM:
        feederFFModel = new SimpleMotorFeedforward(0, 0.5);

        feederkP.initDefault(10);
        feederkI.initDefault(0);
        feederkD.initDefault(0);

        armkG.initDefault(0.29);
        armkV.initDefault(1);
        armkP.initDefault(1.123);
        break;
      default:
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);

        armkG.initDefault(0.29);
        armkV.initDefault(1);
        armkP.initDefault(1.123);
        break;
    }

    this.feeder = feeder;
    feeder.configurePID(feederkP.get(), feederkI.get(), feederkD.get());

    this.arm = arm;
    arm.configurePID(armkP.get(), armkG.get(), armkV.get());
  }

  public void stopArm() {
    arm.stop();
  }

  public void stopFeeder() {
    feeder.stop();
  }

  public void setFeederRPM(double velocityRPM) {
    feeder.setVelocityRPS(velocityRPM / 60.0, feederFFModel.calculate(velocityRPM / 60.0));
  }

  public double getFeederError() {
    return 0; // feedInputs.velocitySetpointRPM - getFeederError();
  }

  public boolean atFeederSetpoint() {
    return Math.abs(getFeederError())
        <= SubsystemConstants.CoralScorerConstants.AlgaeScorerFlywheelConstants.THRESHOLD;
  }
}
