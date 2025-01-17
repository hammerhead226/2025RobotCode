package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ElevatorVis {
  private final String key;
  private final Mechanism2d panel;
  private final MechanismRoot2d root;
  private final MechanismLigament2d mecha;
  // @AutoLogOutput private final LoggedMechanism2d mech = panel;

  public ElevatorVis(String key, Color color) {

    this.key = key;
    this.panel = new Mechanism2d(100, 100, new Color8Bit(Color.kWhite));
    this.root = panel.getRoot("mechanism", 30, 16);
    this.mecha = root.append(new MechanismLigament2d("elevator", 5, 0, 10, new Color8Bit(color)));

    SmartDashboard.putData("ElevatorVis/mechanism2d/" + key, this.panel);
  }

  public void update(double position) {
    mecha.setLength(position);
    Logger.recordOutput("ElevatorVis/mechanism2d/" + key, this.panel);
  }
}
