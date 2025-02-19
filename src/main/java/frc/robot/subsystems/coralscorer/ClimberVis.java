package frc.robot.subsystems.coralscorer;

// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimberVis {
  private final String key;
  private final LoggedMechanism2d panel;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d mecha;

  public ClimberVis(String key, Color color) {

    this.key = key;
    this.panel = new LoggedMechanism2d(100, 100, new Color8Bit(Color.kWhite));
    this.root = panel.getRoot("mechanism", 49.7, 0.1);
    this.mecha =
        root.append(
            new LoggedMechanismLigament2d("climberArms", 0.55, 0, 10, new Color8Bit(color)));

    Logger.recordOutput("ClimberVis/mechanism2d/" + key, this.panel);
  }

  public void update(double angle) {
    // mecha.setLength(position);
    // root.setPosition(50, position);
    mecha.setAngle(angle);
    Logger.recordOutput("ClimberVis/mechanism2d/" + key, this.panel);
  }
}
