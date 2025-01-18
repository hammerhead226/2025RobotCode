package frc.robot.subsystems.arms;

// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class PivotVis {
  private final String key;
  private final LoggedMechanism2d panel;
  private final LoggedMechanismRoot2d root;
  private final LoggedMechanismLigament2d mecha;

  public PivotVis(String key, Color color) {

    this.key = key;
    this.panel = new LoggedMechanism2d(100, 100, new Color8Bit(Color.kWhite));
    this.root = panel.getRoot("mechanism", 30, 16);
    this.mecha =
        root.append(new LoggedMechanismLigament2d("arms", 5, 0, 10, new Color8Bit(color)));

    Logger.recordOutput("PivotVis/mechanism2d/" + key, this.panel);
  }

  public void update(double position) {
    mecha.setLength(position);
    mecha.setAngle(position);
    Logger.recordOutput("PivotVis/mechanism2d/" + key, this.panel);
    // Logger.recordOutput(key, null);
  }
}
