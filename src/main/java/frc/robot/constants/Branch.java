package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefHeight;

public class Branch {
  // indexed
  // 0-11 -> pipe A-L
  // 0,1,2 -> L2, L3, L4
  private static final Branch[][] branches = new Branch[12][3];

  static {
    for (int pipe = 0; pipe < 12; pipe++) {
      int branchPositionsIndex = 13 - pipe;
      branchPositionsIndex %= 12;
      branches[pipe][0] =
          new Branch(pipe, 2, Reef.branchPositions.get(branchPositionsIndex).get(ReefHeight.L2));
      branches[pipe][1] =
          new Branch(pipe, 3, Reef.branchPositions.get(branchPositionsIndex).get(ReefHeight.L3));
      branches[pipe][2] =
          new Branch(pipe, 4, Reef.branchPositions.get(branchPositionsIndex).get(ReefHeight.L4));
    }
  }

  // indexed
  // pipe: 0-11 -> pipe A-L
  // level: 2,3,4 -> L2, L3, L4
  public static Branch getBranch(int pipe, int level) {
    return branches[pipe][level - 2];
  }

  public final int pipe;
  public final int level;
  private final Pose3d pose;

  private Branch(int pipe, int level, Pose3d pose) {
    this.pipe = pipe;
    this.level = level;
    this.pose = pose;
  }

  public int getPipe() {
    return pipe;
  }

  public int getLevel() {
    return level;
  }

  public Pose3d getPose() {
    return pose;
  }

  public String toString() {
    return (char) ('A' + pipe) + " L" + level;
  }
}
