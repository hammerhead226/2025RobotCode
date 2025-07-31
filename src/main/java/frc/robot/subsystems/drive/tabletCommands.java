package frc.robot.subsystems.drive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class tabletCommands {
  private NetworkTableEntry a4, a3, a2, a1, b4, b3, b2, b1, c4, c3, c2, c1;
  private NetworkTableEntry d4, d3, d2, d1, e4, e3, e2, e1, f4, f3, f2, f1;
  private NetworkTableEntry g4, g3, g2, g1, h4, h3, h2, h1, i4, i3, i2, i1;
  private NetworkTableEntry j4, j3, j2, j1, k4, k3, k2, k1, l4, l3, l2, l1;
  private NetworkTableEntry lCS, rCS, prcsr, brg;

  public tabletCommands() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("tablet");

    a4 = table.getEntry("A4");
    a3 = table.getEntry("A3");
    a2 = table.getEntry("A2");
    a1 = table.getEntry("A1");
    b4 = table.getEntry("B4");
    b3 = table.getEntry("B3");
    b2 = table.getEntry("B2");
    b1 = table.getEntry("B1");
    c4 = table.getEntry("C4");
    c3 = table.getEntry("C3");
    c2 = table.getEntry("C2");
    c1 = table.getEntry("C1");
    d4 = table.getEntry("D4");
    d3 = table.getEntry("D3");
    d2 = table.getEntry("D2");
    d1 = table.getEntry("D1");
    e4 = table.getEntry("E4");
    e3 = table.getEntry("E3");
    e2 = table.getEntry("E2");
    e1 = table.getEntry("E1");
    f4 = table.getEntry("F4");
    f3 = table.getEntry("F3");
    f2 = table.getEntry("F2");
    f1 = table.getEntry("F1");
    g4 = table.getEntry("G4");
    g3 = table.getEntry("G3");
    g2 = table.getEntry("G2");
    g1 = table.getEntry("G1");
    h4 = table.getEntry("H4");
    h3 = table.getEntry("H3");
    h2 = table.getEntry("H2");
    h1 = table.getEntry("H1");
    i4 = table.getEntry("I4");
    i3 = table.getEntry("I3");
    i2 = table.getEntry("I2");
    i1 = table.getEntry("I1");
    j4 = table.getEntry("J4");
    j3 = table.getEntry("J3");
    j2 = table.getEntry("J2");
    j1 = table.getEntry("J1");
    k4 = table.getEntry("K4");
    k3 = table.getEntry("K3");
    k2 = table.getEntry("K2");
    k1 = table.getEntry("K1");
    l4 = table.getEntry("L4");
    l3 = table.getEntry("L3");
    l2 = table.getEntry("L2");
    l1 = table.getEntry("L1");

    lCS = table.getEntry("lCS");
    rCS = table.getEntry("rCS");
    prcsr = table.getEntry("prcsr");
    brg = table.getEntry("brg");
  }

  // Generate getters and setters for each entry

  public boolean getA4Val() {
    return a4.getBoolean(false);
  }

  public void setA4Val(boolean value) {
    a4.setBoolean(value);
  }

  public boolean getA3Val() {
    return a3.getBoolean(false);
  }

  public void setA3Val(boolean value) {
    a3.setBoolean(value);
  }

  public boolean getA2Val() {
    return a2.getBoolean(false);
  }

  public void setA2Val(boolean value) {
    a2.setBoolean(value);
  }

  public boolean getA1Val() {
    return a1.getBoolean(false);
  }

  public void setA1Val(boolean value) {
    a1.setBoolean(value);
  }

  public boolean getB4Val() {
    return b4.getBoolean(false);
  }

  public void setB4Val(boolean value) {
    b4.setBoolean(value);
  }

  public boolean getB3Val() {
    return b3.getBoolean(false);
  }

  public void setB3Val(boolean value) {
    b3.setBoolean(value);
  }

  public boolean getB2Val() {
    return b2.getBoolean(false);
  }

  public void setB2Val(boolean value) {
    b2.setBoolean(value);
  }

  public boolean getB1Val() {
    return b1.getBoolean(false);
  }

  public void setB1Val(boolean value) {
    b1.setBoolean(value);
  }

  public boolean getC4Val() {
    return c4.getBoolean(false);
  }

  public void setC4Val(boolean value) {
    c4.setBoolean(value);
  }

  public boolean getC3Val() {
    return c3.getBoolean(false);
  }

  public void setC3Val(boolean value) {
    c3.setBoolean(value);
  }

  public boolean getC2Val() {
    return c2.getBoolean(false);
  }

  public void setC2Val(boolean value) {
    c2.setBoolean(value);
  }

  public boolean getC1Val() {
    return c1.getBoolean(false);
  }

  public void setC1Val(boolean value) {
    c1.setBoolean(value);
  }

  public boolean getD4Val() {
    return d4.getBoolean(false);
  }

  public void setD4Val(boolean value) {
    d4.setBoolean(value);
  }

  public boolean getD3Val() {
    return d3.getBoolean(false);
  }

  public void setD3Val(boolean value) {
    d3.setBoolean(value);
  }

  public boolean getD2Val() {
    return d2.getBoolean(false);
  }

  public void setD2Val(boolean value) {
    d2.setBoolean(value);
  }

  public boolean getD1Val() {
    return d1.getBoolean(false);
  }

  public void setD1Val(boolean value) {
    d1.setBoolean(value);
  }

  public boolean getE4Val() {
    return e4.getBoolean(false);
  }

  public void setE4Val(boolean value) {
    e4.setBoolean(value);
  }

  public boolean getE3Val() {
    return e3.getBoolean(false);
  }

  public void setE3Val(boolean value) {
    e3.setBoolean(value);
  }

  public boolean getE2Val() {
    return e2.getBoolean(false);
  }

  public void setE2Val(boolean value) {
    e2.setBoolean(value);
  }

  public boolean getE1Val() {
    return e1.getBoolean(false);
  }

  public void setE1Val(boolean value) {
    e1.setBoolean(value);
  }

  public boolean getF4Val() {
    return f4.getBoolean(false);
  }

  public void setF4Val(boolean value) {
    f4.setBoolean(value);
  }

  public boolean getF3Val() {
    return f3.getBoolean(false);
  }

  public void setF3Val(boolean value) {
    f3.setBoolean(value);
  }

  public boolean getF2Val() {
    return f2.getBoolean(false);
  }

  public void setF2Val(boolean value) {
    f2.setBoolean(value);
  }

  public boolean getF1Val() {
    return f1.getBoolean(false);
  }

  public void setF1Val(boolean value) {
    f1.setBoolean(value);
  }

  public boolean getG4Val() {
    return g4.getBoolean(false);
  }

  public void setG4Val(boolean value) {
    g4.setBoolean(value);
  }

  public boolean getG3Val() {
    return g3.getBoolean(false);
  }

  public void setG3Val(boolean value) {
    g3.setBoolean(value);
  }

  public boolean getG2Val() {
    return g2.getBoolean(false);
  }

  public void setG2Val(boolean value) {
    g2.setBoolean(value);
  }

  public boolean getG1Val() {
    return g1.getBoolean(false);
  }

  public void setG1Val(boolean value) {
    g1.setBoolean(value);
  }

  public boolean getH4Val() {
    return h4.getBoolean(false);
  }

  public void setH4Val(boolean value) {
    h4.setBoolean(value);
  }

  public boolean getH3Val() {
    return h3.getBoolean(false);
  }

  public void setH3Val(boolean value) {
    h3.setBoolean(value);
  }

  public boolean getH2Val() {
    return h2.getBoolean(false);
  }

  public void setH2Val(boolean value) {
    h2.setBoolean(value);
  }

  public boolean getH1Val() {
    return h1.getBoolean(false);
  }

  public void setH1Val(boolean value) {
    h1.setBoolean(value);
  }

  public boolean getI4Val() {
    return i4.getBoolean(false);
  }

  public void setI4Val(boolean value) {
    i4.setBoolean(value);
  }

  public boolean getI3Val() {
    return i3.getBoolean(false);
  }

  public void setI3Val(boolean value) {
    i3.setBoolean(value);
  }

  public boolean getI2Val() {
    return i2.getBoolean(false);
  }

  public void setI2Val(boolean value) {
    i2.setBoolean(value);
  }

  public boolean getI1Val() {
    return i1.getBoolean(false);
  }

  public void setI1Val(boolean value) {
    i1.setBoolean(value);
  }

  public boolean getJ4Val() {
    return j4.getBoolean(false);
  }

  public void setJ4Val(boolean value) {
    j4.setBoolean(value);
  }

  public boolean getJ3Val() {
    return j3.getBoolean(false);
  }

  public void setJ3Val(boolean value) {
    j3.setBoolean(value);
  }

  public boolean getJ2Val() {
    return j2.getBoolean(false);
  }

  public void setJ2Val(boolean value) {
    j2.setBoolean(value);
  }

  public boolean getJ1Val() {
    return j1.getBoolean(false);
  }

  public void setJ1Val(boolean value) {
    j1.setBoolean(value);
  }

  public boolean getK4Val() {
    return k4.getBoolean(false);
  }

  public void setK4Val(boolean value) {
    k4.setBoolean(value);
  }

  public boolean getK3Val() {
    return k3.getBoolean(false);
  }

  public void setK3Val(boolean value) {
    k3.setBoolean(value);
  }

  public boolean getK2Val() {
    return k2.getBoolean(false);
  }

  public void setK2Val(boolean value) {
    k2.setBoolean(value);
  }

  public boolean getK1Val() {
    return k1.getBoolean(false);
  }

  public void setK1Val(boolean value) {
    k1.setBoolean(value);
  }

  public boolean getL4Val() {
    return l4.getBoolean(false);
  }

  public void setL4Val(boolean value) {
    l4.setBoolean(value);
  }

  public boolean getL3Val() {
    return l3.getBoolean(false);
  }

  public void setL3Val(boolean value) {
    l3.setBoolean(value);
  }

  public boolean getL2Val() {
    return l2.getBoolean(false);
  }

  public void setL2Val(boolean value) {
    l2.setBoolean(value);
  }

  public boolean getL1Val() {
    return l1.getBoolean(false);
  }

  public void setL1Val(boolean value) {
    l1.setBoolean(value);
  }

  public boolean getLCSVal() {
    return lCS.getBoolean(false);
  }

  public void setLCSVal(boolean value) {
    lCS.setBoolean(value);
  }

  public boolean getRCSVal() {
    return rCS.getBoolean(false);
  }

  public void setRCSVal(boolean value) {
    rCS.setBoolean(value);
  }

  public boolean getPrcsrVal() {
    return prcsr.getBoolean(false);
  }

  public void setPrcsrVal(boolean value) {
    prcsr.setBoolean(value);
  }

  public boolean getBrgVal() {
    return brg.getBoolean(false);
  }

  public void setBrgVal(boolean value) {
    brg.setBoolean(value);
  }
}
