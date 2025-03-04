<<<<<<< Updated upstream
package frc.robot.util;

// Copyright (c) 2024 FRC 6328
=======
// Copyright (c) 2025 FRC 6328
>>>>>>> Stashed changes
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

<<<<<<< Updated upstream
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConstants.fieldLength - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(apply(translation.getX()), translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
      return new Translation3d(
          apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
    } else {
      return translation3d;
    }
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
=======


import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;


public class AllianceFlipUtil {
  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  // public static VehicleState apply(VehicleState state) {
  //   return shouldFlip()
  //       ? disableHAL.newBuilder()
  //           .setX(applyX(state.getX()))
  //           .setY(applyY(state.getY()))
  //           .setTheta(apply(Rotation2d.fromRadians(state.getTheta())).getRadians())
  //           .setVx(-state.getVx())
  //           .setVy(-state.getVy())
  //           .setOmega(state.getOmega())
  //           .addAllModuleForces(
  //               state.getModuleForcesList().stream()
  //                   .map(
  //                       forces ->
  //                           ModuleForce.newBuilder()
  //                               .setFx(-forces.getFx())
  //                               .setFy(-forces.getFy())
  //                               .build())
  //                   .toList())
  //           .build()
  //       : state;
  // }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
>>>>>>> Stashed changes
