// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldZone {
  private final double minX, maxX, minY, maxY;

  /** Define bounds in blue alliance coordinates */
  public FieldZone(double minX, double maxX, double minY, double maxY) {
    this.minX = minX;
    this.maxX = maxX;
    this.minY = minY;
    this.maxY = maxY;
  }

  public boolean contains(Translation2d position) {
    // Flip Position if on Red
    // if (Constants.isRedAlliance()) {
    //   // Mirror X across field centerline; Y stays the same
    //   position = FlippingUtil.flipFieldPosition(position);
    // }

    return position.getX() >= minX
        && position.getX() <= maxX
        && position.getY() >= minY
        && position.getY() <= maxY;
  }
}
