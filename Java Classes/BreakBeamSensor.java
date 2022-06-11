// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class BreakBeamSensor {
  DigitalInput beamSensor;
  /**
   * Creates a new BreakBeamSensor.
   * @param channel The DIO channel on the RoboRio for the sensor
   */
  public BreakBeamSensor(int channel) {
    beamSensor = new DigitalInput(channel);
  }

  /**
   * Detects if something has broken the IR beam. If the beam is broken that either means there is some object in between the two sensors
   * OR the sensors are misaligned.
   * @return A boolean indicating if the beam has been broken
   */
  public boolean isBeamBroken() {
    return !beamSensor.get();
  }
}