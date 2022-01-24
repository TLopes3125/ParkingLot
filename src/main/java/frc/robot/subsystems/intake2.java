// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class intake2 extends SubsystemBase {
  /** Creates a new intake. */
  public intake2() {
    DoubleSolenoid solenoid = new DoubleSolenoid(null, 0,1);
    Compressor comp = new Compressor(null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
