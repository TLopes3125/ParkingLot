// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Intake2 extends SubsystemBase {
  DoubleSolenoid solenoid = new DoubleSolenoid(0,1);
  Compressor comp = new Compressor();
  
  /** Creates a new Intake. */
  public Intake2() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
