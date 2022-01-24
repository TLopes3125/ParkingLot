// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

    protected TalonFX intake;
   // private XboxController m_controller;


  public Intake() {

  intake = new TalonFX(Constants.INTAKE);

  }

  public void setSpeed(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
