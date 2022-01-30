// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {

  // Drive Motors
  protected WPI_TalonFX leftLeader;

  protected WPI_TalonFX rightLeader;
  protected PigeonIMU pidgey;
  private final DifferentialDrive m_drive;
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new Chassis. */
  public Chassis() {
    
        // Set the motor IDs
        leftLeader = new WPI_TalonFX(Constants.LLEADER);

        rightLeader = new WPI_TalonFX(Constants.RLEADER);
        
        pidgey = new PigeonIMU(0);
        m_drive = new DifferentialDrive(leftLeader,rightLeader);
        m_odometry = new DifferentialDriveOdometry(new Rotation2d(getAngle()));

        // Set inverted
        leftLeader.setInverted(!Constants.LEFT_INVERTED);

        rightLeader.setInverted(!Constants.LEFT_INVERTED);

        leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void setSpeed(double lSpeed, double rSpeed) {
        leftLeader.set(ControlMode.PercentOutput, lSpeed*0.2);
        rightLeader.set(ControlMode.PercentOutput, rSpeed*0.2);
  }
  public double getAngle() {
    double[] ypr = new double[3];
    pidgey.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void resetEncoder(){
    leftLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    rightLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoder();
    m_odometry.resetPosition(pose, new Rotation2d(getAngle()));
  }

  public void zeroAllSensors() {
    leftLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    rightLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    pidgey.setYaw(0,30);
    pidgey.setAccumZAngle(0,30);
  }

  public void stop() {
    leftLeader.set(ControlMode.PercentOutput, 0);
    rightLeader.set(ControlMode.PercentOutput, 0);
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left encoder", leftLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("right encoder", rightLeader.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }
}