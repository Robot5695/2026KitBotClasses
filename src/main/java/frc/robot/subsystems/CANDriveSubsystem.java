// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  
  /* private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower; */

  private final TalonSRX leftLeader;
  private final TalonSRX leftFollower;
  private final TalonSRX rightLeader;
  private final TalonSRX rightFollower;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    /* leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed); */

    leftLeader = new TalonSRX(LEFT_LEADER_ID);
    leftFollower = new TalonSRX(LEFT_FOLLOWER_ID);
    rightFollower = new TalonSRX(RIGHT_FOLLOWER_ID);
    rightLeader = new TalonSRX(RIGHT_LEADER_ID);

  }

  @Override
  public void periodic() {
  }

  public void driveArcade(double xSpeed, double zRotation) {
    //create drive code
    leftLeader.set(TalonSRXControlMode.PercentOutput,xSpeed-zRotation);
    leftFollower.set(TalonSRXControlMode.PercentOutput,xSpeed-zRotation);
    rightLeader.set(TalonSRXControlMode.PercentOutput,-xSpeed-zRotation);
    rightFollower.set(TalonSRXControlMode.PercentOutput,-xSpeed-zRotation);
    
  }

}
