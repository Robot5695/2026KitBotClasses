// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetLock extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  

  public TargetLock(CANDriveSubsystem driveSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
    // Basic targeting data
double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
double tync = LimelightHelpers.getTYNC("");  // Vertical offset from principal pixel/point to target in degrees
    
if (hasTarget)
{
  driveSubsystem.driveArcade(0,0);
} else {
  driveSubsystem.driveArcade(0,0.2); // Robot is rotating slowly if theres no target
}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
