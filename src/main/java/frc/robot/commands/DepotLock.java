// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DepotLock extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  //IntakeSubsystem intakeSubsystem;
  CANFuelSubsystem fuelSubsystem;
  long depotMoveStartTime;

  public DepotLock(CANDriveSubsystem driveSystem, /*IntakeSubsystem intakeSystem,*/ CANFuelSubsystem fuelSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    //addRequirements(intakeSystem);
    addRequirements(fuelSystem);
    driveSubsystem = driveSystem;
    //intakeSubsystem = intakeSystem;
    fuelSubsystem = fuelSystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    depotMoveStartTime = System.currentTimeMillis();
      fuelSubsystem
        .setIntakeLauncherRoller(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
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

  double maxZ = 0.1;
  double maxX = 0.1;

  //positive is counter-clockwise, negative is clockwise
  double zSpeed = -ta*ty/(60);
  double xSpeed = (0.1); //Movement speed is constant
  if (zSpeed > maxZ)
  {
    zSpeed = maxZ;
  }
  if (zSpeed < -maxZ)
  {
    zSpeed = -maxZ;
  }
  if (xSpeed > maxX)
  {
    xSpeed = maxX;
  }
  if (xSpeed < -maxX)
  {
    xSpeed = -maxX;
  }
  driveSubsystem.driveArcade(xSpeed,zSpeed);//Choose center coordinates
} else { //No target in sight
  driveSubsystem.driveArcade(0.1,0); // Robot is moving slowly backwards if theres no target
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
//    return false;
double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?
    return ((System.currentTimeMillis()-depotMoveStartTime>5000) || Math.abs(ta)<2 && Math.abs(ty)<2 && hasTarget);
  }
}
