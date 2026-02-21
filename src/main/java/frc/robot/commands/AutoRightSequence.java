// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRightSequence extends SequentialCommandGroup {
  /** Creates a new LaunchSequence. */
  public AutoRightSequence(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new TargetLock(driveSubsystem, false),
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(fuelSubsystem).withTimeout(FuelConstants.AUTO_LAUNCH_SECONDS));
     

        //right side rotation
        //shoot
        //rotate intake toward outpost
        //move to outpost, with hopper oriented toward outpost
        //wait 2s
        //rotate to hub
        //move
        //shoot

        
  }
}
