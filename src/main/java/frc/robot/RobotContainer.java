// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.TargetAlign;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class RobotContainer { 
  private final VisionSubsystem visionSub = new VisionSubsystem();
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();

  public RobotContainer() {
    
  }


  public Command getAutonomousCommand() {
    return new TargetAlign(driveSub, visionSub);
  }
}
