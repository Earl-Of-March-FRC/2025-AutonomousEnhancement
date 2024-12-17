// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AlignArmToTag;
import frc.robot.Commands.OneStickCmd;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class RobotContainer { 
  private final CommandXboxController controller = new CommandXboxController(Constants.DriverConstants.DRIVER_PORT);

  private final VisionSubsystem visionSub = new VisionSubsystem();
  private final DrivetrainSubsystem driveSubSystem = new DrivetrainSubsystem();
  private final ArmSubsystem armSub = new ArmSubsystem();

  public RobotContainer() {
    driveSubSystem.setDefaultCommand(new OneStickCmd(driveSubSystem, () -> controller.getLeftX(),() -> controller.getLeftY()));

    configureBindings();
  }


  public Command getAutonomousCommand() {
    return Commands.print("null");
  }

  public void configureBindings() {
    controller.axisGreaterThan(1, 0.1).whileTrue(new AlignArmToTag(armSub, visionSub)); // Axis 1 is y-axis, above 0.1 it should trigger
  }
}
