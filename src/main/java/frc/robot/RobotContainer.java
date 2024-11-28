// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ArmCommands.AlignWithAprilTag;
import frc.robot.Commands.DrivetrainCommands.TankDriveCmd;
import frc.robot.Constants.DriverConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class RobotContainer { 
  private final CommandXboxController driver = new CommandXboxController(DriverConstants.DRIVER_PORT);

  private final VisionSubsystem visionSub = new VisionSubsystem();
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  private final ArmSubsystem armSub = new ArmSubsystem();

  public RobotContainer() {
    driveSub.setDefaultCommand(
      new TankDriveCmd(driveSub,() -> driveSub.radicalSpeed(driver.getRawAxis(DriverConstants.LEFT_AXIS)), () -> driveSub.radicalSpeed(driver.getRawAxis(DriverConstants.RIGHT_AXIS)))
    );
    armSub.setDefaultCommand(Commands.runEnd(() -> {
      armSub.setSpeed(0);
    }, () -> {}, armSub));

    configureBindings();
  }

  public void configureBindings() {
    driver.x().whileTrue(new AlignWithAprilTag(armSub, visionSub));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No auto routine.");
  }
}
