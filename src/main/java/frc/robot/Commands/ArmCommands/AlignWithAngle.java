// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignWithAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public AlignWithAngle(
    ArmSubsystem armSub,
    DoubleSupplier degreeSetpoint
  ) {
    super(
        // The controller that the command will use
        new PIDController(0.04, 0.01, 0),
        // This should return the measurement
        () -> armSub.getArmDegrees(),
        // This should return the setpoint (can also be a constant)
        degreeSetpoint,
        // This uses the output
        output -> {
          armSub.setSpeed(output);
        });
    addRequirements(armSub);
    SmartDashboard.putNumber("ARM_P", getController().getP());
    SmartDashboard.putNumber("ARM_I", getController().getI());
    SmartDashboard.putNumber("ARM_D", getController().getD());
  }

  @Override
  public void execute() {
    super.execute();
    getController().setP(SmartDashboard.getNumber("ARM_P", getController().getP()));
    getController().setI(SmartDashboard.getNumber("ARM_I", getController().getI()));
    getController().setD(SmartDashboard.getNumber("ARM_D", getController().getD()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
