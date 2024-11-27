// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final WPI_TalonSRX rightLift = new WPI_TalonSRX(0);
  private final WPI_TalonSRX leftLift = new WPI_TalonSRX(1);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rightLift.setInverted(true);
    rightLift.setNeutralMode(NeutralMode.Coast);
    leftLift.setNeutralMode(NeutralMode.Coast);

    rightLift.follow(leftLift);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double s) {
    rightLift.set(s);
  }
}
