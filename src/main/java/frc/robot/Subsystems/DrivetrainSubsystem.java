// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  private final TalonFX leftFront = new TalonFX(3);
  private final TalonFX leftBack = new TalonFX(4);
  private final TalonFX rightFront = new TalonFX(1);
  private final TalonFX rightBack = new TalonFX(2);

  private DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    leftBack.setControl(new Follower(leftFront.getDeviceID(), false));
    rightBack.setControl(new Follower(rightFront.getDeviceID(), false));

    MotorOutputConfigs brake = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    MotorOutputConfigs invBrake = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);

    leftFront.getConfigurator().apply(brake);
    leftBack.getConfigurator().apply(brake);
    rightFront.getConfigurator().apply(invBrake);
    rightBack.getConfigurator().apply(invBrake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left", leftFront.get());
    SmartDashboard.putNumber("Right", rightFront.get());
  }

  public void tankDrive(double leftSpeed,double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
}
