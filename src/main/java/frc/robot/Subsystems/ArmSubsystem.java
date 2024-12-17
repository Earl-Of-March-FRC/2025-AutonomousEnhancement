// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX rightLift = new WPI_TalonSRX(ArmConstants.LEFT_ID);
  private final WPI_TalonSRX leftLift = new WPI_TalonSRX(ArmConstants.RIGHT_ID);

  private final Encoder encoder = new Encoder(0, 2);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    leftLift.follow(rightLift);

    leftLift.setInverted(true);
    rightLift.setInverted(false);

    leftLift.setNeutralMode(NeutralMode.Brake);
    rightLift.setNeutralMode(NeutralMode.Brake);

    encoder.setDistancePerPulse(1.00/16.00);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor speed", rightLift.get());
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    rightLift.set(MathUtil.clamp(speed, -ArmConstants.MAX_MOTOR_SPEED, ArmConstants.MAX_MOTOR_SPEED));
  }

  public double getArmPosition() {
    return encoder.getDistance();
  }

  public double getEncoderRate() {
    return encoder.getRate();
  }

  public void stopMotors(){
    leftLift.stopMotor();
    rightLift.stopMotor();
  }
}
