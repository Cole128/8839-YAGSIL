// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private CANSparkMax flyWheelMotorLeft =
    new CANSparkMax(Constants.MotorConstants.MOTOR_FLYWHEEL_LEFT_ID, MotorType.kBrushless);
  private CANSparkMax flyWheelMotorRight = 
    new CANSparkMax(Constants.MotorConstants.MOTOR_FLYWHEEL_RIGHT_ID, MotorType.kBrushless);
  private CANSparkMax feedWheelMotor =
    new CANSparkMax(Constants.MotorConstants.MOTOR_FEEDER_ID, MotorType.kBrushless);



  public ShooterSubsystem() {
    flyWheelMotorLeft.setInverted(false);
    flyWheelMotorRight.setInverted(true);
  }

  public void spinFeedWheel(double speed) {
    feedWheelMotor.set(speed);
  }

  public void spinFlywheel(double lSpeed, double rSpeed) {
    flyWheelMotorLeft.set(lSpeed);
    flyWheelMotorRight.set(rSpeed);
  }

  public void spinFeedandFlywheel(double lSpeed, double rSpeed, double fSpeed) {
    flyWheelMotorLeft.set(lSpeed);
    flyWheelMotorRight.set(rSpeed);
    feedWheelMotor.set(fSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
