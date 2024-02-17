// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeJointSubsystem extends SubsystemBase {
  private final CANSparkMax intakeJoint = new CANSparkMax(Constants.MotorConstants.MOTOR_INTAKE_ANGLE_ID, MotorType.kBrushed);
  private final PIDController intakePID = new PIDController(Constants.IntakePID.kOutP, Constants.IntakePID.ki, Constants.IntakePID.kd);

  double m_encoderPos;
  boolean atPosition;
  double currentPosition;
  /** Creates a new IntakeJointSubsystem. */
  public IntakeJointSubsystem() {
    intakeJoint.restoreFactoryDefaults();
    intakeJoint.setIdleMode(IdleMode.kBrake);

    intakeJoint.setSmartCurrentLimit(25);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
