// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JointConstants;

public class IntakeJoint extends SubsystemBase {
  private final CANSparkMax joint;
  private final AbsoluteEncoder encoder;
  private final SparkPIDController jointPID;

  
  /** Creates a new IntakeJointSubsystem. */
  public IntakeJoint(int jointID) {
    joint = new CANSparkMax(jointID, MotorType.kBrushed);
    joint.restoreFactoryDefaults();

    joint.setSmartCurrentLimit(30);
    joint.setIdleMode(IdleMode.kBrake);

    joint.setInverted(false);

    encoder = joint.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setInverted(false);
    jointPID = joint.getPIDController();
    jointPID.setFeedbackDevice(encoder);

    jointPID.setP(JointConstants.kP);
    jointPID.setI(JointConstants.kI);
    jointPID.setD(JointConstants.kD);
    jointPID.setIZone(JointConstants.kIz);
    jointPID.setOutputRange(JointConstants.kMinOutput, JointConstants.kMaxOutput);

    joint.burnFlash();
  }

  public void runOpenLoop(double supplier) {
    if(getPos() >= JointConstants.kUpperLimit) {
      joint.set(0);
      System.out.println("Too High, Upper Limit");
    }
    else if (getPos() <= JointConstants.kLowerLimit) {
      joint.set(0);
      System.out.println("Too Low, Lower Limit");
    }
    else {
      joint.set(supplier);
    }
  }

  public void hold() {
    jointPID.setReference(encoder.getPosition(), ControlType.kPosition);
  }

  public void runToPosition(double setpoint) {
    if(getPos() >= JointConstants.kUpperLimit) {
      joint.set(0);
      System.out.println("Too High, Upper Limit");
    }
    else if(getPos() <= JointConstants.kLowerLimit) {
      joint.set(0);
      System.out.println("Too Low, Lower Limit" + getPos());
    }
    else {
      jointPID.setReference(setpoint, ControlType.kPosition);
    }
  }

  public double getPos() {
    return encoder.getPosition();
  }

  
}
