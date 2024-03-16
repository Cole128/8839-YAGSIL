// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Elevator extends SubsystemBase {
  private final CANSparkMax leftEl;
  private final CANSparkMax rightEl;
  private static Encoder eEncoder;
  public boolean elResting = true;

  /** Creates a new ElevatorSubsystem. */
  public Elevator() {
    leftEl = new CANSparkMax(MotorConstants.MOTOR_ELEVATOR_LEFT_ID, MotorType.kBrushless);
    rightEl = new CANSparkMax(MotorConstants.MOTOR_ELEVATOR_RIGHT_ID, MotorType.kBrushless);

    leftEl.restoreFactoryDefaults();
    rightEl.restoreFactoryDefaults();

    leftEl.setSmartCurrentLimit(40);
    rightEl.setSmartCurrentLimit(40);

    leftEl.setIdleMode(IdleMode.kBrake);
    rightEl.setIdleMode(IdleMode.kBrake);

    leftEl.setInverted(false);
    rightEl.setInverted(false);

    leftEl.burnFlash();
    rightEl.burnFlash();

    eEncoder = new Encoder(2, 3);
  }

  public static double getEncoderDistance() {
    return eEncoder.getDistance();
  }

  @Override
  public void periodic() {
    if (elResting == true) {
      if (getEncoderDistance() > 400) {
        climbExtend(0.7);
      } else if (getEncoderDistance() > 300) {
        climbExtend(0.5);
      } else if (getEncoderDistance() > 10) {
        climbExtend(0.35);
      } else {
        climbExtend(0);
      }
    } else if (elResting == false) {
      if (getEncoderDistance() < 3400) {
        climbExtend(-0.7);
      } else if (getEncoderDistance() < 3500) {
        climbExtend(-0.5);
      } else if (getEncoderDistance() < 3800) {
        climbExtend(-0.35);
      } else {
        climbExtend(0);
     }
    }
   }

  public void climbExtend(double speed) {
    leftEl.set(speed);
    rightEl.set(speed);
  }

  public void setElOut() {
    elResting = false;
  } 
  
  public void setElIn() {
    elResting = true;
  }

  public void toggleEl() {
    if (elResting == true) {
      elResting = false;
    } else {
      elResting = true;
    }
  }
}
