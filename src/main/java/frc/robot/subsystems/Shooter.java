// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leftFly;
  private final  CANSparkMax rightFly;
  private final CANSparkMax feed;
  private final CANSparkMax sJoint;
  private static DutyCycleEncoder sEncoder;
  public boolean shooterResting = true;

  public Shooter() {
    leftFly = new CANSparkMax(MotorConstants.MOTOR_FLYWHEEL_LEFT_ID, MotorType.kBrushless);
    rightFly = new CANSparkMax(MotorConstants.MOTOR_FLYWHEEL_RIGHT_ID, MotorType.kBrushless);
    feed = new CANSparkMax(MotorConstants.MOTOR_FEEDER_ID, MotorType.kBrushless);
    sJoint = new CANSparkMax(MotorConstants.MOTOR_SHOOTER_ANGLE_ID, MotorType.kBrushless);

    leftFly.restoreFactoryDefaults();
    rightFly.restoreFactoryDefaults();
    feed.restoreFactoryDefaults();
    sJoint.restoreFactoryDefaults();

    leftFly.setSmartCurrentLimit(40);
    rightFly.setSmartCurrentLimit(40);
    feed.setSmartCurrentLimit(40);
    sJoint.setSmartCurrentLimit(40);

    leftFly.setIdleMode(IdleMode.kBrake);
    rightFly.setIdleMode(IdleMode.kBrake);
    feed.setIdleMode(IdleMode.kBrake);
    sJoint.setIdleMode(IdleMode.kBrake);

    leftFly.setInverted(false);
    rightFly.setInverted(true);
    feed.setInverted(false);
    sJoint.setInverted(true);
    
    leftFly.burnFlash();
    rightFly.burnFlash();
    feed.burnFlash();
    sJoint.burnFlash();

    sEncoder = new DutyCycleEncoder(4);
  }

  public static double getEncoderDistance() {
    return (sEncoder.getAbsolutePosition() * 360);
  }

  public void spinFeedWheel(double speed) {
    feed.set(speed);
  }

  public void spinFlywheel(double lSpeed, double rSpeed) {
    leftFly.set(lSpeed);
    rightFly.set(rSpeed);
  }

  public void spinFeedandFlywheel(double lSpeed, double rSpeed, double fSpeed) {
    leftFly.set(lSpeed);
    rightFly.set(rSpeed);
    feed.set(fSpeed);
  }

  public void setShooterOut() {
    shooterResting = false;
  }

  public void setShooterIn() {
    shooterResting = true;
  }


  public void toggleShooter() {
    if (shooterResting == true) {
      shooterResting = false;
    } else {
      shooterResting = true;
    }
  }






  @Override
  public void periodic() {
    if (shooterResting == true) {
      if (getEncoderDistance() > 165) {
        sJoint.set(0.2);
      } else if (getEncoderDistance() > 140) {
        sJoint.set(0.2);
      } else if (getEncoderDistance() > 120) {
        sJoint.set(0.2);
      } else {
        sJoint.set(0);
      }
    } else if (shooterResting == false) {
      if (getEncoderDistance() < 165) {
        sJoint.set(-0.3);
      } else if (getEncoderDistance() < 170) {
        sJoint.set(-0.2);
      } else if (getEncoderDistance() < 198) {
        sJoint.set(-0.1);
      } else {
        sJoint.set(0);

      }
    }
  }
}

