/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax  SMotorChip;
  CANSparkMax SMotorDale;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
  //SMotorChip = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_CHIP, MotorType.kBrushless);
  //SMotorDale = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DALE, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
