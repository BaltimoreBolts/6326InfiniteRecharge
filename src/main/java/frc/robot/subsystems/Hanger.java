/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Hanger extends SubsystemBase {
  public CANSparkMax hangerPlutoMotor;
CANEncoder hangerEncoder;

  /**
   * Creates a new Hanger.
   */
  public Hanger() {
    hangerPlutoMotor = new CANSparkMax (HangerConstants.HANGER_MOTOR_PLUTO, MotorType.kBrushed);
    hangerEncoder = hangerPlutoMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hanger Encoder positions",hangerEncoder.getPosition());
  }
}
