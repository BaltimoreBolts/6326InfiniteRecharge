/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HarvesterConstants;

public class Harvester extends SubsystemBase {
  private CANSparkMax harvesterMickeyMotor;
  private CANSparkMax harvesterMinnieMotor;
  private DigitalInput LimitSwitch0;
  
  /**
   * Creates a new Harvester.
   */
  public Harvester() {
    harvesterMickeyMotor = new CANSparkMax (HarvesterConstants.HARVESTER_MOTOR_MICKEY, MotorType.kBrushless);
    harvesterMinnieMotor = new CANSparkMax (HarvesterConstants.HARVESTER_MOTOR_MINNIE, MotorType.kBrushless);
    LimitSwitch0 = new DigitalInput(HarvesterConstants.HARVESTER_LIMIT_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    harvesterMickeyMotor.set(speed);
    harvesterMinnieMotor.set(speed);
  }
}
