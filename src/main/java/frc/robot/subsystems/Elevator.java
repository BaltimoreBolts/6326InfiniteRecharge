/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
  public CANSparkMax ElevatorGoofyMotor;
  CANEncoder elevatorEncoder;
  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    ElevatorGoofyMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_GOOFY,MotorType.kBrushless);
    elevatorEncoder = ElevatorGoofyMotor.getAlternateEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elvator position encoder", elevatorEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}