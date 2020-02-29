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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;


public class Elevator extends SubsystemBase {
  private CANSparkMax ElevatorGoofyMotor;
  private static double matchTime;
  private Relay safety;


  CANEncoder elevatorEncoder;
  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    ElevatorGoofyMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_GOOFY,MotorType.kBrushless);
    elevatorEncoder = ElevatorGoofyMotor.getAlternateEncoder();
    safety = new Relay(2, Relay.Direction.kForward);
  }
  
  public void setSpeed(double speed){
    ElevatorGoofyMotor.set(speed);
  }

  public double getElevatorEncoder() {
    return elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator pos", elevatorEncoder.getPosition());
    // This method will be called once per scheduler run
    matchTime = Timer.getMatchTime();
    // Match time isn't exact, may want to have separate timer/
    if (matchTime <= 5.0){
      safety.set(Value.kOn);
    }


  }
}
