/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/*THIS PID CONTROLLER NEEDS WORK JMK*/
public class Shooter extends SubsystemBase {
  CANSparkMax SMotorChip;
  CANSparkMax SMotorDale;
  double shooterMotor1Speed = 0;
  double shooterMotor2Speed = 0;

  private CANPIDController shooterPID;
  double pVal = 6e-5; // Updated to reflect REV default JMK
  double iVal = 0; // Updated to reflect REV default JMK
  double dVal = 0; // Updated to reflect REV default JMK
  double pidShooterSpeed = 0.0;
  CANEncoder ShooterEncoder;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    SMotorChip = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_CHIP, MotorType.kBrushed);
    SMotorDale = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DALE, MotorType.kBrushed);

    //Restore factory defaults (not necessary but let's do it anyway)
    //SMotorChip.restoreFactoryDefaults();
    //SMotorDale.restoreFactoryDefaults();

    //shooterPID = SMotorChip.getPIDController();
    ShooterEncoder = SMotorChip.getEncoder();

    // Prints the initial PID values to smart dashboard
    SmartDashboard.putNumber("Current pVal = ", pVal);
    SmartDashboard.putNumber("Current iVal = ", iVal);
    SmartDashboard.putNumber("Current dVal = ", dVal);
    SmartDashboard.putNumber("Calculated PID = ", pidShooterSpeed);
    SmartDashboard.putNumber("Shooter Motor1 Speed = ", shooterMotor1Speed);
    SmartDashboard.putNumber("Shooter Motor2 Speed = ", shooterMotor2Speed);
    SmartDashboard.putBoolean("PID or Value:", true); // Set to true for using "Shooter Motor Speed" to control shooter speed
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder positions",ShooterEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Encoder velocity",ShooterEncoder.getVelocity());
    PIDTuner(); // Comment this out once we figure out our PID values.
  }

  public void PIDTuner() {
    double pTemp = 0;
    double iTemp = 0;
    double dTemp = 0;
    double motor1ShooterSpeed = 0;
    double motor2ShooterSpeed = 0;
    boolean pidOrValue;

    pTemp = SmartDashboard.getNumber("Current pVal = ", -1);
    iTemp = SmartDashboard.getNumber("Current iVal = ", -1);
    dTemp = SmartDashboard.getNumber("Current dVal = ", -1);
    motor1ShooterSpeed = SmartDashboard.getNumber("Shooter Motor1 Speed = ", -1); 
    motor2ShooterSpeed = SmartDashboard.getNumber("Shooter Motor2 Speed = ", -1);
    pidOrValue = SmartDashboard.getBoolean("PID or Value:", true);
  

    // We don't want to set our constants to a negative value so lets prevent that
    if (pTemp >= 0 ) {
      pVal = pTemp;
    }

    if (iTemp >= 0 ) {
      iVal = iTemp;
    }

    if (dTemp >= 0 ) {
      dVal = dTemp;
    }
    /*shooterPID.setP(pVal);
    shooterPID.setI(iVal);
    shooterPID.setD(dVal);*/

    if (motor1ShooterSpeed >= 1.0) {
      motor1ShooterSpeed = 1.0;
    } else if (motor1ShooterSpeed <= -1.0) {
      motor1ShooterSpeed = -1.0;
    }

    if (motor2ShooterSpeed >= 0.8) {
      motor2ShooterSpeed = 0.8;
    } else if (motor2ShooterSpeed <= -0.8) {
      motor2ShooterSpeed = -0.8;
    }

    if (true) {
      SMotorChip.set(motor1ShooterSpeed);
      SMotorDale.set(-motor1ShooterSpeed);
    } else {
      // Use PID value
      // This currently only sets value for one motor!! JMK
      shooterPID.setReference(motor1ShooterSpeed, ControlType.kVelocity);
      SmartDashboard.putNumber("ProcessVariable", ShooterEncoder.getVelocity());
    }
  }


}
