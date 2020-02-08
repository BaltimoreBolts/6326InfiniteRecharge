/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GenConstants;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftDriveMotor1;
  private CANSparkMax leftDriveMotor2;
  private CANSparkMax rightDriveMotor1;
  private CANSparkMax rightDriveMotor2;

  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;

  private DifferentialDrive driveTrain;
  
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    leftDriveMotor1 = new CANSparkMax(DriveConstants.LEFT_DRIVE_MOTOR1, MotorType.kBrushless);
    leftDriveMotor2 = new CANSparkMax(DriveConstants.LEFT_DRIVE_MOTOR2, MotorType.kBrushless);
    rightDriveMotor1 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_MOTOR1, MotorType.kBrushless);
    rightDriveMotor2 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_MOTOR2, MotorType.kBrushless);

    leftDriveMotor2.follow(leftDriveMotor1);
    rightDriveMotor2.follow(rightDriveMotor1);

    leftEncoder = leftDriveMotor1.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    rightEncoder = rightDriveMotor1.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);

    driveTrain = new DifferentialDrive(leftDriveMotor1,rightDriveMotor1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());
  }
  
  public void arcadeDrive(double x, double y) {
		driveTrain.arcadeDrive(y, x); // Moving the stick forward (+y) moves the robot forward, left/right on the stick makes the robot spin
  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }
}
