/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShootPowerCell;
import frc.robot.commands.moveIndexer;
import frc.robot.commands.PowerCellSucker;
import frc.robot.commands.ElevatorGoUp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controller;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain roboDT = new DriveTrain();
  private final Indexer roboIndexer = new Indexer();
  private final Harvester roboHarvest = new Harvester(roboIndexer);
  private final Shooter roboShoot = new Shooter();
  private final Elevator roboElevator = new Elevator();
   // Define CameraServer
   public CameraServer RobotCamera;
   public UsbCamera frontRobotCamera;
 
  
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private XboxController driver = new XboxController(OIConstants.DRIVER_CONTROLLER);

  JoystickButton rightDriverTrigger;
  JoystickButton aDriverButton;
  JoystickButton dUpDriverButton;
  JoystickButton yDriverButton;
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default drive command
    // Negative in the Y direction makes robot go forward 2/6
    roboDT.setDefaultCommand(
      new RunCommand(() -> roboDT
        .arcadeDrive(driver.getRawAxis(Controller.XBOX.STICK.LEFT.X), 
        -driver.getRawAxis(Controller.XBOX.STICK.LEFT.Y)), roboDT));
        RobotCamera = CameraServer.getInstance();
    frontRobotCamera = RobotCamera.startAutomaticCapture(0);
    /** serverOne = CameraServer.getInstance();
	    //serverOne.startAutomaticCapture();
	    //serverOne.startAutomaticCapture(0);
	    camera = serverOne.startAutomaticCapture(0);
	    camera.setResolution(RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT);
	    camera.setBrightness(50);
	    camera.setExposureManual(50); **/

  

  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    rightDriverTrigger = new JoystickButton(driver, Constants.Controller.XBOX.TRIGGER.RIGHT);
    aDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.A);
    //dUpDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.DPAD.UP);
    yDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.Y);
    rightDriverTrigger.whenPressed(new ShootPowerCell(roboShoot));
    aDriverButton.whenPressed(new PowerCellSucker(roboHarvest));
    //dUpDriverButton. whenPressed(new ElevatorGoUp(roboElevator));
    yDriverButton.whenPressed(new moveIndexer(roboIndexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

}
