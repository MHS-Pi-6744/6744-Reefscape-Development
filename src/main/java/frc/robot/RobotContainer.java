// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.AutonomousCommand;
import frc.robot.commands.auto.AutonomousCommand2;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.BuildConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  public void updateshuffleboard(){
    SmartDashboard.updateValues();
  }

  public boolean setRelativeCommandFalse(){
    return fieldrelative = false;
  }
  public boolean setRelativeCommandTrue(){
    return fieldrelative = true;
  }

  public boolean fieldrelative = true;





// The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final AutonomousCommand autoCommand = new AutonomousCommand(m_robotDrive);
  public final AutonomousCommand2 autoCommand2 = new AutonomousCommand2(m_robotDrive);



  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


  //m_chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //m_chooser

    // Shuffleboard.getTab("Autonomous").add(m_chooser);
    m_chooser.addOption("Auto", autoCommand);
    m_chooser.addOption("Auto2", autoCommand2);
    m_chooser.setDefaultOption("Auto", autoCommand);
    SmartDashboard.putData("Auto Chooser", m_chooser);




    // Configure the button bindings
    configureButtonBindings();
      

    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                fieldrelative),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Axis.kRightTrigger.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive
        ));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageLoad),
            m_elevator
        ));
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL1),
            m_elevator
        ));
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL2),
            m_elevator
        ));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL3),
            m_elevator
        ));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL4),
            m_elevator
        ));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageAlgae),
            m_elevator
        ));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }


    // Print Git Data

  public void printGitData() {
    System.out.println("Repo:" + BuildConstants.MAVEN_NAME);
    System.out.println("Branch:" + BuildConstants.GIT_BRANCH);
    System.out.println("Git Date:" + BuildConstants.GIT_DATE);
    System.out.println("Build Date:" + BuildConstants.BUILD_DATE);
  };
  
}
