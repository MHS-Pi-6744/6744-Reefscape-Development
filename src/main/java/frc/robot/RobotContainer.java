// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.subsystems.ShooterSubsystem;
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

  public void setRelativeCommandFalse(){
    fieldrelative = false;
  }
  public void setRelativeCommandTrue(){
    fieldrelative = true;
  }
  public void toggleFieldRelative(){ // is this unused?
    fieldrelative = !fieldrelative;
  }                          

  public boolean fieldrelative = true;





// The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final AutonomousCommand autoCommand = new AutonomousCommand(m_robotDrive);
  public final AutonomousCommand2 autoCommand2 = new AutonomousCommand2(m_robotDrive);




  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverController2Port);
  Sendable m_driverControllerSendable = new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        m_driverController.initSendable(builder);
    };
  };
  Sendable m_driverController2Sendable = new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        m_driverController2.initSendable(builder);
    };
  }; 

  //m_chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //m_chooser

    // Shuffleboard.getTab("Autonomous").add(m_chooser);

    NamedCommands.registerCommand("AutonomousCommand2", autoCommand2);

    m_chooser.addOption("DR-L2 Auto", new PathPlannerAuto("DR-L2 Auto"));
    m_chooser.addOption("DR-Wait Auto", new PathPlannerAuto("DR-Wait Auto"));

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
    m_elevator.setDefaultCommand(
      new RunCommand(
        () -> m_elevator.stickControl(MathUtil.applyDeadband(m_driverController2.getLeftY(), OIConstants.kDriveDeadband)), 
        m_elevator
      )
    );
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

  // Driver controller - mdriverController
    // Right trigger sets swerve in X configuration
    new JoystickButton(m_driverController, XboxController.Axis.kRightTrigger.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive
        ));
    // Right bumper controls field reletive - button relesed set to robot relative for swerve testing
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileFalse(new RunCommand(
          () -> setRelativeCommandTrue()))
        .whileTrue(new RunCommand(
          () -> setRelativeCommandFalse()));      

  //Copilot controller - mdriverController2
    // Left bumper elevator stage Load
    new JoystickButton(m_driverController2, XboxController.Button.kLeftBumper.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageLoad),
            m_elevator
        ));
    // A button elevator stage L1
    new JoystickButton(m_driverController2, XboxController.Button.kA.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL1),
            m_elevator
        ));
    // B button elevator stage L2
    new JoystickButton(m_driverController2, XboxController.Button.kB.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL2),
            m_elevator
        ));
    // X button elevator stage L3
    new JoystickButton(m_driverController2, XboxController.Button.kX.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL3),
            m_elevator
        ));
    // Y button elevator stage L4
    new JoystickButton(m_driverController2, XboxController.Button.kY.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageL4),
            m_elevator
        ));
    //  Right bumber elevator stage Algae
    new JoystickButton(m_driverController2, XboxController.Button.kRightBumper.value)
        .toggleOnTrue(new RunCommand(
            () -> m_elevator.setTargetPosition(ElevatorConstants.kStageAlgae),
            m_elevator,
            m_robotDrive));      
    // Right trigger triggers release command to shoot
    new JoystickButton(m_driverController2, XboxController.Axis.kRightTrigger.value)
          .whileTrue(m_shooter.releaseCommand());
    // Left trigger intakes coral
    new JoystickButton(m_driverController, XboxController.Axis.kLeftTrigger.value)
          .onTrue(m_shooter.intakeCommand()).onFalse(m_shooter.stopMotor());
            
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }


    // Print Git Data (maybe we will try this later)
    //public void printGitData() {
    //  System.out.println("Repo:" + BuildConstants.MAVEN_NAME);
    //  System.out.println("Branch:" + BuildConstants.GIT_BRANCH);
    //  System.out.println("Git Date:" + BuildConstants.GIT_DATE);
    //  System.out.println("Build Date:" + BuildConstants.BUILD_DATE);
    //};
  
}
