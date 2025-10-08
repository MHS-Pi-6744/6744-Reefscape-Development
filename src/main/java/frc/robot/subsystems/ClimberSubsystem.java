package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

/**
 * Climber Subsystem... Not much else to say
 * 
 * @author MattheDev53
 */
public class ClimberSubsystem extends SubsystemBase {

  private SparkMax m_arm;
  private SparkMaxConfig c_arm;
  private RelativeEncoder e_arm;
  private SparkClosedLoopController p_arm; //...esean cheese

  /**
   * 
   * Where should the arm be at?
   * 
   * @author MattheDev53
   * 
   */
  double m_setpoint;
  
  public ClimberSubsystem() {
    c_arm = Configs.ClimberSubsystem.armConfig;

    m_arm = new SparkMax(ArmConstants.kCanId, SparkMax.MotorType.kBrushless);
    e_arm = m_arm.getEncoder();
    p_arm = m_arm.getClosedLoopController();

    m_arm.configure(c_arm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    e_arm.setPosition(0);
  }

  /**
   * Sets the Setpoint and then moves the arm
   * 
   * @author MattheDev53
   * 
   * @param here is where you want to go
   */
  public void goTo(double here) {
    m_setpoint = here;
    moveToSetpoint();
  }

  /**
   * 
   * Moves the motor to variable {@code m_setpoint}
   * 
   * @author MattheDev53
   * 
   */
  private void moveToSetpoint() {
    p_arm.setReference(m_setpoint, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Is the arm at the right place?
   * 
   * @author MattheDev53
   * 
   * @return A boolean telling whether or not the arm is where it should be
   */
  boolean atTarget() {
    return Math.abs(e_arm.getPosition() - m_setpoint) <= ArmConstants.kPositionTolerance;
  }

  public Command stickControl(double stick) {
    return startEnd(
      () -> m_arm.set(stick * ArmConstants.kStickMultiplier),
      () -> m_arm.set(0)
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", e_arm.getPosition());
    SmartDashboard.putNumber("Arm Velocity", e_arm.getVelocity());
    SmartDashboard.putNumber("Set Point", m_setpoint);
    SmartDashboard.putBoolean("Arm At Target?", atTarget());
  }
}
