package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
  private SparkAbsoluteEncoder e_cal;
  private SparkClosedLoopController p_arm;

  private double m_speed;

  /**
   * 
   * Where should the arm be at?
   * 
   * @author MattheDev53
   * 
   */
  public double m_setpoint = 0;
  
  public ClimberSubsystem() {
    c_arm = Configs.ClimberSubsystem.armConfig;

    m_arm = new SparkMax(ArmConstants.kCanId, SparkMax.MotorType.kBrushless);
    e_cal = m_arm.getAbsoluteEncoder();
    e_arm = m_arm.getEncoder();
    p_arm = m_arm.getClosedLoopController();

    p_arm = m_arm.getClosedLoopController();

    m_arm.configure(c_arm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_speed = 0.1;
  }
    public void setTargetPosition(double setpoint) {
      m_setpoint = setpoint;
      moveToSetpoint();
  }

  private void moveToSetpoint() {
    p_arm.setReference(m_setpoint, ControlType.kMAXMotionPositionControl);
  }
  public boolean atTargetRotation() { // Lets the Target rotation by subtracting Encoder pos by the set point
    return Math.abs(avgEncoderPos() - m_setpoint) < ArmConstants.kPositionTolerance;
  }

  public double avgEncoderPos() { // Gets encoder pos
    return e_arm.getPosition();
  }
  public Command resetArm() { // Resets arm value
    return run(() -> e_arm.setPosition(0.0));
  }
  public Command motorFwd() {
    return run(() -> setTargetPosition(72.0));
  }
  public Command motorRev() {
    return run(() -> setTargetPosition(-45.0));
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", e_arm.getPosition());
    SmartDashboard.putNumber("Arm Velocity", e_arm.getVelocity());
  }
}
