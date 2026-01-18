package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
  private SparkAbsoluteEncoder e_arm;

  private double m_speed;

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
    e_arm = m_arm.getAbsoluteEncoder();

    m_arm.configure(c_arm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_speed = 0.1;
  }

  public Command motorFwd() {
    return startEnd(
      () -> m_arm.set(m_speed * -1),
      () -> m_arm.set(0)
    );
  }
  public Command motorRev() {
    return startEnd(
      () -> m_arm.set(m_speed),
      () -> m_arm.set(0)
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", e_arm.getPosition());
    SmartDashboard.putNumber("Arm Velocity", e_arm.getVelocity());
  }
}
