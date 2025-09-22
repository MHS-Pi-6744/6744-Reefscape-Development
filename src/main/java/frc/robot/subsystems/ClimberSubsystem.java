package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax m_arm;
  private SparkMaxConfig c_arm;
  private RelativeEncoder e_arm;
  private SparkClosedLoopController p_arm; //...esean cheese
  
  public ClimberSubsystem() {
    c_arm = Configs.ClimberSubsystem.armConfig;

    m_arm = new SparkMax(ArmConstants.kCanId, SparkMax.MotorType.kBrushless);
    e_arm = m_arm.getEncoder();
    p_arm = m_arm.getClosedLoopController();

    m_arm.configure(c_arm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    e_arm.setPosition(0);
  }
}
