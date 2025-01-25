package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax m_shepherd;
  private SparkMax m_sheep;

  public ElevatorSubsystem() {
    m_shepherd = new SparkMax(ElevatorConstants.kShepherdCanId, SparkMax.MotorType.kBrushless);
    m_sheep = new SparkMax(ElevatorConstants.kSheepCanId, SparkMax.MotorType.kBrushless);

    SparkMaxConfig shepherdConfig = new SparkMaxConfig();
    shepherdConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit);
    shepherdConfig.encoder
      .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
      .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);
    shepherdConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    SparkMaxConfig sheepConfig = shepherdConfig;
    sheepConfig
      .inverted(true);

    m_shepherd.configure(shepherdConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(sheepConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


}
