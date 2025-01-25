package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_shepherd = new SparkMax(ElevatorConstants.kShepherdCanId, SparkMax.MotorType.kBrushless);
    private final SparkMax m_sheep = new SparkMax(ElevatorConstants.kSheepCanId, SparkMax.MotorType.kBrushless);
}
