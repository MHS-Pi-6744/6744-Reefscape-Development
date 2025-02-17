// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Robot-wide constants. This class should not be used for any other purpose. All constants
 * should be declared globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {
  public static final class DriveConstants {
    // Maximum driving speed commands - These are the maximum speeds that can be requested by 
    // the driver or autonomous, they are not the maximum speed cababiity of the robot.
    public static final double kMaxSpeedMetersPerSecond = 1; // originally 4.8    TUNING
    public static final double kMaxAngularSpeed = Math.PI/2 ; // radians per second    originally 2*Pi   TUNING

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final int kGyroCanId = 9;
    public static final String kCanBus = "rio";

    public static final boolean kGyroReversed = false;
  }

  public static final class ElevatorConstants {
    public static final int kShepherdCanId = 8;
    public static final int kSheepCanId = 7;

    public static final int kStartingPosition = 0;
    public static final int kCurrentLimit = 50;

    public static final int kPositionConversionFactor = 360;
    public static final int kVelocityConversionFactor = 1000;
    
    public static final double kP = 0.00000000;
    public static final double kI = 0.00000000;
    public static final double kD = 0.00000000;
    public static final double kF = 0.00000000;
    
    public static final double kFwdSoftLimit = 3600;
    public static final double kRevSoftLimit = -3600;

    public static final double kPositionTolerance = 0.5;

    public static final double kStageLoad = 0;
    public static final double kStageL1 = 180;
    public static final double kStageL2 = 360;
    public static final double kStageL3 = 180*3;
    public static final double kStageL4 = 180*4;
    public static final double kStageAlgae = 180*5;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;   //  TUNING
    // An additional driver control TUNING option to try 
    // would be to square controller inputs that vary from 0 t0 1
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;  // originally 3  TUNING
    public static final double kMaxAccelerationMetersPerSecondSquared = 1; // originally 3  TUNING
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI/2; // originally Pi  TUNING
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; // originally Pi  TUNING

    public static final double kPXController = 1;  //  TUNING
    public static final double kPYController = 1;  //  TUNING
    public static final double kPThetaController = 1; //   TUNING

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
