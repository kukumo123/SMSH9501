package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 4*2.54/100;
        public static final double kDriveMotorGearRatio = 1 / 5.95;
        public static final double kTurningMotorGearRatio = 1 /21;    //19.6115
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.65;
        public static final double kITurning = 0;
        public static final double kDTurning = 0;
    }

    public static final class DriveConstants {
        

        public static final double kTrackWidth = Units.inchesToMeters(21.4);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21.4);
        // Distance between front and back wheels
        // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        //         new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        //         new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        //         new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        //         new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kFrontRightDriveMotorPort = 7;
        public static final int kBackRightDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 4;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed =false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 10;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftAbsoluteEncoderOffsetRad = 0.434116562971636;
        public static final double kBackLeftAbsoluteEncoderOffsetRad = 0.216291291091875;
        public static final double kFrontRightAbsoluteEncoderOffsetRad = 2.51266053055668;
        public static final double kBackRightAbsoluteEncoderOffsetRad = 1.613747788855695; 

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3.5 * 2 * Math.PI;//最大旋轉速度

        public static final double kTeleDriveMaxSpeedMetersPerSecond = (kPhysicalMaxSpeedMetersPerSecond / 4) *2.5;//最大加速度
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kControlPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.06;
    }

    public static class ShooterConstants{
        public static final int kRightShooterMotorPort = 14;
        public static final int kLeftShooterMotorPort = 13;
      }

    public static class IntakeConstants{
        public static final int kIntakeMotorPort = 17;
      }

    public static class ElvatorConstants{
        public static final int kElvatorRightMotorID = 20;
        public static final int kElvatorLeftMotorID = 15;
    }

    public static class BeltConstants{
        public static final int kBeltmotorID = 21;
    }

} 