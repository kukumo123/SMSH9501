package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {

    private final CANcoder FLcancoder = new CANcoder(9, "rio");
    private final CANcoder BLcancoder = new CANcoder(10, "rio");
    private final CANcoder FRcancoder = new CANcoder(11, "rio");
    private final CANcoder BRcancoder = new CANcoder(12, "rio");
    private Field2d field = new Field2d();
    private SwerveDriveOdometry odometry;
    private SwerveDriveKinematics kinematics;


    

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, getRotation2d(), getModulePosition());

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        setupPathPlanner();
    };

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }

    @Override
    public void periodic() {
        field.setRobotPose(getPose());
        odometer.update(gyro.getRotation2d(), getModulePosition());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getSweveModuleStates(){
        return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
        };
    }

    public SwerveModulePosition[] getModulePosition(){
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void resetEncoder(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void getTurningEncoderPosition(){
        SmartDashboard.putNumber("FL", FLcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("BL", BLcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("FR", FRcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("BR", BRcancoder.getAbsolutePosition().getValue()*(Math.PI*2));

        SmartDashboard.putNumber("FrontLeftEncoder", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("BackLeftEncoder", backLeft.getTurningPosition());
        SmartDashboard.putNumber("FrontRightEncoder", frontRight.getTurningPosition());
        SmartDashboard.putNumber("BackRightEncoder", backRight.getTurningPosition());
    }

    public ChassisSpeeds getRobotVelocity() {
        var kinematics = Constants.DriveConstants.kDriveKinematics;
        return kinematics.toChassisSpeeds(this.getSweveModuleStates());
    }

    public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }

    public ChassisSpeeds getSpeeds(){
        return kinematics.toChassisSpeeds(getSweveModuleStates());
    }

    public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getRobotVelocity,     
            this::driveChassisSpeeds,
            new HolonomicPathFollowerConfig( 
                                            new PIDConstants( 0.01, 0, 0, 0),
                                            new PIDConstants(0.03, 0, 0, 0),
                                            Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                                            0.2,
                                            new ReplanningConfig()
            ),
            () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Blue : true;
            },
            this
        );
    }

    public Command getAutonomousCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }


    public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
    {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      if (setOdomToStart)
      {
        resetOdometry(new Pose2d());
      }
      return AutoBuilder.followPath(path);
    }
}
