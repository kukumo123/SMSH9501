package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BeltCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Elvator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {


  
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final POVButton pov0 = new POVButton(driverJoytick, 0);
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final LimelightHelpers limelightHelpers = new LimelightHelpers();

    
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Elvator elvator = new Elvator();
    private final Belt belt = new Belt();
    private static final String kDefaultAuto = "Default";
    private static final String middleStart = "Auto";
    private static final String RightStart = "Auto2";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        
        NamedCommands.registerCommand("Intake", new IntakeCmd(intake, -1));
        NamedCommands.registerCommand("Cum", new SequentialCommandGroup(
          new ShooterCmd(shooter, 1, -1).withTimeout(0.5),
          new ParallelCommandGroup(new BeltCmd(belt, 1), new ShooterCmd(shooter, 1, -1)).withTimeout(1.3) 
        ));
        NamedCommands.registerCommand("belt", new BeltCmd(belt, 1).withTimeout(2));
        NamedCommands.registerCommand("shooter", new ShooterCmd(shooter, 1, -1).withTimeout(15));

        
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("Auto", middleStart);
        m_chooser.addOption("Auto2", RightStart);
        SmartDashboard.putData("Auto choices", m_chooser);  
        m_autoSelected =  m_chooser.getSelected();
        System.out.println("Auto Selected" + m_autoSelected);

        switch (m_autoSelected) {
          case middleStart:
            break;
          case RightStart:
            break;
          case kDefaultAuto:
          default:
          break;
        }

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

                new LimelightCommand(limelightSubsystem ,swerveSubsystem ,limelightHelpers);
        new JoystickButton(driverJoytick, Button.kY.value).whileTrue(
                new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> 0d, //-new LimelightCommand(limelightSubsystem , swerveSubsystem).getmove(),
                        () -> 0d,
                        () -> new LimelightCommand(limelightSubsystem , swerveSubsystem , limelightHelpers).getrot() * -0.2,
                        () -> false));
        new InstantCommand();
        configureButtonBindings();     
    }

  
    private void configureButtonBindings() {
    
    new JoystickButton(driverJoytick, Button.kX.value).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(),swerveSubsystem));
    
    new JoystickButton(driverJoytick, Button.kStart.value).whileTrue(new IntakeCmd(intake, -1));
    new JoystickButton(driverJoytick, Button.kLeftBumper.value).whileTrue(new IntakeCmd(intake, 1));
    new JoystickButton(driverJoytick, Button.kRightBumper.value).whileTrue(new  ShooterCmd(shooter, 1, -1));
    //Intake 控制
    // new JoystickButton(driverJoytick, Button.kA.value).whileTrue(new InstantCommand(() -> elvator.LetRobotUP()));
    // new JoystickButton(driverJoytick, Button.kB.value).whileTrue(new InstantCommand(() -> elvator.LetRobotDOWN()));
    new JoystickButton(driverJoytick, Button.kBack.value).whileTrue(new BeltCmd(belt, 0.8));
    new JoystickButton(driverJoytick, Button.kB.value).whileTrue(new BeltCmd(belt, -0.6));
    }


      public PathPlannerAuto getAutonomousCommand(){
        return new PathPlannerAuto(m_chooser.getSelected());
      }
    // public Command getAutonomousCommand(){
    //   return null;
    // }
  }