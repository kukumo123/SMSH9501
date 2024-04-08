package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightCommand extends Command {
    private final LimelightSubsystem limelightSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    public double turnOutput;
    public double moveOutput;
    public final double getmove() {
        return moveOutput;
    }
    public final double getrot() {
        return turnOutput;
    }


    public LimelightCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(limelightSubsystem, swerveSubsystem);
        execute();
    }

    @Override
    public void initialize() {
        turnOutput = 0;
        moveOutput = 0;
    }

    @Override
    public void execute() {
         turnOutput = limelightSubsystem.getTurnOutput();
         moveOutput = limelightSubsystem.getMoveOutput();

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}