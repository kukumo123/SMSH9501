package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    public double getTX() {
        return LimelightHelpers.getTX("");
    }

    public double getTY() {
        return LimelightHelpers.getTY("");
    }

    public double getMoveOutput() {
        return getTY() * 0.25;
    }

    public double getTurnOutput() {
        return getTX() * 0.25;
    }

}