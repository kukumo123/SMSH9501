package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class LimelightSubsystem extends SubsystemBase {

    private double move;
    private double turn;
    private PIDController moveController;
    private PIDController turnController;

    public LimelightSubsystem() {
        turnController = new PIDController(0.08, 0, 0);
        moveController = new PIDController(0.08, 0, 0);
    }

    public double getTX() {
        return LimelightHelpers.getTX("");
    }

    public double getTY() {
        return LimelightHelpers.getTY("");
    }

    public double getTurnOutput() {
        turn = turnController.calculate(getTX(), 0);
        return turn;
    }

    public double getMoveOutput() {
        move = moveController.calculate(getTY() , 0);
        return move;
    }
    public void setPipelineIndex(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex("", pipelineIndex);
    }

//    public double getTA() {
//        return LimelightHelpers.getTA("");
//    }

//    public Pose2d getBotPose2d() {
//        return LimelightHelpers.getBotPose2d_wpiBlue("");
//    }
//
//    public PoseEstimate getBotPoseEstimate() {
//        return LimelightHelpers.getBotPoseEstimate_wpiBlue("");
//    }
//
//    public LimelightResults getLatestResults() {
//        return LimelightHelpers.getLatestResults("");
//    }

//    public void setLEDMode(LedMode mode) {
//        switch (mode) {
//            case PIPELINE_CONTROL:
//                LimelightHelpers.setLEDMode_PipelineControl("");
//                break;
//            case FORCE_OFF:
//                LimelightHelpers.setLEDMode_ForceOff("");
//                break;
//            case FORCE_BLINK:
//                LimelightHelpers.setLEDMode_ForceBlink("");
//                break;
//            case FORCE_ON:
//                LimelightHelpers.setLEDMode_ForceOn("");
//                break;
//        }
//    }
//
//    private enum LedMode {
//        PIPELINE_CONTROL,
//        FORCE_OFF,
//        FORCE_BLINK,
//        FORCE_ON
//    }
}