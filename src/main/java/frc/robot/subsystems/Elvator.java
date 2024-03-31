package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elvator extends SubsystemBase{
    private CANSparkMax Left = new CANSparkMax(Constants.ElvatorConstants.kElvatorLeftMotorID, MotorType.kBrushless);
    private CANSparkMax Right = new CANSparkMax(Constants.ElvatorConstants.kElvatorRightMotorID, MotorType.kBrushless);
    private RelativeEncoder RightelvatEncoder;
    private SparkPIDController RightelvatorController;
    private RelativeEncoder LeftelvatEncoder;
    private SparkPIDController LeftelvatorController;
    private double kP, kI, kD, kFF, kMaxOutput, kMinOutput;



public Elvator(){
    Left.restoreFactoryDefaults();
    Right.restoreFactoryDefaults();
    Left.setIdleMode(IdleMode.kBrake);
    Right.setIdleMode(IdleMode.kBrake);
    Left.setInverted(true);
    LeftelvatEncoder = Left.getEncoder();
    LeftelvatorController = Left.getPIDController();
    RightelvatEncoder = Right.getEncoder();
    RightelvatorController = Right.getPIDController();

    kP = 0.04;
    kI = 0;
    kD = 0.02;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    RightelvatorController.setP(kP);
    RightelvatorController.setP(kI);
    RightelvatorController.setP(kD);
    RightelvatorController.setP(kFF);
    RightelvatorController.setOutputRange(kMinOutput, kMaxOutput);
    RightelvatorController.setFeedbackDevice(RightelvatEncoder);

    LeftelvatorController.setP(kP);
    LeftelvatorController.setI(kI);
    LeftelvatorController.setD(kD);
    LeftelvatorController.setFF(kFF);
    LeftelvatorController.setOutputRange(kMinOutput, kMaxOutput);
    LeftelvatorController.setFeedbackDevice(LeftelvatEncoder);

}

public void RightMotor(double speed){
    double x = 350;
    SmartDashboard.putNumber("Angle", LeftelvatEncoder.getPosition());
    if (x < RightelvatEncoder.getPosition()) {
        Right.stopMotor();
    } else {
        Right.set(speed);
    }
}
public void LeftMotor(double speed){
    double x = 350;
    SmartDashboard.putNumber("Angle", RightelvatEncoder.getPosition());
    if (x < RightelvatEncoder.getPosition()){
        Right.stopMotor();
    } else {
        Right.set(speed);
    }
}

public void setP(double kP) {
    RightelvatorController.setP(kP);
    LeftelvatorController.setP(kP);
}

public void setI(double kI) {
    RightelvatorController.setI(kI);
    LeftelvatorController.setI(kI);
}

public void setD(double kD) {
    RightelvatorController.setD(kD);
    LeftelvatorController.setD(kD);
}


}
