package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Belt extends SubsystemBase{
    private CANSparkMax BeltMotor = new CANSparkMax(Constants.BeltConstants.kBeltmotorID, MotorType.kBrushless);

    

    public Belt(){
        BeltMotor.restoreFactoryDefaults();
        BeltMotor.setIdleMode(IdleMode.kBrake);
        BeltMotor.setInverted(true);
        BeltMotor.burnFlash();
        BeltMotor.setSmartCurrentLimit(70);
    }

    public void setBeltMotorSpeed (double speed){
        BeltMotor.set(speed);
    }

    public void stopMotor(){
        BeltMotor.stopMotor();
    }

}


