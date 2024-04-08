package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    //吸入
    private CANSparkMax IntakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    //角度(上下)
    private AnalogInput IntakeSensor = new AnalogInput(0); 
    private AnalogInput ShooterSnsor = new AnalogInput(1);
    private boolean NoteInTheRobot;

    public Intake(){
        
        IntakeMotor.restoreFactoryDefaults();
        IntakeMotor.setIdleMode(IdleMode.kBrake);
        IntakeMotor.burnFlash();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("note Value", noteINvalue());
        // SmartDashboard.putBoolean("Note In The Robot ?", NoteInTheRobot);
        // if (noteINvalue() < 450) {
        //     NoteInTheRobot = true;
        //     System.out.println("Note In The Robot");
        // }if (Shoot() > 500){
        //     NoteInTheRobot = false;
        //     System.out.println("Note Not In Robot");
        // }
    }   

    public double noteINvalue(){
        return (double)IntakeSensor.getValue();
    }

    public double Shoot(){
        return (double)ShooterSnsor.getValue();
    }

    public void setIntakeMotor(double speed){
        IntakeMotor.set(speed);
    }

    public void stopMotor(){
        IntakeMotor.stopMotor();
    }
}