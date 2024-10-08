package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChuckConstants;

import com.ctre.phoenix6.hardware.*;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

public class Chuck extends SubsystemBase{

    // Speaker motors
    private final TalonFX motor10 = new TalonFX(ChuckConstants.id10);
    private final TalonFX motor11 = new TalonFX(ChuckConstants.id11);
    // Amp motor
    private final CANSparkMax motor12 = new CANSparkMax(ChuckConstants.id12, MotorType.kBrushless);

    // Initialize new output
    public Chuck() {

        // By default, motors will be stopped
        motor10.setNeutralMode(NeutralModeValue.Brake);
        motor11.setNeutralMode(NeutralModeValue.Brake);
        motor12.setIdleMode(IdleMode.kBrake);
    }

    public void periodic() {
        // called periodically
    }

    public Command IntakeRing() {
        return run(
            () -> {
                motor10.set(.25);
                motor11.set(.25);
                motor12.set(-5.0);
            });
    }

    public Command SpeakerShoot() {
        return run(
            () -> {
                motor10.set(-1.5);
                motor11.set(-1.5);
            });
    }

    public Command LaunchShot() {
        return run(
            () -> {
                motor12.set(5.0);
            });
    }

    public Command stopIntake() {
        return run(
            () -> {
                motor10.set(0);
                motor11.set(0);
                motor12.set(0);
            });
    }

    public Command stopRun() {
        return run(
            () -> {
                motor10.set(0.0);
                motor11.set(0.0);
            });
    }

    public Command stopLaunch() {
        return run(
            () -> {
                motor12.set(0.0);
            });
    }

    // Command functions
    public void cmdPrep() {
        motor10.set(-1.5);
        motor11.set(-1.5);
    }

    public void cmdLaunch() {
        motor12.set(5);
    }

    public void cmdStop() {
        motor10.set(0);
        motor11.set(0);
        motor12.set(0);
    }

    public void cmdFire(Timer timer, double timeout) {
        timer.reset();
        timer.start();
        motor11.set(20);
        if (timer.get() >= timeout){
            motor11.set(0);
            motor10.set(0);
        }
        //motor11.set(TalonFXControlMode.PercentOutput, ChuckConstants.speakerspeed);
    }

}
