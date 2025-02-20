package frc.robot.Utility;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Dashboard;
import frc.robot.Subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

public class ExtraMotor {
    public MotorBrand type;
    public int id;
    public int num;
    public TalonFX motorTFX;
    public SparkMax motorSPM;
    public PIDController pidControllerSPM;
    public SparkMaxConfig motorConfigSPM;
    public TalonFXConfiguration motorConfigTFX;
    public Slot0Configs PIDConfigTFX;
    public RelativeEncoder relativeEncoderSPM;
    public double dc = 0;
    public Boolean toggled = false;
    public double[] PID = {0,0,0};
    public static enum MotorBrand {
        TFX,
        SPM
    }
    /**
     * 
     * @param id CAN ID
     * @param num Motor number (for Actuator Interlocks)
     * @param type can be either SPM or TFX for motor types
     */
    public ExtraMotor(int id, int num, MotorBrand type) {
        this.type = type;
        this.id = id;
        this.num = num;
        if (type == MotorBrand.SPM) {
            this.motorSPM = new SparkMax(id, MotorType.kBrushless);
        } else if (type == MotorBrand.TFX) {
            this.motorTFX = new TalonFX(id);
        }
    }
    public void spin(double dc) {
        if (type == MotorBrand.SPM) {
            ActuatorInterlocks.TAI_SparkMAX_Power(motorSPM, "Motor_"+Integer.toString(this.num)+"_(p)", dc);
        } else {
            ActuatorInterlocks.TAI_TalonFX_Power(motorTFX, "Motor_"+Integer.toString(this.num)+"_(p)", dc);
        }
    }

    /**
     * sets the PID
     * @param kP p
     * @param kI i 
     * @param kD d
     * @param min min degrees of pid mode
     * @param max max degrees of pid mode
     * @param speed max speed in degrees per second
     */
    public void setPID(double kP, double kI, double kD, double min, double max, double speed) {
        if (!(this.PID[0] == kP && this.PID[1] == kI && this.PID[2] == kD)) {
            if (type == MotorBrand.SPM) {
                if (pidControllerSPM == null) {
                    motorConfigSPM = new SparkMaxConfig();
                    relativeEncoderSPM = motorSPM.getEncoder();
                    pidControllerSPM = new PIDController(0.0, 0.0, 0.0);
                }
                pidControllerSPM.setPID(kP, kI, kD);
                motorConfigSPM.idleMode(IdleMode.kBrake);
                motorSPM.configure(motorConfigSPM, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            } else {
                if (motorConfigTFX == null) {
                    motorConfigTFX = new TalonFXConfiguration();
                    PIDConfigTFX = new Slot0Configs();
                }
                PIDConfigTFX.kP = kP;
                PIDConfigTFX.kI = kI;
                PIDConfigTFX.kD = kD;
                this.motorTFX.getConfigurator().apply(motorConfigTFX);
                this.motorTFX.getConfigurator().apply(PIDConfigTFX);
                motorTFX.setPosition(0.0);
            }
            this.PID[0] = kP;
            this.PID[1] = kI;
            this.PID[2] = kD;
        }
    }

    public void goToPos(double pos_rot, double ratio, double maxSpeed_dps) {
        if (type == MotorBrand.SPM) {
                        
            double pidOutput = pidControllerSPM.calculate(Wrist.encoder.get(), pos_rot);
            
            pidOutput = Math.signum(pidOutput) * Math.min(Math.abs(pidOutput), maxSpeed_dps);

            System.out.println(Math.round((relativeEncoderSPM.getPosition()/ratio)*360) + " : " + Math.round(Units.rotationsToDegrees(Wrist.encoder.get())) + " : " + Math.round(Units.rotationsToDegrees(pos_rot)) + " : " + pidOutput);
            
            ActuatorInterlocks.TAI_SparkMAX_Power(motorSPM, "Motor_"+Integer.toString(this.num)+"_(p)", pidOutput);
        } else {
            ActuatorInterlocks.TAI_TalonFX_Position(motorTFX, "Motor_"+Integer.toString(this.num)+"_(p)", pos_rot, 0);
        }
    }
}