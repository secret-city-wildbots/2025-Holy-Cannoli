package frc.robot.Utility;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    public SparkClosedLoopController pidControllerSPM;
    public SparkMaxConfig motorConfigSPM;
    public TalonFXConfiguration motorConfigTFX;
    public Slot0Configs PIDConfigTFX;
    public RelativeEncoder relativeEncoderSPM;
    public double dc = 0;
    public Boolean toggled = false;
    public double[] PID = {0,0,0};
    public static enum MotorBrand {
        SPM,
        TFX
    }
    /**
     * 
     * @param id CAN ID
     * @param num Motor number (for Actuator Interlocks)
     * @param type can be either SPM or TFX for motor types
     */
    public ExtraMotor(int num, MotorBrand type) {
        this.type = type;
        this.id = 40 + num;
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
    public void setPID(double kP, double kI, double kD) {
        if (!(this.PID[0] == kP && this.PID[1] == kI && this.PID[2] == kD)) {
            if (type == MotorBrand.SPM) {
                if (pidControllerSPM == null) {
                    motorConfigSPM = new SparkMaxConfig();
                    relativeEncoderSPM = motorSPM.getEncoder();
                    pidControllerSPM = motorSPM.getClosedLoopController();
                }
                motorConfigSPM.closedLoop.pid(kP, kI, kD);
                motorSPM.configure(motorConfigSPM, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
    public void goToPos(double pos_rot) {
        if (type == MotorBrand.SPM) {
            ActuatorInterlocks.TAI_SparkMAX_Position(motorSPM, pidControllerSPM, "Motor_"+Integer.toString(this.num)+"_(p)", pos_rot, 0);
        } else {
            ActuatorInterlocks.TAI_TalonFX_Position(motorTFX, "Motor_"+Integer.toString(this.num)+"_(p)", pos_rot, 0);
        }
    }
}