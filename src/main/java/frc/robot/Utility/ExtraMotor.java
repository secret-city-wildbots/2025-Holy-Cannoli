package frc.robot.Utility;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ExtraMotor {
    public MotorBrand type;
    public int id;
    public int num;
    public TalonFX motorTFX;
    public SparkMax motorSPM;
    public double dc = 0;
    public Boolean toggled = false;
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
}