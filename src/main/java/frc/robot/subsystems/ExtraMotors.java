package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Dashboard;
import frc.robot.Utility.ExtraMotor;
import frc.robot.Utility.ExtraMotor.MotorBrand;

public class ExtraMotors {
    public static String[] buttonNames = {"Off", "Slider", "D_LT", "D_RT", "D_LB", "D_RB", "D_Y", "D_X", "D_A", "D_B", "MLJoystick_X", "MLJoystick_Y", "M_RJoystick_X", "M_RJoystick_Y", "M_LT", "M_RT", "M_LB", "M_RB", "M_Y", "M_X", "M_A", "M_B"};
    public ExtraMotor[] motors = {new ExtraMotor(1, MotorBrand.TFX),new ExtraMotor(2, MotorBrand.TFX)};
    
    public ExtraMotors() {
    }   

    public void updateOutputs(XboxController driverController) {
        for (int motor = 0; motor < motors.length; motor++){
            switch (buttonNames[(int)Dashboard.motorSelectorStates.get()[motor]]) {
                case "Off":
                    motors[motor].dc = 0;
                    break;
                case "Slider":
                    motors[motor].dc = Dashboard.motorAmplitudes.get()[motor];
                    break;
                case "D_LT":
                    if (driverController.getLeftTriggerAxis() > 0.1) {
                        motors[motor].dc = driverController.getLeftTriggerAxis()*Dashboard.motorAmplitudes.get()[motor];
                    }
                    break;
                case "D_RT":
                    if (driverController.getRightTriggerAxis() > 0.1) {
                        motors[motor].dc = driverController.getLeftTriggerAxis()*Dashboard.motorAmplitudes.get()[motor];
                    }
                    break;
                case "D_LB":
                    if (driverController.getLeftBumperButton()) {
                        motors[motor].dc = Dashboard.motorAmplitudes.get()[motor];
                    }
                    break;
                case "D_RB":
                    if (driverController.getRightBumperButton()) {
                        motors[motor].dc = Dashboard.motorAmplitudes.get()[motor];
                    }
                    break;
                case "D_Y":
                    if (driverController.getYButton()) {
                        motors[motor].dc = Dashboard.motorAmplitudes.get()[motor];
                    }
                    break;
                case "D_X":
                    if (driverController.getXButton()) {
                        motors[motor].dc = Dashboard.motorAmplitudes.get()[motor];
                    }
                    break;
                case "D_A":
                    if (driverController.getAButton()) {
                        motors[motor].dc = Dashboard.motorAmplitudes.get()[motor];
                    }
                    break;
                case "D_B":
                    if (driverController.getBButton()) {
                        motors[motor].dc = Dashboard.motorAmplitudes.get()[motor];
                    }
            }
            motors[motor].spin(motors[motor].dc);
        }
    }
}