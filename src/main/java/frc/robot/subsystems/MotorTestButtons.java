package frc.robot.Subsystems;

import frc.robot.Dashboard;

public class MotorTestButtons {
    public static String[] buttonNames = {"Off", "Slider", "D_LT", "D_RT", "D_LB", "D_RB", "D_Y", "D_X", "D_A", "D_B", "MLJoystick_X", "MLJoystick_Y", "M_RJoystick_X", "M_RJoystick_Y", "M_LT", "M_RT", "M_LB", "M_RB", "M_Y", "M_X", "M_A", "M_B"};
    
    public MotorTestButtons() {
    }   
    public void updateOutputs() {
        System.out.println(Dashboard.motorSelectorStates.get());
        System.out.println(buttonNames[(int)Dashboard.motorSelectorStates.get()[0]]);
    }
}