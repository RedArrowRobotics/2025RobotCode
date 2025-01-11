package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ComponentsControl {
    //Variable Defintions
    
    //Run the components
    public void runComponents(Components components, ControlInputs controlInputs, SensorInputs sensorInputs) {
        //Variable Defintions
        
    }

    private double powerClamp(double power) {
        if (power > 1.0) {
            power = 1.0;
        } else if (power < -1) {
            power = -1.0;
        }
        return power;
    }
}