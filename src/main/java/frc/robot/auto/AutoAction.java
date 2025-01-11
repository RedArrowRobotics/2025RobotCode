package frc.robot.auto;

import frc.robot.Components;
import frc.robot.SwerveDriveTrain;
import frc.robot.SensorInputs;

public interface AutoAction {
    //Ran first when sequence | Ran before Execute
    void init(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensor);
    
    //Ran after init
    boolean execute(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensor);
    
    //Ran once action finished
    void finalize(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensor);

    //Used to override toString() and make logging easier
    String toString();
}