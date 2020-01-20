package org.firstinspires.ftc.teamcode.autonomous.ai;

import org.firstinspires.ftc.teamcode.templates.LinearAutonomous;
import org.firstinspires.ftc.teamcode.templates.ParallelAutonomous;
import org.firstinspires.ftc.teamcode.templates.ThreadController;

public class Controller extends ParallelAutonomous {
    private LinearAutonomous thread = new ThreadA();

    @Override
    public void runner(){
        telemetry.addData("Controller", "Working properly");
        telemetry.update();
    }

    @Override
    public void instance(){
        threadA = new ThreadController(this.thread);
        threads = 1;
    }
}
