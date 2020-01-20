package org.firstinspires.ftc.teamcode.templates;

public class ThreadController extends Thread {
    LinearAutonomous autonomous;

    public ThreadController(LinearAutonomous autonomous){
        this.autonomous = autonomous;
    }
    @Override
    public void run(){
        autonomous.runner();
    }
}
