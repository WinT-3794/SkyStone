package org.firstinspires.ftc.teamcode.autonomous.ai;

import org.firstinspires.ftc.teamcode.templates.LinearAutonomous;

public class ThreadA extends LinearAutonomous {
    @Override
    public void runner() {
        telemetry.addData("Thread A", "Working properly");
    }
}
