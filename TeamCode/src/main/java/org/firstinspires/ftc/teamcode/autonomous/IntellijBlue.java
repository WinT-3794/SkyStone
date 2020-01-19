package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.templates.LinearAutonomous;

@Autonomous
public class IntellijBlue extends LinearAutonomous {
    @Override
    public void runner(){
        switch (stage) {
            case 0:
                moveToPosition(20, 1);
                sleep(100);
                break;
            case 1:
                sleep(100);
                while (nextSkyStone < 4 && !vuforiaRead().equals("Stone Target")) {
                    mecanumToPosition(-Utilities.STONE_LENGTH, 0.8);
                    sleep(400);
                    mecanum.stop();
                    nextSkyStone++;
                    sleep(350);
                }
                speech.speak("Give me the rock, bitch!");
                targetsSkyStone.deactivate();
                mecanum.withoutEncoders();
                sleep(100);
                break;
            case 2:
                mecanum.move(.8, Math.PI, 0);
                sleep(100);
                mecanum.stop();
                clawRed2(1);
                mecanum.move(-1, 0, 0);
                sleep(200);
                mecanum.stop();
                BRZ.setPower(-1);
                turnToPosition(-23, 1);
                sleep(600);
                break;
            case 3:
                mecanum.withoutEncoders();
                mecanum.stop();
                BRZ.setPower(0);
                while (blue < Utilities.BLUE_COLOR || blueV < Utilities.BLUE_VERIFIER_COLOR) {
                    mecanum.move(0, 0, 1);
                    colors = colorSensor.getNormalizedColors();
                    colorsV = colorSensorVerifier.getNormalizedColors();
                    blue = colors.blue;
                    blueV = colorsV.blue;
                }
                mecanum.stop();
                sleep(20);
                break;
            case 4:
                mecanum.targetToPositionEncoders();
                moveToPosition(42, 1);
                turnToPosition(-17.5, 1);
                sleep(500);
                mecanum.stop();
                sleep(50);
                mecanum.withoutEncoders();
                while (!foundation.isPressed()) {
                    mecanum.move(0.4, Math.PI, 0);
                }
                mecanum.stop();
                sleep(50);
                break;
            case 5:
                JL_DRC.setPosition(1);
                JL_IZQ.setPosition(0);
                sleep(350);
                mecanum.targetToPositionEncoders();
                moveToPosition(30,1);
                mecanum.turnToPosition(45, 1);
                mecanum.withoutEncoders();
                sleep(1500);
                mecanum.stop();
                break;
            case 6:
                JL_DRC.setPosition(0);
                JL_IZQ.setPosition(1);
                CUBO.setPosition(0);
                BRZ.setPower(1);
                sleep(250);
                BRZ.setPower(0.19);
                mecanum.move(-1, 0, 0);
                sleep(600);
                CUBO.setPosition(1);
                sleep(200);
                BRZ.setPower(0);
                mecanum.stop();
                sleep(150);
                mecanum.targetToPositionEncoders();
                moveToPosition(10, 1);
                mecanum.stop();
                break;
            case 7:
                targetsSkyStone.activate();
                mecanum.targetToPositionEncoders();
                mecanum.turnToPosition(17.9,1);
                sleep(600);
                mecanum.stop();
                mecanum.withoutEncoders();
                mecanum.move(0, 0, 1);
                sleep(2150);
                mecanum.stop();
                mecanum.targetToPositionEncoders();
                turnToPosition(-22, 1);
                sleep(50);
                mecanum.withoutEncoders();
                mecanum.stop();
                break;
            case 8:
                sleep(300);
                if(!vuforiaRead().equals("Stone Target")) {
                    sleep(300);
                }
                if(!vuforiaRead().equals("Stone Target")){
                    while (!vuforiaRead().equals("Stone Target")) {
                        mecanumToPosition(Utilities.STONE_LENGTH, 0.8);
                        sleep(400);
                        mecanum.stop();
                        sleep(350);
                    }
                }
                speech.speak("Give me the rock, bitch!");
                targetsSkyStone.deactivate();
                mecanum.withoutEncoders();
                sleep(100);
                break;
            case 9:

                mecanum.move(.8, Math.PI, 0);
                sleep(100);
                mecanum.stop();
                if(nextSkyStone == 3){
                    clawRed2(0);
                }else{
                    clawRed2(1);
                }
                mecanum.withoutEncoders();
                turnperseconds(-.5);
                sleep(50);
                mecanum.stop();
                while (!(digitalTouch1.isPressed() || digitalTouch2.isPressed())) {
                    mecanum.move(-1, 0, 0);
                }
                mecanum.stop();
                mecanum.targetToPositionEncoders();
                moveToPosition(3, 1);
                turnToPosition(-19.5, 1);
                sleep(50);
                mecanum.withoutEncoders();
                mecanum.stop();
                sleep(20);
                colors = colorSensor.getNormalizedColors();
                colorsV = colorSensorVerifier.getNormalizedColors();
                blue = colors.red;
                blueV = colorsV.red;
                while (blue < Utilities.BLUE_COLOR || blueV < Utilities.BLUE_VERIFIER_COLOR) {
                    mecanum.move(0, 0, 1);
                    colors = colorSensor.getNormalizedColors();
                    colorsV = colorSensorVerifier.getNormalizedColors();
                    blue = colors.blue;
                    blueV = colorsV.blue;
                }
                mecanum.stop();
                mecanum.targetToPositionEncoders();
                sleep(20);
                BRZ.setPower(-1);
                sleep(280);
                BRZ.setPower(-0.35);
                moveToPosition(15, 1);
                mecanum.withoutEncoders();
                CUBO.setPosition(0);
                sleep(150);
                mecanum.targetToPositionEncoders();
                BRZ.setPower(-0.35);
                moveToPosition(-25, 1);
                mecanum.stop();
                BRZ.setPower(0);
                sleep(20);
                break;
            default:
                telemetry.addData("Step", "Terminado");
                mecanum.stop();
                sleep(50);
                break;
        }
        telemetry.update();
        stage++;
    }
}
