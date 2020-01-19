package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.templates.LinearAutonomous;
import org.firstinspires.ftc.teamcode.util.Utilities;

@Autonomous
public class RedCosmic extends LinearAutonomous {
    @Override
    public void runner() {
        switch (stage) {
            case 0:
                moveToPosition(23,1);
                sleep(100);
                break;
            case 1:
                sleep(100);
                while (nextSkyStone < 4 && !vuforiaRead().equals(Utilities.LABEL_SS)) {
                    mecanumToPosition(Utilities.STONE_LENGTH, 0.8);
                    sleep(400);
                    mecanum.stop();
                    nextSkyStone++;
                    sleep(500);
                }
                speech.speak("Ya se supo Irbing");
                sleep(100);
                mecanum.usingEncoders();
                break;
            case 2:
                mecanum.move(.8, Math.PI, 0);
                sleep(100);
                mecanum.stop();
                clawRed(1);
                mecanum.move(-1, 0, 0);
                sleep(250);
                mecanum.stop();
                turnToPosition(15.5, 1);
                sleep(400);
                break;
            case 3:
                mecanum.withoutEncoders();
                sleep(20);
                mecanum.stop();
                sleep(20);
                mecanum.usingBrake();
                BRZ.setPower(0);
                while (red < Utilities.RED_COLOR || redV < Utilities.RED_VERIFIER_COLOR) {
                    mecanum.move(1,0,0);
                    colors = colorSensor.getNormalizedColors();
                    colorsV = colorSensorVerifier.getNormalizedColors();
                    red = colors.red;
                    redV = colorsV.red;
                }
                mecanum.stop();
                sleep(20);
                break;
            case 4:
                mecanum.targetToPositionEncoders();
                BRZ.setPower(-1);
                sleep(180);
                BRZ.setPower(-0.314);
                moveToPosition(14,1);
                sleep(500);
                CUBO.setPosition(0);
                sleep(200);
                mecanum.withoutEncoders();
                BRZ.setPower(-1);
                sleep(500);
                BRZ.setPower(0);
                while (red < Utilities.RED_COLOR || redV < Utilities.RED_VERIFIER_COLOR) {
                    mecanum.move(0.8, Math.PI, 0);
                    colors = colorSensor.getNormalizedColors();
                    colorsV = colorSensorVerifier.getNormalizedColors();
                    red = colors.red;
                    redV = colorsV.red;
                }
                sleep(35);
                break;
            case 5:
                mecanum.targetToPositionEncoders();
                moveToPosition(-75,1);
                turnToPosition(-19,1);
                moveToPosition(3,1);
                mecanum.stop();
                sleep(600);
                while (nextSkyStone < 4 && !vuforiaRead().equals(Utilities.LABEL_SS)) {
                    mecanumToPosition(-Utilities.STONE_LENGTH, 0.8);
                    sleep(400);
                    mecanum.stop();
                    nextSkyStone++;
                    sleep(350);
                }
                speech.speak("El acosador!");
                sleep(100);
                targetsSkyStone.deactivate();
                mecanum.usingEncoders();
                sleep(100);
                break;

            case 6:
                mecanum.move(.8, Math.PI, 0);
                sleep(100);
                mecanum.stop();
                if(nextSkyStone == 3){
                    clawRed2(0);
                }else{
                    clawRed2(1);
                }
                mecanum.withoutEncoders();
                turnperseconds(.5);
                sleep(70);
                mecanum.stop();
                while (!(digitalTouch1.isPressed() || digitalTouch2.isPressed())) {
                    mecanum.move(-1, 0, 0);
                }
                mecanum.stop();
                mecanum.targetToPositionEncoders();
                moveToPosition(10, 1);
                turnToPosition(17, 1);
                sleep(50);
                mecanum.withoutEncoders();
                mecanum.stop();
                sleep(20);
                colors = colorSensor.getNormalizedColors();
                colorsV = colorSensorVerifier.getNormalizedColors();
                red = colors.red;
                redV = colorsV.red;
                while (red < Utilities.RED_COLOR || redV < Utilities.RED_VERIFIER_COLOR) {
                    mecanum.move(0, 0, 1);
                    colors = colorSensor.getNormalizedColors();
                    colorsV = colorSensorVerifier.getNormalizedColors();
                    red = colors.red;
                    redV = colorsV.red;
                }
                mecanum.stop();
                mecanum.targetToPositionEncoders();
                sleep(20);
                BRZ.setPower(-1);
                sleep(280);
                BRZ.setPower(-0.35);
                moveToPosition(14, 1);
                mecanum.withoutEncoders();
                CUBO.setPosition(0);
                sleep(300);
                mecanum.targetToPositionEncoders();
                BRZ.setPower(-1);
                sleep(250);
                moveToPosition(-20, 1);
                mecanum.stop();
                BRZ.setPower(0);
                sleep(20);
                mecanum.withoutEncoders();
                mecanum.move(1,-Math.PI/2,0);
                sleep(600);
                mecanum.stop();
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
