package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.templates.OpenCVSkyStoneDetector;
import org.firstinspires.ftc.teamcode.util.LibTMOA;
import org.firstinspires.ftc.teamcode.util.Utilities;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous
public class OpenCVRed extends OpenCVSkyStoneDetector {
    public ElapsedTime runtime = new ElapsedTime();
    public Orientation angles;
    public Acceleration gravity;

    public TouchSensor digitalTouch1;
    public TouchSensor digitalTouch2;
    public TouchSensor foundation;

    public boolean exit = false;
    public VuforiaLocalizer vuforia = null;
    public LibTMOA mecanum;

    public DcMotor SP_DRC;
    public DcMotor IN_DRC;
    public DcMotor SP_IZQ;
    public Servo JL_DRC;
    public Servo JL_IZQ;
    public DcMotor IN_IZQ;
    public DcMotor ELE_P;
    public DcMotor BRZ;
    public DcMotor ELE_A1;
    public DcMotor ELE_A2;
    public Servo CUBO;
    public Servo RLQ;
    public NormalizedColorSensor colorSensor;
    public NormalizedColorSensor colorSensorVerifier;

    public List<VuforiaTrackable> allTrackables;
    public VuforiaTrackables targetsSkyStone;
    public AndroidTextToSpeech speech;

    public NormalizedRGBA colors;
    public NormalizedRGBA colorsV;
    public float red;
    public float redV;
    public float blue;
    public float blueV;
    public int stage = 0;
    public int nextSkyStone = -1;
    private int mid = 0;
    private int left = 0;
    private int right = 0;
    public boolean working = false;

    private boolean doStop = false;

    @Override
    public void runOpMode() {
        initRunner();
        while(opModeIsActive()){
            runner();
        }
    }

    private void initRunner() {
        SP_DRC = hardwareMap.dcMotor.get("SP_DRC");
        IN_DRC = hardwareMap.dcMotor.get("IN_DRC");
        SP_IZQ = hardwareMap.dcMotor.get("SP_IZQ");
        JL_DRC = hardwareMap.servo.get("JL_DRC");
        JL_IZQ = hardwareMap.servo.get("JL_IZQ");
        IN_IZQ = hardwareMap.dcMotor.get("IN_IZQ");
        BRZ = hardwareMap.dcMotor.get("BRZ");
        ELE_P = hardwareMap.dcMotor.get("ELE_P");
        ELE_A1 = hardwareMap.dcMotor.get("ELE_A1");
        ELE_A2 = hardwareMap.dcMotor.get("ELE_A2");
        CUBO = hardwareMap.servo.get("CUBO");
        RLQ = hardwareMap.servo.get("RLQ");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        colorSensorVerifier =
                hardwareMap.get(NormalizedColorSensor.class, "colorSensorVerifier");

        SP_IZQ.setDirection(DcMotorSimple.Direction.REVERSE);
        IN_IZQ.setDirection(DcMotorSimple.Direction.REVERSE);
        ELE_A1.setDirection(DcMotorSimple.Direction.REVERSE);
        ELE_A2.setDirection(DcMotorSimple.Direction.REVERSE);
        BRZ.setDirection(DcMotorSimple.Direction.REVERSE);

        SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ELE_P.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SP_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SP_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ELE_P.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        JL_DRC.setPosition(0);
        JL_IZQ.setPosition(1);
        RLQ.setPosition(0);
        CUBO.setPosition(1);

        mecanum =
                new LibTMOA(
                        SP_IZQ,
                        IN_IZQ,
                        SP_DRC,
                        IN_DRC,
                        Utilities.WIDTH,
                        Utilities.CPR,
                        Utilities.GEAR_RATIO,
                        Utilities.DIAMETER
                );

        colors = colorSensor.getNormalizedColors();

        digitalTouch1 = hardwareMap.touchSensor.get("digital_touch1");
        digitalTouch2 = hardwareMap.touchSensor.get("digital_touch2");
        foundation = hardwareMap.touchSensor.get("foundation");

        initOpenCV();

        speech = new AndroidTextToSpeech();
        speech.initialize();
        speech.setLanguageAndCountry("en", "US");

        sleep(1000);
        speech.speak("Waiting to start.");
        while (speech.isSpeaking());
        telemetry.addData("Status", "Waiting");
        telemetry.update();
        runtime.reset();
        waitForStart();
    }

    public void runner(){
        switch (stage) {
            case 0:
                mecanum.targetToPositionEncoders();
                left = readOpenCV(1);
                mid = readOpenCV(2);
                right = readOpenCV(3);
                if(mid == 0 && left == 255 && right == 255) {
                    speech.speak("Give me the rock, bitch!");
                    moveToPosition(23, 1);
                } else if (mid == 255 && left == 0 && right == 255) {
                    speech.speak("Give me the rock, bitch!");
                    mecanumToPosition(Utilities.STONE_LENGTH, 0.8);
                    moveToPosition(23, 1);
                } else if (mid == 0 && left == 255 && right == 255) {
                    speech.speak("Give me the rock, bitch!");
                    mecanumToPosition(-1 * Utilities.STONE_LENGTH, 0.8);
                    moveToPosition(23, 1);
                }
                mecanum.withoutEncoders();
                sleep(100);
                break;
            case 2:
                mecanum.move(.8, Math.PI, 0);
                sleep(100);
                mecanum.stop();
                clawRed(1);
                mecanum.move(-1, 0, 0);
                sleep(150);
                mecanum.stop();
                BRZ.setPower(-1);
                turnToPosition(15, 1);
                sleep(400);
                break;
            case 3:
                mecanum.withoutEncoders();
                mecanum.stop();
                BRZ.setPower(0);
                while (red < Utilities.RED_COLOR || redV < Utilities.RED_VERIFIER_COLOR) {
                    mecanum.move(0, 0, 1);
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
                moveToPosition(41, 1);
                turnToPosition(17.5, 1);
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
                mecanum.turnToPosition(70, 1);
                mecanum.withoutEncoders();
                sleep(1500);
                break;
            case 6:
                JL_DRC.setPosition(0);
                JL_IZQ.setPosition(1);
                CUBO.setPosition(0);
                BRZ.setPower(1);
                sleep(150);
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
                mecanum.turnToPosition(-17.3,1);
                sleep(600);
                mecanum.stop();
                mecanum.withoutEncoders();
                mecanum.move(0, 0, 1);
                sleep(2150);
                mecanum.stop();
                mecanum.targetToPositionEncoders();
                turnToPosition(17, 1);
                sleep(50);
                mecanum.withoutEncoders();
                mecanum.stop();
                break;
            case 8:
                left = readOpenCV(1);
                mid = readOpenCV(2);
                right = readOpenCV(3);
                if(mid == 0) {
                    speech.speak("Give me the rock, bitch!");
                    moveToPosition(23, 1);
                } else if (left == 0) {
                    speech.speak("Give me the rock, bitch!");
                    mecanumToPosition(Utilities.STONE_LENGTH, 0.8);
                    moveToPosition(23, 1);
                } else if (right == 0) {
                    speech.speak("Give me the rock, bitch!");
                    mecanumToPosition(-1 * Utilities.STONE_LENGTH, 0.8);
                    moveToPosition(23, 1);
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
                turnperseconds(.5);
                sleep(70);
                mecanum.stop();
                while (!(digitalTouch1.isPressed() || digitalTouch2.isPressed())) {
                    mecanum.move(-1, 0, 0);
                }
                mecanum.stop();
                mecanum.targetToPositionEncoders();
                moveToPosition(3, 1);
                turnToPosition(16, 1);
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

    protected void moveToPosition(double inches, double speed) {
        int move = (int) (Math.round(inches * Utilities.CONVERTION));

        IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
        SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() + move);
        IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() + move);
        SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() + move);

        SP_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SP_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IN_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IN_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        SP_IZQ.setPower(speed);
        IN_IZQ.setPower(speed);
        SP_DRC.setPower(speed);
        IN_DRC.setPower(speed);

        while (
                SP_IZQ.isBusy() && SP_DRC.isBusy() && IN_IZQ.isBusy() && IN_DRC.isBusy()
        ) {
            if (exit) {
                SP_DRC.setPower(0);
                SP_IZQ.setPower(0);
                IN_DRC.setPower(0);
                IN_IZQ.setPower(0);
                return;
            }
        }
        SP_DRC.setPower(0);
        SP_IZQ.setPower(0);
        IN_DRC.setPower(0);
        IN_IZQ.setPower(0);
    }

    protected void turnWithEncoder(double input) {
        SP_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IN_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SP_IZQ.setPower(input);
        IN_IZQ.setPower(input);
        SP_DRC.setPower(-input);
        IN_DRC.setPower(-input);
    }

    protected void clawRed(double orient) {
        CUBO.setPosition(0);
        mecanum.move(0.700, orient * Math.PI / 2, 0);
        sleep(280);
        mecanum.stop();
        BRZ.setPower(1);
        sleep(750);
        BRZ.setPower(0);
        mecanum.move(1, 0, 0);
        sleep(180);
        mecanum.stop();
        CUBO.setPosition(1);
        sleep(700);
        mecanum.move(-1, 0, 0);
        sleep(130);
    }

    protected void clawRed2(double orient) {
        CUBO.setPosition(0);
        BRZ.setPower(1);
        sleep(750);
        BRZ.setPower(0);
        mecanum.withoutEncoders();
        turnperseconds(-.5);
        sleep(80);
        mecanum.stop();
        mecanum.move(0.8, 0, 0);
        sleep(400);
        mecanum.stop();
        CUBO.setPosition(1);
        sleep(700);
    }

    public void mecanumToPosition(double inches, double speed) {
        mecanum.targetToPositionEncoders();
        int move = (int) (Math.round(inches * Utilities.CONVERTION));

        IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
        SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() - move);
        IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() - move);
        SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() + move);

        SP_IZQ.setPower(mecanum.calc2(speed, 0, 0));
        IN_IZQ.setPower(mecanum.calc1(speed, 0, 0));
        SP_DRC.setPower(mecanum.calc1(speed, 0, 0));
        IN_DRC.setPower(mecanum.calc2(speed, 0, 0));

        return;
    }

    public void turnToPosition(double inches, double speed) {
        mecanum.targetToPositionEncoders();
        int move = (int) (Math.round(inches * Utilities.CONVERTION));

        IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
        IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() - move);
        SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() - move);
        SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() + move);

        SP_IZQ.setPower(speed);
        IN_IZQ.setPower(speed);
        SP_DRC.setPower(speed);
        IN_DRC.setPower(speed);

        while (
                SP_IZQ.isBusy() && SP_DRC.isBusy() && IN_IZQ.isBusy() && IN_DRC.isBusy()
        ) {
            if (exit) {
                SP_DRC.setPower(0);
                SP_IZQ.setPower(0);
                IN_DRC.setPower(0);
                IN_IZQ.setPower(0);
                return;
            }
        }
        return;
    }

    public void turnperseconds (double speed){
        SP_IZQ.setPower(speed);
        IN_IZQ.setPower(speed);
        SP_DRC.setPower(-speed);
        IN_DRC.setPower(-speed);
    }

    public void clawBlue2(double orient) {
        CUBO.setPosition(0);
        BRZ.setPower(1);
        sleep(750);
        BRZ.setPower(0);
        mecanum.withoutEncoders();
        turnperseconds(.5);
        sleep(80);
        mecanum.stop();
        mecanum.move(0.8, 0, 0);
        sleep(400);
        mecanum.stop();
        CUBO.setPosition(1);
        sleep(700);
        return;
    }
}
