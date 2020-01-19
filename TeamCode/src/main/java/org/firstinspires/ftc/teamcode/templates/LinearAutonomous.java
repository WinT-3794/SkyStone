/*
 *   Copyright (c) 2019 under MIT license. All rights reserved.
 *   Created by Manuel Díaz and Paolo Reyes for WinT 15645 Subteam of WinT 3794
 *   This code was used in FIRST Tech Challenge 2019 - 2020
 */
package org.firstinspires.ftc.teamcode.templates;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.teamcode.util.LibTMOA;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.Utilities;

public class LinearAutonomous extends LinearOpMode {
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

    @Override
    public void runOpMode() {
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

        //colorDetector =
        //  hardwareMap.get(NormalizedColorSensor.class, "colorDetector");
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

        int cameraMonitorViewId = hardwareMap
                .appContext.getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
                cameraMonitorViewId
        );

        colors = colorSensor.getNormalizedColors();

        digitalTouch1 = hardwareMap.touchSensor.get("digital_touch1");
        digitalTouch2 = hardwareMap.touchSensor.get("digital_touch2");
        foundation = hardwareMap.touchSensor.get("foundation");

        parameters.vuforiaLicenseKey = Utilities.VUFORIA_KEY;
        parameters.cameraDirection = Utilities.CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone =
                this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(
                OpenGLMatrix
                        .translation(0, 0, Utilities.STONE_Z)
                        .multiplied(
                                Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)
                        )
        );

        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * Utilities.MM_PER_INCH;
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * Utilities.MM_PER_INCH;
        final float CAMERA_LEFT_DISPLACEMENT = 0;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(
                        CAMERA_FORWARD_DISPLACEMENT,
                        CAMERA_LEFT_DISPLACEMENT,
                        CAMERA_VERTICAL_DISPLACEMENT
                )
                .multiplied(
                        Orientation.getRotationMatrix(
                                EXTRINSIC,
                                YZX,
                                DEGREES,
                                Utilities.PHONE_Y_ROTATE,
                                Utilities.PHONE_Z_ROTATE,
                                Utilities.PHONE_X_ROTATE
                        )
                );

        for (VuforiaTrackable trackable : allTrackables) {
            (
                    (VuforiaTrackableDefaultListener) trackable.getListener()
            ).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();
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

        runtime.startTime();
        while (opModeIsActive()){runner();}
    }

    private void climbElevator(double speed) {
        ELE_P.setPower(speed);
        ELE_A1.setPower(speed);
        ELE_A2.setPower(speed);
    }

    private void climbArm(double speed) {
        BRZ.setPower(speed);
    }

    public void moveToPosition(double inches, double speed) {
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
        return;
    }

    public void turnWithEncoder(double input) {
        SP_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IN_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SP_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IN_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SP_IZQ.setPower(input);
        IN_IZQ.setPower(input);
        SP_DRC.setPower(-input);
        IN_DRC.setPower(-input);
    }

    public void clawRed(double orient) {
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
        return;
    }

    public void clawRed2(double orient) {
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
        return;
    }

    public String vuforiaRead() {
        CameraDevice.getInstance().setFlashTorchMode(true);
        String target = "";
        for (VuforiaTrackable trackable : allTrackables) {
            if (
                    ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()
            ) {
                target = trackable.getName();
                CameraDevice.getInstance().setFlashTorchMode(false);
            }
            break;
        }
        return target;
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

    public void runner(){
        //Write here in your Child Auto
    }
}
