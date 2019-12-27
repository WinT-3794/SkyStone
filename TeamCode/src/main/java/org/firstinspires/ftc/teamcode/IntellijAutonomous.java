/*
 *   Copyright (c) 2019 under MIT license. All rights reserved.
 *   Created by Manuel Díaz and Obed García with help of Paolo Reyes for WinT 15645 Subteam of WinT 3794
 *   This code was used in FIRST Tech Challenge 2019 - 2020
 */
package org.firstinspires.ftc.teamcode;

import android.content.ContentValues;
import android.database.sqlite.SQLiteDatabase;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import java.lang.Thread;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.helpers.LibTMOA;
import org.firstinspires.ftc.teamcode.helpers.SQLiteHelper;
import org.firstinspires.ftc.teamcode.helpers.Utilities;

@Autonomous(name = "Intellij Autonomous", group = "Joker")
public class IntellijAutonomous extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();


  public TouchSensor digitalTouch1;
  public TouchSensor digitalTouch2;
  public TouchSensor foundation;

  private boolean isCalcCorrect = true;
  private boolean isRedAlliance = true;
  private double nextSkyStoneInches = 0;
  private boolean flag = true;
  Boolean exit = false;
  private VuforiaLocalizer vuforia = null;
  private float phoneXRotate = 0;
  private float phoneYRotate = 0;
  private float phoneZRotate = 0;
  private LibTMOA mecanum;

  public DcMotor SP_DRC;
  public DcMotor IN_DRC;
  public DcMotor SP_IZQ;
  public Servo JL_DRC;
  public Servo JL_IZQ;
  public DcMotor IN_IZQ;
  private DcMotor ELE_P;
  public DcMotor BRZ;
  private DcMotor ELE_A1;
  private DcMotor ELE_A2;
  public Servo CUBO;
  private Servo RLQ;
  public NormalizedColorSensor colorSensor;
  public NormalizedColorSensor colorSensorVerifier;
  public NormalizedColorSensor colorDetector;

  private List<VuforiaTrackable> allTrackables;
  public VuforiaTrackables targetsSkyStone;
  private AndroidTextToSpeech speech;

  public NormalizedRGBA colors;
  public NormalizedRGBA colorsV;
  public float red = 0;
  public float redV = 0;
  public float blue = 0;
  public float blueV = 0;
  private int stage = 0;
  private int nextSkyStone = -1;

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

    if (Utilities.CAMERA_CHOICE == BACK) {
      phoneYRotate = -90;
    } else {
      phoneYRotate = 90;
    }

    if (Utilities.PHONE_IS_PORTRAIT) {
      phoneXRotate = 90;
    }

    OpenGLMatrix robotFromCamera = OpenGLMatrix
      .translation(
        Utilities.CAMERA_FORWARD_DISPLACEMENT,
        Utilities.CAMERA_LEFT_DISPLACEMENT,
        Utilities.CAMERA_VERTICAL_DISPLACEMENT
      )
      .multiplied(
        Orientation.getRotationMatrix(
          EXTRINSIC,
          YZX,
          DEGREES,
          phoneYRotate,
          phoneZRotate,
          phoneXRotate
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
    speech.speak("Select your alliance");
    telemetry.addData("Status", "Selecting alliance");
    telemetry.update();
    while (speech.isSpeaking());
    sleep(20);

    while (flag && !opModeIsActive()) {
      if (digitalTouch1.isPressed()) {
        speech.speak("Red Alliance Selected");
        while (speech.isSpeaking());
        isRedAlliance = true;
        flag = false;
      } else if (digitalTouch2.isPressed()) {
        speech.speak("Blue Alliance Selected");
        while (speech.isSpeaking());
        isRedAlliance = false;
        flag = false;
      }
    }

    if(!flag) {
      if(isRedAlliance){
        telemetry.addData("Status", "Waiting: Red Alliance Selected");
      } else {
        telemetry.addData("Status", "Waiting: Blue Alliance Selected");
      }
      telemetry.update();
      speech.speak("Waiting to start.");
      while (speech.isSpeaking());
    } else {
      speech.speak("Red Alliance Selected by Default");
    }

    runtime.reset();
    waitForStart();

    runtime.startTime();
    while (runtime.time() < 30 && opModeIsActive()) {
      if(isRedAlliance) {
        switch (stage) {
          case 0:
            moveToPosition(24, 1);
            sleep(100);
            mecanum.stop();
            break;
          case 1:
            sleep(200);
            if (vuforiaRead().equals("Stone Target")) {
              nextSkyStone = 1;
              break;
            }
            while (nextSkyStone < 4 && !vuforiaRead().equals("Stone Target")) {
              mecanumToPosition(Utilities.STONE_LENGHT, 0.8);
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
            claw(1);
            BRZ.setPower(-1);
            turnToPosition(16.2, 1);
            sleep(400);
            break;
          case 3:
            mecanum.withoutEncoders();
            mecanum.stop();
            BRZ.setPower(0);
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
            sleep(20);
            break;
          case 4:
            mecanum.targetToPositionEncoders();
            moveToPosition(34, 1);
            turnToPosition(18, 1);
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
            mecanum.moveWithoutLimits(7 * Math.PI / 4);
            sleep(1300);
            mecanum.stop();
            turnToPosition(20, 1);
            sleep(100);
            mecanum.withoutEncoders();
            mecanum.stop();
            sleep(50);
            break;
          case 6:
            JL_DRC.setPosition(0);
            JL_IZQ.setPosition(1);
            mecanum.move(1, 3 * Math.PI / 2, 0);
            CUBO.setPosition(0);
            BRZ.setPower(0.45);
            sleep(70);
            BRZ.setPower(0.35);
            mecanum.move(-1, 0, 0);
            sleep(1250);
            BRZ.setPower(0);
            mecanum.stop();
            sleep(150);
            break;
          case 7:
            mecanum.move(0, 0, 1);
            CUBO.setPosition(1);
            sleep(2050);
            mecanum.stop();
            turnToPosition(20.4, 1);
            sleep(50);
            mecanum.withoutEncoders();
            mecanum.stop();
            sleep(20);
            break;
          case 8:
            while (!(digitalTouch1.isPressed() || digitalTouch2.isPressed())) {
              mecanum.move(.8, Math.PI, 0);
            }
            mecanum.stop();
            sleep(250);
            mecanum.targetToPositionEncoders();
            moveToPosition(25, 1);
            sleep(150);
            mecanum.stop();
            mecanum.withoutEncoders();
            sleep(20);
            break;
          case 9:
            mecanum.move(1, 3 * -Math.PI / 2, 0);
            sleep(600);
            mecanum.stop();
            sleep(150);
            targetsSkyStone.activate();
            nextSkyStoneInches = (3 - nextSkyStone) * Utilities.STONE_LENGHT + 5;

            if (nextSkyStoneInches < 0) {
              nextSkyStoneInches = 0;
              isCalcCorrect = false;
            }

            if (isCalcCorrect) {
              mecanumToPosition(-nextSkyStoneInches, 0.5);
            } else {
              nextSkyStone = 0;
              sleep(300);
              while (nextSkyStone < 4) {
                if (vuforiaRead().equals("Stone Target")) {
                  break;
                } else {
                  mecanumToPosition(-Utilities.STONE_LENGHT, 0.8);
                  sleep(800);
                  mecanum.stop();
                  sleep(300);
                  nextSkyStone++;
                }
              }
            }

            if (isCalcCorrect) {
              sleep(300);
              if (!vuforiaRead().equals("Stone Target")) {
                telemetry.addData("Status", "Unknown Stone");
              }
            }
            targetsSkyStone.deactivate();
            sleep(20);
            break;
          case 10:
            mecanum.withoutEncoders();
            mecanum.move(.8, Math.PI, 0);
            sleep(100);
            mecanum.stop();
            claw();
            mecanum.stop();
            mecanum.move(-1, 0, 0);
            sleep(320);
            mecanum.stop();
            mecanum.targetToPositionEncoders();
            turnToPosition(17.6, 1);
            sleep(50);
            mecanum.withoutEncoders();
            mecanum.stop();
            sleep(20);
            break;
          case 11:
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
            break;
          case 12:
            BRZ.setPower(-1);
            sleep(300);
            BRZ.setPower(-0.35);
            moveToPosition(34, 1);
            mecanum.withoutEncoders();
            CUBO.setPosition(0);
            sleep(150);
            mecanum.targetToPositionEncoders();
            BRZ.setPower(-0.35);
            moveToPosition(-51, 1);
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
      } else {
        switch (stage) {
          case 0:
            moveToPosition(24, 1);
            sleep(100);
            mecanum.stop();
            break;
          case 1:
            sleep(200);
            if (vuforiaRead().equals("Stone Target")) {
              nextSkyStone = 1;
              break;
            }
            while (nextSkyStone < 4 && !vuforiaRead().equals("Stone Target")) {
              mecanumToPosition(-Utilities.STONE_LENGHT, 0.8);
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
            claw(3);
            BRZ.setPower(-1);
            turnToPosition(-16.2, 1);
            sleep(400);
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
            moveToPosition(34, 1);
            turnToPosition(-18, 1);
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
            mecanum.moveWithoutLimits(Math.PI / 4);
            sleep(1300);
            mecanum.stop();
            turnToPosition(-20, 1);
            sleep(100);
            mecanum.withoutEncoders();
            mecanum.stop();
            sleep(50);
            break;
          case 6:
            JL_DRC.setPosition(0);
            JL_IZQ.setPosition(1);
            mecanum.move(1, Math.PI / 2, 0);
            CUBO.setPosition(0);
            BRZ.setPower(0.45);
            sleep(70);
            BRZ.setPower(0.35);
            mecanum.move(-1, 0, 0);
            sleep(1250);
            BRZ.setPower(0);
            mecanum.stop();
            sleep(150);
            break;
          case 7:
            mecanum.move(0, 0, 1);
            CUBO.setPosition(1);
            sleep(2050);
            mecanum.stop();
            turnToPosition(-20.4, 1);
            sleep(50);
            mecanum.withoutEncoders();
            mecanum.stop();
            sleep(20);
            break;
          case 8:
            while (!(digitalTouch1.isPressed() || digitalTouch2.isPressed())) {
              mecanum.move(.8, Math.PI, 0);
            }
            mecanum.stop();
            sleep(250);
            mecanum.targetToPositionEncoders();
            moveToPosition(25, 1);
            sleep(150);
            mecanum.stop();
            mecanum.withoutEncoders();
            sleep(20);
            break;
          case 9:
            mecanum.move(1, 3 * Math.PI / 2, 0);
            sleep(600);
            mecanum.stop();
            sleep(150);
            targetsSkyStone.activate();
            nextSkyStoneInches = (3 - nextSkyStone) * Utilities.STONE_LENGHT + 5;

            if (nextSkyStoneInches < 0) {
              nextSkyStoneInches = 0;
              isCalcCorrect = false;
            }

            if (isCalcCorrect) {
              mecanumToPosition(nextSkyStoneInches, 0.5);
            } else {
              nextSkyStone = 0;
              sleep(300);
              while (nextSkyStone < 4) {
                if (vuforiaRead().equals("Stone Target")) {
                  break;
                } else {
                  mecanumToPosition(Utilities.STONE_LENGHT, 0.8);
                  sleep(800);
                  mecanum.stop();
                  sleep(300);
                  nextSkyStone++;
                }
              }
            }

            if (isCalcCorrect) {
              sleep(300);
              if (!vuforiaRead().equals("Stone Target")) {
                telemetry.addData("Status", "Unknown Stone");
              }
            }
            targetsSkyStone.deactivate();
            sleep(20);
            break;
          case 10:
            mecanum.withoutEncoders();
            mecanum.move(.8, Math.PI, 0);
            sleep(100);
            mecanum.stop();
            claw();
            mecanum.stop();
            mecanum.move(-1, 0, 0);
            sleep(320);
            mecanum.stop();
            mecanum.targetToPositionEncoders();
            turnToPosition(-17.6, 1);
            sleep(50);
            mecanum.withoutEncoders();
            mecanum.stop();
            sleep(20);
            break;
          case 11:
            colors = colorSensor.getNormalizedColors();
            colorsV = colorSensorVerifier.getNormalizedColors();
            blue = colors.blue;
            blueV = colorsV.blue;
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
            break;
          case 12:
            BRZ.setPower(-1);
            sleep(300);
            BRZ.setPower(-0.35);
            moveToPosition(34, 1);
            mecanum.stop();
            CUBO.setPosition(0);
            sleep(150);
            BRZ.setPower(-0.35);
            moveToPosition(-51, 1);
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
  }

  public void moveToPosition(double inches, double speed) {
    int move = (int) (Math.round(inches * Utilities.CONVERSION));

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

  public void claw(double orient) {
    CUBO.setPosition(0);
    mecanum.move(0.700, orient * Math.PI / 2, 0);
    sleep(220);
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

  public void claw() {
    CUBO.setPosition(0);
    BRZ.setPower(1);
    sleep(750);
    BRZ.setPower(0);
    mecanum.targetToPositionEncoders();
    turnToPosition(-3.8, 1);
    sleep(100);
    mecanum.withoutEncoders();
    mecanum.move(0.8, 0, 0);
    sleep(180);
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
    int move = (int) (Math.round(inches * Utilities.CONVERSION));

    IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
    SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() - move);
    IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() - move);
    SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() + move);

    SP_IZQ.setPower(mecanum.calc2(speed, 0, 0));
    IN_IZQ.setPower(mecanum.calc1(speed, 0, 0));
    SP_DRC.setPower(mecanum.calc1(speed, 0, 0));
    IN_DRC.setPower(mecanum.calc2(speed, 0, 0));
  }

  public void turnToPosition(double inches, double speed) {
    mecanum.targetToPositionEncoders();
    int move = (int) (Math.round(inches * Utilities.CONVERSION));

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
  }

  public void regMatch(int id, double scan, double half, double finish){
    SQLiteHelper db_connect = new SQLiteHelper(null, Utilities.DATABASE_NAME, null, Utilities.DATABASE_VERSION);
    SQLiteDatabase db = db_connect.getWritableDatabase();
    ContentValues values = new ContentValues();
    values.put(Utilities.ID_INDEX, id);
    values.put(Utilities.SCAN_INDEX, scan);
    values.put(Utilities.HALF_INDEX, half);
    values.put(Utilities.FINAL_INDEX, finish);

    Long newID = db.insert(Utilities.TABLE_NAME, Utilities.ID_INDEX, values);
    db.close();
  }
}
