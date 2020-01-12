package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.teamcode.helpers.LibTMOA;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class Threading extends Thread {
  private DcMotor BRZ;
  private Servo CUBO;
  private DcMotor ELE_A1;
  private DcMotor ELE_A2;
  private DcMotor ELE_P;
  private Blinker expansion_Hub_1;
  private Blinker expansion_Hub_2;
  private DcMotor IN_DRC;
  private DcMotor IN_IZQ;
  private Servo JL_DRC;
  private Servo JL_IZQ;
  private DcMotor SP_DRC;
  private DcMotor SP_IZQ;
  private DistanceSensor colorSensor;
  private DistanceSensor colorSensorVerifier;
  private TouchSensor digitalTouch1;
  private TouchSensor digitalTouch2;
  private TouchSensor foundation;
  private Gyroscope imu1;
  private Gyroscope imu2;
  private DistanceSensor colorDetector;
  private IA ia;
  private LibTMOA mecanum;
  private int stage = 0;
  private int nextSkyStone = 0;
  private static double redC;
  private static double redVC;
  private ElapsedTime runtime = new ElapsedTime();
  private static VuforiaLocalizer.CameraDirection CAMERA_CHOICE;
  private static boolean PHONE_IS_PORTRAIT;
  private static String VUFORIA_KEY;

  private static float mmPerInch;
  private static float mmTargetHeight;

  private static float stoneZ;

  public Threading(IA ia) {
    this.ia = ia;

    this.IN_DRC = ia.IN_DRC;
    this.SP_IZQ = ia.SP_IZQ;
    this.IN_IZQ = ia.IN_IZQ;
    this.SP_DRC = ia.SP_DRC;

    this.BRZ = ia.BRZ;
    this.CUBO = ia.CUBO;
    this.JL_IZQ = ia.JL_IZQ;
    this.JL_DRC = ia.JL_DRC;
    this.foundation = foundation;

    this.digitalTouch1 = ia.digitalTouch1;
    this.digitalTouch2 = ia.digitalTouch2;

    this.mecanum =
      new LibTMOA(
        SP_IZQ,
        IN_IZQ,
        SP_DRC,
        IN_DRC,
        ia.width,
        ia.cpr,
        ia.gearratio,
        ia.diameter
      );

    this.CAMERA_CHOICE = ia.CAMERA_CHOICE;
    this.PHONE_IS_PORTRAIT = ia.PHONE_IS_PORTRAIT;
    this.VUFORIA_KEY = ia.VUFORIA_KEY;

    this.mmPerInch = ia.mmPerInch;
    this.mmTargetHeight = ia.mmTargetHeight;
    this.stoneZ = ia.stoneZ;

    this.redC = ia.redC;
    this.redVC = ia.redVC;
  }

  @Override
  public void run() {
    runtime.reset();
    runtime.startTime();
    while (runtime.time() < 30 && ia.opModeIsActive()) {
      if (stage == 0) {
        ia.telemetry.addData("Stage: ", "0");
        mecanum.moveToPosition(26, 1);
        stage++;
      } else if (stage == 1) {
        ia.telemetry.addData("Stage: ", "1");
        while (!ia.vuforiaRead().equals("Stone Target")) {
          mecanum.mecanumToPosition(ia.stoneLength, 0.8);
          delay(600);
          mecanum.stop();
          nextSkyStone++;
          delay(150);
        }
        mecanum.withoutEncoders();
        claw(ia.allianceMultiplicator);
        stage++;
      } else if (stage == 2) {
        ia.telemetry.addData("Stage: ", "2");
        BRZ.setPower(-1);
        delay(500);
        mecanum.turnToPosition(17, 1);
        delay(600);
        mecanum.withoutEncoders();
        mecanum.stop();
        BRZ.setPower(0);
        stage=4;
      } else if (stage == 3) {
        ia.telemetry.addData("Stage: ", "3");
        while (ia.red < redC || ia.redV < redVC) {
          mecanum.move(0, 0, 1);
          ia.colors = ia.colorSensor.getNormalizedColors();
          ia.colorsV = ia.colorSensorVerifier.getNormalizedColors();
          ia.red = ia.colors.red;
          ia.redV = ia.colorsV.red;
        }
        mecanum.stop();
        delay(20);
        stage++;
      } else if (stage == 4) {
        ia.telemetry.addData("Stage: ", "4");
        mecanum.targetToPositionEncoders();
        mecanum.moveToPosition(46, 1);
        mecanum.turnToPosition(17.4, 1);/*
        mecanum.stop();
        delay(700);
        mecanum.withoutEncoders();
        while (!foundation.isPressed()) {
          mecanum.move(0.4, Math.PI, 0);
        }
        mecanum.stop();*/
        stage++;
      } else if (stage == 5) {
        ia.telemetry.addData("Stage: ", "5");
        JL_DRC.setPosition(1);
        JL_IZQ.setPosition(0);
        delay(350);
        mecanum.move(1, 7 * Math.PI / 4, 0);
        delay(1500);
        mecanum.stop();
        CUBO.setPosition(0);
        mecanum.turnToPosition(22, 1);
        delay(800);
        mecanum.withoutEncoders();
        mecanum.stop();
        stage++;
      } else if (stage == 6) {
        ia.telemetry.addData("Stage: ", "6");
        JL_DRC.setPosition(0);
        JL_IZQ.setPosition(1);
        mecanum.move(-Math.PI / 2, 1, 0);
        delay(200);
        mecanum.move(-1, 0, 0);
        delay(1000);
        mecanum.stop();
        stage++;
      } else if (stage == 7) {
        ia.telemetry.addData("Stage: ", "7");
        mecanum.move(Math.PI / 2, 1, 0);
        delay(170);
        mecanum.move(0, 0, 1);
        delay(2000);
        mecanum.stop();
        CUBO.setPosition(1);
        mecanum.turnToPosition(17, 1);
        delay(700);
        mecanum.withoutEncoders();
        mecanum.move(1, Math.PI / 2, 0);
        delay(150);
        ia.targetsSkyStone.activate();
        stage++;
        while (!(digitalTouch1.isPressed() || digitalTouch2.isPressed())) {
          mecanum.move(.8, Math.PI, 0);
        }
        mecanum.stop();
        stage++;
      } else if (stage == 8) {
        ia.telemetry.addData("Stage: ", "8");
        if (nextSkyStone == 1) {
          mecanum.move(0.7, 7 * Math.PI / 4, 0);
          delay(300);
          mecanum.stop();
        } else if (nextSkyStone == 2) {
          mecanum.move(0.7, 15 * Math.PI / 8, 0);
          delay(300);
          mecanum.stop();
        } else if (nextSkyStone == 3) {
          mecanum.targetToPositionEncoders();
          mecanum.moveToPosition(22.8, 1);
        }
        stage++;
      } else if (stage == 9) {
        ia.telemetry.addData("Stage: ", "9");
        claw(ia.allianceMultiplicator);
        delay(500);
        mecanum.stop();
        mecanum.move(-1, 0, 0);
        delay(300);
        mecanum.stop();
        mecanum.turnToPosition(21.5, 1);
        delay(850);
        mecanum.withoutEncoders();
        mecanum.stop();
        stage++;
      } else if (stage == 10) {
        ia.telemetry.addData("Stage: ", "10");
        ia.colors = ia.colorSensor.getNormalizedColors();
        ia.colorsV = ia.colorSensorVerifier.getNormalizedColors();
        ia.red = ia.colors.red;
        ia.redV = ia.colorsV.red;
        while (ia.red < redC || ia.redV < redVC) {
          ia.colors = ia.colorSensor.getNormalizedColors();
          ia.colorsV = ia.colorSensorVerifier.getNormalizedColors();
          ia.red = ia.colors.red;
          ia.redV = ia.colorsV.red;
        }
        stage++;
      } else if (stage == 11) {
        ia.telemetry.addData("Stage: ", "11");
        BRZ.setPower(-1);
        delay(350);
        BRZ.setPower(-0.35);
        mecanum.moveToPosition(25, 1);
        mecanum.withoutEncoders();
        CUBO.setPosition(0);
        mecanum.move(1, Math.PI, 0);
        delay(250);
        BRZ.setPower(0);
        stage++;
      } else if (stage == 12) {
        ia.telemetry.addData("Stage: ", "12");
        ia.colors = ia.colorSensor.getNormalizedColors();
        ia.colorsV = ia.colorSensorVerifier.getNormalizedColors();
        ia.red = ia.colors.red;
        ia.redV = ia.colorsV.red;
        while (ia.red < redC || ia.redV < redVC) {
          mecanum.move(0, 0, -1);
          ia.colors = ia.colorSensor.getNormalizedColors();
          ia.colorsV = ia.colorSensorVerifier.getNormalizedColors();
          ia.red = ia.colors.red;
          ia.redV = ia.colorsV.red;
        }
        mecanum.stop();
        stage++;
      }

      ia.telemetry.update();
    }
  }

  private void delay(int ms, String errMsg) {
    try {
      sleep(ms);
    } catch (InterruptedException e) {
      ia.telemetry.addData("Status", "Thread Failed: " + errMsg);
    }
  }

  private void delay(int ms) {
    try {
      sleep(ms);
    } catch (InterruptedException e) {
      ia.telemetry.addData("Status", "Thread Failed");
    }
  }
  
  public void claw(double orient) {
    CUBO.setPosition(0);
    mecanum.move(-1, 0, 0);
    delay(60);
    mecanum.stop();
    BRZ.setPower(1);
    delay(500);
    BRZ.setPower(0);
    mecanum.move(0.700, orient * Math.PI / 2, 0);
    delay(300);
    mecanum.stop();
    mecanum.move(.45, 0, 0);
    delay(500);
    mecanum.stop();
    CUBO.setPosition(1);
    delay(700);
    return;
  }
}
