/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.auto.main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.lib.hardware.base.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.hardware.manip.Lift;

import java.util.List;

@Autonomous(group = "main")

public class    ScoreLeft extends LinearOpMode {


    private static final String TFOD_MODEL_ASSET = "betterpp7571sleeve.tflite";

    private static final String[] LABELS = {
            "1 Yellow",
            "2 Blue",
            "3 Green"
    };

    private static final String VUFORIA_KEY =
            "AeSrvI3/////AAABmUSQfCByrUcZjWTMUHPjMY1IIUpYyRMjVhj2brmEUl+sUo+t82OGML/Oq35Z7QZ8BdgUssPKUr409P5eQ/JF/jCY6mmqkPcEBBZqcoSaIR0XbIVSZGgsI7pgrDm9XvOGmbnjoFYIop/KIIrHeRt99uj2qC0P0AmWbwABM6Ma09jitjXGbNdAdnn2KJRGWPYvAjutSJ0e9UpgZBl30uLyhIp9VNolBAL8DsuBZs9FDmrY6jf7CftowUxGbDk6jfymEqoqPOqNH0L/AbCHlsr2UJmTSj8mZEqPabCnbULMcamnTYwR7J14bjmGGCrjQ5JifN/H33RwUFAhCYhl7fmBiqZKvAu3jCdvGmVR/QUYpPz3";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private DriveTrain dt = new DriveTrain();
    private ElapsedTime runtime = new ElapsedTime();
    private WebcamName Webcam1;
    private DcMotorEx[] motors;
    private BNO055IMU gyro;
    private int degreeOffset = 2;
    public Lift lift = new Lift();
    public Servo claw;
    public int park = 2;
    public int tracker = 5;

    @Override
    public void runOpMode() {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        motors = new DcMotorEx[]{hardwareMap.get(DcMotorEx.class, "fl"), hardwareMap.get(DcMotorEx.class, "fr"), hardwareMap.get(DcMotorEx.class, "bl"), hardwareMap.get(DcMotorEx.class, "br")};
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        lift.init(hardwareMap.get(DcMotorEx.class, "lift"));
        claw = hardwareMap.get(Servo.class, "claw");
        Webcam1 = hardwareMap.get(WebcamName.class, "Webcam1");



        dt.initMotors(motors);
        dt.initGyro(gyro);
        claw.setPosition(0.1);
        waitForStart();

        //Auto Commands
        sleep(500);
        if(tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size() == 1) {
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    if (recognition.getLabel() == "1 Yellow") {
                        park--;
                    } else if (recognition.getLabel() == "3 Green") {
                        park++;
                    }
                    telemetry.addData("Parking Space", park);
                }
                telemetry.update();
            }
        }
        dt.strafeDistance(24,1000,opModeIsActive());
        dt.driveDistance(-51,1500,opModeIsActive());
        dt.strafeDistance(-15,1000,opModeIsActive());
        score(3);
        turnDegrees(88,500);
        dt.driveDistance(-2,500,opModeIsActive());
        dt.strafeDistance(2,500,opModeIsActive());
        dt.driveDistance(-28,1000,opModeIsActive());
        for (int i = 1; i <= 5; i++){
            cycle();
            tracker--;
        }
        dt.driveDistance(4,500,opModeIsActive());
        if (park == 1){
            dt.strafeDistance(36, 1000,opModeIsActive());
        }
        else if(park == 3){
            dt.strafeDistance(-36,1000,opModeIsActive());
        }
        while (opModeIsActive()){}
    }


    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void turnDegrees(double turnDegrees,int velocity){
        dt.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double offset;
        offset = dt.getGyroRotation(AngleUnit.DEGREES);
        int predictedTicks = (int)(turnDegrees*9.34579439252+2000);
        double correctedDegrees;


        correctedDegrees = offset + turnDegrees;
        if(correctedDegrees>180){
            correctedDegrees-=360;
        }
        if(turnDegrees>0){
            dt.setDrivetrainPositions(-predictedTicks,predictedTicks,-predictedTicks,predictedTicks);
            dt.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
            dt.setDrivetrainVelocity(velocity);
            while(correctedDegrees-degreeOffset>dt.getGyroRotation(AngleUnit.DEGREES)&&opModeIsActive()){
                telemetry.addLine("Running left");
                telemetry.addData("gyro Target: ", correctedDegrees);
                telemetry.addData("gyro: ", dt.getGyroRotation(AngleUnit.DEGREES));
                telemetry.addData("Offset: ", offset);
                telemetry.update();
            }
        }else if(turnDegrees<0){
            dt.setDrivetrainPositions(predictedTicks,-predictedTicks,predictedTicks,-predictedTicks);
            dt.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
            dt.setDrivetrainVelocity(velocity);
            while(correctedDegrees+degreeOffset<dt.getGyroRotation(AngleUnit.DEGREES)&&opModeIsActive()){
                telemetry.addLine("Running right");
                telemetry.addData("gyro: ", dt.getGyroRotation(AngleUnit.DEGREES));
                telemetry.addData("gyro Target: ", correctedDegrees);
                telemetry.addData("Offset: ", offset);
                telemetry.update();
            }

        }
        dt.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void grab(int level){
        if (level == 5){
            lift.targetDistance(6, 2000);
        }
        else if (level == 4){
            lift.targetDistance(4.5, 2000);
        }
        else if (level == 3){
            lift.targetDistance(3, 2000);
        }
        while(lift.lift.isBusy()){}
        claw.setPosition(0.1);
        sleep(1000);
        if (level == 5){
            lift.targetDistance(12, 2000);
        }
        else if (level == 4){
            lift.targetDistance(10.5, 2000);
        }
        else if (level == 3){
            lift.targetDistance(9, 2000);
        }
        else{
            lift.targetDistance(6,2000);
        }
        while(lift.lift.isBusy()){}
    }

    public void score(int level){
        if (level == 1){
            lift.targetDistance(16,2000);
        }
        else if (level == 2){
            lift.targetDistance(24, 2000);
        }
        else{
            lift.targetDistance(34, 2000);
        }
        while(lift.lift.isBusy()){}
        dt.driveDistance(-2,500,opModeIsActive());
        sleep(500);
        claw.setPosition(0.4);
        dt.driveDistance(2,500,opModeIsActive());
        lift.lift.setTargetPosition(0);
    }

    public void cycle(){
        grab(tracker);
        dt.driveDistance(10, 1000, opModeIsActive());
        turnDegrees(88,750);
        score(1);
        turnDegrees(-88,500);
        dt.driveDistance(-10,1000,opModeIsActive());
    }

}
