package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Stats.CIRCLING_HIGH;
import static org.firstinspires.ftc.teamcode.Stats.CIRCLING_LOW;
import static org.firstinspires.ftc.teamcode.Stats.LEFT;
import static org.firstinspires.ftc.teamcode.Stats.RED_TEAM;
import static org.firstinspires.ftc.teamcode.Stats.RIGHT;
import static org.firstinspires.ftc.teamcode.Stats.TERMINAL;

public class Robot {
    private HardwareMap hardwareMap;
    private ElapsedTime runtime;
    private Telemetry telemetry;
    private DcMotor leftDriveA, leftDriveB, rightDriveA, rightDriveB, armLeft, armRight, suckerRight, suckerLeft;
    private Servo clawRight, clawLeft, julie;
    private ColorSensor julieSensor;
    private BNO055IMU gyro;
    private ArrayList<Action> actions = new ArrayList<>();
    private ArrayList<String> permanent_messages = new ArrayList<>();
    private int MODE = Stats.MULTI;
    //Vuforia
    private boolean vuforiaStatus = false;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackable relicTemplate = null;
    private VuforiaTrackables relicTrackables = null;

    public Robot(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    public void init() {
        hardwareInit();
        collapse();
    }

    public void glue(String message) {
        permanent_messages.add(message);
    }

    public void peel() {
        permanent_messages.clear();
    }

    public void emergency() {
        emergencyStop();
    }

    public void loop() {
        handleActions();
        showMessages();
    }

    public void addAction(Action a) {
        actions.add(a);
    }

    public void addActions(Action[] a) {
        actions.addAll(Arrays.asList(a));
    }

    public void addActions(ArrayList<Action> a) {
        actions.addAll(a);
    }

    public void prepareForAutonomous() {
        resetRobot();
        actions.clear();
        MODE = Stats.LINEBYLINE;
    }

    public void collapse() {
    }

    private void handleActions() {
        if (MODE == Stats.LINEBYLINE) {
            if (actions.size() != 0) {
                if (actions.get(0).isAlive()) {
                    if (!actions.get(0).isSetup()) {
                        actions.get(0).setup();
                    } else {
                        actions.get(0).loop();
                    }
                } else {
                    actions.remove(0);
                }
            }
        } else if (MODE == Stats.MULTI) {
            for (int a = 0; a < actions.size(); a++) {
                Action act = actions.get(a);
                if (act.isAlive()) {
                    if (!act.isSetup()) {
                        act.setup();
                    } else {
                        act.loop();
                    }
                }
            }
        }
    }

    private void showMessages() {
        for (int s = 0; s < permanent_messages.size(); s++) {
            telemetry.addLine(permanent_messages.get(s));
        }
        showStats();
    }

    private void showStats() {
        telemetry.addData("Actions:", actions.size());
        if (MODE == Stats.MULTI) {
            telemetry.addData("Mode", "MultiAction");
        } else {
            telemetry.addData("Mode", "ActionByAction");
        }
        showPower();
        telemetry.addData("Gyro 1:", gyro.getAngularOrientation().firstAngle);
        telemetry.addData("Gyro 2:", gyro.getAngularOrientation().secondAngle);
        telemetry.addData("Gyro 3:", gyro.getAngularOrientation().thirdAngle);
        //        telemetry.addData("Seconds", (int) runtime.seconds());
        //        telemetry.addData("Motor", leftDrive.getPower() + " " + rightDrive.getPower());
    }

    private void showPower() {
        telemetry.addData("LA:", leftDriveA.getPower());
        telemetry.addData("LB:", leftDriveB.getPower());
        telemetry.addData("RA:", rightDriveA.getPower());
        telemetry.addData("RB:", rightDriveB.getPower());
    }

    private void resetRobot() {
        leftDriveA.setPower(0);
        leftDriveB.setPower(0);
        rightDriveA.setPower(0);
        rightDriveB.setPower(0);
        leftDriveA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //        resetArm();
    }

    private void resetArm() {
        armLeft.setPower(0);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setPower(0);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void emergencyStop() {
        resetRobot();
        actions.clear();
        telemetry.addLine("Emergency Stop!");
    }

    private void hardwareInit() {
        //        initServos();
        initMotors();
        //        initColor();
        initGyro();
    }

    private void collapseJulie() {
        julie.setPosition(0);
    }

    private void initServos() {
        initClaw();
        initJulie();
    }

    private void initClaw() {
        clawLeft = hardwareMap.get(Servo.class, "s1");
        clawRight = hardwareMap.get(Servo.class, "s2");
        clawLeft.setDirection(Servo.Direction.REVERSE);
        clawRight.setDirection(Servo.Direction.FORWARD);
    }

    private void initJulie() {
        julie = hardwareMap.get(Servo.class, "jul");
        julie.setDirection(Servo.Direction.FORWARD);
    }

    private void initMotors() {
        initLeftDrive();
        initRightDrive();
        initArm();
    }

    private void initLeftDrive() {
        leftDriveA = hardwareMap.get(DcMotor.class, "leftDriveA");
        leftDriveB = hardwareMap.get(DcMotor.class, "leftDriveB");
        leftDriveA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveA.setDirection(DcMotor.Direction.REVERSE);
        leftDriveB.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initRightDrive() {
        rightDriveA = hardwareMap.get(DcMotor.class, "rightDriveA");
        rightDriveB = hardwareMap.get(DcMotor.class, "rightDriveB");
        rightDriveA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveA.setDirection(DcMotor.Direction.FORWARD);
        rightDriveB.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initArm() {
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setDirection(DcMotor.Direction.FORWARD);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setDirection(DcMotor.Direction.REVERSE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initSucker() {
        suckerLeft = hardwareMap.get(DcMotor.class, "suckerLeft");
        suckerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        suckerLeft.setDirection(DcMotor.Direction.REVERSE);
        suckerLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        suckerRight = hardwareMap.get(DcMotor.class, "suckerRight");
        suckerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        suckerRight.setDirection(DcMotor.Direction.REVERSE);
        suckerRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initColor() {
        julieSensor = hardwareMap.get(ColorSensor.class, "color");
        julieSensor.enableLed(false);
    }

    private void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = Stats.Vuforia.key;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        vuforiaStatus = true;
    }

    public void julieUp() {
        julie.setPosition(Stats.julZero);
    }

    public void julieDown() {
        julie.setPosition(Stats.fullDown);
    }

    public void drifttt() {
        leftDriveA.setPower(-1);
        leftDriveB.setPower(1);
        rightDriveA.setPower(-1);
        rightDriveB.setPower(1);
    }

    public void drive(double power) {
        leftDriveA.setPower(power);
        leftDriveB.setPower(power);
        rightDriveA.setPower(power);
        rightDriveB.setPower(power);
    }

    public void turn(double power, double turn) {
        if (power == 0) {
            //Commented Is The "OneOmni" Config.
            leftDriveA.setPower(turn);
            leftDriveB.setPower(turn);
            rightDriveA.setPower(-turn);
            rightDriveB.setPower(-turn);
            //            leftDriveA.setPower(turn);
            //            rightDriveA.setPower(-turn);
        } else {
            //            if (turn < 0) {
            //                leftDriveA.setPower(power/2+turn/2);
            //                leftDriveB.setPower(power/2+turn/2);
            //                rightDriveA.setPower(power);
            //                rightDriveB.setPower(power);
            //            } else {
            //                rightDriveA.setPower(power/2-turn/2);
            //                rightDriveB.setPower(power/2-turn/2);
            //                leftDriveA.setPower(power);
            //                leftDriveB.setPower(power);
            //            }
            leftDriveA.setPower(power / 2 + turn);
            leftDriveB.setPower(power / 2 + turn);
            rightDriveA.setPower(power / 2 - turn);
            rightDriveB.setPower(power / 2 - turn);
        }
    }

    public void pump(double speed) {
        suckerRight.setPower(speed);
        suckerLeft.setPower(speed);
    }

    public void arm(double speed) {
        armLeft.setPower(speed);
        armRight.setPower(speed);
    }

    public boolean isRobotBusy() {
        return (actions.size() != 0);
    }

    private boolean isJulieDown() {
        return (julie.getPosition() > Stats.julZero + 0.05);
    }

    public double getTurnGyro() {
        return gyro.getAngularOrientation().firstAngle;
    }

    public double trim(double orginal, double orginalRangeMin, double orginalRangeMax, double newRangeMin, double newRangeMax) {
        return ((orginal / orginalRangeMax - orginalRangeMin) * (newRangeMax - newRangeMin)) + newRangeMin;
    }

    @Deprecated
    public Action autonomousCircleByEncoder(final int direction, final double degree, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDriveA.setTargetPosition(leftDriveA.getCurrentPosition() + direction * ((int) (Stats.tickPerDegreeAtPlace * degree)));
                leftDriveA.setPower(power);
                leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDriveB.setTargetPosition(leftDriveB.getCurrentPosition() + direction * ((int) (Stats.tickPerDegreeAtPlace * degree)));
                leftDriveB.setPower(power);
                rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveA.setTargetPosition(rightDriveA.getCurrentPosition() - direction * ((int) (Stats.tickPerDegreeAtPlace * degree)));
                rightDriveA.setPower(power);
                rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveB.setTargetPosition(rightDriveB.getCurrentPosition() - direction * ((int) (Stats.tickPerDegreeAtPlace * degree)));
                rightDriveB.setPower(power);
            }

            @Override
            public boolean onLoop() {
                if (!rightDriveA.isBusy() && !rightDriveB.isBusy() && !leftDriveA.isBusy() && !leftDriveB.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
    }

    public Action autonomousCircle(final int direction, final double degree, final double marginalError) {
        return new Action(new Action.Execute() {
            double gyroBegin, gyroToGo;
            Formula noMiss = new Formula() {

                @Override
                public double calculate(double progress) {
                    //                    double marginalError=0.02;
                    //                    progress*=direction;
                    if ((progress >= 1 - marginalError && progress <= 1 + marginalError)) {
                        return 0;
                    } else {
                        double power = 1 - progress;
                        if(progress>=0.5){
                            return (0.5-power/2)*(CIRCLING_HIGH-CIRCLING_LOW)+CIRCLING_LOW;
                        }else{
                            return (power)*(CIRCLING_HIGH-CIRCLING_LOW)+CIRCLING_LOW;

                        }
                    }
                }
            };

            @Override
            public void onSetup() {
                gyroBegin = getTurnGyro();
                gyroToGo = gyroBegin + (direction * degree);
//                if (gyroToGo > TERMINAL) {
//                    gyroToGo = gyroToGo - 360;
//                } else if (gyroToGo < -TERMINAL) {
//                    gyroToGo = gyroToGo + 360;
//                }
            }

            @Override
            public boolean onLoop() {
                double p = getTurnGyro();
                double stageA = p - gyroBegin;
                double stageB = gyroToGo - gyroBegin;
                double progress = stageA / stageB;
                //                progress=Math.abs(progress);
//                telemetry.addData("Progress:", progress);
//                progress=Range.clip(progress,0,1);
                //                telemetry.addData("ToGo:", gyroToGo);
                //                telemetry.addData("Start:", gyroBegin);
                //                telemetry.addData("Current:", p);
                double of = noMiss.calculate(progress);
                turn(0, -direction*of);
                return (of == 0);
            }
        });
    }
    //    public Action autonomousArmMove(final double centimeters, final double power) {
    //        return new Action(new Action.Execute() {
    //            @Override
    //            public void onSetup() {
    //                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //                arm.setTargetPosition((int) (arm.getCurrentPosition() + (Stats.tickPerArmCentimeter * centimeters)));
    //                arm.setPower(power);
    //            }
    //
    //            @Override
    //            public boolean onLoop() {
    //                if (!arm.isBusy()) {
    //                    resetArm();
    //                    return true;
    //                }
    //                return false;
    //            }
    //        });
    //    }

    public Action autonomousJulieUp() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julie.setPosition(Stats.julZero);
            }

            @Override
            public boolean onLoop() {
                return !isJulieDown();
            }
        });
    }

    public Action autonomousJulieDown() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julie.setPosition(Stats.fullDown);
            }

            @Override
            public boolean onLoop() {
                return isJulieDown();
            }
        });
    }

    public Action autonomousJulieScanPosition() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julie.setPosition(Stats.readDown);
            }

            @Override
            public boolean onLoop() {
                return isJulieDown();
            }
        });
    }

    public Action autonomousJulieColorScan(final int team) {
        return new Action(new Action.Execute() {
            ArrayList<Action> miniaction = new ArrayList<>();

            @Override
            public void onSetup() {
                miniaction.add(autonomousWait(500));
                miniaction.add(autonomousCircle(RIGHT, 8, 0.01));
                miniaction.add(autonomousJulieDown());
                julieSensor.enableLed(false);
                boolean isRed = (julieSensor.red() > julieSensor.blue());
                //                permanent_messages.add("Color: "+color.red()+" "+color.green()+" "+color.blue());
                if (team == RED_TEAM) {
                    if (!isRed) {
                        miniaction.add(autonomousCircle(LEFT, 18, 0.3));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(RIGHT, 10, 0.3));
                    } else {
                        miniaction.add(autonomousCircle(RIGHT, 18, 0.3));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(LEFT, 26, 0.3));
                    }
                } else {
                    if (isRed) {
                        miniaction.add(autonomousCircle(LEFT, 18, 0.3));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(RIGHT, 10, 0.3));
                    } else {
                        miniaction.add(autonomousCircle(RIGHT, 18, 0.3));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(LEFT, 26, 0.3));
                    }
                }
            }

            @Override
            public boolean onLoop() {
                if (miniaction.size() != 0) {
                    if (miniaction.get(0).isAlive()) {
                        if (!miniaction.get(0).isSetup()) {
                            miniaction.get(0).setup();
                        } else {
                            miniaction.get(0).loop();
                        }
                    } else {
                        miniaction.remove(0);
                    }
                }
                return (miniaction.size() == 0);
            }
        });
    }

    public Action autonomousVuforia(final Scenario s) {
        return new Action(new Action.Execute() {
            ArrayList<Action> miniaction = new ArrayList<>();
            private boolean foundMark = false;
            private RelicRecoveryVuMark mark = null;

            private void searchVuforia() {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    mark = vuMark;
                    foundMark = true;
                    addActions();
                }
            }

            private void addActions() {
                if (mark == RelicRecoveryVuMark.CENTER) {
                    miniaction.addAll(Arrays.asList(s.getC()));
                } else if (mark == RelicRecoveryVuMark.LEFT) {
                    miniaction.addAll(Arrays.asList(s.getL()));
                } else if (mark == RelicRecoveryVuMark.RIGHT) {
                    miniaction.addAll(Arrays.asList(s.getR()));
                }
            }

            @Override
            public void onSetup() {
                if (!vuforiaStatus)
                    initVuforia();
            }

            @Override
            public boolean onLoop() {
                if (foundMark) {
                    if (miniaction.size() != 0) {
                        if (miniaction.get(0).isAlive()) {
                            if (!miniaction.get(0).isSetup()) {
                                miniaction.get(0).setup();
                            } else {
                                miniaction.get(0).loop();
                            }
                        } else {
                            miniaction.remove(0);
                        }
                    }
                    return (miniaction.size() == 0);
                } else {
                    searchVuforia();
                    return false;
                }
            }
        });
    }

    public Action autonomousVuforiaInit() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                if (!vuforiaStatus)
                    initVuforia();
            }

            @Override
            public boolean onLoop() {
                return true;
            }
        });
    }

    public Action autonomousWait(final int millis) {
        return new Action(new Action.Execute() {
            int targetTime = 0;

            @Override
            public void onSetup() {
                targetTime = (int) runtime.milliseconds() + millis;
            }

            @Override
            public boolean onLoop() {
                return (runtime.milliseconds() >= targetTime);
            }
        });
    }

    public Action autonomousSuck(final int millis, final double speed) {
        return new Action(new Action.Execute() {
            int targetTime = 0;

            @Override
            public void onSetup() {
                targetTime = (int) runtime.milliseconds() + millis;
                pump(-speed);
            }

            @Override
            public boolean onLoop() {
                if (runtime.milliseconds() >= targetTime) {
                    pump(0);
                }
                return (runtime.milliseconds() >= targetTime);
            }
        });
    }

    public Action autonomousSpit(final int millis, final double speed) {
        return new Action(new Action.Execute() {
            int targetTime = 0;

            @Override
            public void onSetup() {
                targetTime = (int) runtime.milliseconds() + millis;
                pump(speed);
            }

            @Override
            public boolean onLoop() {
                if (runtime.milliseconds() >= targetTime) {
                    pump(0);
                }
                return (runtime.milliseconds() >= targetTime);
            }
        });
    }

    public Action autonomousDrive(final double centimeters, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDriveA.setTargetPosition(rightDriveA.getCurrentPosition() + (int) (Stats.tickPerCentimeter * centimeters));
                leftDriveA.setTargetPosition(leftDriveA.getCurrentPosition() + (int) (Stats.tickPerCentimeter * centimeters));
                rightDriveB.setTargetPosition(rightDriveB.getCurrentPosition() + (int) (Stats.tickPerCentimeter * centimeters));
                leftDriveB.setTargetPosition(leftDriveB.getCurrentPosition() + (int) (Stats.tickPerCentimeter * centimeters));
                leftDriveA.setPower(power);
                rightDriveA.setPower(power);
                leftDriveB.setPower(power);
                rightDriveB.setPower(power);
            }

            @Override
            public boolean onLoop() {
                if (!rightDriveA.isBusy() && !rightDriveB.isBusy() && !leftDriveA.isBusy() && !leftDriveB.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
    }

    public Action autonomousDriveToGyro(final double power, final int direction, final double marginalError, final double... angles) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDriveA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDriveA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDriveB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            @Override
            public boolean onLoop() {
                double angletogo = gyro.getAngularOrientation().secondAngle;
                for (int x = 0; x < angles.length; x++) {
                    if (angletogo == angles[x] || (angletogo >= angles[x] - marginalError && angletogo <= angles[x] + marginalError)) {
                        drive(0);
                        return true;
                    } else {
                        drive(direction * power);
                    }
                }
                return false;
                //                if (angletogo < angle) {
                //                    drive(-power);
                //                } else if (angletogo > angle) {
                //                    drive(power);
                //                } else if (angletogo == angle) {
                //                    drive(0);
                //                    return true;
                //                }
                //                return false;
            }
        });
    }

    public Action autonomousDone() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
            }

            @Override
            public boolean onLoop() {
                resetRobot();
                MODE = Stats.MULTI;
                actions.clear();
                return true;
            }
        });
    }

    static class Scenario {
        private Action[] L, C, R;

        public Scenario(Action[] L, Action[] C, Action[] R) {
            this.L = L;
            this.C = C;
            this.R = R;
        }

        public Action[] getL() {
            return L;
        }

        public Action[] getC() {
            return C;
        }

        public Action[] getR() {
            return R;
        }
    }

    static class Field {
        static final double field = 365.76;
        static final double balacing = 60;
    }

    interface Formula {
        double calculate(double progress);
    }
}

class Stats {
    static final int MULTI = 1;
    static final double CIRCLING_LOW = 0.13;
    static final double CIRCLING_HIGH = 0.18;
    static final double TERMINAL = 179.99;
    static final int LINEBYLINE = 2;
    static final int RIGHT = -1;
    static final int LEFT = 1;
    static final int FORWARD = 1;
    static final int BACKWARDS = -1;
    static final int BLUE_TEAM = 1;
    static final int RED_TEAM = 2;
    static final int WHEEL_RADIUS = 5;
    static final double marginalError = 0.015;
    static final double offthebalancing = 30 + WHEEL_RADIUS;
    static final double intoChypher = 45;
    static final double redEastLocation3 = 54;
    static final double redEastLocation2 = 31 + redEastLocation3;
    static final double redEastLocation1 = redEastLocation2 + 31;

    //    static final double westCenterForwardCm = ofthebalance + 19;
    static final double tipper = 15;
    static final double backToBalance = -30;
    static final int eastLeftCm = 54;
    static final int eastCenterCm = 36;
    static final int eastRightCm = 19;
    static final int motorTickPerRevolution = (1440 / 86) * 100;//1674.41
    static final int armMotorTickPerRevolution = (1440);
    static final double PI = 3.14;
    static final double wheelDiameter = 2 * 5.1 * PI;
    static final double distanceBetween = 39;
    static final double placeSpinDiameter = 2 * distanceBetween * PI;
    static final double placeOutSpinDiameter = distanceBetween * PI;//122.46
    static final double tickPerCentimeter = (motorTickPerRevolution) / (wheelDiameter);//52.27
    static final double armToGear = 3 / 6.5;
    static final double armLength = 28.8;
    static final double armCmPerMotorRevolution = (2 * armLength * PI);
    static final double tickPerArmCentimeter = (armMotorTickPerRevolution / armToGear) / armCmPerMotorRevolution;
    static final double tickPerDegree = (((placeSpinDiameter * tickPerCentimeter) / 360) / 231) * 260;
    static final double tickPerDegreeAtPlace = (((placeOutSpinDiameter * tickPerCentimeter) / 360) / 231) * 245;//18.69023569
    static final double fullDown = 0.54;
    static final double readDown = 0.48;
    static final double julZero = 0.02;
    static final double clawOpen = 0.15;
    static final double clawClose = 0.0;
    static final int armUp = 60;

    static class Vuforia {
        static final String key = "AcJU0s7/////AAAAmYAF+R25rU5elxvWG+jzOfkipjv/EqlAWEJ12K0WFESUlxRj3trJYggUrl1cGvVLLpEbh56nvKa7zL9mYbG3P6lcCeUUNtXMKwg7QzPfJBG8HRas8zkQPFahOEgvii83GQCKLihiRGi2UFmlK3RkzVi6NU2d/9v8q1lrFfAT2lJQsqyThtcaYKHbDt9DFQzSTwcQLz7Pblr1cM57P7H3T/XovWdMOZBKeXzT8G00c5J4rRtWmq+Pvk83YFxfAiA22sFPepjkt2eke/QeMiCFE6hwRoq/WlGFsKDhMnGDAy16UlexgrjEQn85vclQxkJh6/Rd7IJ6FF0aCp8ewg29ZV14bWxdr1GEyWwh1BJ50YFQ";
    }
}

class Action {
    @Deprecated
    static class TimeableAction {
        Execute e;
        int time;

        TimeableAction(int time, @Nullable Execute execute) {
            this.time = time;
            this.e = execute;
        }

        Execute getExecutable() {
            return e;
        }

        int getTime() {
            return time;
        }

        interface Execute {
            void onExecute();
        }
    }

    Execute e;

    Action(@NonNull Execute execute) {
        e = execute;
    }

    private boolean alive = true;
    private boolean setup = false;

    boolean isAlive() {
        return alive;
    }

    boolean isSetup() {
        return setup;
    }

    void setup() {
        e.onSetup();
        setup = true;
    }

    void loop() {
        alive = !e.onLoop();
    }

    interface Execute {
        void onSetup();

        boolean onLoop();
    }
}