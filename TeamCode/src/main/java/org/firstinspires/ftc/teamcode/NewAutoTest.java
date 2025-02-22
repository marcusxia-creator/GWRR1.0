package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Roadrunner 1.0 Auto Example Test", group = "Autonomous")
public class NewAutoTest extends LinearOpMode {
    public static class Lift {
        private DcMotorEx liftLeft;
        private DcMotorEx liftRight;

        public Lift(HardwareMap hardwareMap) {
            liftLeft = hardwareMap.get(DcMotorEx.class, "VS_Left_Motor");
            liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
            liftRight = hardwareMap.get(DcMotorEx.class, "VS_Right_Motor");
            liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftRight.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftLeft.setPower(0.8);
                    initialized = true;
                }

                double pos = liftLeft.getCurrentPosition();
                packet.put("liftLeftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    liftLeft.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() { return new LiftUp();}

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftLeft.setPower(-0.8);
                    initialized = true;
                }

                double pos = liftLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    liftLeft.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action CloseClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    @Override
    public void runOpMode() {
        // Define the starting position
        Pose2d startPose = new Pose2d(0, 0, 0);

        // Initialize MecanumDrive with start pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // Build the action-based trajectory
        TrajectoryActionBuilder tab2 = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(20,20),Math.toRadians(180));// Turn 90 degrees



        waitForStart();
        if (isStopRequested()) return;

        // Run the action sequence
        Actions.runBlocking( new SequentialAction(
                tab2.build(),
                new ParallelAction(
                lift.liftUp(),
                claw.openClaw(),
                lift.liftDown()))
        );
    }
}
