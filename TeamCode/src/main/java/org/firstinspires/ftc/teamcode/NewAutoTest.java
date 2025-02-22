package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "Roadrunner 1.0 Auto Example Test", group = "Autonomous")
public class NewAutoTest extends LinearOpMode {
    public static class Lift {
        private DcMotorEx liftLeft;
        private DcMotorEx liftRight;
        private ElapsedTime timer;
        private GoBildaPinpointDriver pinpoint;

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
                    liftLeft.setPower(0.4);
                    liftRight.setPower(0.4);
                    initialized = true;
                }

                double pos = liftLeft.getCurrentPosition();
                packet.put("liftLeftPos", pos);
                if (pos > 3000.0) {
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
                    liftLeft.setPower(-0.4);
                    liftRight.setPower(-0.4);
                    initialized = true;
                }

                double pos = liftLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 100.0) {
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
            claw = hardwareMap.get(Servo.class, "Deposit_Claw_Servo");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.08);
                return false;
            }
        }
        public Action CloseClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.36);
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

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        pinpoint.resetPosAndIMU();
        double initialAngle = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);

        ElapsedTime timer = new ElapsedTime();

        // Build the action-based trajectory
        TrajectoryActionBuilder traj0= drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(20,20),Math.toRadians(90))
                .afterDisp(10, claw.CloseClaw())
                .afterTime(2, claw.openClaw())
                .stopAndAdd(claw.CloseClaw())
                .setReversed(true);// Turn 90 degrees


        Action trajpath = traj0.build();
        Action trajectoryActionCloseOut = traj0.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(30, 0), Math.toRadians(-90))
                .build();

        Actions.runBlocking(claw.openClaw());
        telemetry.addData("IMU Initialized", "Initial Angle: %.2f degree", initialAngle);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Run the action sequence in parallel (without `Actions.delay()`)
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        trajpath,// Move forward
                        trajectoryActionCloseOut  // Move back
                ),
                new SequentialAction(
                        claw.CloseClaw(),  // Close claw
                        new SleepAction( 10),
                        claw.openClaw(),    // Open claw
                        new SleepAction( 12),    // Custom wait 1.5 seconds
                        claw.CloseClaw()     // Close again
                )
        ));

        // Optional: Display timer telemetry
        // ✅ Display final telemetry and IMU angle
        while (opModeIsActive()) {
            double currentAngle = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
            telemetry.addData("Final Timer", "Elapsed: %.2f sec", getRuntime());
            telemetry.addData("Initial IMU Angle", "%.2f degree", initialAngle);
            telemetry.addData("Current IMU Angle", "%.2f degree", currentAngle);
            telemetry.update();
        }
    }
}
