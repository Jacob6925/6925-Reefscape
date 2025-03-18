package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BallIntakeSubsys;
import frc.robot.subsystems.BallIntakeSubsys.BallIntakeSpeed;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ElevatorSubsys.ElevatorPosition;
import frc.robot.subsystems.PipeIntakeSubsys;
import frc.robot.subsystems.PipeIntakeSubsys.PipeIntakeSpeed;
import frc.robot.subsystems.WristSubsys;
import frc.robot.subsystems.WristSubsys.WristSetpoint;

public final class RobotCommands {
    private static ElevatorSubsys elevatorSubsys;
    private static PipeIntakeSubsys pipeIntakeSubsys;
    private static BallIntakeSubsys ballIntakeSubsys;
    private static WristSubsys wristSubsys;
    
    public static void init(ElevatorSubsys elevatorSubsys, PipeIntakeSubsys pipeIntakeSubsys, BallIntakeSubsys ballIntakeSubsys, WristSubsys wristSubsys) {
        RobotCommands.elevatorSubsys = elevatorSubsys;
        RobotCommands.pipeIntakeSubsys = pipeIntakeSubsys;
        RobotCommands.ballIntakeSubsys = ballIntakeSubsys;
        RobotCommands.wristSubsys = wristSubsys;
    }

    public static Command setSpeedPipeAndBall(PipeIntakeSpeed speed) {
        return new SequentialCommandGroup(
            pipeIntakeSubsys.setSpeedCommand(speed),
            ballIntakeSubsys.setSpeedFromPipeSpeedCommand(speed)
        );
    }

    public static Command resetElevator() {
        return new SequentialCommandGroup(
            wristSubsys.goTo(WristSetpoint.START_POS),
            new WaitCommand(0.25),
            elevatorSubsys.goTo(ElevatorPosition.MIN_HEIGHT)
        );
    }

    public static Command elevatorLevelActions(ElevatorPosition elevatorPosition, WristSetpoint wristSetpoint) {
        return new SequentialCommandGroup(
            elevatorSubsys.goTo(elevatorPosition),
            new WaitCommand(0.5),
            wristSubsys.goTo(wristSetpoint)
        );
    }

    public static Command moveElevAndIntakeBall(ElevatorPosition elevatorPosition) {
        return new SequentialCommandGroup(
            elevatorLevelActions(elevatorPosition, WristSetpoint.REMOVE_ALGAE).until(() -> Math.abs(elevatorSubsys.getMotorRotations() - elevatorPosition.rotations) > 1),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.INTAKE)
        );
    }

    public static Command resetIntakesAndElevator() {
        return new SequentialCommandGroup(
            pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.OFF),
            ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF),
            resetElevator()
        );
    }

    public static Command humanPlayerActionsCommand() {
        return new SequentialCommandGroup(
            elevatorSubsys.goTo(ElevatorPosition.HUMAN_PLAYER_INTAKE),
            new WaitCommand(0.5),
            wristSubsys.goTo(WristSetpoint.HUMAN_PLAYER_INTAKE)
        ).andThen(
            pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.INTAKE),
            ballIntakeSubsys.setSpeedFromPipeSpeedCommand(PipeIntakeSpeed.INTAKE)
        );
    }

    public static Command ejectIntakes(boolean pipe, boolean ball) {
        Command ejectCommand;
        if (pipe && ball) {
            ejectCommand = setSpeedPipeAndBall(PipeIntakeSpeed.EJECT);
        } else if (pipe) {
            ejectCommand = pipeIntakeSubsys.setSpeedCommand(PipeIntakeSpeed.OFF);
        } else {
            ejectCommand = ballIntakeSubsys.setSpeedCommand(BallIntakeSpeed.OFF);
        }
        return new SequentialCommandGroup(
            ejectCommand,
            new WaitCommand(3.0),
            setSpeedPipeAndBall(PipeIntakeSpeed.OFF)
        );
    }
}