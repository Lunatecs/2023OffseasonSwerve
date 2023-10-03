package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.SetPointSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driverJoystick = new Joystick(JoystickConstants.DRIVER_USB);
    private final Joystick operatorJoystick = new Joystick(JoystickConstants.OPERATOR_USB);
    /* Drive Controls */
    private final int translationAxis = JoystickConstants.LEFT_Y_AXIS;
    private final int strafeAxis = JoystickConstants.LEFT_X_AXIS;
    private final int rotationAxis = JoystickConstants.RIGHT_X_AXIS;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();


    /* Subsystems */
    private final SwerveDrive swerve = new SwerveDrive();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final LEDSubsystem led = LEDSubsystem.getInstance();
    private final LimeLightSubsystem limelight = new LimeLightSubsystem();

    private SetPointSupplier elevatorSetpoint = new SetPointSupplier();
    boolean isCone = false;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driverJoystick.getRawAxis(translationAxis), 
                () -> -driverJoystick.getRawAxis(strafeAxis), 
                () -> -driverJoystick.getRawAxis(rotationAxis), 
                () -> true,
                () -> driverJoystick.getRawButton(JoystickConstants.LEFT_BUMPER) // slowMode
            )
        );
        
        intake.setDefaultCommand(new RunCommand(() -> intake.runIntake(-0.2), intake));
        arm.setDefaultCommand(new LockArmCommand(arm, new SetPointSupplier()));

        configureButtonBindings();
        configureAutos();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
            // Driver Button Bindings
        new JoystickButton(driverJoystick, JoystickConstants.BACK_BUTTON).onTrue(new InstantCommand(() -> swerve.zeroGyro(), swerve));
    // Intake
        new Trigger(() -> {return Math.abs(driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER)) > 0.1;}).onTrue(new RunIntakeCommand(() -> -driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER), intake))
                                                                                                            .onFalse(new InstantCommand(() -> {}, intake));
        new Trigger(() -> {return Math.abs(driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER)) > 0.1;}).onTrue(new ParallelCommandGroup(new RunIntakeCommand(() -> driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER), intake), 
                                                                                                                                                new RunCommand(() -> led.removeColorBack(led.PICKED_UP))))
                                                                                                                .onFalse(new InstantCommand(() -> {}, intake));

        new POVButton(driverJoystick, JoystickConstants.POV_DOWN).onTrue(new ParallelCommandGroup(new RunIntakeCommand(() -> 0.3, intake),
                                                                                                    new RunCommand(() -> led.removeColorBack(led.PICKED_UP), led)))
                                                                    .onFalse(new InstantCommand(() -> {}, intake));

        new POVButton(driverJoystick, JoystickConstants.POV_UP).onTrue(new ParallelCommandGroup(new RunIntakeCommand(() -> 0.5, intake),
                                                                                                new RunCommand(() -> led.removeColorBack(led.PICKED_UP), led)))
                                                                .onFalse(new InstantCommand(() -> {}, intake));




        //Setpoint Wrist Control
        new JoystickButton(driverJoystick, JoystickConstants.BLUE_BUTTON).onTrue(new SetWristAngleAndLockCommand(wrist, WristConstants.GROUND_INTAKE_CONE));

        new JoystickButton(driverJoystick, JoystickConstants.GREEN_BUTTON).onTrue(new SetWristAngleAndLockCommand(wrist, WristConstants.GROUND_INTAKE_CUBE));

        new JoystickButton(driverJoystick, JoystickConstants.YELLOW_BUTTON).onTrue(new SetWristAngleAndLockCommand(wrist, WristConstants.WRIST_HOME));



        // Operator Button Bindings
        //Elevator Setpoints
        new JoystickButton(operatorJoystick, JoystickConstants.GREEN_BUTTON).onTrue(new SetOtherLevelsCommand(elevator, arm, wrist, ElevatorConstants.BOTTOM, WristConstants.WRIST_HOME, 0.00004));
        //.onFalse(new InstantCommand(() -> {}, elevator));

        new JoystickButton(operatorJoystick, JoystickConstants.RED_BUTTON).onTrue(new SetOtherLevelsCommand(elevator, arm, wrist, ElevatorConstants.STATION_HEIGHT, WristConstants.GROUND_INTAKE_CONE, 0.00006));

        new JoystickButton(operatorJoystick, JoystickConstants.BLUE_BUTTON).onTrue(new SetOtherLevelsCommand(elevator, arm, wrist, ElevatorConstants.MID_HEIGHT, WristConstants.CONE_SETPOINT, 0.00006));
        //.onFalse(new InstantCommand(() -> {}, elevator));

        new JoystickButton(operatorJoystick, JoystickConstants.YELLOW_BUTTON).onTrue(new SetTopLevelCommand(arm, elevator, wrist)); 


        //Manual Wrist Control
        new Trigger(() -> {return Math.abs(operatorJoystick.getRawAxis(JoystickConstants.LEFT_Y_AXIS)) > 0.2;}).whileTrue(new RepeatCommand(new RunCommand(() -> wrist.turnWrist(.5*operatorJoystick.getRawAxis(JoystickConstants.LEFT_Y_AXIS)), wrist)))
                                                                                                                .onFalse(new WristBrakeCommand(new SetPointSupplier(), wrist));
        new JoystickButton(operatorJoystick, JoystickConstants.START_BUTTON).onTrue(new WristBrakeCommand(new SetPointSupplier(), wrist))
                                                                            .onFalse(new InstantCommand(() -> {}, wrist));

        //LED Controls
        new JoystickButton(operatorJoystick, JoystickConstants.RIGHT_BUMPER).onTrue(new RunCommand(() -> {
            led.removeColorFront(led.INTAKE_CONE);
            led.addColorFront(led.INTAKE_CUBE);
            }, led))
                                                                            .onFalse(new RunCommand(() -> led.removeColorFront(led.INTAKE_CUBE), led));

        new JoystickButton(operatorJoystick, JoystickConstants.LEFT_BUMPER).onTrue(new RunCommand(() -> {
            led.removeColorFront(led.INTAKE_CUBE);
            led.addColorFront(led.INTAKE_CONE);
            }, led))
                                                                            .onFalse(new RunCommand(() -> led.removeColorFront(led.INTAKE_CONE), led));
}

    public void configureAutos() {
        autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
        autoChooser.addOption("Test", new FirstAuto(swerve));
        autoChooser.addOption("Move Forward", new AutoSwerveMoveCommand(swerve, 82.0, 0.00001));
        autoChooser.addOption("Deliver Cone and Move", new DeliverConeAndMoveCommand(swerve, elevator, arm, wrist, intake));
        autoChooser.addOption("Deliver Cone and Move Bump Side", new DeliverConeAndMoveBumpSideCommand(swerve, elevator, arm, wrist, intake));
        autoChooser.addOption("Test AutoBalance", new AutoDeliverConeandBalance(swerve, elevator, arm, wrist, intake));
        
        SmartDashboard.putData(autoChooser);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
