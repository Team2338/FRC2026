// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import team.gif.robot.commands.agitator.AgitatorPercent;
import team.gif.robot.commands.autos.AutonCenterShoot;
import team.gif.robot.commands.autos.AutonEject;
import team.gif.robot.commands.autos.AutonStealShoot;
import team.gif.robot.commands.collector.CollectorPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPivotDown;
import team.gif.robot.commands.autos.AutonShoot;
import team.gif.robot.commands.collector.collectorautos.CollectorAutonPivotDown;
import team.gif.robot.commands.drivetrain.DetectBeach;
import team.gif.robot.commands.drivetrain.StopModules;
import team.gif.robot.commands.shooter.ShooterRPM;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.stream.Stream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private  SendableChooser<Command> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("Collector Run", new CollectorPercent(1.0).alongWith(new AgitatorPercent()));
        NamedCommands.registerCommand("Collector Down", new CollectorAutoPivotDown());
        NamedCommands.registerCommand("OC-autonshoot", new AutonShoot(true).withTimeout(4.5).andThen(new ShooterRPM(4000).withTimeout(0.25)));
        NamedCommands.registerCommand("OC-autonshoot-final", new AutonShoot(true));
        NamedCommands.registerCommand("AutonCollectorDown", new CollectorAutonPivotDown());
        NamedCommands.registerCommand("Center Shoot Sequence", new AutonCenterShoot());
        NamedCommands.registerCommand("Steal Shoot Sequence", new AutonStealShoot());
        NamedCommands.registerCommand("DetectBeach", new DetectBeach());
        NamedCommands.registerCommand("Auton Eject", new AutonEject());
        NamedCommands.registerCommand("StopSwerve", new StopModules());

//        NamedCommands.registerCommand("CC-autonshoot", new AutonInitialShoot()); // no longer used since bot no longer shoots first. Would need additional testing

        // Configure the trigger bindings
        configureBindings();

        autoChooser = build2338AutoChooser((stream) -> stream);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static SendableChooser<Command> build2338AutoChooser(Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {

        if(!AutoBuilder.isConfigured()) { throw new RuntimeException("AutoBuilder was not configured before attempting to build an auto chooser."); }

        SendableChooser<Command> autoChooser = new SendableChooser<>();

        ArrayList<String> names = new ArrayList<>();
        names.add("Left-FuelSafe-NeutralMid");
        names.add("Right-FuelSafe-NeutralMid");
        names.add("Left-FuelSafeLong-NeutralMid");
        names.add("Right-FuelSafeLong-NeutralMid");
        names.add("Center");
        names.add("Left-FuelSafeLong-FuelSweep");
        names.add("Right-FuelSafeLong-FuelSweep");
        names.add("Left-FuelSafe-NeutralFar");
        names.add("Right-FuelSafe-NeutralFar");
        names.add("Left-FuelCross-HubNear");
        names.add("Right-FuelCross-HubNear");
        names.add("Right-NeutralMid-HubNear");
        names.add("Left-NeutralMid-HubNear");
        names.add("Left-FuelSafeLong-DoubleTrench");
        names.add("Left-Fuel-Steal");
        names.add("Right-FuelSafeLong-DoubleTrench");
        names.add("Right-Fuel-Steal");
        names.add("Misc-Left-HubAim-FieldCross");

        ArrayList<PathPlannerAuto> autoChoices = new ArrayList<>();

        for(String n : names) {
            PathPlannerAuto auto = new PathPlannerAuto(n);
            autoChoices.add(auto);
        }

        autoChooser.setDefaultOption("** None **", Commands.none());

        optionsModifier.apply(autoChoices.stream()).forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        return autoChooser;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
