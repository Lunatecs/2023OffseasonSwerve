// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Comparator;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem ledSubsystem = null;
  private Spark ledFrontControl = new Spark(0);
  private Spark ledBackControl = new Spark(1);

  private PriorityQueue<PriorityColor> frontQueue = null;
  private PriorityQueue<PriorityColor> backQueue = null;
  

  public final PriorityColor DEFAULT = new PriorityColor(LEDConstants.FIRE_MED, 100, "fire");
  public final PriorityColor BALANCED = new PriorityColor(LEDConstants.CONFETTI, 20, "party");
  public final PriorityColor INTAKE_CONE = new PriorityColor(LEDConstants.STROBE_GOLD, 31, "cone"); // 30
  public final PriorityColor INTAKE_CUBE = new PriorityColor(LEDConstants.SOLID_VIOLET, 30, "cube");
  public final PriorityColor PICKED_UP = new PriorityColor(LEDConstants.SOLID_GREEN, 25, "picked up");
  public final PriorityColor DEFAULT_BACK = new PriorityColor(LEDConstants.SOLID_WHITE, 100, "white");

  
  private LEDSubsystem() {
    //ledBack = new LEDBack();
    frontQueue = new PriorityQueue<PriorityColor>(new PriorityColor());
    backQueue = new PriorityQueue<PriorityColor>(new PriorityColor());
    
    frontQueue.add(DEFAULT);
    backQueue.add(DEFAULT_BACK);
  }

  public static LEDSubsystem getInstance() {
    if (ledSubsystem == null) {
      ledSubsystem = new LEDSubsystem();
    }
    return ledSubsystem;
  }

  public boolean frontQueueContains(PriorityColor color) {
    return frontQueue.contains(color);
  }

  public boolean backQueueContains(PriorityColor color) {
    return backQueue.contains(color);
  }

  public void addColorFront(PriorityColor color) {
    if(!frontQueue.contains(color)) {
      frontQueue.add(color);
    }
  }

  public void addColorBack(PriorityColor color) {
    if(!backQueue.contains(color)) {
      backQueue.add(color);
    }
  }

  public void removeColorFront(PriorityColor color) {
    if(frontQueue.contains(color)) {
      frontQueue.remove(color);
    }
  }

  public void removeColorBack(PriorityColor color) {
    if(backQueue.contains(color)) {
      backQueue.remove(color);
    }
  }

  public String printQueueFront() {
    String queueItems = "";
    for (PriorityColor color : frontQueue) {
      queueItems += color.name + ": " + color.priority + ", ";
    }
    
    return queueItems;  
  }

  public String printQueueBack() {
    String queueItems = "";
    for (PriorityColor color : backQueue) {
      queueItems += color.name + ": " + color.priority + ", ";
    }
    
    return queueItems;  
  }

  @Override
  public void periodic() {
    ledFrontControl.set(frontQueue.peek().color);
    ledBackControl.set(backQueue.peek().color);
    
  }

  

  private class PriorityColor implements Comparator<PriorityColor>, Comparable<PriorityColor> {

    public double color;
    public String name;
    public int priority;
    public int loopCount = -10;

    public PriorityColor() {}
    
    public PriorityColor(double color, int priority, String name) {
      this.color = color;
      this.priority = priority;
    }

    public int compare(PriorityColor p1 , PriorityColor p2) {
        if(p1.priority>= p2.priority) {
          return 1;
        } else {
          return -1;
        }
    }

    public int compareTo(PriorityColor p2) {
      return compare(this, p2);
    }
  }
}