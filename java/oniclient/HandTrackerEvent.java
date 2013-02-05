package oniclient;

import java.util.EventObject;

public class HandTrackerEvent extends EventObject{

  HandTracker _tracker;
  
  public HandTrackerEvent(Object source, HandTracker tracker) {
    super(source);
    _tracker = tracker;
  }
  
  public HandTracker getTracker() {
    return _tracker;
  }
  
}