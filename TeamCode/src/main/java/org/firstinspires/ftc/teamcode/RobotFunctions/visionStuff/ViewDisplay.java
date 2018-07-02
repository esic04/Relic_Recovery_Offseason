package org.firstinspires.ftc.teamcode.RobotFunctions.visionStuff;

import android.content.Context;
import android.view.View;

public interface ViewDisplay { //adds methods to add image to screen
    void setCurrentView(Context context, View view);
    void removeCurrentView(Context context);
}
