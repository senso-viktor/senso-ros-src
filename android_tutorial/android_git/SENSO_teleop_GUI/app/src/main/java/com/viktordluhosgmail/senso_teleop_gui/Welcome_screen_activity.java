package com.viktordluhosgmail.senso_teleop_gui;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

public class Welcome_screen_activity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_welcome_screen_activity);
        Thread my_thread = new Thread(){
            @Override void run(){
                try {
                    sleep(3000);
                    Intent my_intent = new Intent(getApplicationCOntext(),Menu_activity.class)
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        };
        my_thread.start();
    }
}
