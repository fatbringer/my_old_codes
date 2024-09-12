package com.example.sensorout;
// a small sample app to demo sending of real time data
//

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;

// your own imports
import android.os.StrictMode;
import android.provider.ContactsContract;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import java.io.DataOutputStream;
import java.io.OutputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.util.List;
import java.net.UnknownHostException;
import android.widget.Toast;
import java.util.List;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;


public class MainActivity extends AppCompatActivity {
    SensorManager sm = null;
    TextView textView1 = null;
    TextView textView2 = null;
    TextView textView3 = null;

    // client settings
    private Socket clientSocket;

    List list;
    String AccelCombined = "Initialised AccelCombined";



    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toast.makeText(getApplicationContext(),"App starting",Toast.LENGTH_LONG).show();

        /* Get a SensorManager instance */
        sm = (SensorManager)getSystemService(SENSOR_SERVICE);
        textView1 = (TextView)findViewById(R.id.textView1); // sensor data
        textView2 = (TextView)findViewById(R.id.textView2); // errors
        //textView3.setText(ServerIPport);

        list = sm.getSensorList(Sensor.TYPE_ACCELEROMETER);
        if(list.size()>0){
            sm.registerListener(sel, (Sensor) list.get(0), SensorManager.SENSOR_DELAY_NORMAL);
        }else{
            Toast.makeText(getBaseContext(), "Error: No Accelerometer.", Toast.LENGTH_LONG).show();
        }
        new Thread (SocketThread).start();


        Button sendData = (Button) findViewById(R.id.buttonSend);
        sendData.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // Do something in response to button click

                DataOutputStream buttony = null;
                try {
                    buttony = new DataOutputStream(clientSocket.getOutputStream());
                    buttony.writeBytes("button pressed");
                    // when you press the button named "buttonSend", the string gets sent out
                } catch (IOException e) {
                    e.printStackTrace();
                    String error = e.toString() + "\n";
                    textView3.append(error);
                }

            }
        });
    }

    @Override
    protected void onStop() {
        if(list.size()>0){
            sm.unregisterListener(sel);
        }
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        // TODO Auto-generated method stub
        super.onDestroy();
        sm.unregisterListener(sel);
    }

    @Override
    protected void onStart() {
        // TODO Auto-generated method stub
        super.onStart();
    }

        //added code below
    SensorEventListener sel = new SensorEventListener(){
        public void onAccuracyChanged(Sensor sensor, int accuracy) {}
        public void onSensorChanged(SensorEvent event) {
            float[] values = event.values;
            String xAccel = String.valueOf(values[0]);
            String yAccel = String.valueOf(values[1]);
            String zAccel = String.valueOf(values[2]);

            // how you format this data is really up to you
            AccelCombined = "x: "+ xAccel +"  y: "+ yAccel +"  z: "+ zAccel;
            textView1.setText(AccelCombined);

            SocketThread.run();
        }
    };



        // creates a separate thread for running the network stuff
        // this is because android forbids running network connection tasks on the main activity thread
        // running network connection tasks on main activity leads to some os exceptions
        Runnable SocketThread = new Runnable() {
        @Override
        public void run() {
            try {
                textView2.append("SocketThread is running\n");
                String SERVER_ADDRESS = "your ip address8";
                int SERVER_PORT = 9000;

                if (clientSocket == null) {
                    clientSocket = new Socket(InetAddress.getByName(SERVER_ADDRESS), SERVER_PORT);
                    SocketAddress socketAddress = new InetSocketAddress(SERVER_ADDRESS, SERVER_PORT);
                    textView2.append("new socket made\n");

                    if (!clientSocket.isConnected() ) {
                        clientSocket.connect(socketAddress);
                        textView2.append("Socket connected\n");
                    }
                    if (!clientSocket.isBound()) {
                        clientSocket.bind(socketAddress);
                        textView2.append("Socket binded\n");
                    }
                    if (clientSocket.isClosed()) {
                        textView2.append("Socket is closed :(\n");
                    }
                }

                DataOutputStream DOS = new DataOutputStream(clientSocket.getOutputStream());
                //debug textView2.append("DOS variable assigned\n");
                DOS.writeBytes(AccelCombined);
                //debug textView2.append("AccelCombined sent out\n");

            } catch (Exception e) {
                e.printStackTrace();
                String error = e.toString() + "\n";
                textView2.append(error);
            }
        }
    }; // end of SocketThread


} // end of whole thing
