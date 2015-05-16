package org.opencv.samples.facedetect;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.objdetect.CascadeClassifier;

import org.opencv.core.Mat;
import org.opencv.video.KalmanFilter;
import android.app.Activity;
import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.view.KeyEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import android.view.View.OnKeyListener;


import android.os.SystemClock;


public class FdActivity extends Activity implements CvCameraViewListener2 {

    private static final String    TAG                 = "OCVSample::Activity";
    private static final Scalar    FACE_RECT_COLOR     = new Scalar(0, 255, 0, 255);
    public static final int        JAVA_DETECTOR       = 1;
    public static final int        NATIVE_DETECTOR     = 0;
    public static final float ALPHA = 0.15f;
    private static final  int MARGEN = 100;
    private static final  int MIN_RANGE = 120;
    private static final  int MAX_RANGE = 260;

    boolean TURINING;
    private int last_x,last_y;
    int Y_CENTER,X_CENTER;
    private int tracking_enable = 0;
    private int forward_en = 0;
    private String Data_in ="NO DATA";

    private MenuItem               mItemFace50;
    private MenuItem               mItemFace40;
    private MenuItem               mItemFace30;
    private MenuItem               mItemFace20;
    private MenuItem               mItemType;

    private Mat                    mRgba;
    private Mat                    mGray;
    private File                   mCascadeFile;
    private CascadeClassifier      mJavaDetector;
    private DetectionBasedTracker  mNativeDetector;

    private int                    mDetectorType       = JAVA_DETECTOR;
    private String[]               mDetectorName;

    private float                  mRelativeFaceSize   = 0.2f;
    private int                    mAbsoluteFaceSize   = 0;

    private CameraBridgeViewBase   mOpenCvCameraView;
////////////////////// Bluetooth

    Handler bluetoothIn;


    final int handlerState = 0;        				 //used to identify handler message
    private BluetoothAdapter btAdapter = null;
    private BluetoothSocket btSocket = null;
    private StringBuilder recDataString = new StringBuilder();

    private ConnectedThread mConnectedThread;

    // SPP UUID service - this should work for most devices
    private static final UUID BTMODULEUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    // String for MAC address
    private static String address;

    //// sensor values :
    String s0,s1,s2,s3;
    double smoothedValue = 0;
    int smoothing = 5;
    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");

                    // Load native library after(!) OpenCV initialization
                    System.loadLibrary("detection_based_tracker");

                    try {
                        // load cascade file from application resources
                        //InputStream is = getResources().openRawResource(R.raw.lbpcascade_frontalface);
                        InputStream is = getResources().openRawResource(R.raw.haarcascade_mcs_upperbody);
                        File cascadeDir = getDir("cascade", Context.MODE_PRIVATE);
                       // mCascadeFile = new File(cascadeDir, "lbpcascade_frontalface.xml");
                        mCascadeFile = new File(cascadeDir, "haarcascade_mcs_upperbody.xml");
                        FileOutputStream os = new FileOutputStream(mCascadeFile);

                        byte[] buffer = new byte[4096];
                        int bytesRead;
                        while ((bytesRead = is.read(buffer)) != -1) {
                            os.write(buffer, 0, bytesRead);
                        }
                        is.close();
                        os.close();

                        mJavaDetector = new CascadeClassifier(mCascadeFile.getAbsolutePath());
                        if (mJavaDetector.empty()) {
                            Log.e(TAG, "Failed to load cascade classifier");
                            mJavaDetector = null;
                        } else
                            Log.i(TAG, "Loaded cascade classifier from " + mCascadeFile.getAbsolutePath());

                        mNativeDetector = new DetectionBasedTracker(mCascadeFile.getAbsolutePath(), 0);

                        cascadeDir.delete();

                    } catch (IOException e) {
                        e.printStackTrace();
                        Log.e(TAG, "Failed to load cascade. Exception thrown: " + e);
                    }

                    mOpenCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public FdActivity() {
        mDetectorName = new String[2];
        mDetectorName[JAVA_DETECTOR] = "Java";
        mDetectorName[NATIVE_DETECTOR] = "Native (tracking)";

        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.face_detect_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.fd_activity_surface_view);

        mOpenCvCameraView.setMaxFrameSize(640,480); //// smallest size of the front camera which is working fine !
        // other options//Size1: 1280 X 960 ,,,,,,,,,, Size2: 1392 X 1392
        mOpenCvCameraView.setCameraIndex(1);
        mOpenCvCameraView.setCvCameraViewListener(this);

        ///////////////////////////////////bluetooth

        bluetoothIn = new Handler() {
            public void handleMessage(android.os.Message msg) {
                if (msg.what == handlerState) {                                        //if message is what we want
                    String readMessage = (String) msg.obj;                          // msg.arg1 = bytes from connect thread
                    recDataString.append(readMessage);                                    //keep appending to string until ~
                    int endOfLineIndex = recDataString.indexOf("~");                    // determine the end-of-line
                   // Data_in = recDataString.substring(1);
                    if (endOfLineIndex > 0) {                                           // make sure there data before ~
                        String dataInPrint = recDataString.substring(0, endOfLineIndex);    // extract string
                       // txtString.setText("Data Received = " + dataInPrint);
                        int dataLength = dataInPrint.length();                            //get length of data received
                        //txtStringLength.setText("String Length = " + String.valueOf(dataLength));
                        Data_in = recDataString.substring(1,recDataString.indexOf("~"));
                        smoothedValue += (Integer.parseInt(Data_in) - smoothedValue) * 0.15f;/// smoothing;
                        if (recDataString.charAt(0) == '#')                                //if it starts with # we know it is what we are looking for
                        {
                           /*String sensor0 = recDataString.substring(1, 5);          //get sensor value from string between indices 1-5
                           String sensor1 = recDataString.substring(6,10);            //same again...
                           String sensor2 = recDataString.substring(11, 15);
                           String sensor3 = recDataString.substring(16, 20);
*/
                        //    int x = recDataString.indexOf("+", 0);
                        //    int y = recDataString.indexOf("+", x + 1);
                       //  //   int z = recDataString.indexOf("+", y + 1);
                         //   int w = recDataString.indexOf("+", z + 1);
                         //   String s0 = recDataString.substring(1, x);
                         //   String s1 = recDataString.substring(x + 1, y);
                          //  String s2 = recDataString.substring(y + 1, z);
                          //  String s3 = recDataString.substring(z + 1, w);
                           // sensorView0.setText(" Sensor 0 Distance = " /*+ sensor0*/ + s0 + "Cm");    //update the textviews with sensor values
                           // sensorView1.setText(" Sensor 1 Distance = " /*+ sensor1*/ + s1 + "Cm");
                           // sensorView2.setText(" Sensor 2 Distance = " /*+ sensor2*/ + s2 + "Cm");
                           // sensorView3.setText(" Sensor 3 Distance = " /*+ sensor3*/ + s3 + "Cm");
                        }
                        recDataString.delete(0, recDataString.length());                    //clear all string data
                        // strIncom =" ";
                        dataInPrint = " ";
                    }
                }
            }
        };

        Intent intent = getIntent();
        if(intent.getStringExtra(DeviceListActivity.EXTRA_DEVICE_ADDRESS).equals("")) return;
        btAdapter = BluetoothAdapter.getDefaultAdapter();       // get Bluetooth adapter
        checkBTState();

    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);


        //////////////////////// bluetooth
        Intent intent = getIntent();

        //Get the MAC address from the DeviceListActivty via EXTRA
        address = intent.getStringExtra(DeviceListActivity.EXTRA_DEVICE_ADDRESS);

        if(address.equals("")) return;


        //create device and set the MAC address
        BluetoothDevice device = btAdapter.getRemoteDevice(address);

        try {
            btSocket = createBluetoothSocket(device);
        } catch (IOException e) {
            Toast.makeText(getBaseContext(), "Socket creation failed", Toast.LENGTH_LONG).show();
        }
        // Establish the Bluetooth socket connection.
        try
        {
            btSocket.connect();
        } catch (IOException e) {
            try
            {
                btSocket.close();
            } catch (IOException e2)
            {
                //insert code to deal with this
            }
        }
        mConnectedThread = new ConnectedThread(btSocket);
        mConnectedThread.start();

        //I send a character when resuming.beginning transmission to check device is connected
        //If it is not an exception will be thrown in the write method and finish() will be called
        mConnectedThread.write("x");
    }

    public void onDestroy() {
        super.onDestroy();
        mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mGray = new Mat();
        mRgba = new Mat();
    }

    public void onCameraViewStopped() {
        mGray.release();
        mRgba.release();
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

       last_x = 0;

        mRgba = inputFrame.rgba();
        mGray = inputFrame.gray();


        MatOfRect faces = new MatOfRect();

        if (mDetectorType == JAVA_DETECTOR) {
            if (mJavaDetector != null)
                mJavaDetector.detectMultiScale(mGray, faces, 1.1, 2, 6, // TODO: objdetect.CV_HAAR_SCALE_IMAGE
                   //     new Size(mAbsoluteFaceSize, mAbsoluteFaceSize), new Size());
                        new Size(150, 150), new Size());
        }
        else if (mDetectorType == NATIVE_DETECTOR) {
            if (mNativeDetector != null)
                mNativeDetector.detect(mGray, faces);
        }
        else {
            Log.e(TAG, "Detection method is not selected!");
        }



        Rect[] facesArray = faces.toArray();
        for (int i = 0; i < facesArray.length; i++) {
            last_x = facesArray[i].x ;

            Core.rectangle(mRgba, facesArray[i].tl(), facesArray[i].br(), FACE_RECT_COLOR, 3);
            Core.putText(mRgba, "X: " + facesArray[i].x + " Y: " + facesArray[i].y  , new Point(60, 450), 4, 1, new Scalar(255, 0, 0, 0), 2);
        }

        Core.putText(mRgba, "X: " + last_x , new Point(60, 450), 4, 1, new Scalar(255, 0, 0, 0), 2);
        Core.putText(mRgba,"Distance: " + Data_in +"  ,Filtered: "+smoothedValue,new Point(60, 380), 4, 1, new Scalar(255, 0, 0, 0), 2);

        boolean forward_flag = false;
        boolean left_flag = false;
        boolean right_flag = false;
        boolean slight_left_flag = false;
        boolean slight_right_flag = false;
        if(tracking_enable == 1 && last_x>0) {
            if (smoothedValue > 140 && last_x >110 && last_x <300){

                if(!forward_flag) {
                    mConnectedThread.write("f");    // Send "f" via Bluetooth
                    forward_flag = true;
                }
            }

            else {
                mConnectedThread.write("q");    // Send "q" via Bluetooth
                forward_flag = false;
                left_flag = false;
                right_flag = false;
            }
        }else {//if(tracking_enable == 1 ) && smoothedValue < 140) { //Avoid obstecales
            mConnectedThread.write("q");    // Send "q" via Bluetooth
            forward_flag = false;
            left_flag = false;
            right_flag = false;
        }


        //SystemClock.sleep(200);

        //////////// do the the turning somehow !!
        if(tracking_enable == 1  && last_x> 0 && last_x < 110 ) { /// old value = 180

            if(!left_flag) {
                goLeft();
                left_flag= true;
            }
          //  SystemClock.sleep(100);

        } else if(tracking_enable == 1 && last_x > 300 &&  last_x> 0 ) {  // old value = 480

            if(!right_flag) {
                goRight();
                right_flag= true;
            }
           //SystemClock.sleep(100);

        } else if(tracking_enable == 1 && smoothedValue < 140 && last_x >110 && last_x <300){
            mConnectedThread.write("q");
            left_flag = false;
            right_flag = false;
        } //else {
         //   mConnectedThread.write("q");
         //   left_flag = false;
        // //   right_flag = false;
       // }

        return mRgba;
    }



    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        Log.i(TAG, "called onCreateOptionsMenu");
        mItemFace50 = menu.add("Face size 50%");
        mItemFace40 = menu.add("Enable tracking");
        mItemFace30 = menu.add("Forward");
        mItemFace20 = menu.add("Item 4");
        mItemType   = menu.add(mDetectorName[mDetectorType]);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        Log.i(TAG, "called onOptionsItemSelected; selected item: " + item);
        if (item == mItemFace50)
            setMinFaceSize(0.5f);
        else if (item == mItemFace40) {
            if(tracking_enable==0) {
                SystemClock.sleep(4000);
                tracking_enable = 1;
            }
            else tracking_enable = 0;
        } else if (item == mItemFace30) {
            //mConnectedThread.write("e");    // Enable
            if(forward_en==0) {
                //SystemClock.sleep(4000);
                forward_en = 1;
            }
            else forward_en = 0;

        }

        else if (item == mItemFace20)
            setMinFaceSize(0.2f);
        else if (item == mItemType) {
            int tmpDetectorType = (mDetectorType + 1) % mDetectorName.length;
            item.setTitle(mDetectorName[tmpDetectorType]);
            setDetectorType(tmpDetectorType);
        }
        return true;
    }

    private void setMinFaceSize(float faceSize) {
        mRelativeFaceSize = faceSize;
        mAbsoluteFaceSize = 0;
    }

    private void setDetectorType(int type) {
        if (mDetectorType != type) {
            mDetectorType = type;

            if (type == NATIVE_DETECTOR) {
                Log.i(TAG, "Detection Based Tracker enabled");
                mNativeDetector.start();
            } else {
                Log.i(TAG, "Cascade detector enabled");
                mNativeDetector.stop();
            }
        }
    }

    /// bluetooth

    private class ConnectedThread extends Thread {
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        //creation of the connect thread
        public ConnectedThread(BluetoothSocket socket) {
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            try {
                //Create I/O streams for connection
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
            }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }


        public void run() {
            byte[] buffer = new byte[256];
            int bytes;

            // Keep looping to listen for received messages
            while (true) {
                try {
                    bytes = mmInStream.read(buffer);        	//read bytes from input buffer
                    String readMessage = new String(buffer, 0, bytes);
                    // Send the obtained bytes to the UI Activity via handler
                    bluetoothIn.obtainMessage(handlerState, bytes, -1, readMessage).sendToTarget();
                } catch (IOException e) {
                    break;
                }
            }
        }
        //write method
        public void write(String input) {
            byte[] msgBuffer = input.getBytes();           //converts entered String into bytes
            try {
                mmOutStream.write(msgBuffer);                //write bytes over BT connection via outstream
            } catch (IOException e) {
                //if you cannot write, close the application
                Toast.makeText(getBaseContext(), "Connection Failure", Toast.LENGTH_LONG).show();
                finish();

            }
        }
        // Method to send data


        }
    private void checkBTState() {

        if(btAdapter==null) {
            Toast.makeText(getBaseContext(), "Device does not support bluetooth", Toast.LENGTH_LONG).show();
        } else {
            if (btAdapter.isEnabled()) {
            } else {
                Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableBtIntent, 1);
            }
        }
    }

    private BluetoothSocket createBluetoothSocket(BluetoothDevice device) throws IOException {

        return  device.createRfcommSocketToServiceRecord(BTMODULEUUID);
        //creates secure outgoing connecetion with BT device using UUID
    }


    private void goLeft() {
        mConnectedThread.write("a");    // Send "f" via Bluetooth
    }

    private void goRight() {
        mConnectedThread.write("d");    // Send "d" via Bluetooth
    }

    protected float[] lowPass( float[] input, float[] output ) {
        if ( output == null ) return input;

        for ( int i=0; i<input.length; i++ ) {
            output[i] = output[i] + ALPHA * (input[i] - output[i]);
        }
        return output;
    }


}
