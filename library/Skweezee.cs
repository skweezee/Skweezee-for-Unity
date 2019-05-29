using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text.RegularExpressions;
using UnityEngine;

public sealed class Skweezee
{
    // constant values
    private readonly static int MEASUREMENT_DIMENSION = 34;//-----------//28 Skweezee values + 6 IMU values 
    private readonly static int FLAG_A = 10;//--------------------------// A combination of flag A and flag B is used to detect
    private readonly static int FLAG_B = 11;//--------------------------// when a new measurement cycle begins

    static SerialPort port;//-------------------------------------------// reference to the serial connection
    static private bool started;//--------------------------------------// wether or not the data capturing has started
    static private List<int> buffer;//----------------------------------// temp buffer for capturing the data from usb port

    static private int[] measurement = new int[MEASUREMENT_DIMENSION];//// set of values in one sensing cycle
    static private bool useIMU = false;//-------------------------------// boolean on wether or not the imu data needs to be processed
    static private int[] imuData = new int[6];//------------------------//array for IMU data
    static private int dimension;//-------------------------------------// number of values in one measurement
    static private Vector squeezeVector;//------------------------------// squeeze vector, based on measurement
    static private Vector imuVector;//----------------------------------// imu vector, based on measurement
    static private Vector[] slidingWindow;//----------------------------// used to store a time series of vectors
    static private float mag;//-----------------------------------------// momentary magnitude of vector
    static private float max;//-----------------------------------------// maximum observed magnitude of vector
    static private float[] submag;//------------------------------------// momentary magnitudes per electrode
    static private float[] submax;//------------------------------------// maximum observed magnitudes per electrode
    static private List<Form> forms;//----------------------------------// stored forms

    static private readonly string s_error = "<b><color=maroon>" + "\nX" + "</color></b> ";//--// Standard console string : Error  - Red
    static private readonly string s_succes = "<b><color=green>" + "\nV" + "</color></b> ";//--// Standard console string : Succes - Green
    static private readonly string s_info = "<b><color=blue>" + "\no" + "</color></b> ";//-----// Standard console string : Info   - Blue
    static private string log = "";//----------------------------------------------------------// String to hold log information during usb capture

    static private int windowLength = 1;
    static private int windowLengthsmoothingFactor;
    static private int magnitude;
    static private int direction;

    static private List<int> input = new List<int>();//----------------------------------------// inputlist
    static private List<int> inputSelected = new List<int>();//--------------------------------// selected area of the inputlist
    static private List<int> output = new List<int>();//---------------------------------------// parsed data to be added to correct vectors
    static private List<float[]> hist = new List<float[]>();//---------------------------------// history of complete measurement
    static private int[] IMUPrevious = new int[6];//-------------------------------------------// previous IMU measurements to calculate differnce
    static private List<float> shakeHistory = new List<float>();//-----------------------------// history for averaging the shaking behaviour


    static Skweezee()
    {
        port = null;
        started = false;
        buffer = new List<int>();
        measurement = null;
        useIMU = false;
        dimension = 0;
        squeezeVector = null;
        slidingWindow = null;
        mag = 0;
        max = 0;
        submag = null;
        submax = null;
        forms = new List<Form>();
    }

    public static Skweezee Instance { get; } = new Skweezee();



    /*  
     *  InitVariables : Initialize the measurement arrays
     */
    public static void InitVariables()
    {
        measurement = new int[MEASUREMENT_DIMENSION];
        dimension = MEASUREMENT_DIMENSION;
        imuData = new int[6];
    }



    /*
     *  ConnectUSB : Establish the USB connection without giving a prefered port
     */
    public static void ConnectUSB()
    {
        ConnectUSB(""); // If no argument is passed, give it a empty string
    }



    /*
     *   ConnectUSB : Establish the USB connection with a given port
     */
    public static void ConnectUSB(string portnr)
    {
        string[] options = GetValidPorts(); // get all the valid ports as a string array
        bool found = false;                 // Correct port hasn't been found yet
        InitVariables();                    // Initialise measurement arrays

        // See if the specified port is available on this device
        foreach (string x in options)
        {
            if (portnr.Contains(x)) // If we found the port
            {
                port = new SerialPort(portnr, 9600);
                found = true;
            }
        }

        if (!found)           // If we didn't find the port
        {
            if (portnr != "") // If we did specify a port
            {
                log += s_error + "Could not find specified port : <b>" + portnr + "</b>";

                if (options.Length > 0) // Connect to the first available option instead
                {
                    port = new SerialPort(options[0], 9600);
                    port.RtsEnable = true;
                    port.DtrEnable = true;
                    log += s_info + "Trying to connect to <b>" + options[0] + "</b> instead...";
                }
            }
            else // If we didn't specify a port
            {
                if (options.Length > 0) // Just connect to the first available port
                {
                    port = new SerialPort(options[0], 9600);
                    port.RtsEnable = true;
                    port.DtrEnable = true;
                    log += s_info + "Trying to connect to <b>" + options[0] + "</b>...";
                }
            }
        }

        // Evaluate the connection attempt
        // If a connection was made succesfully, open the port and set the readtimeout
        // If not, notify the user that the connection failed
        if (port != null) 
        {
            log += s_succes + "USB Connected!";
            port.Open();
            port.ReadTimeout = 5;
        }
        else
        {
            log += s_error + "Connection failed!";
        }
    }


    /*
     *  GetValidPorts : Return a string array of valid and connected usb ports on the device 
     */
    public static string[] GetValidPorts()
    {
        string[] ports = SerialPort.GetPortNames();
        switch (ports.Length)
        {
            case 0: // No ports found
                {
                    log += s_error + "No ports found";
                    break;
                }
            case 1: // Only one port found
                {
                    log += s_succes + "One port found: <b>" + ports[0] + "</b>";
                    break;
                }
            default: // Mulitple ports found
                {
                    log += s_info + "Mulitple ports found: <b>";
                    foreach (string s in ports)
                    {
                        log += " " + s;
                    }
                    log += "</b>";
                    break;
                }
        }
        return ports;
    }


    /*
     *   Disconnect : Close the current USB connection if currently active
     */
    public static void Disconnect()
    {

        if (port != null)
        {
            port.Close();
            log += s_succes + "USB disconnected";
        }
        else
        {
            log += s_error + "unable to disconnect (Serial port not connected)";
        }

    }



    /*
     * withIMU : indicates that the IMU values needs to be processed from now on 
     */
    public static void withIMU()
    {
        useIMU = true;
    }



   /*
    * withoutIMU : indicates that the IMU values no longer need to be processed
    */
    public static void withoutIMU()
    {
        useIMU = false;
    }




    /*
	 * Sense : collect the data from the serial USB connection
	 */
    static public void Sense()
    {   
        output.Clear();
        if (port != null)
        {
            // check number of available bytes
            int n = port.BytesToRead;
            //log += "Bytes to read: " + n +" ";
            if (n > 0)
            {
                // if data available
                // create byte array that fits the available data
                // and write the available data to that byte array
                byte[] u = new byte[n];
                port.Read(u, 0, n);
                List<int> buffer = new List<int>();
                for (int v = 0; v < u.Length; v++)
                {
                    // for each element in the byte array
                    // cast array element (byte) to integer
                    int t = (int)u[v] & 0xff;
                    input.Add(t);
                }


                // All inputs are now integers, and will be processed
                for (int i = 0; i < input.Count; i++)
                {
                    int val = input[i];
                    int index = Mathf.Max(0, i - 1);
                    int prev = input[index];
                    if (val == FLAG_B && prev == FLAG_A) // check for flags in sequence
                    {
                        inputSelected.RemoveAt(inputSelected.Count - 1); // Remove the previous flag from the measurements
                        for (int a = 0; a < Mathf.Min(inputSelected.Count,Mathf.Min(28,measurement.Length)) ; a++)   // add the Skweezee data to the output buffer
                        {
                            measurement[a] = inputSelected[a];
                            output.Add(inputSelected[a]);
                        }

                        if (useIMU)
                        {
                            imuData = new int[6];
                            for (int a = 28; a < Mathf.Min(MEASUREMENT_DIMENSION, inputSelected.Count); a++) //add the IMU data to the measurement set
                            {
                                int array_index = a - 28;
                                if (array_index < imuData.Length)
                                {
                                    imuData[a - 28] = inputSelected[a];
                                    measurement[a] = inputSelected[a];
                                }
                            }
                        }
                        inputSelected = new List<int>();
                    }
                    else
                    {
                        inputSelected.Add(val);
                    }
                }
                input.Clear();
            }
        }
    }




    /*
     *  Vectorize : parse the data and store in the correct global vectors 
     */
    public static void Vectorize()
    {
        if (measurement != null)
        {
            if (useIMU) // parse Skweezee and IMU data
            {
                float[] x = new float[dimension - imuData.Length]; // for squeeze vector
                float[] y = new float[imuData.Length]; // for imu vector

                for (int i = 0; i < x.Length; i++)
                {
                    // copy each queeze measurement element to squeezeVector
                    // after inverting (255 - element) and scaling (/255)
                    // the resulting vector element is a number from 0 (min squeeze) to 1 (max squeeze)
                    x[i] = (255 - (float)measurement[i]) / 255;

                }
                for (int i = 28; i < 28 + imuData.Length; i++)
                {
                    // copy each IMU measurement element to imuVector
                    y[i - 28] = ((float)measurement[i]) / 255;

                }
                squeezeVector = new Vector(x);
                imuVector = new Vector(y);

            }
            else // only parse the Skweezee data
            {
                float[] x = new float[dimension];

                for (int i = 0; i < dimension; i++)
                {

                    x[i] = (255 - (float)measurement[i]) / 255;

                }
                squeezeVector = new Vector(x);

            }
        }
        else
        {
            // if measurement does not contain actual data
            squeezeVector = null;
        }
    }




    /*
     * Perform basic analysis and store results in global variables 
     */
    static public void BasicAnalysis()
    {

        if (squeezeVector != null)
        {

            mag = squeezeVector.Magnitude();

            if (mag > max)
            {

                max = mag;

            }

            if (output.Count == 28)
            {

                submag = new float[8];
                submax = new float[8];

                for (int i = 0; i < 8; i++)
                {

                    float[] subvector = new float[7];

                    for (int j = 0; j < 7; j++)
                    {

                        subvector[j] = squeezeVector.Component(Shield.SUBVECTOR_MAP[i, j]);

                    }

                    submag[i] = Vector.Magnitude(subvector);


                    if (submag[i] > submax[i])
                    {

                        submax[i] = submag[i];

                    }


                }

            }

        }

    }





    /*
    * Store
    */
    static public void Store()
    {

        if (slidingWindow.Length < 5)
        {

            slidingWindow = new Vector[5];

            for (int i = 0; i < 5; i++)
            {

                slidingWindow[i] = squeezeVector;

            }

        }
        else
        {

            for (int i = 0; i < 5 - 1; i++)
            {

                slidingWindow[5 - 1 - i] = slidingWindow[5 - 1 - i - 1];

            }

            slidingWindow[0] = squeezeVector;

        }

    }




    /*
     *  IsShaking : returns wether or not the IMU is shaking based on a hardcoded threshold
     */
    public static bool IsShaking()
    {
        return (GetShakingFactor() > 20);
    }




    /*
    *  IsShaking : returns wether or not the IMU is shaking based on a parameter threshold
    */
    public static bool IsShaking( float treshold )
    {
        return (GetShakingFactor() > treshold);
    }




    /*
     *  GetShakingFactor : Returns the current shaking factor as a float, averaged out over 100 samples 
     */
    public static float GetShakingFactor()
    {
        int[] data = GetIMU();     
        float diff = 0;
        float sum = 0;

        // Get the 3 gyroscope axis
        for (int i = 0 ; i < Mathf.Min(3,data.Length); i++)
        {
            diff += Mathf.Abs(data[i] - IMUPrevious[i]);
        }

        // Add the rate of change
        shakeHistory.Add(diff);

        // Only keep the last 100 samples
        while(shakeHistory.Count > 100)
        {
            shakeHistory.RemoveAt(0);
        }

        // Calculate the average of the history
        foreach(float f in shakeHistory)
        {
            sum += f;
        }
        sum /= shakeHistory.Count;

        // Set the current data as the previous for the next frame
        IMUPrevious = data;
        return sum;
    }




    /*
     *  GetImu  : Returns the int[] that contains all parse IMU measurements 
     */
    public static int[] GetIMU()
    {
            return imuData;
    }




    /*
     *  GetIMUAngle  : Returns the angle around the z and x axis, useful for steering wheel applications
     */
    public static float[] GetIMUAngle()
    {
        int[] IMU = Skweezee.GetIMU();
        float[] IMUFloat = new float[6];
        // Convert them all to floats
        for (int i = 0; i < IMUFloat.Length; i++)
        {
            IMUFloat[i] = (1 / 0.75f) * 180 * ((float)IMU[i] - 127f) / 255f;
        }

        // Average out over history
        hist.Add(IMUFloat);
        IMUFloat = new float[6];
        if (hist.Count > 10) hist.RemoveAt(0); // Only use the last 10 samples
        for (int i = 0; i < hist.Count; i++)
        {
            for (int index = 0; index < 6; index++)
                IMUFloat[index] += hist[i][index];
        }

        for (int i = 0; i < 6; i++)
        {
            IMUFloat[i] /= hist.Count;
        }

        Vector3 acc = new Vector3(IMUFloat[0], IMUFloat[1], IMUFloat[2]);
        acc.Normalize();
        Vector3 gyro = new Vector3(IMUFloat[3], IMUFloat[4], IMUFloat[5]);

        float az = 180 * Mathf.Acos(acc.x) / Mathf.PI * Mathf.Sign(acc.z); // Rotation around the Z-axis (pointed towards user)
        float ay = 180 * Mathf.Acos(acc.z) / Mathf.PI * Mathf.Sign(acc.x);
        return new float[] { az, ay };
    }




    /**
	 * Skweezee dimension
	 * 
	 * @return	dimension
	 */
    public static int Dimension()
    {

        return dimension;

    }




    /**
	 * Raw measurement
	 * 
	 * @param	electrode
	 * @return	measurement
	 */
    public static int[] Raw(params int[] e)
    {

        int[] m = null;

        if (e.Length == 0)
        {

            m = measurement;

        }
        else if (e.Length == 1)
        {

            if (dimension == 28)
            {

                m = new int[] { 0, 0, 0, 0, 0, 0, 0 };

                for (int i = 0; i < 7; i++)
                {

                    m[i] = measurement[Shield.SUBVECTOR_MAP[e[0], i]];

                }

            }

        }

        return m;

    }

   
    
    
    /**
	 * Squeeze vector
	 * 
	 * @param	e	electrode (optional)
	 * @return	Array of numbers representing the momentary squeeze vector
	 */
    public static float[] GetVector(params int[] e)
    {

        float[] x = null;

        if (e.Length == 0)
        {

            x = squeezeVector.Array();

        }
        else if (e.Length == 1)
        {

            if (dimension == 28)
            {

                x = new float[] { 0, 0, 0, 0, 0, 0, 0 };

                for (int i = 0; i < 7; i++)
                {

                    x[i] = squeezeVector.Component(Shield.SUBVECTOR_MAP[e[0], i]);

                }

            }

        }

        return x;

    }




    /**
	 * Magnitude of squeeze vector
	 * 
	 * @param	e	electrode (optional)
	 * @return	Number representing the vector magnitude of the momentary squeeze vector
	 */
    public static float Mag(params int[] e)
    {
        if (e.Length == 0)
        {
            return mag;
        }

        else if (e.Length == 1)
        {

            return submag[e[0]];

        }
        return 0;
    }




    /**
	 * Direction of squeeze vector
	 * 
	 * @param	e	electrode (optional)
	 * @return	Array of numbers representing the unit vector of the momentary squeeze vector. A unit vector is a vector with magnitude '1' and represents the direction of the vector.
	 */
    public static float[] Dir(params int[] e)
    {

        float[] dir = null;

        if (e.Length == 0)
        {

            dir = squeezeVector.Direction();

        }
        else if (e.Length == 1)
        {

            dir = squeezeVector.Direction(GetVector(e));

        }

        return dir;

    }
    /*
     *  Return the current direction of the IMU 
     */
    public static float[] DirIMU(params int[] e)
    {
        float[] dir = null;

        if (useIMU)
        {
            if (e.Length == 0)
            {

                dir = imuVector.Direction();

            }
            else if (e.Length == 1)
            {

                dir = imuVector.Direction(GetVector(e));

            }
        }

        return dir;
    }




    /**
	 * Maximum: data normalization, rescale
	 * 
	 * @param	e	electrode (optional)
	 * @return	Number representing the observed maxiumum magnitude
	 */
    public static float Max(params int[] e)
    {

        float m = 0;

        if (e.Length == 0)
        {

            m = max;


        }
        else if (e.Length == 1)
        {

            m = submax[e[0]];

        }

        return m;

    }




    /**
	 * Moving average: smooth
	 * 
	 * @param	electrode
	 * @return	average
	 */
    public static float Avg(params int[] e)
    {

        float avg = 0;

        if (e.Length == 0)
        {

            for (int i = 0; i < 5; i++)
            {

                avg += slidingWindow[i].Magnitude();

            }

        }
        else if (e.Length == 1)
        {


            float[] sub = new float[] { 0 };

            for (int j = 0; j < 5; j++)
            {

                sub = new float[] { 0, 0, 0, 0, 0, 0, 0 };

                for (int i = 0; i < 7; i++)
                {

                    sub[i] = slidingWindow[j].Component(Shield.SUBVECTOR_MAP[e[0], i]);

                }

                avg += Vector.Magnitude(sub);

            }


        }

        return avg / 5;

    }




    /**
	 * Moving standard deviation: variance, stability
	 * 
	 * @param	e	electrode (optional)
	 * @return	standard deviation
	 */
    public static float Stdev(params int[] e)
    {

        float sd = 0;
        float avg = 0;

        if (e.Length == 0)
        {

            avg = Avg();

            for (int i = 0; i < 5; i++)
            {

                float d = slidingWindow[i].Magnitude() - avg;
                sd += d * d;

            }

        }
        else if (e.Length == 1)
        {

            avg = Avg(e);
            float[] sub = new float[] { 0, 0, 0, 0, 0, 0, 0 };

            for (int j = 0; j < 5; j++)
            {

                for (int i = 0; i < 7; i++)
                {

                    sub[i] = slidingWindow[j].Component(Shield.SUBVECTOR_MAP[e[0], i]);

                }

                float d = Vector.Magnitude(sub) - avg;
                sd += d * d;

            }

        }

        return (float)Mathf.Sqrt(sd);

    }




    /**
	 * 
	 * Differentiation, first derivative: rate of change
	 * 
	 * @param	electrode
	 * @return	difference
	 */
    public static float Diff(params int[] e)
    {

        float t1 = 0;
        float t2 = 0;
        float t4 = 0;
        float t5 = 0;

        if (e.Length == 0)
        {

            t1 = (-1 * slidingWindow[0].Magnitude());
            t2 = (8 * slidingWindow[1].Magnitude());
            t4 = (-8 * slidingWindow[3].Magnitude());
            t5 = (1 * slidingWindow[4].Magnitude());

        }
        else if (e.Length == 1)
        {

            float[] sub = new float[] { 0, 0, 0, 0, 0, 0, 0 };

            for (int i = 0; i < 7; i++)
            {

                sub[i] = slidingWindow[0].Component(Shield.SUBVECTOR_MAP[e[0], i]);

            }

            t1 = (-1 * Vector.Magnitude(sub));

            sub = new float[] { 0, 0, 0, 0, 0, 0, 0 };
            for (int i = 0; i < 7; i++)
            {

                sub[i] = slidingWindow[1].Component(Shield.SUBVECTOR_MAP[e[0], i]);

            }

            t2 = (8 * Vector.Magnitude(sub));

            sub = new float[] { 0, 0, 0, 0, 0, 0, 0 };
            for (int i = 0; i < 7; i++)
            {

                sub[i] = slidingWindow[3].Component(Shield.SUBVECTOR_MAP[e[0], i]);

            }

            t4 = (-8 * Vector.Magnitude(sub));

            sub = new float[] { 0, 0, 0, 0, 0, 0, 0 };
            for (int i = 0; i < 7; i++)
            {

                sub[i] = slidingWindow[4].Component(Shield.SUBVECTOR_MAP[e[0], i]);

            }

            t5 = (1 * Vector.Magnitude(sub));

        }

        return (t1 + t2 + t4 + t5) / 12;

    }




    /**
	 * Normalized signal
	 * 
	 * @param	electrode
	 * @return	norm
	 */
    public static float Norm(params int[] e)
    {

        float x = 0;

        if (e.Length == 0)
        {

            x = Avg() / max;

        }
        else if (e.Length == 1)
        {

            x = Avg(e) / submax[e[0]];
        }

        return x * x;

    }




    /**
	 * Inverse normalized signal
	 * 
	 * @param	electrode
	 * @return	inverse
	 */
    public static float Inv(params int[] e)
    {

        float x = 0;

        if (e.Length == 0)
        {

            x = Norm();

        }
        else if (e.Length == 1)
        {

            x = Norm(e[0]);
        }

        return 1 - x;

    }




    /**
	 * Squared normalized signal
	 * 
	 * @param	electrode
	 * @return	square
	 */

    public static float Square(params int[] e)
    {

        float x = 0;

        if (e.Length == 0)
        {

            x = Norm();

        }
        else if (e.Length == 1)
        {

            x = Norm(e[0]);
        }

        return x * x;

    }




    /**
	 * 
	 * Square root of normalized signal
	 * 
	 * @param	electrode
	 * @return	square root
	 */

    public static float Root(params int[] e)
    {

        float x = 0;

        if (e.Length == 0)
        {

            x = Norm();

        }
        else if (e.Length == 1)
        {

            x = Norm(e[0]);
        }

        return (float)Mathf.Sqrt(x);

    }




    /* FORM RECOGNITION */
    /**
	 * Record: stores the momentary form (as direction) with specified label, to
	 * be used in combination with the recognition function (rcg).
	 * 
	 * @param label	form label
	 */
    public static void Rcd()
    {

        Boolean found = false;

        for (int i = 0; i < forms.Count; i++)
        {

            if (forms[i].Is(""))
            {

                forms[i].Record();
                found = true;

            }

        }

        if (!found)
        {

            Form form = new Form();
            form.Record();
            forms.Add(form);

        }

    }





    /**
	 * Record: stores the momentary form (as direction) with specified label.
	 * 
	 * @param label	form label
	 */
    public static void Rcd(String label)
    {

        Boolean found = false;

        for (int i = 0; i < forms.Count; i++)
        {

            if (forms[i].Is(label))
            {

                forms[i].Record();
                found = true;

            }

        }

        if (!found)
        {

            Form form = new Form(label);
            form.Record();
            forms.Add(form);

        }



    }





    /**
	 * Returns set of recorded Skweezee forms. The resulting List contains Form
	 * objects.
	 * 
	 * @return List of Form objects
	 */
    public static List<Form> Forms()
    {
        return forms;
    }




   /*   
    *   GetLog : Returns the log data from the Skweezee. Useful for detecting connection problems
    */
    public static string GetLog()
    {
        return log;
    }




   /*   
    *   Clears the log data from the Skweezee
    */
    public static void ClearLog()
    {
        log = "";
    }

}