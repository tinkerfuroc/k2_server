using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using Newtonsoft.Json;

namespace Kinect2Server
{
    public class BodyInfo
    {
        public Body Body { get; set; }
        public int FromX { get; set; }
        public int ToX { get; set; }
        public int FromY { get; set; }
        public int ToY { get; set; }
    };

    public class FlaggedSocket
    {
        public Socket socket;
        public bool prevSendDoneFlag;

        public FlaggedSocket()
        {
            this.prevSendDoneFlag = true;
        }
    };
    public class AsyncNetworkConnectorServer
    {
        public IPAddress selfIPaddress = null;
        public IPEndPoint selfEndPoint = null;
        public int selfPortNumber;
        public string localPCName;

        public Socket listenerSocket = null;
        public SynchronizedCollection<FlaggedSocket> connectedClientList = null;
        public bool connectedToAtleastOne = false;

        public AsyncNetworkConnectorServer(int selfPortNum)
        {
            this.listenerSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            this.selfPortNumber = selfPortNum;
            this.finishSocketGroundWork();
        }

        public void finishSocketGroundWork()
        {
            this.localPCName = System.Environment.MachineName;
            IPHostEntry selfInfo = Dns.GetHostEntry(this.localPCName);
            this.selfIPaddress = IPAddress.Parse("0.0.0.0");
            this.selfEndPoint = new IPEndPoint(this.selfIPaddress, this.selfPortNumber);
            this.connectedClientList = new SynchronizedCollection<FlaggedSocket>();
            this.listenerSocket.Bind(this.selfEndPoint);
        }

        public void startListening()
        {
            this.listenerSocket.Listen(100);
            this.listenerSocket.BeginAccept(new AsyncCallback(startListeningCallBack), this);
        }

        public void startListeningCallBack(IAsyncResult ar)
        {
            try
            {
                AsyncNetworkConnectorServer connector = (AsyncNetworkConnectorServer)ar.AsyncState;
                FlaggedSocket tempSocket = new FlaggedSocket();
                tempSocket.socket = (connector.listenerSocket.EndAccept(ar));
                connector.connectedClientList.Add(tempSocket);
                Console.WriteLine("Added socket to client list");
                connector.listenerSocket.BeginAccept(new AsyncCallback(startListeningCallBack), connector);
            }
            catch (Exception e)
            {
                Console.WriteLine("Error in startListeningCallBack: " + e.ToString());
            }
        }

        public void recieveCallBack(IAsyncResult ar)
        {

        }

        public void sendCallBack(IAsyncResult ar)
        {
            try
            {
                FlaggedSocket tempSocket = (FlaggedSocket)ar.AsyncState;
                if (tempSocket.socket.Connected)
                {
                    int sentBytes = tempSocket.socket.EndSend(ar);
                    //Console.WriteLine("Number of Bytes sent = " + sentBytes.ToString());
                    tempSocket.prevSendDoneFlag = true;
                }
            }
            catch (Exception e)
            {
                FlaggedSocket tempSocket = (FlaggedSocket)ar.AsyncState;
                Console.WriteLine("Error in sendCallBack: " + e.ToString());
                tempSocket.socket.Close();
                tempSocket.socket.Dispose();
                this.connectedClientList.Remove(tempSocket);
            }
        }

        public void receive()
        {

        }

        public void send(FlaggedSocket tempSocket, byte[] data)
        {
            try
            {
                if (tempSocket.prevSendDoneFlag)
                {
                    tempSocket.prevSendDoneFlag = false;
                    tempSocket.socket.BeginSend(data, 0, data.Length, 0, new AsyncCallback(this.sendCallBack), tempSocket);
                }
                else
                {
                    //Console.WriteLine("Skipping");
                }

            }
            catch (Exception e)
            {
                Console.WriteLine("Error in send: " + e.ToString());
                tempSocket.socket.Close();
                tempSocket.socket.Dispose();
                this.connectedClientList.Remove(tempSocket);
            }
        }

        public void sendToAll(byte[] data)
        {
            IEnumerator<FlaggedSocket> enumerator = this.connectedClientList.GetEnumerator();
            try
            {
                while (enumerator.MoveNext())
                {
                    this.send(enumerator.Current, data);
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("Error in sentToAll: " + e.ToString());
            }
        }

        public void closeSocket()
        {
            IEnumerator<FlaggedSocket> enumerator = this.connectedClientList.GetEnumerator();
            while (enumerator.MoveNext())
            {
                try
                {
                    enumerator.Current.socket.Shutdown(SocketShutdown.Both);
                    enumerator.Current.socket.Close();
                }
                catch (Exception e)
                {
                    Console.WriteLine("Error in closing: " + e.ToString());
                }
            }
        }
    };

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinect = null;
        private Stopwatch stopwatch = null;
        private MultiSourceFrameReader reader = null;

        private byte[] colourArray = null;
        private CoordinateMapper coordinateMapper = null;
        private ushort[] depthArray = null;
        private CameraSpacePoint[] mappedSpaceArray = null;
        private short[] locationArray = null;
        private byte[] byteLocationArray = null;
        private Body[] bodyArray = null;
        private byte[] bodyBuffer = null;

        private double numFramesPassed = 0;
        private double deltaTimeForFPS = 0.02;//In seconds
        private DateTime updateFPSMilestone = DateTime.Now;

        private readonly int bytesPerColorPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        private readonly int bytesPerLocationPixel = 6;

        //TCP/IP crap
        private AsyncNetworkConnectorServer colorConnector = null;
        private AsyncNetworkConnectorServer locationConnector = null;
        private AsyncNetworkConnectorServer bodyConnector = null;

        private const int BUFFER_SIZE_COLOR = 1920 * 1080 * 4;
        private const int BUFFER_SIZE_LOCATION = 1920 * 1080 * 3;
        private const int BUFFER_SIZE_BODY = 60000;

        private const int colorPort = 9000;
        private const int bodyPort = 9003;
        private const int locationPort = 18000;

        private const string hostName = "herb2";
        //End of TCP/IP crap

        private BodyFrameReader bodyReader = null;

        private int cnt = 0;

        public MainWindow()
        {
            this.InitializeComponent();
            this.stopwatch = new Stopwatch();
            this.stopwatch.Start();
            this.kinect = KinectSensor.GetDefault();
            this.coordinateMapper = kinect.CoordinateMapper;

            if (this.kinect != null)
            {
                this.kinect.Open();

                int colorHeight = kinect.ColorFrameSource.FrameDescription.Height;
                int colorWidth = kinect.ColorFrameSource.FrameDescription.Width;
                int depthHeight = kinect.DepthFrameSource.FrameDescription.Height;
                int depthWidth = kinect.DepthFrameSource.FrameDescription.Width;

                Console.WriteLine(colorWidth);
                Console.WriteLine(colorHeight);
                Console.WriteLine(depthWidth);
                Console.WriteLine(depthHeight);

                this.reader = this.kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
                this.bodyReader = this.kinect.BodyFrameSource.OpenReader();

                this.colourArray = new byte[colorHeight * colorWidth * bytesPerColorPixel];
                this.depthArray = new ushort[depthHeight * depthWidth];
                this.mappedSpaceArray = new CameraSpacePoint[colorHeight * colorWidth];
                this.locationArray = new short[colorHeight * colorWidth * 3];
                this.byteLocationArray = new byte[colorHeight * colorWidth * bytesPerLocationPixel];
                this.bodyArray = new Body[kinect.BodyFrameSource.BodyCount];
                this.bodyBuffer = new byte[BUFFER_SIZE_BODY];

                this.reader.MultiSourceFrameArrived += this.frameArrivedCallback;
                this.bodyReader.FrameArrived += this.bodyArrivedCallback;
                this.updateFPSMilestone = DateTime.Now + TimeSpan.FromSeconds(this.deltaTimeForFPS);
            }
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            //instantiate sockets and get ip addresses. 
            this.colorConnector = new AsyncNetworkConnectorServer(colorPort);
            this.locationConnector = new AsyncNetworkConnectorServer(locationPort);
            this.bodyConnector = new AsyncNetworkConnectorServer(bodyPort);

            this.colorIPBox.Text = this.colorConnector.selfEndPoint.ToString();
            this.locationIPBox.Text = this.locationConnector.selfEndPoint.ToString();
            //Create the connections
            this.colorConnector.startListening();
            this.locationConnector.startListening();
            this.bodyConnector.startListening();
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.reader != null)
            {
                this.reader.Dispose();
                this.reader = null;
            }
            if (this.reader != null)
            {
                this.kinect.Close();
                this.kinect = null;
            }
            this.colorConnector.closeSocket();
            this.locationConnector.closeSocket();
            this.bodyConnector.closeSocket();
        }

        private void bodyArrivedCallback(object sender, BodyFrameArrivedEventArgs e)
        {
            //Console.WriteLine("body frame arrived!");
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                BodyInfo[] infoes = null;
                if (bodyFrame != null)
                {
                    bodyFrame.GetAndRefreshBodyData(this.bodyArray);
                    int num_body = bodyFrame.BodyCount;
                    infoes = new BodyInfo[num_body];
                    for (int i = 0; i < num_body; i++)
                    {
                        infoes[i] = new BodyInfo();
                        infoes[i].Body = bodyArray[i];
                        infoes[i].FromX = int.MaxValue;
                        infoes[i].ToX = int.MinValue;
                        infoes[i].FromY = int.MaxValue;
                        infoes[i].ToY = int.MinValue;
                        foreach (Joint joint in bodyArray[i].Joints.Values )
                        {
                            CoordinateMapper mapper = bodyFrame.BodyFrameSource.KinectSensor.CoordinateMapper;
                            ColorSpacePoint point = mapper.MapCameraPointToColorSpace(joint.Position);
                            int x = (int)point.X;
                            int y = (int)point.Y;
                            if (x >= 0 && y >= 0)
                            {
                                infoes[i].FromX = x < infoes[i].FromX ? x : infoes[i].FromX;
                                infoes[i].FromY = y < infoes[i].FromY ? y : infoes[i].FromY;
                                infoes[i].ToX = x > infoes[i].ToX ? x : infoes[i].ToX;
                                infoes[i].ToY = y > infoes[i].ToY ? y : infoes[i].ToY;
                            }
                        }
                    }
                    string jsonString = JsonConvert.SerializeObject(infoes);
                    System.Buffer.BlockCopy(ASCIIEncoding.ASCII.GetBytes(jsonString), 0, bodyBuffer, 0, jsonString.Length);
                    this.bodyConnector.sendToAll(bodyBuffer);
                }
            }
        }

        private void frameArrivedCallback(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            this.numFramesPassed++;
            if (DateTime.Now > updateFPSMilestone)
            {
                this.stopwatch.Stop();
                double fps = this.numFramesPassed / this.stopwatch.Elapsed.TotalSeconds;
                this.stopwatch.Reset();
                this.stopwatch.Start();
                this.updateFPSMilestone = DateTime.Now + TimeSpan.FromSeconds(this.deltaTimeForFPS);
                this.numFramesPassed = 0;
                this.statusBox.Text = fps.ToString();
            }
            else return;
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            using (ColorFrame cFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
            {
                if (cFrame != null)
                {
                    cFrame.CopyConvertedFrameDataToArray(this.colourArray, ColorImageFormat.Bgra);
                    colorConnector.sendToAll(this.colourArray);
                }
            }
            using (DepthFrame dFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
            {
                if (dFrame != null)
                {
                    dFrame.CopyFrameDataToArray(this.depthArray);   //Ushort Array ! Use BitConverter.getBytes() to convert to two bytes per each uShort. it gives low byte followed by high byte
                    coordinateMapper.MapColorFrameToCameraSpace(depthArray, mappedSpaceArray);
                    for (int i = 0; i < mappedSpaceArray.Length; i++)
                    { 
                        locationArray[3 * i] = ToShortLocation(mappedSpaceArray[i].X);
                        locationArray[3 * i + 1] = ToShortLocation(mappedSpaceArray[i].Y);
                        locationArray[3 * i + 2] = ToShortLocation(mappedSpaceArray[i].Z);
                    }
                    Buffer.BlockCopy(this.locationArray, 0, this.byteLocationArray, 0, this.byteLocationArray.Length);
                    this.locationConnector.sendToAll(this.byteLocationArray);
                }
            }
        }

        private short ToShortLocation(float location)
        {
            if (location < -100) return 0;
            if (location < -10 || location > 10)
                Console.WriteLine(location);
            return (short)(location * 1000);
        } 

        private void statusBox_TextChanged(object sender, TextChangedEventArgs e)
        {

        }
    }
}