/* tinker2016:
 *  the Kinect server on Windows, send info to ROS
 * 
 * audio(raw data, 16kHz)
 * 
 */

using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
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
        // Kinect and its readers
        private KinectSensor kinect = null;
        private Stopwatch stopwatch = null;
        private AudioBeamFrameReader audioReader = null;

        private int sec = 0;
        private int cnt = 0;

        // Thread
        Thread readingAudioThread; 

        // Data
        private byte[] audioBuffer = null;

        private double numFramesPassed = 0;
        private double deltaTimeForFPS = 0.002;//In seconds
        private DateTime updateFPSMilestone = DateTime.Now;

        private KinectAudioStream convertStream = null;

        //TCP/IP crap
        private AsyncNetworkConnectorServer audioConnector = null;
        private AsyncNetworkConnectorServer audioStreamConnector = null;

        private const int BUFFER_SIZE_AUDIO = 1024;


        private const int audioPort = 9009;
        private const int audioStreamPort = 9012;
        private const int locationPort = 18000;

        private const string hostName = "herb2";
        //End of TCP/IP crap

        public MainWindow()
        {
            this.InitializeComponent();
            this.stopwatch = new Stopwatch();
            this.stopwatch.Start();
            this.kinect = KinectSensor.GetDefault();

            if (this.kinect != null)
            {
                // audioReader
                AudioSource audioSource = this.kinect.AudioSource;
                this.audioBuffer = new byte[audioSource.SubFrameLengthInBytes];
                this.audioReader = audioSource.OpenReader();
                this.audioReader.FrameArrived += this.audioArrivedCallback;

                // fps
                this.updateFPSMilestone = DateTime.Now + TimeSpan.FromSeconds(this.deltaTimeForFPS);

                // Audio Stream
                IReadOnlyList<AudioBeam> audioBeamList = this.kinect.AudioSource.AudioBeams;
                System.IO.Stream audioStream = audioBeamList[0].OpenInputStream();
                // create the convert stream
                this.convertStream = new KinectAudioStream(audioStream);
                this.convertStream.SpeechActive = true;

                // initialization finished
                this.kinect.Open();
            }
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            //instantiate sockets and get ip addresses. 
            this.audioConnector = new AsyncNetworkConnectorServer(audioPort);
            this.audioStreamConnector = new AsyncNetworkConnectorServer(audioStreamPort);

            //Create the connections
            this.audioConnector.startListening();
            this.audioStreamConnector.startListening();

            // thread
            readingAudioThread = new Thread(this.endlessAudioReading);
            readingAudioThread.Start();
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            this.convertStream.SpeechActive = false;
            readingAudioThread.Abort();

            if (this.kinect != null)
            {
                this.kinect.Close();
                this.kinect = null;
            }
            this.audioConnector.closeSocket();
            this.audioStreamConnector.closeSocket();
        }

        private void audioArrivedCallback(object sender, AudioBeamFrameArrivedEventArgs e) 
        {
            AudioBeamFrameReference frameReference = e.FrameReference;
            AudioBeamFrameList frameList = frameReference.AcquireBeamFrames();

            if (frameList != null)
            {
                // AudioBeamFrameList is IDisposable
                using (frameList)
                {
                    // Only one audio beam is supported. Get the sub frame list for this beam
                    IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                    if (subFrameList.Count > 0) 
                    {
                        AudioBeamSubFrame subFrame = subFrameList[0];  // TODO 
                        if (subFrame.BeamAngleConfidence > 0.6)
                        {
                            float angle = subFrame.BeamAngle * 3.1415926f / 2; // TODO: check the formula
                            Debug.WriteLine("audio angle:" + angle + " " + subFrame.BeamAngleConfidence);
                            // send socket here
                            byte[] tempBuffer = BitConverter.GetBytes(angle);
                            this.audioConnector.sendToAll(tempBuffer);
                        }

                        //subFrame.CopyFrameDataToArray(this.audioBuffer);
                        //this.audioStreamConnector.sendToAll(audioBuffer);
                        //if (DateTime.Now.Second != sec)
                        //{
                        //    Debug.WriteLine(cnt);
                        //    sec = DateTime.Now.Second;
                        //    cnt = 0;
                        //}
                        //cnt++;
                    }
                }
            }
        }

        private void endlessAudioReading()
        {
            while (this.convertStream.SpeechActive)
            {
                if (1024 == this.convertStream.Read(audioBuffer, 0, 1024))
                {
                    audioStreamConnector.sendToAll(audioBuffer);
                    //Console.WriteLine(DateTime.Now+" audio received");
                    if (DateTime.Now.Second != sec)
                    {
                        Debug.WriteLine(cnt);
                        sec = DateTime.Now.Second;
                        cnt = 0;
                    }
                    cnt++;
                }

            }
        }

        private void statusBox_TextChanged(object sender, TextChangedEventArgs e)
        {

        }
    }
}