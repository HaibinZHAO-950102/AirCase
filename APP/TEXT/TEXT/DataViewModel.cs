using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using Plugin.BLE.Abstractions.Contracts;
using Xamarin.Essentials;
using Xamarin.Forms;
using Xamarin.Forms.PlatformConfiguration;

namespace TEXT
{
    public class DataViewModel : INotifyPropertyChanged
    {
        // Variables about BLE Connection
        public IDevice selectedDevice;
        public IBluetoothLE ble;
        public IAdapter adapter;

        public string Characteristic_UUID_1 = "5bfdc0be-6476-11eb-ae93-0242ac130002";
        public string Characteristic_UUID_2 = "d2c03edc-64b4-11eb-ae93-0242ac130002";
        public string Characteristic_UUID_3 = "d8eaca70-64b4-11eb-ae93-0242ac130002";
        public string Characteristic_UUID_4 = "dd5fa4ea-64b4-11eb-ae93-0242ac130002";
        public string Characteristic_UUID_5 = "e1441bf4-64b4-11eb-ae93-0242ac130002";
        public string Characteristic_UUID_6 = "84e1aa56-64b5-11eb-ae93-0242ac130002";
        public string Characteristic_UUID_7 = "893ce75a-64b5-11eb-ae93-0242ac130002";

        // Variables about BLE Signals
        public string temperature = "0", pressure = "0", humidity = "0", voc = "0", altitude = "0", co2 = "0";
        public string light = "0", rotate = "0", red = "0", green = "0", blue = "0", violet = "0", orange = "0", yellow = "0";

        // Variables about Data Recording
        public string Label3 = "Defaut", Label_Running;
        public bool recording = false;
        public string record_state = "Not Recording";
        public string year, month, day, hour, minute, second, time;

        public ArrayList year_data = new ArrayList();
        public ArrayList month_data = new ArrayList();
        public ArrayList day_data = new ArrayList();
        public ArrayList hour_data = new ArrayList();
        public ArrayList minute_data = new ArrayList();
        public ArrayList second_data = new ArrayList();
        public ArrayList temperature_data = new ArrayList();
        public ArrayList pressure_data = new ArrayList();
        public ArrayList humidity_data = new ArrayList();
        public ArrayList voc_data = new ArrayList();
        public ArrayList altitude_data = new ArrayList();
        public ArrayList co2_data = new ArrayList();
        public ArrayList light_data = new ArrayList();
        public ArrayList rotate_data = new ArrayList();
        public ArrayList red_data = new ArrayList();
        public ArrayList green_data = new ArrayList();
        public ArrayList blue_data = new ArrayList();
        public ArrayList violet_data = new ArrayList();
        public ArrayList orange_data = new ArrayList();
        public ArrayList yellow_data = new ArrayList();

        // Methods about PropertyChanged
        public event PropertyChangedEventHandler PropertyChanged;
        public void OnPropertyChanged([CallerMemberName] string name = "")
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }

        // Methods about Setting & Getting BLE Data
        public void record_device(IDevice device)
        {
            selectedDevice = device;
            Console.WriteLine(selectedDevice.Name.ToString() + " was successfully recorded!");
        }
        public IDevice GetDevice()
        {
            if(selectedDevice != null)
            {
                Console.WriteLine("Got the Device " + selectedDevice.Name.ToString() + "!");
                return selectedDevice;
            }
            else
            {
                Console.WriteLine("There is no connected Device!");
                return null;
            }
        }
        public void record_BLE(IBluetoothLE bluetooth)
        {
            ble = bluetooth;
        }
        public IBluetoothLE GetBLE()
        {
            if(ble != null)
            {
                Console.WriteLine("Got the BLE!");
                return ble;
            }
            else
            {
                Console.WriteLine("There is no BLE!");
                return null;
            }
        }
        public void record_Adapter(IAdapter ADA)
        {
            adapter = ADA;
        }
        public IAdapter GetAdapter()
        {
            if (adapter != null)
            {
                Console.WriteLine("Got the Adapter!");
                return adapter;
            }
            else
            {
                Console.WriteLine("There is no Adapter!");
                return null;
            }
        }

        // Methods about Signals
        internal string messagespliter(string message, int v)
        {
            int comma = message.IndexOf(',');
            if (v == 0)
            {
                return message.Substring(0, comma);
            }
            else
            {
                return message.Substring(comma + 1);
            }
        }
        public string TEMPERATURE
        {
            get { return temperature; }
            set
            {
                temperature = value;
                //Console.WriteLine("Get the Temperature, which is " + temperature);
                OnPropertyChanged();
                if (recording == true)
                {
                    temperature_data.Add(Convert.ToInt32(temperature));
                    Device.StartTimer(TimeSpan.FromSeconds(0.1), () =>
                    {
                        year = DateTime.Now.ToString("yyyy");
                        month = DateTime.Now.ToString("MM");
                        day = DateTime.Now.ToString("dd");
                        hour = DateTime.Now.ToString("HH");
                        minute = DateTime.Now.ToString("mm");
                        second = DateTime.Now.ToString("ss");
                        time = DateTime.Now.ToString(" yyyy-MM-dd HH:mm:ss");
                        return true;
                    });
                    //Console.WriteLine(second);
                    year_data.Add(Convert.ToInt32(year));
                    month_data.Add(Convert.ToInt32(month));
                    day_data.Add(Convert.ToInt32(day));
                    hour_data.Add(Convert.ToInt32(hour));
                    minute_data.Add(Convert.ToInt32(minute));
                    second_data.Add(Convert.ToInt32(second));
                }
            }
        }
        public string PRESSURE
        {
            get { return pressure; }
            set
            {
                pressure = value;
                //Console.WriteLine("Get the Pressure, which is " + pressure);
                OnPropertyChanged();
                if (recording == true)
                {
                    pressure_data.Add(Convert.ToInt32(pressure));
                }
            }
        }
        public string HUMIDITY
        {
            get { return humidity; }
            set
            {
                humidity = value;
                //Console.WriteLine("Get the Humidity, which is " + humidity);
                OnPropertyChanged();
                if (recording == true)
                {
                    humidity_data.Add(Convert.ToInt32(humidity));
                }
            }
        }
        public string VOC
        {
            get { return voc; }
            set
            {
                voc = value;
                //Console.WriteLine("Get the VOC, which is " + voc);
                OnPropertyChanged();
                if (recording == true)
                {
                    voc_data.Add(Convert.ToInt32(voc));
                }
            }
        }
        public string ALTITUDE
        {
            get { return altitude; }
            set
            {
                altitude = value;
                //Console.WriteLine("Get the Altitude, which is " + altitude);
                OnPropertyChanged();
                if (recording == true)
                {
                    altitude_data.Add(Convert.ToInt32(altitude));
                }
            }
        }
        public string CO2
        {
            get { return co2; }
            set
            {
                co2 = value;
                //Console.WriteLine("Get the CO2, which is " + co2);
                OnPropertyChanged();
                if (recording == true)
                {
                    co2_data.Add(Convert.ToInt32(co2));
                }
            }
        }
        public string LIGHT
        {
            get { return light; }
            set
            {
                light = value;
                //Console.WriteLine("Light Strength is " + light);
                if (recording == true)
                {
                    light_data.Add(Convert.ToInt32(light));
                }
            }
        }
        public string ROTATE
        {
            get { return rotate; }
            set
            {
                rotate = value;
                //Console.WriteLine("Rotate Speed is " + rotate);
                if (recording == true)
                {
                    rotate_data.Add(Convert.ToInt32(rotate));
                }
            }
        }
        public string RED
        {
            get { return red; }
            set
            {
                red = value;
                //Console.WriteLine("Red Spectrum is " + red);
                if (recording == true)
                {
                    red_data.Add(Convert.ToInt32(red));
                }
            }
        }
        public string GREEN
        {
            get { return green; }
            set
            {
                green = value;
                //Console.WriteLine("Green Spectrum is " + green);
                if (recording == true)
                {
                    green_data.Add(Convert.ToInt32(green));
                }
            }
        }
        public string BLUE
        {
            get { return blue; }
            set
            {
                blue = value;
                //Console.WriteLine("Blue Spectrum is " + blue);
                if (recording == true)
                {
                    blue_data.Add(Convert.ToInt32(green));
                }
            }
        }
        public string VIOLET
        {
            get { return violet; }
            set
            {
                violet = value;
                //Console.WriteLine("Voilet Spectrum is " + voilet);
                if (recording == true)
                {
                    violet_data.Add(Convert.ToInt32(violet));
                }
            }
        }
        public string YELLOW
        {
            get { return yellow; }
            set
            {
                yellow = value;
                //Console.WriteLine("Yellow Spectrum is " + yellow);
                if (recording == true)
                {
                    yellow_data.Add(Convert.ToInt32(yellow));
                }
            }
        }
        public string ORANGE
        {
            get { return orange; }
            set
            {
                orange = value;
                //Console.WriteLine("Orange Spectrum is " + orange);
                if (recording == true)
                {
                    orange_data.Add(Convert.ToInt32(orange));
                }
            }
        }

        // Methods about Data Recording
        public string LABEL3
        {
            get { return Label3; }
            set
            {
                Label3 = value;
                Label_Running = Label3;
                Console.WriteLine("Label was set to be " + Label3 + "!");
            }
        }

        internal void start_recording()
        {
            recording = true;
            RECORD_STATE = "Recording " + Label_Running;
        }
        internal void stop_recording()
        {
            recording = false;
            RECORD_STATE = "Not Recording";
   
            int[,] signals = Datahandler();
            Savedata(signals);
            Cleardata();
        }

        private int[,] Datahandler()
        {
            int[,] signal = Combine((int[])year_data.ToArray(typeof(int)), (int[])month_data.ToArray(typeof(int)),
                                    (int[])day_data.ToArray(typeof(int)), (int[])hour_data.ToArray(typeof(int)),
                                    (int[])minute_data.ToArray(typeof(int)), (int[])second_data.ToArray(typeof(int)),
                                    (int[])temperature_data.ToArray(typeof(int)), (int[])pressure_data.ToArray(typeof(int)),
                                    (int[])humidity_data.ToArray(typeof(int)), (int[])voc_data.ToArray(typeof(int)),
                                    (int[])altitude_data.ToArray(typeof(int)), (int[])co2_data.ToArray(typeof(int)),
                                    (int[])light_data.ToArray(typeof(int)), (int[])rotate_data.ToArray(typeof(int)),
                                    (int[])red_data.ToArray(typeof(int)), (int[])green_data.ToArray(typeof(int)),
                                    (int[])blue_data.ToArray(typeof(int)), (int[])violet_data.ToArray(typeof(int)),
                                    (int[])orange_data.ToArray(typeof(int)), (int[])yellow_data.ToArray(typeof(int)));
            return signal;
        }

        private int[,] Combine(int[] a1, int[] a2, int[] a3, int[] a4, int[] a5,
                                int[] a6, int[] a7, int[] a8, int[] a9, int[] a10,
                                int[] a11, int[] a12, int[] a13, int[] a14, int[] a15,
                                int[] a16, int[] a17, int[] a18, int[] a19, int[] a20)
        {
            if (a1.Length >= 2)
            {
                int[,] signal = new int[a1.Length - 1, 20];
                for (int i = 1; i < a1.Length - 1; i += 1)
                {
                    signal[i, 0] = a1[i];
                    signal[i, 1] = a2[i];
                    signal[i, 2] = a3[i];
                    signal[i, 3] = a4[i];
                    signal[i, 4] = a5[i];
                    signal[i, 5] = a6[i];
                    signal[i, 6] = a7[i];
                    signal[i, 7] = a8[i];
                    signal[i, 8] = a9[i];
                    signal[i, 9] = a10[i];
                    signal[i, 10] = a11[i];
                    signal[i, 11] = a12[i];
                    signal[i, 12] = a13[i];
                    signal[i, 13] = a14[i];
                    signal[i, 14] = a15[i];
                    signal[i, 15] = a16[i];
                    signal[i, 16] = a17[i];
                    signal[i, 17] = a18[i];
                    signal[i, 18] = a19[i];
                    signal[i, 19] = a20[i];
                }
                return signal;
            }
            else
            {
                return null;
            }
            
        }

        public string RECORD_STATE
        {
            get { return record_state; }
            set
            {
                record_state = value;
                OnPropertyChanged();
            }
        }

        private void Savedata(int[,] signals)
        {
            string textname = Label_Running + time + ".txt";
            string file = System.IO.Path.Combine(FileSystem.CacheDirectory, textname);
            string data_to_be_wrote = "";
            if (signals != null)
            {
                for (int i = 0; i < signals.GetLength(0); i += 1)
                {
                    for (int j = 0; j < 20; j += 1)
                    {
                        data_to_be_wrote += signals[i, j].ToString();
                        data_to_be_wrote += "\t";
                    }
                    data_to_be_wrote += "\n";
                }

            }
            System.IO.File.WriteAllText(file, data_to_be_wrote);
            Share.RequestAsync(new ShareFileRequest
            {
                Title = "title",
                File = new ShareFile(file)
            });
        }

        private void Cleardata()
        {
            year_data.Clear();
            month_data.Clear();
            day_data.Clear();
            hour_data.Clear();
            minute_data.Clear();
            second_data.Clear();
            temperature_data.Clear();
            pressure_data.Clear();
            humidity_data.Clear();
            voc_data.Clear();
            altitude_data.Clear();
            co2_data.Clear();
            light_data.Clear();
            rotate_data.Clear();
            red_data.Clear();
            green_data.Clear();
            blue_data.Clear();
            violet_data.Clear();
            orange_data.Clear();
            yellow_data.Clear();
        }
    }
}
