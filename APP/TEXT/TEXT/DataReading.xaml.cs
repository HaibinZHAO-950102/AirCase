using System;
using System.Collections.Generic;
using Plugin.BLE.Abstractions.Contracts;
using Xamarin.Forms;

namespace TEXT
{
    public partial class DataReading : ContentPage
    {
        public DataViewModel dataviewmodel;

        IBluetoothLE ble;
        IAdapter adapter;
        IDevice device;
        ICharacteristic Characteristic1, Characteristic2, Characteristic3, Characteristic4, Characteristic5, Characteristic6, Characteristic7,
            Characteristic8, Characteristic9, Characteristic10, Characteristic11, Characteristic12, Characteristic13, Characteristic14;

        public DataReading(DataViewModel m)
        {
            InitializeComponent();
            BindingContext = m;
            this.dataviewmodel = m;
            device = dataviewmodel.GetDevice();
            ble = dataviewmodel.GetBLE();
            adapter = dataviewmodel.GetAdapter();
        }

        async void Button_Clicked_2(object sender, EventArgs e)
        {
            await Navigation.PopModalAsync();
        }

        async void Button_Clicked_3(object sender, EventArgs e)
        {
            var datarecording= new DataRecording(dataviewmodel);
            await Navigation.PushModalAsync(datarecording);
        }

        async void Button_Clicked(object sender, EventArgs e)
        {
            if (device == null)
            {
                await DisplayAlert("No Bluetooth", "No Bluetooth connected, " +
                    "please connect BLE first.", "OK");
                await Navigation.PopModalAsync();
            }

            await adapter.ConnectToDeviceAsync(device);
            var services = await device.GetServicesAsync();
            foreach (var service in services)
            {
                var characteristics = await service.GetCharacteristicsAsync();

                foreach (var characteristic in characteristics)
                {
                    Console.WriteLine(service.Id + " - " + characteristic.Id);

                    if (characteristic.Id == Guid.Parse(dataviewmodel.Characteristic_UUID_1))
                    {
                        Characteristic1 = characteristic;
                        Console.WriteLine("Find Characteristic 1.");
                    }
                    if (characteristic.Id == Guid.Parse(dataviewmodel.Characteristic_UUID_2))
                    {
                        Characteristic2 = characteristic;
                        Console.WriteLine("Find Characteristic 2.");
                    }
                    if (characteristic.Id == Guid.Parse(dataviewmodel.Characteristic_UUID_3))
                    {
                        Characteristic3 = characteristic;
                        Console.WriteLine("Find Characteristic 3.");
                    }
                    if (characteristic.Id == Guid.Parse(dataviewmodel.Characteristic_UUID_4))
                    {
                        Characteristic4 = characteristic;
                        Console.WriteLine("Find Characteristic 4.");
                    }
                    if (characteristic.Id == Guid.Parse(dataviewmodel.Characteristic_UUID_5))
                    {
                        Characteristic5 = characteristic;
                        Console.WriteLine("Find Characteristic 5.");
                    }
                    if (characteristic.Id == Guid.Parse(dataviewmodel.Characteristic_UUID_6))
                    {
                        Characteristic6 = characteristic;
                        Console.WriteLine("Find Characteristic 6.");
                    }
                    if (characteristic.Id == Guid.Parse(dataviewmodel.Characteristic_UUID_7))
                    {
                        Characteristic7 = characteristic;
                        Console.WriteLine("Find Characteristic 7.");
                    }
                }
            }

            Characteristic1.ValueUpdated += (o, args) =>
            {
                var bytes = args.Characteristic.Value;
                string message = System.Text.Encoding.ASCII.GetString(bytes);
                dataviewmodel.TEMPERATURE = dataviewmodel.messagespliter(message, 0);
                dataviewmodel.PRESSURE = dataviewmodel.messagespliter(message, 1);
            };
            await Characteristic1.StartUpdatesAsync();

            Characteristic2.ValueUpdated += (o, args) =>
            {
                var bytes = args.Characteristic.Value;
                string message = System.Text.Encoding.ASCII.GetString(bytes);
                dataviewmodel.HUMIDITY = dataviewmodel.messagespliter(message, 0);
                dataviewmodel.VOC = dataviewmodel.messagespliter(message, 1);
            };
            await Characteristic2.StartUpdatesAsync();

            Characteristic3.ValueUpdated += (o, args) =>
            {
                var bytes = args.Characteristic.Value;
                string message = System.Text.Encoding.ASCII.GetString(bytes);
                dataviewmodel.ALTITUDE = dataviewmodel.messagespliter(message, 0);
                dataviewmodel.CO2 = dataviewmodel.messagespliter(message, 1);
            };
            await Characteristic3.StartUpdatesAsync();

            Characteristic4.ValueUpdated += (o, args) =>
            {
                var bytes = args.Characteristic.Value;
                string message = System.Text.Encoding.ASCII.GetString(bytes);
                dataviewmodel.LIGHT = dataviewmodel.messagespliter(message, 0);
                dataviewmodel.ROTATE = dataviewmodel.messagespliter(message, 1);
            };
            await Characteristic4.StartUpdatesAsync();

            Characteristic5.ValueUpdated += (o, args) =>
            {
                var bytes = args.Characteristic.Value;
                string message = System.Text.Encoding.ASCII.GetString(bytes);
                dataviewmodel.RED = dataviewmodel.messagespliter(message, 0);
                dataviewmodel.GREEN = dataviewmodel.messagespliter(message, 1);
            };
            await Characteristic5.StartUpdatesAsync();

            Characteristic6.ValueUpdated += (o, args) =>
            {
                var bytes = args.Characteristic.Value;
                string message = System.Text.Encoding.ASCII.GetString(bytes);
                dataviewmodel.BLUE = dataviewmodel.messagespliter(message, 0);
                dataviewmodel.VIOLET = dataviewmodel.messagespliter(message, 1);
            };
            await Characteristic6.StartUpdatesAsync();

            Characteristic7.ValueUpdated += (o, args) =>
            {
                var bytes = args.Characteristic.Value;
                string message = System.Text.Encoding.ASCII.GetString(bytes);
                dataviewmodel.YELLOW = dataviewmodel.messagespliter(message, 1);
                dataviewmodel.ORANGE = dataviewmodel.messagespliter(message, 0);
            };
            await Characteristic7.StartUpdatesAsync();
        }
    }
}
