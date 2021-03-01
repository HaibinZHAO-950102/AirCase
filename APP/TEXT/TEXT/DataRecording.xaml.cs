using System;
using System.Collections.Generic;
using Plugin.BLE.Abstractions.Contracts;
using Xamarin.Forms;

namespace TEXT
{
    public partial class DataRecording : ContentPage
    {
        public DataViewModel dataviewmodel;

        IBluetoothLE ble;
        IAdapter adapter;
        IDevice device;

        public string Label3 = "Default";

        public DataRecording(DataViewModel m)
        {
            InitializeComponent();
            this.dataviewmodel = m;
            BindingContext = m;
            device = dataviewmodel.GetDevice();
            ble = dataviewmodel.GetBLE();
            adapter = dataviewmodel.GetAdapter();
        }

        async void Button_Clicked_2(object sender, EventArgs e)
        {
            await Navigation.PopModalAsync();
        }


        void Entry_Completed(System.Object sender, System.EventArgs e)
        {
            var entry = sender as Entry;
            Label3 = entry.Text;
            dataviewmodel.LABEL3 = Label3;
        }

        void Button_Clicked(System.Object sender, System.EventArgs e)
        {
            if (!dataviewmodel.recording)
            {
                dataviewmodel.Label_Running = "Inside Pocket";
                dataviewmodel.start_recording();
            }
        }

        void Button_Clicked_1(System.Object sender, System.EventArgs e)
        {
            dataviewmodel.stop_recording();
        }

        void Button_Clicked_3(System.Object sender, System.EventArgs e)
        {
            if (!dataviewmodel.recording)
            {
                dataviewmodel.Label_Running = "Outside Pocket";
                dataviewmodel.start_recording();
            }

        }

        void Button_Clicked_4(System.Object sender, System.EventArgs e)
        {
            dataviewmodel.stop_recording();
        }

        void Button_Clicked_5(System.Object sender, System.EventArgs e)
        {
            if (!dataviewmodel.recording)
            {
                dataviewmodel.start_recording();
            }

        }

        void Button_Clicked_6(System.Object sender, System.EventArgs e)
        {
            dataviewmodel.stop_recording();
        }
    }
}
