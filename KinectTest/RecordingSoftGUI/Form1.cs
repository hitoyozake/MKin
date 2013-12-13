using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RecordingSoftGUI
{
    public partial class Form1 : Form
    {
        private Process area_selector = null;
        private Process recorder = null;
        private Boolean record_started = false;
        private int minutes = 0;
        private int seconds = 0;


        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load( object sender , EventArgs e )
        {
            button1.Enabled = false;
            button2.Enabled = false;
            button2.Visible = false;
            //button1.Enabled = false;

            if ( recorder == null )
            {
                recorder = new Process();
                recorder.StartInfo.CreateNoWindow = false;
                recorder.StartInfo.RedirectStandardInput = true;
                recorder.StartInfo.UseShellExecute = false;
                //area_selector.EnableRaisingEvents = true;
                recorder.StartInfo.FileName = @"./KinectTest.exe";
                //Exitした時の処理を追加
                recorder.Exited += recorder_exited;
            }
            recorder.Start();
            String areaFilename = GetAreaFileName();
            System.Threading.Thread.Sleep( 100 );
            recorder.StandardInput.WriteLine( areaFilename );

            while ( true )
            {
                var str = recorder.StandardOutput.ReadLine();

                if ( str.IndexOf( "ready" ) >= 0 )
                {
                    break;
                }
            }

            button1.Enabled = true;

        }

        public String GetAreaFileName()
        {
            String[] files = System.IO.Directory.GetFiles( System.IO.Directory.GetCurrentDirectory() , "area*.txt" );

            List<String> areaFiles = new List<String>();

            foreach( var file in files )
            {
                areaFiles.Add( file );
            }

            areaFiles.Sort( delegate( String f1, String f2 ){ return DateTime.Compare( System.IO.Directory.GetCreationTime( f1 ),
            System.IO.Directory.GetCreationTime( f2 ) ); } );

            areaFiles.Reverse();

            if ( areaFiles.Count > 0 )
                return areaFiles[0];
            else
                return "NULL";
        }


        //Recordボタン
        private void button1_Click( object sender , EventArgs e )
        {
            if ( record_started == false )
            {
                button2.Enabled = false;
                record_started = true;

                button1.Text = "Stop";

                recorder.StandardInput.WriteLine( "start" );
                timer1.Start(); //タイマー開始

            }
            else
            {
                timer1.Stop();
                seconds = 0;
                minutes = 0;
                recorder.StandardInput.WriteLine( "end" );
                recorder.WaitForExit();
                Close();                
                //終了処理
            }
        }

        //範囲選択ボタン
        private void button2_Click( object sender , EventArgs e )
        {
            button1.Enabled = false;
            button2.Enabled = false;
            if( area_selector == null )
            {
                area_selector = new Process();
                area_selector.StartInfo.RedirectStandardInput = true;
                area_selector.StartInfo.UseShellExecute = false;
                area_selector.EnableRaisingEvents = true;
                area_selector.StartInfo.FileName = @"./RecordRangeSelector.exe";
                //Exitした時の処理を追加
                area_selector.Exited += area_selector_exited;
            }
            area_selector.Start();
        }

        void recorder_exited( object sender, EventArgs e )
        {
            //recorderが終了するとソフトも閉じる
        }

        
        void area_selector_exited( object sender, EventArgs e )
        {
            area_selector = null;
            button1.Enabled = true;
            button2.Enabled = true;
        }

        private void timer1_Tick( object sender , EventArgs e )
        {
            ++seconds;

            if( seconds > 0 && seconds % 60 == 0 )
            {
                ++minutes;
            }

            //描画更新
            label2.Text = minutes.ToString( "D2" ) + "："
                + ( seconds % 60 ).ToString( "D2" );

        }
    }
}
