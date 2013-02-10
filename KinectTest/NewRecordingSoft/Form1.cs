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

namespace NewRecordingSoft
{
    public partial class Form1 : Form
    {
        public Boolean started = false;
        public System.IO.StreamWriter debugLog = null;

        public Process process = null;
        public int timerCounter = 0;

        public String filename = "recorder.exe";

        public Form1()
        {
            InitializeComponent();
        }

        private void InitProcess()
        {
            process = new Process();
            process.StartInfo.FileName = filename;
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.CreateNoWindow = true;
            process.StartInfo.RedirectStandardError = true;
            process.StartInfo.RedirectStandardOutput = false;
        }

        private void CloseProcess()
        {
            if( process != null )
            {
                process.StandardInput.WriteLine( "end" );
                //タイマーでExitを待ち続ける
                timer1.Enabled = true;
            }
        }

        private String GenerateFileName()
        {
            var year = System.DateTime.Now.Year.ToString( "0000" );
            var month = System.DateTime.Now.Month.ToString("00");
            var day = System.DateTime.Now.Day.ToString("00");
            var hour = System.DateTime.Now.Hour.ToString("00");
            var minute = System.DateTime.Now.Minute.ToString("00");

            return year + "_" + month + "_" + day + "_" + hour + "_" + minute;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (started)
            {
                button1.Text = "START RECORDING";
                started = false;
                button1.Enabled = false;

                CloseProcess();
            }
            else
            {
                if (System.IO.File.Exists(filename))
                {
                    button1.Enabled = false;
                    button1.Text = "STOP RECORDING";

                    InitProcess();
                    process.Start();

                    started = true;

                    button1.Enabled = true;
                    //撮影開始まで待機
                }
                else
                {
                    MessageBox.Show("計測ソフトが存在しません(recorder.exe does not exist");
                    debugLog.WriteLine("REDORDER.EXE DOES NOT EXIST");
                }

            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            debugLog = new System.IO.StreamWriter("debugLog.txt");

            //現在時刻を書き込む
            debugLog.WriteLine(DateTime.Now);
            debugLog.WriteLine("******************************");

        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (process != null)
            {
                if (process.HasExited)
                {
                    started = false;
                    button1.Enabled = true;
                    timerCounter = 0;
                }
                else if (timerCounter > 1000)
                {
                    //何らかの理由で終了できない
                    process.Kill(); //本当はよろしくない
                }
                ++timerCounter;
            }
            else
            {
                started = false;
                button1.Enabled = true;
                timerCounter = 0;
            }
        }
    }
}
