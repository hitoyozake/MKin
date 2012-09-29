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

namespace motorGUI
{
    public partial class Form1 : Form
    {
        //process
        Process process = null;

        //param
        int[] angle = null;
        int kinect_num = 3;

        int max_angle = 30;
        int min_angle = -30;


        public Form1()
        {
            InitializeComponent();
        }
        void SendCloseMessage()
        {
            process.StandardInput.WriteLine("end");
        }


        void SendMessage( int index, int angle )
        {
            String mes = "";

            if (index == -1)
            {
                mes += "all";
            }
            else
            {
                mes += index.ToString();
            }

            mes += ",";

            mes += angle.ToString();

            process.StandardInput.WriteLine(mes);

        }

        void UpdateCurrentAngleLabel()
        {
            labelCurrentAngle.Text = angle[listBox1.SelectedIndex].ToString();
        }

        private void Form1_Load(object sender, EventArgs e)
        {

            angle = new int[kinect_num];

            for (int i = 0; i < kinect_num; ++i)
                angle[i] = 0;
            listBox1.SelectedIndex = 0;

            process = new Process();
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.RedirectStandardInput = true;
            //process.StartInfo.RedirectStandardOutput = true;
 
            process.StartInfo.FileName = @"motor_setting.exe";

            process.Start();
            comboBox1.BeginUpdate();
            for (int i = min_angle; i <= max_angle; ++i)
            {
                comboBox1.Items.Add(i.ToString());
            }
            comboBox1.EndUpdate();

        }

        private void buttonUp_Click(object sender, EventArgs e)
        {
            angle[listBox1.SelectedIndex] += 5;
            angle[listBox1.SelectedIndex] = Math.Max( min_angle, Math.Min( max_angle,
                angle[listBox1.SelectedIndex] ) );
            SendMessage(listBox1.SelectedIndex, angle[listBox1.SelectedIndex]);

            UpdateCurrentAngleLabel();
        }

        private void buttonEnd_Click(object sender, EventArgs e)
        {
            SendCloseMessage();
            process.WaitForExit();

            Close();
        }

        private void buttonDown_Click(object sender, EventArgs e)
        {
            angle[listBox1.SelectedIndex] += -5;
            angle[listBox1.SelectedIndex] = Math.Max(min_angle, Math.Min(max_angle,
                angle[listBox1.SelectedIndex]));
            SendMessage(listBox1.SelectedIndex, angle[listBox1.SelectedIndex]);

            UpdateCurrentAngleLabel();
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
            
        }

        //角度設定ボタン
        private void button1_Click(object sender, EventArgs e)
        {
            int num;
            if (int.TryParse(comboBox1.Text, out num))
            {
                num = Math.Max(min_angle, Math.Min(max_angle, num));
                comboBox1.Text = num.ToString();
            }
            else
            {
                comboBox1.Text = "0";
                num = 0;
            }

            angle[listBox1.SelectedIndex] = num;
            SendMessage(listBox1.SelectedIndex, angle[listBox1.SelectedIndex]);

            UpdateCurrentAngleLabel();
        }

        private void listBox1_SelectedValueChanged(object sender, EventArgs e)
        {
            UpdateCurrentAngleLabel();
        }
    }
}
