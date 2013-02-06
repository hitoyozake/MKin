using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NewRecordingSoft
{
    public partial class Form1 : Form
    {
        public Boolean started_ = false;

        public Form1()
        {
            InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (started_)
            {
                button1.Text = "START RECORDING";
                started_ = false;
            }
            else
            {
                button1.Text = "STOP RECORDING";
                button1.Enabled = false;
                started_ = true;

                //撮影開始まで待機
            }
        }
    }
}
