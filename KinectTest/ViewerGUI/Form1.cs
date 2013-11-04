using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Diagnostics;

namespace ViewerGUI
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load( object sender , EventArgs e )
        {
            comboBox1.SelectedIndex = 0;
            comboBox2.SelectedIndex = 0;
            comboBox3.SelectedIndex = 0;

        }

        void prepare_process()
        {
            viewer = new Process();
            string current_path = System.IO.Directory.GetCurrentDirectory();
            viewer.StartInfo.FileName = @"" + current_path + "\\PCLViewer.exe";
            viewer.StartInfo.RedirectStandardInput = true;
            viewer.StartInfo.RedirectStandardOutput = true;
            viewer.StartInfo.UseShellExecute = false;
            viewer.Start();
        }

        String[] axis = { "x" , "y" , "z" };
        String[] operation = { "rotate" , "move" };
        Process viewer = null;


        void send_command_end()
        {
            viewer.StandardInput.WriteLine( "ie" );
            
        }

        void send_cmd( String num, String operation, String axis, String value )
        {
            viewer.StandardInput.WriteLine( "cmd" );

            viewer.StandardInput.WriteLine( num ); //画像番号

            viewer.StandardInput.WriteLine( operation ); //コマンド

            viewer.StandardInput.WriteLine( axis ); //軸
            viewer.StandardInput.WriteLine( value );

            richTextBox1.Text += num + "," + operation + "," + axis + ","
                + value  + "\n";
        }

        private void button1_Click( object sender , EventArgs e )
        {
            //起動確認
            if ( viewer == null )
            {
                prepare_process();
            }

            button1.Enabled = false;

            if ( checkBox1.Checked )
            {
                while ( true )
                {
                    if ( richTextBox2.Lines.Length == 0 )
                    {
                        send_command_end();
                        button1.Enabled = true;
                        return;                        
                    }
                    //,区切りで格納
                    String[] param = richTextBox2.Lines[0].Split( ',' );　//ボタンの有効化

                    send_cmd( param[0] , param[1] , param[2] , param[3] );

                    String[] strs = new String[richTextBox2.Lines.Length - 1];
                    for ( int i = 1; i < richTextBox2.Lines.Length; ++i )
                    {
                        strs[i - 1] = richTextBox2.Lines[i];
                    }

                    richTextBox2.Clear();
                    for ( int i = 0; i < strs.Length; ++i )
                        richTextBox2.Text += strs[i] + "\n";

                    while ( true )
                    {
                        var output = viewer.StandardOutput.ReadLine();

                        richTextBox3.Text += output + "\n";

                        if ( output == "nf" )
                        {
                            break;
                        }
                    }
                }
            }
            else
            {
                send_cmd( comboBox1.SelectedItem.ToString() , comboBox2.SelectedItem.ToString() ,
                comboBox3.SelectedItem.ToString() , numericUpDown1.Value.ToString() );

                while ( true )
                {
                    var output = viewer.StandardOutput.ReadLine();

                    richTextBox3.Text += output + "\n";

                    if ( output == "nf" )
                    {
                        break;
                    }
                }
                button1.Enabled = true; //ボタンの有効化
            }

        }

        private void button2_Click( object sender , EventArgs e )
        {
            if ( viewer == null )
            {
                prepare_process();
            }

            button2.Enabled = false;

            viewer.StandardInput.WriteLine( "n" );

            button2.Enabled = true;
        }

        private void comboBox2_SelectedIndexChanged( object sender , EventArgs e )
        {

        }

        private void button3_Click( object sender , EventArgs e )
        {
            send_command_end();
        }
    }
}
