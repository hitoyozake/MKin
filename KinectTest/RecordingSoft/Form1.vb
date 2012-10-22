Imports System.Diagnostics

Public Class Form1
    Dim process As Process = Nothing
    Dim lastMinute As Integer
    Dim started As Boolean = False

    Sub InitProcess()
        process = New Process()
        process.StartInfo.FileName = "recorder.exe"
        process.StartInfo.UseShellExecute = False
        process.StartInfo.CreateNoWindow = True
        process.StartInfo.RedirectStandardInput = True
        process.StartInfo.RedirectStandardOutput = False
    End Sub

    Function GenerateFileName() As String
        Dim year As String = System.DateTime.Now.Year.ToString("0000")
        Dim month As String = System.DateTime.Now.Month.ToString("00")
        Dim day As String = System.DateTime.Now.Day.ToString("00")
        Dim hour As String = System.DateTime.Now.Hour.ToString("00")
        Dim minute As String = System.DateTime.Now.Minute.ToString("00")

        Return year + "_" + month + "_" + day + "_" + hour + "_" + minute
    End Function

    Private Sub Form1_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        RadioButton1.Checked = True
        RadioButton3.Checked = True
    End Sub

    Sub CloseProcess()
        If process IsNot Nothing Then
            process.StandardInput.WriteLine("end")
            process.WaitForExit()
        End If

    End Sub


    Private Sub Button1_Click(sender As Object, e As EventArgs) Handles Button1.Click
        
        If started = False Then
            Dim minutes(5) As Integer
            minutes(0) = 0
            minutes(1) = 10
            minutes(2) = 30
            minutes(3) = 60
            minutes(4) = 180

            Dim time As Integer = 0

            If RadioButton3.Checked = True Then
                time = minutes(0)
            End If
            If RadioButton4.Checked = True Then
                time = minutes(1)
            End If

            If RadioButton5.Checked = True Then
                time = minutes(2)
            End If
            If RadioButton6.Checked = True Then
                time = minutes(3)
            End If
            If RadioButton7.Checked = True Then
                time = minutes(4)
            End If

            If time > 0 Then
                Timer1.Interval = time * 1000 * 60
                Timer1.Enabled = True
                Timer1.Start()
                Timer2.Enabled = True
                Timer2.Start()

                Label3.Text = time
                lastMinute = time
            End If
            InitProcess()
            process.Start()

            'ファイルの保存
            '日時・メモの保存
            Button1.Text = "停止・終了"
            started = True
        Else
            '終了イベント
            ToolStripStatusLabel1.Text = "Closing. Please Wait...."
            Close()
        End If

    End Sub

    Private Sub Form1_FormClosing(sender As Object, e As FormClosingEventArgs) Handles MyBase.FormClosing
        StatusStrip1.Text = "Closing. Please Wait........."
        CloseProcess()

    End Sub

    Private Sub RichTextBox1_TextChanged(sender As Object, e As EventArgs) Handles RichTextBox1.TextChanged

    End Sub

    Private Sub Button2_Click(sender As Object, e As EventArgs) Handles Button2.Click
        '終了イベント
        ToolStripStatusLabel1.Text = "Closing. Please Wait...."
        Close()
    End Sub

    Private Sub 終了ToolStripMenuItem_Click(sender As Object, e As EventArgs) Handles 終了ToolStripMenuItem.Click
        '終了イベント
        ToolStripStatusLabel1.Text = "Closing. Please Wait...."
        Close()
    End Sub

    Private Sub Timer1_Tick(sender As Object, e As EventArgs) Handles Timer1.Tick
        '終了イベント
        ToolStripStatusLabel1.Text = "Closing. Please Wait...."
        Close()
    End Sub

    Private Sub Timer2_Tick(sender As Object, e As EventArgs) Handles Timer2.Tick
        lastMinute -= 1
        If lastMinute < 0 Then
            lastMinute = 0
            Timer2.Enabled = False
        End If
        Label3.Text = lastMinute
    End Sub
End Class
