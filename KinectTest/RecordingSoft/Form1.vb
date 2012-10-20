Imports System.Diagnostics

Public Class Form1
    Dim process As Process = Nothing

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
        MessageBox.Show(GenerateFileName())

    End Sub

    Sub CloseProcess()
        If process IsNot Nothing Then
            process.StandardInput.WriteLine("end")
            process.WaitForExit()
        End If

    End Sub


    Private Sub Button1_Click(sender As Object, e As EventArgs) Handles Button1.Click

        If started = False Then
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
End Class
