Imports System.Diagnostics

Public Class Form1
    Dim process As Process = Nothing

    Dim started As Boolean = False

    Private Sub Form1_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        process = New Process()
        process.StartInfo.FileName = "recorder.exe"
        process.StartInfo.UseShellExecute = False
        process.StartInfo.CreateNoWindow = True
        process.StartInfo.RedirectStandardInput = True
        process.StartInfo.RedirectStandardOutput = False

    End Sub

    Sub CloseProcess()
        process.StandardInput.WriteLine("end")
        process.WaitForExit()
    End Sub


    Private Sub Button1_Click(sender As Object, e As EventArgs) Handles Button1.Click

        If started = False Then
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
End Class
