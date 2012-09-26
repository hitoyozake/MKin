Public Class Form1

    '録画ソフト起動用プロセス
    Public process As Process = Nothing
    Public standardInput As String
    Public standardOutput As String

    Private Sub InitInstance()

        process = New Process()
        process.StartInfo.UseShellExecute = False
        process.StartInfo.FileName = "hoge.exe"


    End Sub


    Private Sub NewInstance()

        If process Is Nothing Then
        End If

    End Sub



    Private Sub Form1_Load(sender As System.Object, e As System.EventArgs) Handles MyBase.Load

        NewInstance()

    End Sub
End Class
