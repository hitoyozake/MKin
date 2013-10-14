using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PoseEstimationStudio
{
    class ImageLoader
    {
        private System.IO.StreamReader [] depthStreams = null;
        private System.IO.StreamReader rangeFile = null;
        private System.IO.StreamReader[] backStreams = null;

        private int OpenRangeFile( String filename )
        {
            try
            {
                rangeFile = new System.IO.StreamReader( filename );
            }
            catch( Exception e )
            {
                Console.WriteLine( e.Message );
                rangeFile = null;
                return -1;
            }

            int kinectNum = 0;

            kinectNum = int.Parse( rangeFile.ReadLine() );
            return kinectNum;
      
        }


    }
}
