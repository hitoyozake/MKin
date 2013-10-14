using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using Microsoft.Kinect;


namespace PoseEstimationStudio.Kinect
{
    class Range
    {
        public int width;
        public int height;

    };

    class Runtime
    {
        BinaryReader depth;
        BinaryReader color;
        BinaryReader backGround;

        public Range range = new Range();
    }

    class KinectManager
    {
        public int kinectNum = 0;
        public BinaryReader [] depthReader = null;
        public BinaryReader [] colorReader = null;
        public String filename;
        public Runtime[] kinects;

        KinectSensor device = KinectSensor.KinectSensors[ 0 ];


        public KinectManager()
        {
            device.Start();
        }

        public void TransformDepthTo3D( Byte [] data )
        {
            CoordinateMapper mapper = new CoordinateMapper( device );
            DepthImagePoint  pixels = new DepthImagePoint();
            SkeletonPoint [] points = new SkeletonPoint[ 320 * 120 ];
            var p = mapper.MapDepthPointToSkeletonPoint( DepthImageFormat.Resolution640x480Fps30,
                pixels );
        }

        public void OpenRangeFile()
        {
            var reader = new StreamReader( filename );

            this.kinectNum = int.Parse( reader.ReadLine() );

            kinects = new Runtime[ kinectNum ];

            for( int i = 0; i < kinectNum; ++i )
            {
                kinects[ i ] = new Runtime();
                var range = kinects[ i ].range;
                String str = reader.ReadLine();
               // range.x = int.Parse( str.Substring( str.IndexOf( ":" ) ) ); //使わない(ReadLineは必要)
                str = reader.ReadLine();
               // range.y = int.Parse( str.Substring( str.IndexOf( ":" ) ) ); //使わない
                str = reader.ReadLine();
                range.width = int.Parse( str.Substring( str.IndexOf( ":" ) ) );
                str = reader.ReadLine();
                range.height = int.Parse( str.Substring( str.IndexOf( ":" ) ) );
            }
        }

        public Byte[] ReadDepth( BinaryReader data, int width, int height )
        {
            return data.ReadBytes( width * height * 2 );
        }

        public void ReadColor( Color [] bufferArray, BinaryReader data, int width, int height )
        {
            var readData = data.ReadBytes( width * height * 4 );

            for( int y = 0; y < height; ++y )
            { 
                for( int x = 0; x < width; ++x )
                {
                    bufferArray[ x + y * width ].B = readData[ 4 * ( x + y * width ) ];
                    bufferArray[ x + y * width ].G = readData[ 4 * ( x + y * width ) + 1 ];
                    bufferArray[ x + y * width ].R = readData[ 4 * ( x + y * width ) + 2 ];
                }
            }
        }

        public Boolean OpenKinect( String filename, int num )
        {
            depthReader = new BinaryReader[ num ];
            colorReader = new BinaryReader[ num ];

            try
            {
                for( int i = 0; i < num; ++i )
                {
                    depthReader[ i ] = new BinaryReader( File.OpenRead( filename + num.ToString() + ".txt" ) );
                }
            }
            catch( Exception e )
            {
                Console.WriteLine( e.Message );
                return false;
            }
            return true;
        }
    }
}
