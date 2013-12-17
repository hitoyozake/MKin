using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;

namespace BoneEstimation
{


    public class PoseEvaluater
    {

        Color[] pixels1 = new Color[ 640 * 480 ];

        Color[] pixels2 = new Color[ 640 * 480 ];

        Color[] pixelsForSave = new Color[ 640 * 480 ];

        Color XNABackColor = new Color( 68, 34, 136 );


        private void InitializePixelsForFile()
        {
            for( int i = 0; i < 640 * 480; ++i )
            {
                pixelsForSave[ i ].R = 0;
                pixelsForSave[ i ].G = 0;
                pixelsForSave[ i ].B = 0;
                pixelsForSave[ i ].A = 255;
            }
        }

        public float evaluate( Texture2D texture1, Texture2D texture2 )
        {
            int sum = 0;
            int prod = 0;
            int allSum = 0;
            double lmd = 0.4;//0.1;

            double depthEvaluation = 0.0;

            texture1.GetData( pixels1 );
            texture2.GetData( pixels2 );


            InitializePixelsForFile();

            for( int i = 0; i < pixels1.Length; ++i )
            {
                //両方共背景色ではない AND
                if( pixels1[ i ] != XNABackColor && pixels2[ i ] != XNABackColor )
                {
                    ++prod;
                    //一致してたら計算
                    depthEvaluation += Math.Min( 40.0, Math.Abs( pixels1[ i ].R - pixels2[ i ].R ) );

                    var hoge = ( byte )Math.Min( 190, 255 - Math.Abs( pixels1[ i ].R - pixels2[ i ].R ) * 14 );
                    pixelsForSave[ i ].R = hoge;
                }

                //どちらか一方は存在する(重なっていても良い) OR
                if( pixels1[ i ] != XNABackColor || pixels2[ i ] != XNABackColor )
                {
                    ++sum;
                    //この部分少し修正必要
                    //    depthEval += ( float )255.0;

                    if( pixels1[ i ] != XNABackColor )
                    {
                        ++allSum;
                    }

                    if( pixels2[ i ] != XNABackColor )
                    {
                        ++allSum;
                    }


                    if( pixels1[ i ] == XNABackColor && pixels2[ i ] != XNABackColor )
                    {
                        pixelsForSave[ i ].G = 255;
                        depthEvaluation += 50.0f;
                    }

                    if( pixels1[ i ] != XNABackColor && pixels2[ i ] == XNABackColor )
                    {
                        pixelsForSave[ i ].B = 255;
                        depthEvaluation += 50.0f;
                    }
                }
            }

            double silhouetteValue = 1.0 - ( ( 2.0 * prod ) / ( sum + prod ) );
            double depthValue = lmd * depthEvaluation / ( allSum + 0.0000001 );
            double result =  silhouetteValue + depthValue;

            return ( float )result;
        }

        float GetCoveringPercentage( Texture2D modelRendered, Texture2D depth )
        {
            int depthThreshold = 12;

            modelRendered.GetData( pixels1 );
            depth.GetData( pixels2 );

            int counter = 0;
            int sum = 0;

            for( int i = 0; i < pixels1.Length; ++i )
            {
                if( pixels1[ i ] != XNABackColor )
                {
                    if( pixels2[ i ] != XNABackColor )
                    {
                        if( Math.Abs( pixels1[ i ].R - pixels2[ i ].R ) <= depthThreshold )
                        {
                            ++counter;
                        }
                    }
                    ++sum;
                }
            }
            float result = ( float )counter / ( sum + 0.0001f );
            return result;
        }

        public void MargeImage( Color [] image1, Color [] image2, Color [] output )
        {
            for( int i = 0; i < image1.Length; ++i )
            {
                if( image1[ i ] != XNABackColor && image2[ i ] != XNABackColor )
                {
                    output[ i ].R = Math.Max( image1[ i ].R, image2[ i ].R );
                }
                else
                {
                    if( image1[ i ] != XNABackColor )
                    {
                        output[ i ].R = image1[ i ].R;
                    }
                    else if( image2[ i ] != XNABackColor )
                    {
                        output[ i ].R = image2[ i ].R;
                    }
                    else
                    {
                        output[ i ] = XNABackColor;
                    }
                }
            }
        }


    }
}
