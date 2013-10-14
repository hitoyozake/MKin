using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;

namespace PoseEstimationStudio
{
    class Particle
    {
        public int degreeNum = 27;
        public float[] degreePositions = null;
        public float[] maxDegree = null;
        public float[] minDegree = null;

        public float[] vectors = null;

        public float[] localBestPosition = null;
        public Boolean badPosition = false;

        public float evaluationMin = 1000f;

        public float maxVel = 17.0f;
        public float minVel = -17.0f;
        //+-16.0fがなかなか良かったパラメータ


        public void Initialize()
        {
            degreePositions = new float[  degreeNum ];
            //degreesのみ[0,359]で初期化

            maxDegree = new float[ degreeNum ];
            minDegree = new float[ degreeNum ];

            for( int i = 0; i < degreeNum; ++i )
            {
                maxDegree[ i ] = 80f;
                minDegree[ i ] = -80f;
            }

            localBestPosition = new float[ degreeNum ];

            vectors = new float[ degreeNum ];
        }

        public Particle()
        {
        }

        public void SetRnage( int index, float min, float max )
        {
            minDegree[ index ] = MathHelper.Min( min, max );
            maxDegree[ index ] = MathHelper.Max( min, max );
        }


        public void Update( float allWeight, float weight, float c1, float c2, float r1, float r2, float [] globalBest, float evaluation )
        {
            for( int i = 0; i < degreeNum; ++i )
            {
                vectors[ i ] = allWeight * ( weight * vectors[ i ]   + c1 * r1 * ( localBestPosition[ i ] - degreePositions[ i ] )
                   + c2 * r2 * ( globalBest[ i ] - degreePositions[ i ] ) );
                //vectors[ i ] = weight * ( vectors[ i ] + c1 * r1 * ( localBestPosition[ i ] - degreePositions[ i ] )
                //    + c2 * r2 * ( globalBest[ i ] - degreePositions[ i ] ) );

                vectors[ i ] = MathHelper.Clamp( vectors[ i ], this.minVel, this.maxVel );

                degreePositions[ i ] += vectors[ i ];

                degreePositions[ i ] = MathHelper.Clamp( degreePositions[ i ], minDegree[ i ], maxDegree[ i ] );

                //degreePositions[ i ] = degreePositions[ i ] % 360.0f;

                //if ( degreePositions[i] > 360f || degreePositions[i] < -360f )
                //{
                //    badPosition = true;
                //}
            }

            if( evaluationMin > evaluation )
            {
                evaluationMin = evaluation;

                for( int i = 0; i < degreeNum; ++i )
                {
                    localBestPosition[ i ] = degreePositions[ i ];
                }
            }
        }


    }
}
