using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PoseEstimationStudio
{
  

    class ParticleSwarmOptimizer
    {
        public float weight = 0.95f;//0.85f;
        public float allWeight = 0.96f;
        public float c1 = 1.8f;//1.4f;//1.1f;//2.8f;
        public float c2 = 1.2f;//1.1f;//0.9f;//1.3f;//1.3f;
        public float e = 0f;

        public float[] weights = null;

        public float random1 = 0.1f;
        public float random2 = 0.1f;
        public System.IO.StreamWriter log = new System.IO.StreamWriter( "var.csv" );
        public int indexRem = 0;

        Random randGen = null;

        public int particleNum = 64;
        public Particle[] particles = null;

        public float globalBestEvaluation = 1000000f;
        public Particle globalBestParticle = null;
        public float[] evaluations = null;
        public Boolean finished = false;
        public int frameCount = 0;

        public float[] maxDegrees = null;
        public float[] minDegrees = null;
        public float threshold = 0.005f;

        public Particle prevFrame = null;

        //論文によると
        //c1 = 2.8, c2 = 1.3
        // e = c1 + c2;
        //w = 2.0 / | 2 - e - sqrt( e*e - 4 * e ) |

        public void InitializeWeightByRelatedWork()
        {
            weight = 1.0f;
            c1 = 2.8f;
            c2 = 1.3f;
            e = c1 + c2;
            allWeight = 2.0f / ( float )( Math.Abs( 2f - e - Math.Sqrt( e * e - 4 * e ) ) );
        }
        /// <summary>
        /// 最大値設定と最小値設定の間違いをなくすためのメソッド
        /// </summary>
        /// <param name="particle"></param>
        /// <param name="index"></param>
        /// <param name="value1"></param>
        /// <param name="value2"></param>
        public void SetMaxMinDegree( Particle particle, int index, float value1, float value2 )
        {
            particle.maxDegree[ index ] = Math.Max( value1, value2 );
            particle.minDegree[ index ] = Math.Min( value1, value2 );
        }


        public Boolean SetBodyParams()
        {
            if( weights.Length != ( int )BodyPartsName.Size * 3 )
            {
                Console.WriteLine( "weights.Length != BodyPartsName.Size * 3" );
                return false;
            }

            for( int i = 0; i < particles.Length; ++i )
            {

                SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.Trunk * 3, 360f, -360f );
                SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.Trunk * 3 + 1, 360f, -360f );
                SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.Trunk * 3 + 2, 360f, -360f );

                //腰

                SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UnderTrunk * 3, 50f, -50f );
                SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UnderTrunk * 3 + 1, 60f, -20f );
                SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UnderTrunk * 3 + 2, 50f, -50f );

                //肩・上腕
                {
                    //ZとY反転
                    //屈曲・伸展
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UpperArmR * 3, 165f, -60f );

                    //外転・内転
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UpperArmR * 3 + 1, 165f, 0f );

                    //内旋・外旋
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UpperArmR * 3 + 2, 90f, -70f );

                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UpperArmL * 3, -165f, 60f );
                    
                    //外転・内転
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UpperArmL * 3 + 1, 165f, 0f );

                    //内旋・外旋
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.UpperArmL * 3 + 2, 70f, -90f );

                }

                //肘・前腕
                {
                    //屈曲
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ArmR * 3, 0f, 165f );

                    //回内回外
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ArmR * 3 + 1, 80f, -80f );

                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ArmL * 3, 0f, 165f );

                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ArmL * 3 + 1, 80f, -80f );
                }

                //太もも
                {
                    //ZとY反転
                    //屈曲・伸展
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ThighR * 3, 120f, -20f );

                    //外転・内転
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ThighR * 3 + 1, 45f, -30f );
                    
                    //内旋・外旋
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ThighR * 3 + 2, 40f, -40f );


                    //ZとY反転
                    //屈曲・伸展
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ThighL * 3, 120f, -20f );

                    //外転・内転
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ThighL * 3 + 1, 30f, -45f );
                    
                    //内旋・外旋
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.ThighL * 3 + 2, 40f, -40f );
                }

                //脚
                {
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.LegR * 3, -145f, 0f );
                    SetMaxMinDegree( particles[ i ], ( int )BodyPartsName.LegL * 3, -145f, 0f );
                }
            }
            
            return true;
        }




        public void Initialize( int degreeNum )
        {
            globalBestEvaluation = 1000000f;

            //InitializeWeightByRelatedWork();    //allWeight, c1, c2, weightを書き換える
            allWeight = 0.92f;
            particles = new Particle[ particleNum ];

            randGen = new Random();

            evaluations = new float[ particleNum ];

            weights = new float[ degreeNum ];
            maxDegrees = new float[ degreeNum ];
            minDegrees = new float[ degreeNum ];

            prevFrame = new Particle();
            prevFrame.degreeNum = degreeNum;
            prevFrame.Initialize();

            for( int i = 0; i < particles.Length; ++i )
            {
                particles[ i ] = new Particle();
                particles[ i ].degreeNum = degreeNum;
                particles[ i ].Initialize();

                for( int j = 0; j < particles[i].degreePositions.Length; ++j )
                {
                    particles[ i ].degreePositions[ j ] = ( float )randGen.Next( -180, 180 );
                }
            }

            globalBestParticle = new Particle();
            globalBestParticle.Initialize();
            for( int j = 0; j < globalBestParticle.degreePositions.Length; ++j )
            {
                globalBestParticle.degreePositions[ j ] = ( float )randGen.Next( -180, 180 );
            }
            SetBodyParams();

        }

        void OutputLog()
        {
            for( int i = 0; i < particles[ 0 ].vectors.Length; ++i )
            {
                log.Write( particles[ 0 ].vectors[ i ] );

                log.Write( "," );

                log.Write( particles[0].degreePositions[i] );

                if( i < particles[ 0 ].vectors.Length - 1 )
                {
                    log.Write( "," );
                }
                else
                {
                    log.Write( "\n" );
                }
            }
        }


        public void Update()
        {
            this.frameCount++;

            if( globalBestEvaluation <= 0.0001 )
            {
                Console.WriteLine( "Finish" );
                finished = true;
            }
            else
            {
                for( int i = 0; i < particles.Length; ++i )
                {
                    random1 = ( float )( randGen.NextDouble());
                    random2 = ( float )( randGen.NextDouble());
                    particles[ i ].Update( allWeight, weight, c1, c2, random1, random2, globalBestParticle.degreePositions, evaluations[ i ] );
                    if( evaluations[ i ] < globalBestEvaluation )
                    {
                        globalBestEvaluation = evaluations[ i ];
                        globalBestParticle.evaluationMin = globalBestEvaluation;
                        indexRem = i;
                        particles[ i ].degreePositions.CopyTo( globalBestParticle.degreePositions, 0 );

                        if( particles[ i ].badPosition )
                        {
                            particles[ i ].badPosition = false;

                            float sumMin = 10000f;
                            int sumMinIndex = 0;
                            for( int j = 0; j < particles.Length; ++j )
                            {
                                float sum = 0;
                                if( j == i )
                                {
                                    continue;
                                }
                                for( int k = 0; k < particles[ i ].degreePositions.Length; ++k )
                                {
                                    sum += Math.Abs( particles[ i ].degreePositions[ k ] - particles[ j ].degreePositions[ k ] );
                                }

                                if( sumMin > sum )
                                {
                                    sumMin = sum;
                                    sumMinIndex = j;
                                }
                            }
                            particles[ sumMinIndex ].degreePositions.CopyTo( particles[ i ].degreePositions, 0 );
                            particles[ sumMinIndex ].vectors.CopyTo( particles[ i ].vectors, 0 );
                        }
                        if( frameCount % 20 == 0 )
                        {
                            if( allWeight >= 0.5f )
                            {
                                allWeight *= 0.92f;
                            }
                            if( weight >= 0.5f )
                            {
                                weight *= 0.94f;
                            }
                        }
                    }
                }

                if( frameCount % 5 == 0 )
                {
                    for( int i = 0; i < particles.Length / 2; ++i )
                    {
                        if( particles[ i ] != globalBestParticle )
                        {
                            for( int j = 0; j < particles[ i ].degreeNum; ++j )
                            {
                                //if( randGen.NextDouble() < ( double )threshold )
                                {
                                    if( j != ( int )BodyPartsName.Trunk )
                                    {
                                        particles[ i ].degreePositions[ j ] = ( float )prevFrame.degreePositions[ j ] + 3.0f * ( float )randGen.NextDouble();
                                        particles[ i ].vectors[ j ] = 0f;
                                        Console.WriteLine( i + "," + j + " : " + particles[ i ].degreePositions[ j ] );
                                    }
                               }
                            }

                        }
                    }
                }

                    OutputLog();
                // weight -= 0.01f;
                //weight = Math.Max( 0.65f, weight );
                Console.WriteLine( "Eval:" + globalBestEvaluation );
                //Console.WriteLine( "Best :" + indexRem );

                //foreach( var p in particles )
                //{
                //    if( globalBestParticle == p )
                //    {
                //        Console.WriteLine( "foo " );
                //    }
                //}

            }

        }
    }
}

/*
 * 合計軸数：3+(3+1+1+3+1+3+3)*2
 * = 31?
 * 
 * 
 * 
 * 首：3軸(てか首の座りっていつよ？)
 * 屈曲と伸展は同じ軸
 * 
 * ----7 * 2
 * 3軸
 * 肩：屈曲・伸展 -60～165，外転 0-165(y)，内旋外旋-70～90
 * 
 * 1軸
 * * 前腕：回内回外 -80～80(Y軸統一)
 * 
 * 1軸
 * 肘：屈曲(X軸):0-165
 *
 *  
 * 2軸
 * 手：-70～90, -25～30
 * 
 * 
 * ------6 * 2
 * 3軸
 * 股：屈曲伸展 -20～120，外転内点-30～45 内旋外旋-40～40
 * 
 * 1軸
 * 膝：屈曲0-145(X軸)
 * 
 * 2軸
 * 足
 */


