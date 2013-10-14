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
        public float c1 = 1.4f;//1.1f;//2.8f;
        public float c2 = 1.1f;//0.9f;//1.3f;//1.3f;
        public float random1 = 0.1f;
        public float random2 = 0.1f;
        public System.IO.StreamWriter log = new System.IO.StreamWriter( "var.csv" );
        public int indexRem = 0;

        Random randGen = null;

        public int particleNum = 64;
        public Particle[] particles = null;

        public float globalBestEvaluation = 10000f;
        public Particle globalBestParticle = null;
        public float[] evaluations = null;
        public Boolean finished = false;
        public int frameCount = 0;

        public void Initialize( int degreeNum )
        {
            particles = new Particle[ particleNum ];

            randGen = new Random();

            evaluations = new float[ particleNum ];

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
                    random1 = ( float )randGen.NextDouble();
                    random2 = ( float )randGen.NextDouble();
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

                OutputLog();
                // weight -= 0.01f;
                //weight = Math.Max( 0.65f, weight );
                Console.WriteLine( "Eval:" + globalBestEvaluation );
                Console.WriteLine( "Best :" + indexRem );

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
