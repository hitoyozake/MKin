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

namespace BoneEstimation
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class Game1 : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        Random randGen = new Random();

        public Model[] models = new Model[ ( int )BodyPartsName.Size ];
        public Color XNABackColor = new Color( 68, 34, 136 );

        public Effect depthEffect;
        public BasicEffect basicEffect;

        public BodyParts [] bodyParts;
        public ScreenCapture2D captureDevice = new ScreenCapture2D();
        public PoseEvaluater poseEvaluater = new PoseEvaluater();

        public int width = 640, height = 480;
        public Texture2D TeacherTex;
        public Texture2D tmpTexture;
        public Texture2D PartsTexture;

        public Color[] tmpPixels = new Color[ 640 * 480 ];

        public Color[] copyPixels = new Color[ 640 * 480 ];

        public List<BodyParts> drawList = new List<BodyParts>();

        public Boolean testFlag = false;
        public Boolean nextFrame = false;

        public TestEnv testEnv = null;
        public Boolean readyfinishedForPSO = false;
        public Boolean useDefaultEffect = false;

        public int nextFrameWaitCounter = 0;

        KeyboardState keyState;

        ParticleSwarmOptimization PSOManager = new ParticleSwarmOptimization();

        public Vector3 cameraPosition = new Vector3( 0f, 0f, 0.3f );

        public System.IO.StreamWriter poseLog = new System.IO.StreamWriter( "poseLog.csv" );

        public Logger logFiles = new Logger();


        #region SingleJointTest
        void InitializeSingleJointTest()
        {
            PSOManager.Initialize( 32, 9 );
            SetFirstPose();
            ReadyForPSO() ;

            readyfinishedForPSO = true;
        }

        void MakeNoise()
        {
            foreach( Particle p in PSOManager.particles )
            {
                for( int j = 0; j < p.degrees.Length; ++j )
                {
                    p.degrees[ j ] = p.degrees[ j ] + ( randGen.Next() % 2 == 0 ? -1f : 1f )
                        * ( ( float )randGen.NextDouble() * 2f + 4f ); 
                }
            }
        }

        void MakeNoiseToDrawingModel( BodyParts [] parts )
        {
            foreach( var p in parts )
            {
                p.angle.X = p.angle.X +( randGen.Next() % 2 == 0 ? -1f : 1f )
                        * ( ( float )randGen.NextDouble() * 2f + 4f );
                p.angle.Y = p.angle.Y  +( randGen.Next() % 2 == 0 ? -1f : 1f )
                        * ( ( float )randGen.NextDouble() * 2f + 4f );
                p.angle.Z = p.angle.Z + ( randGen.Next() % 2 == 0 ? -1f : 1f )
                        * ( ( float )randGen.NextDouble() * 2f + 4f );

                p.Update();
            }

        }

        #region 次のフレーム
        void NextFrame()
        {
            testEnv.RotateModelAndNextFrame( drawList, PSOManager );

            ReadyForPSO();
            nextFrame = false;
        }
        #endregion

        void SetFirstPose()
        {
            bodyParts[ ( int )BodyPartsName.ArmR ].angle.X = -100 + 200 * ( float )randGen.NextDouble();
            bodyParts[ ( int )BodyPartsName.ArmR ].angle.Y = -100 + 200 * ( float )randGen.NextDouble();
            bodyParts[ ( int )BodyPartsName.ArmR ].angle.Z = -100 + 200 * ( float )randGen.NextDouble();

            bodyParts[ ( int )BodyPartsName.UpperArmR ].angle.X = -100 + 200 * ( float )randGen.NextDouble();
            bodyParts[ ( int )BodyPartsName.UpperArmR ].angle.Y = -100 + 200 * ( float )randGen.NextDouble();
            bodyParts[ ( int )BodyPartsName.UpperArmR ].angle.Z = -100 + 200 * ( float )randGen.NextDouble();

            //bodyParts[ ( int )BodyPartsName.Trunk ].angle.X = -40 + 80 * ( float )randGen.NextDouble();
            //bodyParts[ ( int )BodyPartsName.Trunk ].angle.Y = -40 + 80 * ( float )randGen.NextDouble();
            //bodyParts[ ( int )BodyPartsName.Trunk ].angle.Z = -40 + 80 * ( float )randGen.NextDouble();

            drawList = new List<BodyParts>();
            //drawList.Add( bodyParts[ ( int )BodyPartsName.Trunk ] );
            drawList.Add( bodyParts[ ( int )BodyPartsName.UpperArmR ] );
            drawList.Add( bodyParts[ ( int )BodyPartsName.ArmR ] );

            for( int i = 0; i < 32; ++i )
            { 
                PSOManager.SetBodyPartsToParticle( drawList.ToArray(), i );
            }

            //初期姿勢を記憶
            for( int i = 0; i < PSOManager.particles[ 0 ].degreeCount; ++i )
            {
                poseLog.Write( PSOManager.particles[ 0 ].degrees[ i ] + "," );
            }

            testEnv.RememberPose( PSOManager.particles[ 0 ].degrees );

            poseLog.Write( "0\n" );

            MakeNoise();
            PSOManager.SetCurrentDegreesToLocalBest();

            for( int i = 0; i < 32; ++i )
            {
                PSOManager.SetDegreesToPrevFrame( drawList.ToArray(), i );
            }

        }
        #endregion

        void SaveGroupBestPose()
        {
            for( int i = 0; i < PSOManager.groupBestDegrees.Length; ++i )
            {
                poseLog.Write( PSOManager.groupBestDegrees[ i ] + "," );
            }
            poseLog.Write( "0\n" );

        }


        #region PSO
        public void ReadyForPSO()
        {
            //スクリーンショット作成

            for( int i = 0; i < drawList.Count; ++i )
            {
                drawList[ i ].UseEffect( models, depthEffect );
                drawList[ i ].SetView( cameraPosition );
                drawList[ i ].Update();
                drawList[ i ].Draw( models );
            }

            TakeScreenCapture( drawList, TeacherTex );
            TakeScreenCapture( drawList, TeacherTex );

            using ( var file = System.IO.File.Create( "psoTeacherImage.png"))
            {
                TeacherTex.SaveAsPng( file, 640, 480 );
            }

            logFiles.OutputPose( drawList.ToArray() );
            logFiles.WriteBlankPose( drawList.ToArray() );
            //testEnv.StretchModel( drawList );

        }

        public void PSOLoop()
        {
            for( int i = 0; i < PSOManager.particles.Length; ++i )
            {
                PSOManager.SetParticleToBodyParts( drawList.ToArray(), i );
                GraphicsDevice.Clear( XNABackColor );
                captureDevice.BeginCapture( GraphicsDevice );

                for( int j = 0; j < drawList.Count; ++j )
                {
                    drawList[ j ].SetView( cameraPosition );
                    drawList[ j ].UseEffect( models, depthEffect );
                    drawList[ j ].Update();
                    drawList[ j ].Draw( models );
                }
                captureDevice.Capture( tmpTexture );
                
                //SaveAsPng( tmpTexture, "particle_" + i + ".png" );
                //SaveAsPng( TeacherTex, "teacher.png" );
                //評価
                PSOManager.currentFrameEvaluation[ i ] = poseEvaluater.evaluate( tmpTexture, TeacherTex );
                Console.WriteLine( PSOManager.currentFrameEvaluation[ i ] );

            }

            PSOManager.Update();

            PSOManager.SetGroupBestDegreesToBodyParts( drawList.ToArray() );
            SaveGroupBestPose();

        }

        #endregion

        void SaveAsPng( Texture2D tex, String filename )
        {
            using( var file = System.IO.File.Create( filename ) )
            {
                tex.SaveAsPng( file, 640, 480 );
            }

        }

        #region はさみうち
        public void Loop( BodyParts [] parts, PoseEvaluater ev )
        {
            const int MIN = 0;
            const int MAX = 1;

            //値のセット
            float[] middles = null;
            float[][] parameters = null;
            float[] minValues = null;

            int dimention = 6;
            float allowedRange = 14f;

            parameters = new float[ dimention ][];
            middles = new float[ dimention ];
            minValues = new float[ dimention ];
            float [] parameterSlot = new float[ dimention ];

            for( int i = 0; i < parameters.Length; ++i )
            {
                parameters[ i ] = new float[ 2 ];
            }

            parameters[ 0 ][ 0 ] = parts[ 0 ].angle.X - allowedRange;
            parameters[ 0 ][ 1 ] = parts[ 0 ].angle.X + allowedRange;
            parameters[ 1 ][ 0 ] = parts[ 0 ].angle.Y - allowedRange;
            parameters[ 1 ][ 1 ] = parts[ 0 ].angle.Y + allowedRange;
            parameters[ 2 ][ 0 ] = parts[ 0 ].angle.Z - allowedRange;
            parameters[ 2 ][ 1 ] = parts[ 0 ].angle.Z + allowedRange;
            parameters[ 3 ][ 0 ] = parts[ 1 ].angle.X - allowedRange;
            parameters[ 3 ][ 1 ] = parts[ 1 ].angle.X + allowedRange;
            parameters[ 4 ][ 0 ] = parts[ 1 ].angle.Y - allowedRange;
            parameters[ 4 ][ 1 ] = parts[ 1 ].angle.Y + allowedRange;
            parameters[ 5 ][ 0 ] = parts[ 1 ].angle.Z - allowedRange;
            parameters[ 5 ][ 1 ] = parts[ 1 ].angle.Z + allowedRange;

            float [] degrees = new float[ dimention ];

            float minValue = 1000f;
            int [] minIndex = new int[ 6 ];

            int challangeCount = 9;

            for( int counter = 0; counter < challangeCount; ++counter )
            {
                for (int i = 0; i < parameters[0].Length; i++)
			    {
			        for (int j = 0; j < parameters[1].Length; j++)
			        {
			            for (int k = 0; k < parameters[2].Length; k++)
			            {
			                for (int l = 0; l < parameters[3].Length; l++)
	           	            {
			                    for (int m = 0; m < parameters[4].Length; m++)
	           	                {
			                        for(int n = 0; n < parameters[5].Length; n++)
	           	                    {
                                        //値をセット
                                        parts[ 0 ].angle.X = parameters[ 0 ][ i ];
                                        parts[ 0 ].angle.Y = parameters[ 1 ][ j ];
                                        parts[ 0 ].angle.Z = parameters[ 2 ][ k ];
                                        parts[ 1 ].angle.X = parameters[ 3 ][ l ];
                                        parts[ 1 ].angle.Y = parameters[ 4 ][ m ];
                                        parts[ 1 ].angle.Z = parameters[ 5 ][ n ];

                                        GraphicsDevice.Clear( XNABackColor );

                                        captureDevice.BeginCapture( GraphicsDevice );

                                        parts[ 0 ].Update();
                                        parts[ 1 ].Update();

                                        parts[ 0 ].Draw( models );
                                        parts[ 1 ].Draw( models );

                                        captureDevice.Capture( tmpTexture );

                                        var evalresult = poseEvaluater.evaluate( tmpTexture, TeacherTex );

                                        //SaveAsPng( tmpTexture, "loopTmpTex.png" );
                                        //SaveAsPng( TeacherTex, "loopTeacher.png" );

                                        if( evalresult < minValue )
                                        {
                                            minValue = evalresult;

                                            minIndex[ 0 ] = i;
                                            minIndex[ 1 ] = j;
                                            minIndex[ 2 ] = k;
                                            minIndex[ 3 ] = l;
                                            minIndex[ 4 ] = m;
                                            minIndex[ 5 ] = n;
                                        }
		                            }
		                        }
		                    }
			            }
			        }
			    }
                //最小のindexを覚えたのでここでセット
                for( int i = 0; i < 6; ++i )
                {
                    var tmpMin = Math.Min( parameters[ i ][ minIndex[ i ] ], middles[ i ] );
                    var tmpMax = Math.Max( parameters[ i ][ minIndex[ i ] ], middles[ i ] );

                    parameters[ i ][ MIN ] = tmpMin;
                    parameters[ i ][ MAX ] = tmpMax;

                    middles[ i ] = ( tmpMax + tmpMin ) / 2f;
                }
                minValue = 1000f;//リセット
            }

            Draw2D( TeacherTex, Color.CadetBlue );

            parts[ 0 ].angle.X = parameters[ 0 ][ minIndex[ 0 ] ];
            parts[ 0 ].angle.Y = parameters[ 1 ][ minIndex[ 1 ] ];
            parts[ 0 ].angle.Z = parameters[ 2 ][ minIndex[ 2 ] ];
            parts[ 1 ].angle.X = parameters[ 3 ][ minIndex[ 3 ] ];
            parts[ 1 ].angle.Y = parameters[ 4 ][ minIndex[ 4 ] ];
            parts[ 1 ].angle.Z = parameters[ 5 ][ minIndex[ 5 ] ];

            logFiles.OutputPose( parts );


        }
        #endregion


        #region Draw内処理

        void DrawMain()
        {
            nextFrameWaitCounter++;

            if( useDefaultEffect == false )
            {
                foreach( var parts in bodyParts )
                {
                    parts.UseEffect( models, depthEffect );
                }
            }
            else
            {
                foreach( var parts in bodyParts )
                {
                    parts.UseBasicEffect( models, basicEffect );
                }
            }

            if( testFlag == true )
            {
                if( readyfinishedForPSO == false )
                {
                    InitializeSingleJointTest();
                    testEnv.StretchModel( drawList );
                    MakeNoiseToDrawingModel( drawList.ToArray() );
                }
                else
                {
                    if( nextFrame == true )
                    {
                        NextFrame();
                        MakeNoiseToDrawingModel( drawList.ToArray() );
                    }

                    Loop( drawList.ToArray(), this.poseEvaluater );
                   //PSOLoop();
                   Draw2D( TeacherTex, Color.CadetBlue );

                }
            }
        }

        #endregion

        public void InitializeViewer()
        {
            drawList = new List<BodyParts>();
            drawList.Add( bodyParts[ ( int )BodyPartsName.Trunk ] );
            drawList.Add( bodyParts[ ( int )BodyPartsName.ArmR ] );
            drawList.Add( bodyParts[ ( int )BodyPartsName.UpperArmR ] );
        }


        #region 基本操作

        void Command()
        {
            if( keyState.IsKeyDown( Keys.T ))
            {
                cameraPosition.Z += 0.001f;
            }
            if( keyState.IsKeyDown( Keys.Y ) )
            {
                cameraPosition.Z += -0.001f;
            }


            if( keyState.IsKeyDown(Keys.F3 ))
            {
                testFlag = true;
            }

            if( keyState.IsKeyDown( Keys.F4 ) )
            {
                if( nextFrame == false && nextFrameWaitCounter >= 10 )
                {
                    nextFrameWaitCounter = 0;
                    nextFrame = true;
                }
            }

        }

        public Game1()
        {
            graphics = new GraphicsDeviceManager( this );
            Content.RootDirectory = "Content";
            graphics.PreferredBackBufferWidth = 640;
            graphics.PreferredBackBufferHeight = 480;
            IsMouseVisible = true;
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            // TODO: Add your initialization logic here
            base.Initialize();
        }

        public void CopyTexture( Texture2D src, Texture2D dest )
        {
            src.GetData( copyPixels );
            dest.SetData( copyPixels );
        }


        public void InitializeTexture()
        {
            TeacherTex = new Texture2D( GraphicsDevice, width, height );
            tmpTexture = new Texture2D( GraphicsDevice, width, height );
        }


        void InitializeObjects()
        {
            captureDevice.Init( GraphicsDevice );
            testEnv = new TestEnv( GraphicsDevice );
        }

        public void TakeScreenCapture( List<BodyParts> draws, Texture2D saveTexture )
        {
            GraphicsDevice.Clear( XNABackColor );

            captureDevice.BeginCapture( GraphicsDevice );

            foreach( var d in draws )
            {
                d.Draw( models );
            }
            captureDevice.Capture( saveTexture );
        }

        /// <summary>
        /// モデルを読み込む
        /// </summary>
        void LoadModel()
        {
            models[ ( int )BodyPartsName.ArmR ] = Content.Load<Model>( "rarm" );
            models[ ( int )BodyPartsName.UpperArmR ] = Content.Load<Model>( "ruarm" );
            models[ ( int )BodyPartsName.Trunk ] = Content.Load<Model>( "body" );


            bodyParts = new BodyParts[ ( int )BodyPartsName.Size ];

            for( int i = 0; i < ( int )BodyPartsName.Size; ++i )
            {
                bodyParts[ i ] = new BodyParts();
                bodyParts[ i ].bodyPartsName = ( BodyPartsName ) i;
                
            }

            bodyParts[ ( int )BodyPartsName.ArmR ].parent =
                bodyParts[ ( int )BodyPartsName.UpperArmR ];
            //bodyParts[ ( int )BodyPartsName.UpperArmR ].parent =
            //   bodyParts[ ( int )BodyPartsName.Trunk ];


            bodyParts[ ( int )BodyPartsName.UpperArmR ].basePosition.Y = 0.5f;
            bodyParts[ ( int )BodyPartsName.UpperArmR ].position.Y = 0.5f;
            bodyParts[ ( int )BodyPartsName.UpperArmR ].position.X = -1f;


            bodyParts[ ( int )BodyPartsName.ArmR ].basePosition.Y = 0.5f;
            bodyParts[ ( int )BodyPartsName.ArmR ].position.Y = 0.8f;

        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch( GraphicsDevice );
            InitializeObjects();
            LoadModel();
            // TODO: use this.Content to load your game content here
            depthEffect = Content.Load<Effect>( "depth" );
            basicEffect = new BasicEffect( GraphicsDevice );
            InitializeTexture();

            InitializeViewer();
        }

        void DrawModels( List< BodyParts > drawlist)
        {
            foreach( var d in drawlist )
            {
                d.Draw( models );
            }
        }

        void Draw2D( Texture2D texture, Color backColor )
        {
            spriteBatch.Begin();

            spriteBatch.Draw( texture, Vector2.Zero, backColor );

            spriteBatch.End();

        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// all content.
        /// </summary>
        protected override void UnloadContent()
        {
            // TODO: Unload any non ContentManager content here
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update( GameTime gameTime )
        {
            Command();

            // Allows the game to exit
            if( GamePad.GetState( PlayerIndex.One ).Buttons.Back == ButtonState.Pressed )
                this.Exit();

            keyState = Keyboard.GetState();
            // TODO: Add your update logic here
            base.Update( gameTime );
        }
        #endregion
        
        void TestDraw()
        {
            drawList = new List<BodyParts>();
            drawList.Add( bodyParts[ ( int )BodyPartsName.UpperArmR ] );
            drawList.Add( bodyParts[ ( int )BodyPartsName.ArmR ] );

 //           drawList[ 0 ].angle = new Vector3( -43.06471f,	38.83652f,	-59.49479f

 //);
 //           drawList[ 1 ].angle = new Vector3( 72.72168f,	-60.64774f,	47.39542f

 //);

            drawList[ 0 ].angle = new Vector3( -25.43162f,	24.83652f,	-43.36222f

);
            drawList[ 1 ].angle = new Vector3( 86.72168f,	-44.09669f,	33.39542f
 );


            //drawList[ 0 ].angle = new Vector3( -23.788f, 31.3782f, -94.375f );
            //drawList[ 1 ].angle = new Vector3( 23.714f, 57.202f, -59.1012f );

            drawList[ 0 ].SetView( cameraPosition );
            drawList[ 1 ].SetView( cameraPosition );


            drawList[ 0 ].UseEffect( models, depthEffect);
            drawList[ 1 ].UseEffect( models, depthEffect );

            drawList[ 0 ].Update();
            drawList[ 1 ].Update();


        }

        
        
        
        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw( GameTime gameTime )
        {
            GraphicsDevice.Clear( XNABackColor );

            DrawMain();
            //TestDraw();

            foreach( var parts in bodyParts )
            {
                parts.SetView( cameraPosition );
            }

            
            // TODO: Add your drawing code here
            foreach( var parts in bodyParts )
            {
                parts.Update();
            }

            foreach( var m in drawList)
            {
                m.Draw( models );
            }

            base.Draw( gameTime );
        }

        /// <summary>
        /// 終了処理
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="args"></param>
        protected override void OnExiting( object sender, EventArgs args )
        {
            PSOManager.tracer.Close();
            poseLog.Close();
            logFiles.Close();
            PSOManager.evaluationTracer.Close();

            base.OnExiting( sender, args );
        }
    }
}
