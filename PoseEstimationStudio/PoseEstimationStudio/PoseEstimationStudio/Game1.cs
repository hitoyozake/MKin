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
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class Game1 : Microsoft.Xna.Framework.Game
    {
        Random randGen = new Random();

        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        int frameCount = 0;
        BodyParts arm = new BodyParts();
        BodyParts uArm = new BodyParts();
        BodyParts hand = new BodyParts();
        BodyParts underTrunk = new BodyParts();

        BodyParts uArmR = new BodyParts();
        BodyParts uArmL = new BodyParts();
        BodyParts armL = new BodyParts();
        BodyParts armR = new BodyParts();
        BodyParts head = new BodyParts();
        BodyParts body = new BodyParts();
        BodyParts legL = new BodyParts();
        BodyParts legR = new BodyParts();
        BodyParts thighL = new BodyParts();
        BodyParts thighR = new BodyParts();

        BodyParts uArmRTearcher = new BodyParts();
        BodyParts uArmLTearcher = new BodyParts();
        BodyParts armLTearcher = new BodyParts();
        BodyParts armRTearcher = new BodyParts();
        BodyParts headTearcher = new BodyParts();
        BodyParts bodyTearcher = new BodyParts();
        BodyParts legLTearcher = new BodyParts();
        BodyParts legRTearcher = new BodyParts();
        BodyParts thighLTearcher = new BodyParts();
        BodyParts thighRTearcher = new BodyParts();
        BodyParts footR = new BodyParts();
        BodyParts footL = new BodyParts();
        BodyParts handL = new BodyParts();
        BodyParts handR = new BodyParts();

        Model xAxis;
        Model yAxis;
        Model zAxis;
        Boolean visibleAxis = true;

        Boolean psoFlag = false;
        PoseEvaluater poseEvaluater = new PoseEvaluater();

        PointPrimitive primitive = new PointPrimitive();

        //PSOするやつ
        ParticleSwarmOptimizer PSOManager = new ParticleSwarmOptimizer();

        List<BodyParts> bodyParts = new List<BodyParts>();
        //教師データ
        List<BodyParts> teacherParts = new List<BodyParts>();

        List<String> displayStrings = new List<String>();

        SpriteFont font = null;

        KeyboardState keyState;

        Texture2D loadedImage;

        //XNAの背景色(塗りつぶしに使用)
        Color xnaBackColor = new Color( 68, 34, 136 );
        
        //2Dキャプチャ映像の背景(3Dだけでやってキャプチャする場合，将来的にはいらない
        Color texture2DBackColor = new Color( 0, 0, 0 );

        ScreenCapture2D screenCapture = new ScreenCapture2D();

        //スクリーンショット用
        Texture2D tmpScreenCaptured = null;

        Texture2D [] capturedTextures = new Texture2D[2];

        Texture2D[] teacherTexture = new Texture2D[ 3 ];

        //多方面レンダリング用
        Vector3[] cameraPositions = new Vector3[ 3 ];

        public uint otherView = 0;    

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";

            graphics.PreferredBackBufferWidth = 640;
            graphics.PreferredBackBufferHeight = 480;

            IsMouseVisible = true;
            PSOManager.Initialize( ( int )BodyPartsName.Size * 3 );

            cameraPositions[ 0 ] = new Vector3( 0f, 0f, 15f );
            cameraPositions[ 1 ] = new Vector3( 4f, 17f, 5f );
            cameraPositions[ 2 ] = new Vector3( -6f, -12f, 3f );
        }

        void ReplaceColor( ref Texture2D texture, Color source, Color replaced )
        {
            Color [] pixels = new Color[ texture.Width * texture.Height ];
            texture.GetData( pixels );
            for( int x = 0; x < texture.Width; ++x )
            {
                for( int y = 0; y < texture.Height; ++y )
                {
                    if( pixels[ x + y * texture.Width ] == source )
                    {
                        pixels[ x + y * texture.Width ] = replaced;
                    }
                }
            }
            texture.SetData( pixels );
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

        void InitializeAxises( Vector3 cameraPosition )
        {
            float distance = 0.5f;
            foreach( var mesh in xAxis.Meshes )
            {
                foreach( BasicEffect effect in mesh.Effects )
                {
                    effect.EnableDefaultLighting();

                    effect.View = Matrix.CreateLookAt( cameraPosition, Vector3.Zero, Vector3.Up );

                    effect.Projection = Matrix.CreatePerspectiveFieldOfView(
                        MathHelper.ToRadians( 45.0f ), ( float )this.GraphicsDevice.Viewport.Width /
                        ( float )this.GraphicsDevice.Viewport.Height, 1f, 100f );

                    effect.World = Matrix.CreateRotationZ( MathHelper.ToRadians( 90f ) ) *
                        Matrix.CreateTranslation( distance, .0f, 0f );

                }
            }

            foreach( var mesh in yAxis.Meshes )
            {
                foreach( BasicEffect effect in mesh.Effects )
                {
                    effect.EnableDefaultLighting();

                    effect.View = Matrix.CreateLookAt( cameraPosition, Vector3.Zero, Vector3.Up );

                    effect.Projection = Matrix.CreatePerspectiveFieldOfView(
                        MathHelper.ToRadians( 45.0f ), ( float )this.GraphicsDevice.Viewport.Width /
                        ( float )this.GraphicsDevice.Viewport.Height, 1f, 100f );

                    effect.World = Matrix.CreateRotationZ( MathHelper.ToRadians( 0f ) ) *
                        Matrix.CreateTranslation( .0f, distance, 0f );

                }
            }
            foreach( var mesh in zAxis.Meshes )
            {
                foreach( BasicEffect effect in mesh.Effects )
                {
                    effect.EnableDefaultLighting();

                    effect.View = Matrix.CreateLookAt( cameraPosition, Vector3.Zero, Vector3.Up );

                    effect.Projection = Matrix.CreatePerspectiveFieldOfView(
                        MathHelper.ToRadians( 45.0f ), ( float )this.GraphicsDevice.Viewport.Width /
                        ( float )this.GraphicsDevice.Viewport.Height, 1f, 100f );

                    effect.World = Matrix.CreateRotationX( MathHelper.ToRadians( 90f ) ) *
                        Matrix.CreateTranslation( .0f, .0f, distance );

                }
            }

        }

        void InitializeBodyParts()
        {
            head.parent = body;
            uArmL.parent = body;
            armL.parent = uArmL;

            uArmR.parent = body;
            armR.parent = uArmR;

            thighL.parent = underTrunk;
            thighR.parent = underTrunk;
            legL.parent = thighL;
            legR.parent = thighR;
            handR.parent = armR;
            handL.parent = armL;
            footL.parent = legL;
            footR.parent = legR;
            underTrunk.parent = body;
            
            head.position.Y = 2f;
            arm.position.Y = 1.0f;

            uArmL.position.Y = 1.0f;
            uArmL.position.X = -1.2f;
            armL.position.Y = 1.2f;
            handL.position.Y = 0.8f;

            uArmR.position.Y = 1.0f;
            uArmR.position.X = 1.2f;
            armR.position.Y = 1.2f;
            handR.position.Y = 0.8f;

            legL.position.Y = 1.8f;
            legR.position.Y = 1.8f;
            thighR.position.Y = -0.2f;
            thighL.position.Y = -0.2f;
            thighR.position.X = -1f;
            thighL.position.X = 1f;

            armL.baseAngle.Z = 0f;
            uArmL.baseAngle.Z = 90f;
            armR.baseAngle.Z = -0f;
            uArmR.baseAngle.Z = -90f;
            hand.baseAngle.Z = 0f;
            thighL.baseAngle.Z = 180f;
            thighR.baseAngle.Z = 180f;

            footL.position.Z = 0.4f;
            footR.position.Z = 0.4f;

            footL.position.Y = 1.3f;
            footR.position.Y = 1.3f;

            underTrunk.position.Y = -1.0f;

            head.Init( GraphicsDevice );
            armL.Init( GraphicsDevice );
            armR.Init( GraphicsDevice );
            uArmL.Init( GraphicsDevice );
            uArmR.Init( GraphicsDevice );
            hand.Init( GraphicsDevice );
            body.Init( GraphicsDevice );
            thighL.Init( GraphicsDevice );
            thighR.Init( GraphicsDevice );
            legL.Init( GraphicsDevice );
            legR.Init( GraphicsDevice );

            handR.Init( GraphicsDevice );
            handL.Init( GraphicsDevice );
            footR.Init( GraphicsDevice );
            footL.Init( GraphicsDevice );
            underTrunk.Init( GraphicsDevice );


            armL.Update();
            armR.Update();
            uArmL.Update();
            uArmR.Update();
            hand.Update();
            body.Update();
            head.Update();
            legR.Update();
            legL.Update();
            thighL.Update();
            thighR.Update();

            bodyParts[ ( int )BodyPartsName.Head ] = head;
            bodyParts[ ( int )BodyPartsName.Trunk ] = body;
            bodyParts[ ( int )BodyPartsName.LegR ] = legR;
            bodyParts[ ( int )BodyPartsName.LegL ] = legL;
            bodyParts[ ( int )BodyPartsName.ThighR ] = thighR;
            bodyParts[ ( int )BodyPartsName.ThighL ] = thighL;
            bodyParts[ ( int )BodyPartsName.UpperArmL ] = uArmL;
            bodyParts[ ( int )BodyPartsName.UpperArmR ] = uArmR;
            bodyParts[ ( int )BodyPartsName.ArmR ] = armR;
            bodyParts[ ( int )BodyPartsName.ArmL ] = armL;
            bodyParts[ ( int )BodyPartsName.HandR ] = handR;
            bodyParts[ ( int )BodyPartsName.HandL ] = handL;
            bodyParts[ ( int )BodyPartsName.FootR ] = footR;
            bodyParts[ ( int )BodyPartsName.FootL ] = footL;
            bodyParts[ ( int )BodyPartsName.UnderTrunk ] = underTrunk;
          
        }


        #region LoadContentメソッドによる初期化
        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);

            screenCapture.Init( GraphicsDevice );

            primitive.Initialize( GraphicsDevice );

            for( int i = 0; i < 2; ++i )
            {
                capturedTextures[ i ] = new Texture2D( GraphicsDevice, 640, 480 );
            }

            //Font
            font = Content.Load<SpriteFont>("myfont");

            // TODO: use this.Content to load your game content here
            arm.model = Content.Load<Model>("arm");
            uArm.model = Content.Load<Model>("uArm");
            hand.model = Content.Load<Model>("hand");
            head.model = Content.Load<Model>("head");
            body.model = Content.Load<Model>( "body" );//("body");

            uArmR.model = Content.Load<Model>( "Ruarm" );
            uArmL.model = Content.Load<Model>( "Luarm" );
            armR.model = Content.Load<Model>( "Rarm" );
            armL.model = Content.Load<Model>( "Larm" );

            legL.model = Content.Load<Model>("Lleg");
            legR.model = Content.Load<Model>("Rleg");
            thighL.model = Content.Load<Model>("Lthigh");
            thighR.model = Content.Load<Model>("Rthigh");

            footL.model = Content.Load<Model>( "footL" );
            footR.model = Content.Load<Model>( "footR" );
            handL.model = Content.Load<Model>( "handL" );
            handR.model = Content.Load<Model>( "handR" );
            underTrunk.model = Content.Load<Model>( "underTrunk" );
            
            xAxis = Content.Load<Model>("x");
            yAxis = Content.Load<Model>("y");
            zAxis = Content.Load<Model>("z");

            Vector3 cameraPosition = new Vector3(0f, 0, 15f);

            //*******************************************
            loadedImage = this.Content.Load<Texture2D>( "sampleImage" );
            tmpScreenCaptured = new Texture2D( GraphicsDevice, 640, 480 );

            Color[] pixels = new Color[ loadedImage.Width * loadedImage.Height ];
            loadedImage.GetData( pixels );
            texture2DBackColor = pixels[ 0 ];

            ReplaceColor( ref loadedImage, texture2DBackColor, xnaBackColor );

            //*******************************************
            InitializeAxises( cameraPosition : cameraPosition );
            //リストの初期化

            bodyParts.Clear();
            for( int i = 0; i < ( int )BodyPartsName.Size; ++i )
            {
                bodyParts.Add( null );
            }
            //*******************************************
                body.Init( GraphicsDevice );
            //uArmL.Init( GraphicsDevice );
            //uArmL.position.Y = 1.1f;

            //uArmL.parent = body;

            //bodyParts.Add( body );
            //bodyParts.Add( uArmL );
            InitializeBodyParts();

            for( int i = 0; i < cameraPositions.Length; ++i )
                teacherTexture[ i ] = new Texture2D( GraphicsDevice, 640, 480 );

            foreach( var parts in bodyParts )
            {
                parts.ChangeCameraPosition( cameraPositions[ 0 ].X, cameraPositions[ 0 ].Y, cameraPositions[ 0 ].Z );
                parts.Update();
            }

            Kinect.KinectManager manager = new Kinect.KinectManager();
            byte [] d = new byte[ 10 ];
            manager.TransformDepthTo3D( d );

        }
        #endregion
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
        protected override void Update(GameTime gameTime)
        {
            // Allows the game to exit
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                this.Exit();

            // TODO: Add your update logic here
            keyState = Keyboard.GetState();


            base.Update(gameTime);
        }


        #region モデルのカーソル操作
        /// <summary>
        /// モデル操作関数
        /// </summary>
        private void modelMoveEvent()
        {
            if( this.keyState.IsKeyDown( Keys.LeftShift ) || this.keyState.IsKeyDown( Keys.RightShift ) )
            {
                if( this.keyState.IsKeyDown( Keys.A ) )
                {
                    //displayStrings.Add( "Z angle rotate" );
                    body.angle.Z += 1f;
                }
                if( this.keyState.IsKeyDown( Keys.S ) )
                {
                    //displayStrings.Add( "Y angle rotate" );
                    body.angle.Y += 1f;
                }
                if( this.keyState.IsKeyDown( Keys.D ) )
                {
                    //displayStrings.Add( "Y angle rotate" );
                    body.angle.X += 1f;
                }
                if( this.keyState.IsKeyDown( Keys.OemSemicolon ) )
                {
                    //displayStrings.Add( "Z angle rotate" );
                    body.angle.Z += -1f;
                }
                if( this.keyState.IsKeyDown( Keys.L ) )
                {
                    //displayStrings.Add( "Y angle rotate" );
                    body.angle.Y += -1f;
                }
                if( this.keyState.IsKeyDown( Keys.K ) )
                {
                    //displayStrings.Add( "Y angle rotate" );
                    body.angle.X += -1f;
                }
            }
            else
            {
                if( this.keyState.IsKeyDown( Keys.A ) )
                {
                    //displayStrings.Add( "Z angle rotate" );
                    body.position.Z += 0.06f;
                }
                if( this.keyState.IsKeyDown( Keys.S ) )
                {
                    //displayStrings.Add( "Y angle rotate" );
                    body.position.Y += 0.06f;
                }
                if( this.keyState.IsKeyDown( Keys.D ) )
                {
                    //displayStrings.Add( "Y angle rotate" );
                    body.position.X += 0.06f;
                }
                if( this.keyState.IsKeyDown( Keys.OemSemicolon ) )
                {
                    //displayStrings.Add( "Z angle rotate" );
                    body.position.Z += -0.06f;
                }
                if( this.keyState.IsKeyDown( Keys.L ) )
                {
                    //displayStrings.Add( "Y angle rotate" );
                    body.position.Y += -0.06f;
                }
                if( this.keyState.IsKeyDown( Keys.K ) )
                {
                    //displayStrings.Add( "Y angle rotate" );
                    body.position.X += -0.06f;
                }
            }
        }
        #endregion
        private void showAxis()
        {
            if( visibleAxis == true )
            {
                foreach( var mesh in xAxis.Meshes )
                {
                    mesh.Draw();
                }

                foreach( var mesh in yAxis.Meshes )
                {
                    mesh.Draw();
                }

                foreach( var mesh in zAxis.Meshes )
                {
                    mesh.Draw();
                }
            }
        }

        #region Draw2D
        public void Draw2D()
        {
           spriteBatch.Begin();

            float scale = 0.5f;

            spriteBatch.Draw( loadedImage, new Vector2( 0, 0 ), null,
        Color.White, 0f, Vector2.Zero, scale, SpriteEffects.None, 0f );

            spriteBatch.End();
        }
        public void Draw2D( Texture2D texture )
        {
            spriteBatch.Begin();

            float scale = 1f;

            spriteBatch.Draw( texture, new Vector2( 0, 0 ), null,
        Color.White, 0f, Vector2.Zero, scale, SpriteEffects.None, 0f );

            spriteBatch.End();
        }
        #endregion
        void SetParamToParts( BodyParts part, float x, float y, float z )
        {
            part.angle.X = x;
            part.angle.Y = y;
            part.angle.Z = z;
        }


        #region スクリーンショットを二次元のテクスチャに落としこむコード
        void ExecuteScreenShotModel()
        {
            for( int i = 0; i < cameraPositions.Length; ++i )
            {
                GraphicsDevice.Clear( xnaBackColor );
                screenCapture.BeginCapture( GraphicsDevice );

                foreach( var parts in bodyParts )
                {
                    parts.ChangeCameraPosition( cameraPositions[ i ].X, cameraPositions[ i ].Y,
                        cameraPositions[ i ].Z );
                    parts.Update();
                    foreach( var mesh in parts.model.Meshes )
                    {
                        mesh.Draw();
                    }
                }

                screenCapture.Capture( teacherTexture[i] );
                         
            }

            for( int i = 0; i < cameraPositions.Length; ++i )
            {
                var t = System.IO.File.Create( "tmpScreenshot" + i + ".png" );
                teacherTexture[ i ].SaveAsPng( t, 640, 480 );
                t.Close();
            }

            //もとに戻す
            foreach( var parts in bodyParts )
            {
                parts.ChangeCameraPosition( cameraPositions[ 0 ].X, cameraPositions[ 0 ].Y, cameraPositions[ 0 ].Z );
            }
            SetNearDefaultPostion();
            System.Threading.Thread.Sleep( 100 );
        }
        #endregion

        #region particleのパラメータをbodyPartsに順番に代入していく
        void setParamToBodyParts( Particle ptc )
        {
            for( int i = 0; i < bodyParts.Count; ++i )
            {
                SetParamToParts( bodyParts[ i ], ptc.degreePositions[ 3 * i ], 
                    ptc.degreePositions[ 3 * i + 1 ], ptc.degreePositions[ 3 * i + 2 ] );
            }
        }
        #endregion

        #region PSOコード
        void ExecutePSO()
        {
            ++frameCount;
            //Console.WriteLine( frameCount );
            //Console.WriteLine( PSOManager.indexRem );

            GraphicsDevice.Clear( xnaBackColor );
            screenCapture.BeginCapture( GraphicsDevice );

            Draw2D();

            screenCapture.Capture( capturedTextures[ 0 ] );
            GraphicsDevice.Clear( xnaBackColor );

            for( int i = 0; i < PSOManager.particleNum; ++i )
            {
                //体の各部位に対してangleセット
                var ptc = PSOManager.particles[ i ];
                setParamToBodyParts( ptc );

                screenCapture.BeginCapture( GraphicsDevice );

                // TODO: Add your drawing code here
                foreach( var parts in bodyParts )
                {
                    parts.Update();
                    foreach( var mesh in parts.model.Meshes )
                    {
                        mesh.Draw();
                    }
                }

                screenCapture.Capture( capturedTextures[ 1 ] );
                PSOManager.evaluations[ i ] = poseEvaluater.EvaluateTexture( tmpScreenCaptured, capturedTextures[ 1 ] );
                //PSOManager.evaluations[ i ] = poseEvaluater.EvaluateTexture( capturedTextures[ 0 ], capturedTextures[ 1 ] );
            }

            //更新
            PSOManager.Update();

            //最良のモデルポーズを描画
            GraphicsDevice.Clear( xnaBackColor );
            Draw2D( tmpScreenCaptured );
            var bestPtc = PSOManager.globalBestParticle;

            setParamToBodyParts( bestPtc );

            // TODO: Add your drawing code here
            foreach( var parts in bodyParts )
            {
                parts.Update();
                foreach( var mesh in parts.model.Meshes )
                {
                    mesh.Draw();
                }
            }
        }

        #region 初期姿勢をチューニング
        void SetNearDefaultPostion()
        {
            var r = 1;

            for( int i = 0; i < PSOManager.particles.Length; ++i )
            {
                for( int j = 0; j < bodyParts.Count; ++j )
                {
                    r = 1;
                    if( randGen.Next( 0, 1 ) == 0 )
                    {
                        r = -1;
                    }
                    PSOManager.particles[ i ].degreePositions[ j * 3 ] = bodyParts[ j ].angle.X +( float )r * randGen.Next( 10, 15 );
                    PSOManager.particles[ i ].degreePositions[ j * 3 + 1 ] = bodyParts[ j ].angle.Y + ( float )r * randGen.Next( 10, 15 );
                    PSOManager.particles[ i ].degreePositions[ j * 3 + 2 ] = bodyParts[ j ].angle.Z + ( float )r * randGen.Next( 10, 15 );
                }
                PSOManager.particles[ i ].SetLocalBestPositionByCurrentPos();
                PSOManager.particles[ 0 ].degreePositions.CopyTo( PSOManager.globalBestParticle.degreePositions, 0 );
            }
            for( int j = 0; j < bodyParts.Count; ++j )
            {
                r = 1;
                if( randGen.Next( 0, 1 ) == 0 )
                {
                    r = -1;
                }
                PSOManager.prevFrame.degreePositions[ j * 3 ] = bodyParts[ j ].angle.X + ( float )r * randGen.Next( 10, 25 );
                PSOManager.prevFrame.degreePositions[ j * 3 ] = bodyParts[ j ].angle.X + ( float )r * randGen.Next( 10, 25 );
                PSOManager.prevFrame.degreePositions[ j * 3 ] = bodyParts[ j ].angle.X + ( float )r * randGen.Next( 10, 25 );
            }
        }
        #endregion
        void ExecutePSO2()
        {
            ++frameCount;
            //Console.WriteLine( frameCount );
            //Console.WriteLine( PSOManager.indexRem );

            GraphicsDevice.Clear( xnaBackColor );

            for( int i = 0; i < PSOManager.particleNum; ++i )
            {
                //体の各部位に対してangleセット
                var ptc = PSOManager.particles[ i ];
                setParamToBodyParts( ptc );

                PSOManager.evaluations[ i ] = 0f;

                for( int k = 0; k < cameraPositions.Length; ++k )
                {
                    screenCapture.BeginCapture( GraphicsDevice );

                    // TODO: Add your drawing code here
                    foreach( var parts in bodyParts )
                    {
                        parts.ChangeCameraPosition( cameraPositions[ k ].X, cameraPositions[ k ].Y, cameraPositions[ k ].Z );
                        parts.Update();
                        foreach( var mesh in parts.model.Meshes )
                        {
                            mesh.Draw();
                        }
                    }
                    screenCapture.Capture( capturedTextures[ 1 ] );
                    PSOManager.evaluations[ i ] += poseEvaluater.EvaluateTexture( teacherTexture[ k ], capturedTextures[ 1 ] );
                }
            }

            //更新
            PSOManager.Update();


            if( this.frameCount % 10 >= 5 )
            {
                otherView++;
            }

            //最良のモデルポーズを描画
            GraphicsDevice.Clear( xnaBackColor );
            if( visibleAxis )
            {
                Draw2D( teacherTexture[ 0 + otherView % cameraPositions.Length ] );
            }
            var bestPtc = PSOManager.globalBestParticle;//particles[ (frameCount / 10 )% PSOManager.particleNum ];

            setParamToBodyParts( bestPtc );
            
            // TODO: Add your drawing code here
            foreach( var parts in bodyParts )
            {
                parts.ChangeCameraPosition( this.cameraPositions[ 0 + otherView % 3 ].X, this.cameraPositions[ 0 + otherView % 3 ].Y, this.cameraPositions[ 0 + otherView % 3 ].Z );
                parts.Update();
                foreach( var mesh in parts.model.Meshes )
                {
                    mesh.Draw();
                }
            }
        }
        #endregion

        #region Drawメソッド
        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            if( psoFlag )
            {
                ExecutePSO2();
            }
            else
            {
                GraphicsDevice.Clear( xnaBackColor);

                Draw2D();
                // TODO: Add your drawing code here
                foreach( var parts in bodyParts )
                {
                    parts.ChangeCameraPosition( cameraPositions[ 0 ].X, cameraPositions[ 0 ].Y, cameraPositions[ 0 ].Z );
                    parts.Update();
                    foreach( var mesh in parts.model.Meshes )
                    {
                        mesh.Draw();
                    }
                }

                //***************************************************
                //軸の描画 visibleAxis依存
                showAxis();
                //***********************************************

                //ログの描画
                if( this.keyState.IsKeyDown( Keys.Space ) )
                {
                    spriteBatch.Begin();

                    for( int i = 0; i < displayStrings.Count; ++i )
                    {
                        this.spriteBatch.DrawString( this.font, displayStrings[ i ], new Vector2( 0f, ( 15f * i ) ), Color.Black );
                    }

                    spriteBatch.End();
                }

                //************************************************
            }

            primitive.Draw();

            //コマンド
            //消去コマンド(ログのクリア)
            if( this.keyState.IsKeyDown( Keys.C ) )
            {
                this.displayStrings.Clear();
            }

            if( this.keyState.IsKeyDown( Keys.D1 ) )
            {
                this.visibleAxis = true;
            }

            if( this.keyState.IsKeyDown( Keys.D2 ) )
            {
                this.visibleAxis = false;
            }

            if( this.keyState.IsKeyDown( Keys.D3 ) )
            {
                this.psoFlag = true;
            }
            if( this.keyState.IsKeyDown( Keys.D4 ) )
            {
                this.psoFlag = false;
            }
            if( this.keyState.IsKeyDown( Keys.P ) )
            {
                this.ExecuteScreenShotModel();
            }

            // KL; ASD <Shift> キー使用
            modelMoveEvent();
            //************************************************
            base.Draw(gameTime);
        }
        #endregion

        public void Get3DColl()
        {
            Viewport viewport = this.GraphicsDevice.Viewport;
            Vector3 screenPosition = new Vector3( new Vector2(0, 0), 1.0f );

            Ray ray = new Ray( this.cameraPositions[0],
                Vector3.Normalize( new Vector3( 0, 0, 0)));

            foreach( var mesh in body.model.Meshes )
            {
                if( mesh.BoundingSphere.Intersects( ray ) != null )
                {
                    //衝突
                }
            }

        }

    }
}
