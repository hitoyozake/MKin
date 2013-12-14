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

        public Model[] models = new Model[ ( int )BodyPartsName.Size ];
        public Color XNABackColor = new Color( 68, 34, 136 );

        public Effect depthEffect;
        public BasicEffect basicEffect;

        public BodyParts [] bodyParts;
        public ScreenCapture2D captureDevice = new ScreenCapture2D();
        public int width = 640, height = 480;
        public Texture2D TeacherTex;
        public Texture2D tmpTexture;
        public Texture2D PartsTexture;

        public Color[] tmpPixels = new Color[ 640 * 480 ];

        public Color[] copyPixels = new Color[ 640 * 480 ];


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
        }

        /// <summary>
        /// ÉÇÉfÉãÇì«Ç›çûÇﬁ
        /// </summary>
        void LoadModel()
        {
            models[ ( int )BodyPartsName.ArmR ] = Content.Load<Model>( "rarm" );
            models[ ( int )BodyPartsName.UpperArmR ] = Content.Load<Model>( "ruarm" );


            bodyParts = new BodyParts[ ( int )BodyPartsName.Size ];

            for( int i = 0; i < ( int )BodyPartsName.Size; ++i )
            {
                bodyParts[ i ] = new BodyParts();
                bodyParts[ i ].bodyPartsName = ( BodyPartsName ) i;
            }

            bodyParts[ ( int )BodyPartsName.ArmR ].parent =
                bodyParts[ ( int )BodyPartsName.UpperArmR ];

            bodyParts[ ( int )BodyPartsName.ArmR ].position.Y = 1f;

        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch( GraphicsDevice );

            LoadModel();

            // TODO: use this.Content to load your game content here
            depthEffect = Content.Load<Effect>( "depth" );
            basicEffect = new BasicEffect( GraphicsDevice );

            InitializeTexture();

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
            // Allows the game to exit
            if( GamePad.GetState( PlayerIndex.One ).Buttons.Back == ButtonState.Pressed )
                this.Exit();

            // TODO: Add your update logic here


            base.Update( gameTime );
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw( GameTime gameTime )
        {
            bodyParts[ 0 ].angle.Z += 1f;
            bodyParts[ 1 ].angle.Z = 87f;
            
            
            GraphicsDevice.Clear( XNABackColor );

            // TODO: Add your drawing code here
            bodyParts[ 0 ].Update();
            bodyParts[ 1 ].Update();

            bodyParts[ 0 ].Draw( models );
            bodyParts[ 1 ].Draw( models );


            base.Draw( gameTime );
        }

        /// <summary>
        /// èIóπèàóù
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="args"></param>
        protected override void OnExiting( object sender, EventArgs args )
        {
            base.OnExiting( sender, args );
        }
    }
}
