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
    class PointPrimitive
    {
        public VertexDeclaration vertexDeclaration;
        public VertexPositionNormalTexture[] pointList;
        public VertexBuffer vertexBuffer;
        public int points = 6;
        public BasicEffect basicEffect;
        public Matrix viewMatrix;
        public Matrix projectionMatrix;
        public GraphicsDevice graphicsDevice = null;
        public Vector3 position = new Vector3();
        public float scale = 0.2f;

        private void InitializeView()
        {
            //カメラ
            viewMatrix = Matrix.CreateLookAt( new Vector3( 0f, 0f, 10f ), Vector3.Zero, Vector3.Up );
            
            projectionMatrix = Matrix.CreatePerspectiveFieldOfView(
            MathHelper.ToRadians( 45 ),
            ( float )graphicsDevice.Viewport.Width /
            ( float )graphicsDevice.Viewport.Height, 1.0f, 100.0f );

        }

        private void InitializeEffect()
        {
            basicEffect = new BasicEffect( graphicsDevice );

            //放散する色
            basicEffect.DiffuseColor = new Vector3( 1f, 1f, 1f );

            basicEffect.View = viewMatrix;

            basicEffect.Projection = projectionMatrix;
        }

        public void Initialize( GraphicsDevice gd )
        {
            this.graphicsDevice = gd;

            InitializeView();
            InitializeEffect();

            vertexDeclaration = new VertexDeclaration( VertexPositionNormalTexture.VertexDeclaration.GetVertexElements() );

            double angle = MathHelper.TwoPi / 8;

            pointList = new VertexPositionNormalTexture[ points ];
            //for( int i = 1; i <= points; ++i )
            //{
            //    pointList[ i ] = new VertexPositionNormalTexture(
            //        new Vector3( ( float )Math.Round( Math.Sin( angle * i ), 4 ),
            //            ( float )Math.Round( Math.Cos( angle * i ), 4 ),
            //            0.0f ), Vector3.Backward, new Vector2() );
            //}
            pointList[ 0 ] = new VertexPositionNormalTexture(
                   new Vector3( position.X + scale * ( float )Math.Round( Math.Sin( angle * 1 ), 4 ),
                       position.Y + scale * ( float )Math.Round( Math.Cos( angle * 1 ), 4 ),
                       0.0f ), Vector3.Backward, new Vector2() );

            pointList[ 1 ] = new VertexPositionNormalTexture(
                 new Vector3( position.X + scale * ( float )Math.Round( Math.Sin( angle * 3 ), 4 ),
                     position.Y + scale * ( float )Math.Round( Math.Cos( angle * 3 ), 4 ),
                     0.0f ), Vector3.Backward, new Vector2() );

            pointList[ 2 ] = new VertexPositionNormalTexture(
                 new Vector3( position.X + scale * ( float )Math.Round( Math.Sin( angle * 5 ), 4 ),
                     position.Y + scale * ( float )Math.Round( Math.Cos( angle * 5 ), 4 ),
                     0.0f ), Vector3.Backward, new Vector2() );

            pointList[ 3 ] = new VertexPositionNormalTexture(
                 new Vector3( position.X + scale * ( float )Math.Round( Math.Sin( angle * 5 ), 4 ),
                     position.Y + scale * ( float )Math.Round( Math.Cos( angle * 5 ), 4 ),
                     0.0f ), Vector3.Backward, new Vector2() );

            pointList[ 4 ] = new VertexPositionNormalTexture(
                 new Vector3( position.X + scale * ( float )Math.Round( Math.Sin( angle * 7 ), 4 ),
                     position.Y + scale * ( float )Math.Round( Math.Cos( angle * 7 ), 4 ),
                     0.0f ), Vector3.Backward, new Vector2() );

            pointList[ 5 ] = new VertexPositionNormalTexture(
                 new Vector3( position.X + scale * ( float )Math.Round( Math.Sin( angle * 1 ), 4 ),
                     position.Y + scale * ( float )Math.Round( Math.Cos( angle * 1 ), 4 ),
                     0.0f ), Vector3.Backward, new Vector2() );

            //頂点バッファを初期化し，拡張点についてメモリを割り当てる
            vertexBuffer = new VertexBuffer( graphicsDevice, 
                VertexPositionNormalTexture.VertexDeclaration,
                pointList.Length, BufferUsage.None );

            //頂点の配列に頂点バッファのデータを設定
            vertexBuffer.SetData( pointList );
        }
    
        public void Draw()
        {
            foreach( EffectPass pass in basicEffect.CurrentTechnique.Passes )
            {
                pass.Apply();
                
                
                graphicsDevice.SetVertexBuffer( this.vertexBuffer );
                
                graphicsDevice.DrawPrimitives( PrimitiveType.TriangleList, 0, points + 1 );

            }
        }


    }
}
