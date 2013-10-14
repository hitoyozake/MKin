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
    class ScreenCapture2D
    {
        public GraphicsDevice graphicsDevice = null;
        public RenderTarget2D renderTarget = null;
        public Texture2D tmpTexture;
        public Color[] tmpPixels = new Color[ 640 * 480 ];
        public ScreenCapture2D()
        {
        }

        public void Init( GraphicsDevice device )
        {
            if( graphicsDevice == null )
            {
                graphicsDevice = device;
            }

            if( renderTarget == null )
            {
                renderTarget = new RenderTarget2D( device, 640, 480, true, SurfaceFormat.Color, DepthFormat.Depth24 );
            }
        }

        public void BeginCapture( GraphicsDevice gd )
        {
            //graphicsDevice.Clear( Color.White );
            gd.SetRenderTarget( renderTarget );
        }

        public Texture2D Capture( Texture2D texture )
        {
            //キャプチャ
            tmpTexture = renderTarget;

            //var x = System.IO.File.Create("hoge.jpg");
            //レンダーターゲットをバックバッファーに戻す
            graphicsDevice.SetRenderTarget( null );

            tmpTexture.GetData( tmpPixels );

            texture.SetData( tmpPixels );

            //texture.SaveAsJpeg(System.IO.File.Create( "tex" + ".jpg" ), 640, 480);

            //x.Close();

            return texture;
        }
    }
}
