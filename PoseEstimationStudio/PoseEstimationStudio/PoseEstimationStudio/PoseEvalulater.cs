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
    class PoseEvaluater
    {
        Color[] pixels1 = new Color[ 640 * 480 ];
        Color[] pixels2 = new Color[ 640 * 480 ];
        Color back = new Color( 68, 34, 136 );

        static int counter = 0;
        /// <summary>
        /// 和と積を返す
        /// </summary>
        /// <param name="texture1"></param>
        /// <param name="texutture2"></param>
        /// <returns></returns>
        public float EvaluateTexture( Texture2D texture1, Texture2D texture2 )
        {
            if( texture1.Height != texture2.Height || texture1.Width != texture2.Width )
            {
                return -1;
            }

            texture1.GetData( pixels1 );
            texture2.GetData( pixels2 );
            
           // texture1.SaveAsJpeg( tex1, 640, 480 );
           // texture2.SaveAsJpeg( tex2, 640, 480 );
            //レンダーターゲットをバックバッファーに戻す
            counter++;
            //tex1.Close();
            //tex2.Close();

            int sum = 0;
            int prod = 0;
            back = pixels1[ 0 ];

            back = pixels2[ 0 ];

            for( int i = 0; i < pixels1.Length; ++i )
            {
                //重なっている
                if( pixels1[ i ] != back && pixels2[ i ] != back )
                {
                    ++prod;
                }

                //どちらか一方は存在する(重なっていても良い)
                if( pixels1[ i ] != back || pixels2[ i ] != back )
                {
                    ++sum;
                }
            }
            var result = 1.0f - ( ( 2.0 * prod ) / ( sum + prod ) );
            return  ( float )( 1.0 - ( ( 2.0 * prod ) / ( sum + prod ) ) );
        }

    }
}
