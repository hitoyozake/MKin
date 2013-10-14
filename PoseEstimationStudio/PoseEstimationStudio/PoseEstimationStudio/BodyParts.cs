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

namespace PoseEstimationStudio
{

    enum BodyPartsName
    {
        Head,//頭
        Trunk,//体幹
        UnderTrunk,
        UpperArmR,
        UpperArmL,
        ArmR,
        ArmL,
        HandR,
        HandL,
        ThighR,
        ThighL,
        LegR,
        LegL,
        FootR,
        FootL,
        Size
    }

    //あるパーツに対する初期相対座標を全て移動させてから回転させれば良い

    class BodyParts
    {
        public String partsName = "";
        public Model model;
        
        //親パーツとの相対角度
        public Vector3 angle = new Vector3( 0f, 0f, 0f );
        public Vector3 baseAngle = new Vector3( 0f, 0f, 0f );

        //親パーツとの相対位置
        public Vector3 position = new Vector3(0f, 0f, 0f);
        public BodyParts parent = null;
        public Vector3 modelSize = new Vector3(0f, 0.5f, 0f);
        public float modelAngle = 90.0f;
        public bool updated = false;

        GraphicsDevice device;

        public BodyParts()
        {
        }

        public void InitializeParam( String partsName, float x, float y, float z )
        {
            this.partsName = partsName;
            position.X = x;
            position.Y = y;
            position.Z = z;
        }

        public void ChangeCameraPosition( float x, float y, float z )
        {
            Vector3 cameraPos = new Vector3( x, y, z );

            foreach( var mesh in model.Meshes )
            {
                foreach( BasicEffect effect in mesh.Effects )
                {
                    effect.EnableDefaultLighting();
                    effect.View = Matrix.CreateLookAt( cameraPos, Vector3.Zero, Vector3.Up );
                    
                    effect.Projection = Matrix.CreatePerspectiveFieldOfView(
                        MathHelper.ToRadians( 45.0f ), ( float )device.Viewport.Width /
                        ( float )device.Viewport.Height, 1f, 100f );
                }
            }
        }

        public void Init(GraphicsDevice graphicsDevice)
        {
            device = graphicsDevice;
            Vector3 cameraPosition = new Vector3(0f, 0, 25f);
            foreach (var mesh in model.Meshes)
            {
                foreach (BasicEffect effect in mesh.Effects)
                {
                    effect.EnableDefaultLighting();
                    effect.View = Matrix.CreateLookAt(cameraPosition, Vector3.Zero, Vector3.Up);

                    effect.Projection = Matrix.CreatePerspectiveFieldOfView(
                        MathHelper.ToRadians(45.0f), (float)graphicsDevice.Viewport.Width /
                        (float)graphicsDevice.Viewport.Height, 1f, 100f);
                }
            }
        }


        public void Update()
        {
            foreach (var mesh in model.Meshes)
            {
                foreach( BasicEffect effect in mesh.Effects )
                {
                    effect.World = Matrix.CreateTranslation(modelSize) *
                        //Matrix.CreateRotationZ(MathHelper.ToRadians(modelAngle)) *
                        Matrix.CreateRotationX( MathHelper.ToRadians( angle.X + baseAngle.X ) ) *
                        Matrix.CreateRotationY( MathHelper.ToRadians( angle.Y + baseAngle.Y) ) *
                        Matrix.CreateRotationZ( MathHelper.ToRadians( angle.Z + baseAngle.Z) ) * 
                        Matrix.CreateTranslation(position);//相対座標移動


                    var p = parent;
                    while( p != null )
                    {
                        effect.World *= Matrix.CreateRotationX( MathHelper.ToRadians( p.angle.X + p.baseAngle.X ) ) *
                        Matrix.CreateRotationY( MathHelper.ToRadians( p.angle.Y + p.baseAngle.Y ) ) *
                        Matrix.CreateRotationZ( MathHelper.ToRadians( p.angle.Z + p.baseAngle.Z ) ) *
                        Matrix.CreateTranslation(p.position);

                        p = p.parent;
                    }

                }
            }
        }

    }
}
