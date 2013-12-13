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
namespace BoneEstimation
{
    public enum BodyPartsName
    {
        UpperArmR,
        ArmR,
        Size
    }


    public class BodyParts
    {
        public BodyPartsName bodyPartsName;

        public Vector3 basePosition = Vector3.Zero;
        public Vector3 position = Vector3.Zero;
        public Vector3 angle = Vector3.Zero;
        public Vector3 baseAngle = Vector3.Zero;

        public BodyParts parent = null;
        public Vector3 scales = new Vector3( 1f, 1f, 1f );
        public bool usingDefaultEffect = true;
        public Matrix LightView =  Matrix.CreateLookAt( new Vector3( 0f, 0f, 255f ), Vector3.Zero, Vector3.Up );
        public Matrix Projection = Matrix.CreatePerspectiveFieldOfView(
                        MathHelper.ToRadians(45.0f), 640f/480f, 0.00001f, 1000f);
        public Matrix View = Matrix.CreateLookAt( new Vector3( 0f, 0f, 15f ), Vector3.Zero, Vector3.Up );
        public Matrix World = Matrix.CreateTranslation( 0f, 0f, 0f );


        public BodyParts()
        {

        }

        public void Update()
        {
            World = 
                Matrix.CreateRotationX( MathHelper.ToRadians( angle.X + baseAngle.X ) )
                *
                Matrix.CreateRotationY( MathHelper.ToRadians( angle.Y + baseAngle.Y ) )
                *
                Matrix.CreateRotationZ( MathHelper.ToRadians( angle.Z + baseAngle.Z ) )
                *
                Matrix.CreateTranslation( position );
            var p = parent; 
            
            while( p != null )
            {
                World *= 
                Matrix.CreateRotationX( MathHelper.ToRadians( p.angle.X + p.baseAngle.X ) )
                *
                Matrix.CreateRotationY( MathHelper.ToRadians( p.angle.Y + p.baseAngle.Y ) )
                *
                Matrix.CreateRotationZ( MathHelper.ToRadians( p.angle.Z + p.baseAngle.Z ) )
                *
                Matrix.CreateTranslation( p.position );

                p = p.parent;
            }

            World *= Matrix.CreateScale( scales );
        }

        public void UseBasicEffect( Model [] models, BasicEffect basicEffect )
        {
            usingDefaultEffect = true;

            foreach( var mesh in models[ ( int )bodyPartsName ].Meshes )
            {
                for( int i = 0; i < mesh.MeshParts.Count; ++i )
                {
                    mesh.MeshParts[ i ].Effect = basicEffect;
                }
            }
        }

        public void UseEffect( Model [] models, Effect effect )
        {
            usingDefaultEffect = false;
            foreach( var mesh in models[ ( int )bodyPartsName ].Meshes )
            {
                for( int i = 0; i < mesh.MeshParts.Count; ++i )
                {
                    mesh.MeshParts[ i ].Effect = effect;
                }
            }
        }

        public void Draw( Model [] models )
        {
            if( usingDefaultEffect )
            {
                foreach( var mesh in models[ ( int )bodyPartsName ].Meshes )
                {
                    foreach( BasicEffect effect in mesh.Effects )
                    {
                        effect.EnableDefaultLighting();
                        effect.View = View;
                        effect.Projection = Projection;
                        effect.World = World;
                    }
                    mesh.Draw();
                }
            }
            else
            {
                foreach( var mesh in models[ ( int )bodyPartsName ].Meshes )
                {
                    foreach( Effect effect in mesh.Effects )
                    {
                        effect.Parameters[ "View" ].SetValue( View );
                        effect.Parameters[ "Projection" ].SetValue( Projection );
                        effect.Parameters[ "LightProjection" ].SetValue( Projection );
                        effect.Parameters[ "World" ].SetValue( World );
                    }
                    mesh.Draw();
                }
            }
        }

    }
}
