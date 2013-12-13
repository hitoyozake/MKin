float4x4 World;
float4x4 View;
float4x4 Projection;
float4x4 LightView;
float4x4 LightProjection;
texture ShadowMap;
// TODO: add effect parameters here.


// 頂点シェーダへ入力する情報
struct ShadowVertexShaderInput
{
	float4 Position : POSITION0;

	// UV座標
	//float2 TexCoord : TEXCOORD0;
};

// 頂点シェーダから出力される情報
struct ShadowVertexShaderOutput
{
	float4 Position : POSITION0;
	// 深度
	// 深度
	float4 Depth : TEXCOORD0;
};

//// ピクセルシェーダ
//float4 ShadowPixelShaderFunction( ShadowVertexShaderOutput input ) : COLOR0
//{
//	float depth = input.Depth.z;// ( input.Depth.z - 10 ) / 16;//( input.Depth.z / input.Depth.w );
//	float r = 0, g = 0, b = 0;
//	if( depth > 10 && depth <= 80 )
//	{
//		b = -( depth - 10.0f ) / 70.0f;
//	}
//
//	if( depth > 1.1 && depth <= 10 )
//	{
//		r = ( 8.5f - depth ) / 8.9f;
//		b = 1.0f;
//	}
//	if( depth >= 0 && depth <= 0.6f )
//	{
//		b = 0.0f; //1.0f;
//		r = 0.0f; //1.0f;
//		g = ( 0.6f - depth ) / 0.6f;
//	}
//
//	return float4( r, g, b, 1.0f );
//}
// ピクセルシェーダ
float4 ShadowPixelShaderFunction( ShadowVertexShaderOutput input ) : COLOR0
{
	float depth = input.Depth.z;// ( input.Depth.z - 10 ) / 16;//( input.Depth.z / input.Depth.w );
	float r = 0, g = 0, b = 0;

	if( depth > 0 && depth <= 1 )
	{
		r = ( 1.0f - depth ) / 1.0f;
		b = 1.0f;
	}

	return float4( r, g, b, 1.0f );
}

// 頂点シェーダ
ShadowVertexShaderOutput ShadowVertexShaderFunction( ShadowVertexShaderInput input )
{
	ShadowVertexShaderOutput output;

	float4 worldPosition = mul( input.Position, World );
		float4 viewPosition = mul( worldPosition, View );
		output.Position = mul( viewPosition, Projection );

	output.Depth = output.Position; // 出力する深度情報の作成
	//output.Depth = 1.0f - output.Position.z / output.Position.w;
	return output;
}


technique Technique1
{
	pass Pass1
	{
		// TODO: set renderstates here.
		VertexShader = compile vs_2_0 ShadowVertexShaderFunction( );
		PixelShader = compile ps_2_0 ShadowPixelShaderFunction( );
	}
}
