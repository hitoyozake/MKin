xof 0302txt 0064
template Header {
 <3D82AB43-62DA-11cf-AB39-0020AF71E433>
 WORD major;
 WORD minor;
 DWORD flags;
}

template Vector {
 <3D82AB5E-62DA-11cf-AB39-0020AF71E433>
 FLOAT x;
 FLOAT y;
 FLOAT z;
}

template Coords2d {
 <F6F23F44-7686-11cf-8F52-0040333594A3>
 FLOAT u;
 FLOAT v;
}

template Matrix4x4 {
 <F6F23F45-7686-11cf-8F52-0040333594A3>
 array FLOAT matrix[16];
}

template ColorRGBA {
 <35FF44E0-6C7C-11cf-8F52-0040333594A3>
 FLOAT red;
 FLOAT green;
 FLOAT blue;
 FLOAT alpha;
}

template ColorRGB {
 <D3E16E81-7835-11cf-8F52-0040333594A3>
 FLOAT red;
 FLOAT green;
 FLOAT blue;
}

template IndexedColor {
 <1630B820-7842-11cf-8F52-0040333594A3>
 DWORD index;
 ColorRGBA indexColor;
}

template Boolean {
 <4885AE61-78E8-11cf-8F52-0040333594A3>
 WORD truefalse;
}

template Boolean2d {
 <4885AE63-78E8-11cf-8F52-0040333594A3>
 Boolean u;
 Boolean v;
}

template MaterialWrap {
 <4885AE60-78E8-11cf-8F52-0040333594A3>
 Boolean u;
 Boolean v;
}

template TextureFilename {
 <A42790E1-7810-11cf-8F52-0040333594A3>
 STRING filename;
}

template Material {
 <3D82AB4D-62DA-11cf-AB39-0020AF71E433>
 ColorRGBA faceColor;
 FLOAT power;
 ColorRGB specularColor;
 ColorRGB emissiveColor;
 [...]
}

template MeshFace {
 <3D82AB5F-62DA-11cf-AB39-0020AF71E433>
 DWORD nFaceVertexIndices;
 array DWORD faceVertexIndices[nFaceVertexIndices];
}

template MeshFaceWraps {
 <4885AE62-78E8-11cf-8F52-0040333594A3>
 DWORD nFaceWrapValues;
 Boolean2d faceWrapValues;
}

template MeshTextureCoords {
 <F6F23F40-7686-11cf-8F52-0040333594A3>
 DWORD nTextureCoords;
 array Coords2d textureCoords[nTextureCoords];
}

template MeshMaterialList {
 <F6F23F42-7686-11cf-8F52-0040333594A3>
 DWORD nMaterials;
 DWORD nFaceIndexes;
 array DWORD faceIndexes[nFaceIndexes];
 [Material]
}

template MeshNormals {
 <F6F23F43-7686-11cf-8F52-0040333594A3>
 DWORD nNormals;
 array Vector normals[nNormals];
 DWORD nFaceNormals;
 array MeshFace faceNormals[nFaceNormals];
}

template MeshVertexColors {
 <1630B821-7842-11cf-8F52-0040333594A3>
 DWORD nVertexColors;
 array IndexedColor vertexColors[nVertexColors];
}

template Mesh {
 <3D82AB44-62DA-11cf-AB39-0020AF71E433>
 DWORD nVertices;
 array Vector vertices[nVertices];
 DWORD nFaces;
 array MeshFace faces[nFaces];
 [...]
}

Header{
1;
0;
1;
}

Mesh {
 21;
 0.00000;0.00000;-1.40000;,
 0.00000;-1.12000;0.00000;,
 0.45458;0.00000;-1.32414;,
 0.85990;0.00000;-1.10480;,
 1.17203;0.00000;-0.76573;,
 1.35716;0.00000;-0.34368;,
 1.39522;-0.00000;0.11561;,
 1.28208;-0.00000;0.56237;,
 1.03001;-0.00000;0.94819;,
 0.66633;-0.00000;1.23126;,
 0.23043;-0.00000;1.38091;,
 -0.23043;0.00000;1.38091;,
 -0.66633;0.00000;1.23126;,
 -1.03001;0.00000;0.94819;,
 -1.28208;0.00000;0.56237;,
 -1.39522;0.00000;0.11561;,
 -1.35716;0.00000;-0.34368;,
 -1.17203;0.00000;-0.76573;,
 -0.85990;0.00000;-1.10480;,
 -0.45458;0.00000;-1.32415;,
 0.00000;0.00000;0.00000;;
 
 38;
 3;2,1,0;,
 3;3,1,2;,
 3;4,1,3;,
 3;5,1,4;,
 3;6,1,5;,
 3;7,1,6;,
 3;8,1,7;,
 3;9,1,8;,
 3;10,1,9;,
 3;11,1,10;,
 3;12,1,11;,
 3;13,1,12;,
 3;14,1,13;,
 3;15,1,14;,
 3;16,1,15;,
 3;17,1,16;,
 3;18,1,17;,
 3;19,1,18;,
 3;0,1,19;,
 3;2,0,20;,
 3;3,2,20;,
 3;4,3,20;,
 3;5,4,20;,
 3;6,5,20;,
 3;7,6,20;,
 3;8,7,20;,
 3;9,8,20;,
 3;10,9,20;,
 3;11,10,20;,
 3;12,11,20;,
 3;13,12,20;,
 3;14,13,20;,
 3;15,14,20;,
 3;16,15,20;,
 3;17,16,20;,
 3;18,17,20;,
 3;19,18,20;,
 3;0,19,20;;
 
 MeshMaterialList {
  1;
  38;
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0;;
  Material {
   1.000000;1.000000;1.000000;1.000000;;
   0.000000;
   0.000000;0.000000;0.000000;;
   0.000000;0.000000;0.000000;;
  }
 }
}
