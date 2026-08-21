#version 330 compatibility

uniform float uPointSize;
uniform int uLodLevel;
uniform float uNodeOriginZ;

out vec4 vOriginalColor;
flat out int vLodLevel;
out float vWorldHeight;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    gl_PointSize = max(uPointSize, 1.0);
    vOriginalColor = gl_Color;
    vLodLevel = uLodLevel;
    vWorldHeight = gl_Vertex.z + uNodeOriginZ;
}
