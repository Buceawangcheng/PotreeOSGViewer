#version 330 compatibility

uniform float uPointSize;
uniform int uLodLevel;

out vec4 vOriginalColor;
flat out int vLodLevel;
out float vHeightAboveMinimum;

void main()
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    gl_PointSize = max(uPointSize, 1.0);
    vOriginalColor = gl_Color;
    vLodLevel = uLodLevel;
    vHeightAboveMinimum = gl_Vertex.z;
}
