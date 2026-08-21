#version 330 compatibility

uniform int uColorMode;
uniform sampler1D uTurboPalette;
uniform sampler1D uLodPalette;
uniform float uHeightMin;
uniform float uHeightMax;

in vec4 vOriginalColor;
flat in int vLodLevel;
in float vWorldHeight;
out vec4 fragmentColor;

void main()
{
    vec2 pointOffset = gl_PointCoord * 2.0 - vec2(1.0);
    if (dot(pointOffset, pointOffset) > 1.0) {
        discard;
    }

    vec4 color = vOriginalColor;
    if (uColorMode == 1) {
        int paletteIndex = vLodLevel % 8;
        color = texture(uLodPalette, (float(paletteIndex) + 0.5) / 8.0);
    } else if (uColorMode == 2) {
        float range = uHeightMax - uHeightMin;
        float heightFactor = abs(range) > 1e-6
            ? clamp((vWorldHeight - uHeightMin) / range, 0.0, 1.0)
            : 0.5;
        color = texture(uTurboPalette, heightFactor);
    }

    fragmentColor = color;
}
