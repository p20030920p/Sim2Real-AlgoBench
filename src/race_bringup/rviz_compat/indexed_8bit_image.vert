#version 420 compatibility

// Draws an 8-bit image using RGB values from a 256x1 palette texture.
// This vertex shader just passes through the UV texture coordinates.
// I'm not sure if it is necessary or not.

in vec4 uv0;
out vec2 UV;

void main()
{
  gl_Position = ftransform();
  UV = vec2(uv0);
}
