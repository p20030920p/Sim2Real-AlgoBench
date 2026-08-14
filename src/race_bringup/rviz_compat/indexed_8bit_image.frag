#version 420 compatibility

// Draws an 8-bit image using RGB values from a 256x1 palette texture.

in vec2 UV;
// Mesa validates unlike sampler types during program linking, before OGRE's
// material parameters are applied. Bind distinct units at link time.
layout(binding = 0) uniform sampler2D eight_bit_image;
layout(binding = 1) uniform sampler1D palette;
uniform float alpha;

void main()
{
  // The 0.999 multiply is needed because brightness value 255 comes
  // out of texture2D() as 1.0, which wraps around to 0.0 in the
  // palette texture.
  vec4 color = texture( palette, 0.999 * texture( eight_bit_image, UV ).x );
  gl_FragColor = vec4( color.rgb, color.a * alpha );
}
