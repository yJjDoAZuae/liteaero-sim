// terrain_grade.gdshader — runtime color grade for terrain tiles.
//
// Applied as a per-tile material_override by TerrainLoader.gd: the tile's original
// glTF albedo texture is passed in `albedo_tex`, and the grade is driven live from
// TerrainLoader's exported Terrain Appearance controls (sourced from configs/viewer.json).
//
// Color pipeline (per channel unless noted):
//   c = albedo * gain + offset        // affine: multiplicative factor + additive offset
//   c = (c - 0.5) * contrast + 0.5    // contrast about mid-grey
//   c = mix(luma(c), c, saturation)   // saturation (cross-channel, BT.709 luma)
//   c = c * value                     // scalar value/brightness (uniform darken/lighten)
//
// The terrain is forced fully matte (SPECULAR = 0, ROUGHNESS = 1, METALLIC = 0): the
// default specular is a white highlight that does NOT scale with albedo, so dialing the
// albedo down would neutralize the surface toward grey instead of darkening it. With
// specular off, `value` (and `gain`) cleanly control the rendered brightness.
//
// Output is written to ALBEDO (a lit spatial shader), so scene lighting and shadows
// still apply exactly as with the original StandardMaterial3D.

shader_type spatial;
render_mode cull_back;

uniform sampler2D albedo_tex : source_color, filter_linear_mipmap;
uniform vec3  gain = vec3(1.0);    // multiplicative factor, per channel
uniform vec3  offset = vec3(0.0);  // additive offset, per channel (may be negative)
uniform float contrast : hint_range(0.0, 3.0) = 1.0;
uniform float saturation : hint_range(0.0, 3.0) = 1.0;
uniform float value : hint_range(0.0, 3.0) = 1.0;      // scalar brightness/value multiplier
uniform float specular : hint_range(0.0, 1.0) = 0.0;   // 0 = matte (albedo-independent highlight off)

void fragment() {
	vec3 c = texture(albedo_tex, UV).rgb;
	c = c * gain + offset;                                 // affine
	c = (c - vec3(0.5)) * contrast + vec3(0.5);            // contrast about mid-grey
	float luma = dot(c, vec3(0.2126, 0.7152, 0.0722));
	c = mix(vec3(luma), c, saturation);                    // desaturate/oversaturate
	c *= value;                                            // scalar value/brightness
	ALBEDO = clamp(c, vec3(0.0), vec3(1.0));
	ROUGHNESS = 1.0;
	METALLIC = 0.0;
	SPECULAR = specular;  // default 0 = matte; the highlight does not scale with albedo
}
