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

void fragment() {
	vec3 c = texture(albedo_tex, UV).rgb;
	c = c * gain + offset;                                 // affine
	c = (c - vec3(0.5)) * contrast + vec3(0.5);            // contrast about mid-grey
	float luma = dot(c, vec3(0.2126, 0.7152, 0.0722));
	c = mix(vec3(luma), c, saturation);                    // desaturate/oversaturate
	ALBEDO = clamp(c, vec3(0.0), vec3(1.0));
}
