// terrain_grade.gdshader — runtime color grade + conformal grid for terrain tiles.
//
// Applied as a per-tile material_override by TerrainLoader.gd: the tile's original glTF albedo
// texture is passed in `albedo_tex`, and the grade + grid are driven live from TerrainLoader's
// exported controls (sourced from configs/viewer.json).
//
// Color pipeline (per channel unless noted):
//   c = albedo * gain + offset        // affine: multiplicative factor + additive offset
//   c = (c - 0.5) * contrast + 0.5    // contrast about mid-grey
//   c = mix(luma(c), c, saturation)   // saturation (cross-channel, BT.709 luma)
//   c = c * value                     // scalar value/brightness (uniform darken/lighten)
//   c = mix(c, grid_color, cover)     // conformal grid overlay (independent color)
//
// The terrain is forced fully matte (SPECULAR = 0, ROUGHNESS = 1, METALLIC = 0): the default
// specular is a white highlight that does NOT scale with albedo, so dialing the albedo down would
// neutralize the surface toward grey instead of darkening it.
//
// Conformal grid (§Conformal Terrain Grid): north/east-aligned lines at equal metric spacing from
// the terrain-region centroid (= world origin).  Computed per fragment from the world-ENU position
// (a `spatial` shader, so it is on the terrain surface — never above/below it, at any LOD — and
// not baked into the texture).  grid_color/grid_opacity are independent of the color grade above.
//
// Output is written to ALBEDO (a lit spatial shader), so scene lighting and shadows still apply.

shader_type spatial;
render_mode cull_back;

uniform sampler2D albedo_tex : source_color, filter_linear_mipmap;
uniform vec3  gain = vec3(1.0);    // multiplicative factor, per channel
uniform vec3  offset = vec3(0.0);  // additive offset, per channel (may be negative)
uniform float contrast : hint_range(0.0, 3.0) = 1.0;
uniform float saturation : hint_range(0.0, 3.0) = 1.0;
uniform float value : hint_range(0.0, 3.0) = 1.0;      // scalar brightness/value multiplier
uniform float specular : hint_range(0.0, 1.0) = 0.0;   // 0 = matte (albedo-independent highlight off)

// Conformal reference grid — a major level and an independent minor level (drawn under major).
uniform float grid_enabled = 0.0;                      // 0 = off, 1 = on (both levels)
uniform float grid_spacing_m = 1000.0;                 // major line spacing, metres from centroid
uniform vec3  grid_color : source_color = vec3(0.0, 1.0, 0.0);
uniform float grid_opacity : hint_range(0.0, 1.0) = 0.5;
uniform float grid_width_m = 5.0;                      // major physical line width, metres
uniform float grid_minor_divisions = 5.0;              // minor spacing = major / divisions (< 2 = no minor)
uniform vec3  grid_minor_color : source_color = vec3(0.0, 1.0, 0.0);
uniform float grid_minor_opacity : hint_range(0.0, 1.0) = 0.2;
uniform float grid_minor_width_m = 2.0;                // minor physical line width, metres

varying highp vec3 v_world;  // fragment world position (world-origin ENU: x=east, y=up, z=-north)

// Coverage of the nearest grid line to `coord` on one axis — mirrors grid_overlay.line_coverage.
// `aa` is the pixel size in metres (clamped fwidth): analytic 1-D pixel coverage, robust at
// grazing angles (a derivative-scaled smoothstep would over-widen / NaN and fill the screen).
float grid_line(float coord, float spacing, float width_m, float aa) {
	float g = coord / spacing;
	float f = abs(fract(g - 0.5) - 0.5);   // grid units to nearest line, [0, 0.5]
	float d = f * spacing;                 // metres to nearest line (>= 0)
	float half_w = 0.5 * width_m;
	float overlap = min(half_w, d + 0.5 * aa) - max(-half_w, d - 0.5 * aa);
	return clamp(overlap / aa, 0.0, 1.0);
}

// Minor-line coverage on one axis, suppressed where the minor line coincides with a major line
// (so a coincident intersection shows only the major line) — mirrors grid_overlay.minor_line_coverage.
float minor_line(float coord, float minor_spacing, float major_spacing, float width_m, float aa) {
	float cov = grid_line(coord, minor_spacing, width_m, aa);
	if (major_spacing > 0.0) {
		float nearest = round(coord / minor_spacing) * minor_spacing;  // nearest minor line position
		float ratio = nearest / major_spacing;
		if (abs(ratio - round(ratio)) < 1e-3) {
			cov = 0.0;   // this minor line is also a major line — skip it
		}
	}
	return cov;
}

void vertex() {
	v_world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	vec3 c = texture(albedo_tex, UV).rgb;
	c = c * gain + offset;                                 // affine
	c = (c - vec3(0.5)) * contrast + vec3(0.5);            // contrast about mid-grey
	float luma = dot(c, vec3(0.2126, 0.7152, 0.0722));
	c = mix(vec3(luma), c, saturation);                    // desaturate/oversaturate
	c *= value;                                            // scalar value/brightness

	// Conformal grid: on-surface, north/east aligned, centroid-anchored, LOD-independent.
	// Minor level first, major level on top; each independent and independent of the grade.
	// (fwidth clamped so grazing angles — huge fwidth — don't blow up and fill the screen.)
	if (grid_enabled > 0.5) {
		float e = v_world.x;    // east
		float n = -v_world.z;   // north
		if (grid_spacing_m > 0.0 && grid_minor_divisions >= 2.0) {
			float minor_spacing = grid_spacing_m / grid_minor_divisions;
			float aa_e = clamp(fwidth(e), 1e-4, minor_spacing);
			float aa_n = clamp(fwidth(n), 1e-4, minor_spacing);
			float cover = max(minor_line(e, minor_spacing, grid_spacing_m, grid_minor_width_m, aa_e),
			                  minor_line(n, minor_spacing, grid_spacing_m, grid_minor_width_m, aa_n));
			c = mix(c, grid_minor_color, cover * grid_minor_opacity);
		}
		if (grid_spacing_m > 0.0) {
			float aa_e = clamp(fwidth(e), 1e-4, grid_spacing_m);
			float aa_n = clamp(fwidth(n), 1e-4, grid_spacing_m);
			float cover = max(grid_line(e, grid_spacing_m, grid_width_m, aa_e),
			                  grid_line(n, grid_spacing_m, grid_width_m, aa_n));
			c = mix(c, grid_color, cover * grid_opacity);
		}
	}

	ALBEDO = clamp(c, vec3(0.0), vec3(1.0));
	ROUGHNESS = 1.0;
	METALLIC = 0.0;
	SPECULAR = specular;  // default 0 = matte; the highlight does not scale with albedo
}
