// Vertex shader

struct VertexInput {
    [[location(0)]] position: vec2<f32>;
	[[location(1)]] motion: vec2<f32>;
	[[location(2)]] contains_tracked: f32;
};

struct VertexOutput {
    [[builtin(position)]] clip_position: vec4<f32>;
	[[location(0)]] color: vec2<f32>;
	[[location(1)]] tex_coords: vec2<f32>;
	[[location(2)]] contains_tracked: f32;
};

let pi_approx: f32 = 3.1415;

fn motion_to_rgb(
	motion: vec2<f32>
) -> vec3<f32> {
	var angle = (atan2(motion.x, motion.y) + pi_approx) / (2.0 * pi_approx);
	var magnitude = length(motion);
	var K = vec4<f32>(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
	var p = abs(fract(vec3<f32>(angle) + K.xyz) * 6.0 - K.www);
	return magnitude * clamp(p - K.xxx, vec3<f32>(0.0), vec3<f32>(1.0));
}

[[stage(vertex)]]
fn vs_main(
    [[builtin(vertex_index)]] in_vertex_index: u32,
    model: VertexInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = vec4<f32>(model.position.x, model.position.y, 0.0, 1.0);
	out.color = model.motion;
	out.tex_coords = (model.position * vec2<f32>(1.0, -1.0) + vec2<f32>(1.0)) * 0.5;
	out.contains_tracked = model.contains_tracked;
    return out;
}

[[group(0), binding(0)]]
var t_diffuse: texture_2d<f32>;
[[group(0), binding(1)]]
var s_diffuse: sampler;

[[stage(fragment)]]
fn fs_main(in: VertexOutput) -> [[location(0)]] vec4<f32> {
	var color = motion_to_rgb(in.color) * 0.1;
	var frame = textureSample(t_diffuse, s_diffuse, in.tex_coords) * 0.5;
	var tracked = vec4<f32>(1.0, 0.0, 0.0, 1.0) * in.contains_tracked;
	return (frame * (1.0 - length(color))) + 5.0 * vec4<f32>(color, 1.0) + tracked;
}
