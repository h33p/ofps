// Line renderer shader

struct VertexInput {
	[[location(0)]] colour: vec4<f32>;
    [[location(1)]] position: vec3<f32>;
	[[location(2)]] normal: vec3<f32>;
	[[location(3)]] uv: vec2<f32>;
};

struct InstanceInput {
	[[location(4)]] proj1: vec4<f32>;
	[[location(5)]] proj2: vec4<f32>;
	[[location(6)]] proj3: vec4<f32>;
	[[location(7)]] proj4: vec4<f32>;
	[[location(8)]] colour: vec4<f32>;
};

struct CameraUniform {
	proj: mat4x4<f32>;
	res: vec2<f32>;
};

[[group(0), binding(0)]]
var<uniform> uniform_buf: CameraUniform;

struct VertexOutput {
    [[builtin(position)]] clip_position: vec4<f32>;
	[[location(0)]] colour: vec4<f32>;
	[[location(1)]] tex_coords: vec2<f32>;
};

[[stage(vertex)]]
fn vs_main(
    [[builtin(vertex_index)]] in_vertex_index: u32,
    model: VertexInput,
	instance: InstanceInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = vec4<f32>(model.position.x, model.position.y, 0.0, 1.0);

	var start = uniform_buf.proj * instance.proj1;
	var end = uniform_buf.proj * instance.proj2;
	var dir = normalize(end - start);
	var cross_dir = cross(vec3<f32>(0.0, 0.0, 1.0), dir.xyz);
	var res = vec2<f32>(1.0) / uniform_buf.res;
	//var res = vec2<f32>(1.0) / vec2<f32>(1280.0, 720.0);
	var thickness = instance.proj3.xx * res;
	var pos_base = start + (end - start) * model.position.y;
	out.clip_position = pos_base + vec4<f32>(cross_dir * vec3<f32>(thickness, 0.0), 0.0) * ((model.position.x - 0.5) * pos_base.w);

	out.colour = model.colour * instance.colour;
	out.tex_coords = model.uv;
    return out;
}

[[stage(fragment)]]
fn fs_main(in: VertexOutput) -> [[location(0)]] vec4<f32> {
	return in.colour;
}
