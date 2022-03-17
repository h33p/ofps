// Unlit 3D shader

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
};

[[group(0), binding(0)]]
var<uniform> camera: CameraUniform;

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
	var instance_proj = mat4x4<f32>(instance.proj1, instance.proj2, instance.proj3, instance.proj4);
	out.clip_position = camera.proj * instance_proj * vec4<f32>(model.position, 1.0);
	out.colour = model.colour * instance.colour;
	out.tex_coords = model.uv;
    return out;
}

[[stage(fragment)]]
fn fs_main(in: VertexOutput) -> [[location(0)]] vec4<f32> {
	return in.colour;
}
