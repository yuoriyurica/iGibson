#version 410
uniform mat4 V[6];
uniform mat4 P;
uniform mat4 pose_rot;
uniform mat4 pose_trans;
uniform vec3 instance_color;
uniform vec3 diffuse_color;

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;
layout (location=2) in vec2 texCoords;

out VS_OUT
{
    vec2 theCoords;
    vec3 Normal;
    vec3 FragPos;
    vec3 Normal_cam;
    vec3 Instance_color;
    vec3 Pos_cam;
    vec3 Diffuse_color;
} vs_out;

void main() {
    gl_Position = pose_trans * pose_rot * vec4(position, 1);
    vec4 world_position4 = pose_trans * pose_rot * vec4(position, 1);
    vs_out.FragPos = vec3(world_position4.xyz / world_position4.w); // in world coordinate
    vs_out.Normal = normalize(mat3(pose_rot) * normal); // in world coordinate
    vs_out.Normal_cam = mat3(pose_rot) * normal; // in camera coordinate
    vec4 pos_cam4 = V[0] * pose_trans * pose_rot * vec4(position, 1);
    vs_out.Pos_cam = pos_cam4.xyz / pos_cam4.w;
    vs_out.theCoords = texCoords;
    vs_out.Instance_color = instance_color;
    vs_out.Diffuse_color = diffuse_color;
}