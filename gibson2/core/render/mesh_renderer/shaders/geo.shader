#version 410
uniform mat4 V[6];
uniform mat4 P;

layout(triangles) in;
layout(triangle_strip, max_vertices = 18) out;

in VS_OUT
{
    vec2 theCoords;
    vec3 Normal;
    vec3 FragPos;
    vec3 Normal_cam;
    vec3 Instance_color;
    vec3 Pos_cam;
    vec3 Diffuse_color;
} gs_in[];

out VS_OUT
{
    vec2 theCoords;
    vec3 Normal;
    vec3 FragPos;
    vec3 Normal_cam;
    vec3 Instance_color;
    vec3 Pos_cam;
    vec3 Diffuse_color;
} gs_out;

void main() {
    int i, layer;
    for (layer = 0; layer < 6; layer++) {
        for (i = 0; i < gl_in.length(); i++) {
            gl_Layer = layer;
            gs_out.theCoords = gs_in[i].theCoords;
            gs_out.Normal = gs_in[i].Normal;
            gs_out.FragPos = gs_in[i].FragPos;
            gs_out.Normal_cam = gs_in[i].Normal_cam;
            gs_out.Instance_color = gs_in[i].Instance_color;
            gs_out.Pos_cam = gs_in[i].Pos_cam;
            gs_out.Diffuse_color = gs_in[i].Diffuse_color;
            gl_Position = P * V[layer] * gl_in[i].gl_Position;
            EmitVertex();
        }
        EndPrimitive();
    }
}