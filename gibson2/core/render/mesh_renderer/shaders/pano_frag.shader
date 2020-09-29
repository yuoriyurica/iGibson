#version 410
uniform sampler2D texUnit;
uniform float use_texture;

in VS_OUT
{
    vec2 theCoords;
    vec3 Normal;
    vec3 FragPos;
    vec3 Normal_cam;
    vec3 Instance_color;
    vec3 Pos_cam;
    vec3 Diffuse_color;
} fs_in;

layout (location = 0) out vec4 outputColour;
layout (location = 1) out vec4 NormalColour;
layout (location = 2) out vec4 InstanceColour;
layout (location = 3) out vec4 PCColour;

uniform vec3 light_position;  // in world coordinate
uniform vec3 light_color; // light color

void main() {
    float ambientStrength = 0.2;
    vec3 ambient = ambientStrength * light_color;
    vec3 lightDir = normalize(light_position - fs_in.FragPos);
    float diff = 0.5 + 0.5 * max(dot(fs_in.Normal, lightDir), 0.0);
    vec3 diffuse = diff * light_color;

    if (use_texture == 1) {
        outputColour = texture(texUnit, fs_in.theCoords);// albedo only
    } else {
        outputColour = vec4(fs_in.Diffuse_color,1) * diff; //diffuse color
    }

    NormalColour =  vec4((fs_in.Normal_cam + 1) / 2,1);
    InstanceColour = vec4(fs_in.Instance_color,1);
    PCColour = vec4(vec3(length(fs_in.Pos_cam) / 8),1);
}