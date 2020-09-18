#version 410
uniform samplerCube texUnit;
// uniform sampler2D texUnit;

layout (location = 0) out vec4 outputColour;
layout (location = 1) out vec4 NormalColour;
layout (location = 2) out vec4 InstanceColour;
layout (location = 3) out vec4 PCColour;

in VS_OUT
{
	vec2 texcoord;
} fs_in;

double pi = 3.141592653589793;
double pi2 = 6.283185307179587;

void main() {
    // if (fs_in.texcoord.y < 0.5) {
        vec2 coords = vec2(fs_in.texcoord.x - 0.5, fs_in.texcoord.y * 2 - 0.5);
        vec2 uv = vec2(coords.x * pi2, -coords.y * pi);
        vec3 e2cCoords = vec3(cos(uv.y) * sin(uv.x), sin(uv.y), cos(uv.y) * cos(uv.x));
        outputColour = texture(texUnit, e2cCoords);
    // }
    // outputColour = vec4(e2cCoords, 1.0);
    // outputColour = texture(texUnit, vec3(1.0, fs_in.texcoord));
    // outputColour = texture(texUnit, fs_in.texcoord);
    // outputColour = vec4(fs_in.texcoord, 0.0, 1.0);
}