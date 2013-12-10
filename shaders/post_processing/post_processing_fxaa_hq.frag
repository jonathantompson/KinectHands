#version 150
// This is a stripped down version of fxaa3_11.frag: the following includes
// were used:
// #define FXAA_PC 1
// #define FXAA_GLSL_130 1
// #define FXAA_QUALITY__PRESET 13
#define QUALITY_EDGE_THRESHOLD 0.125  // 0.166 - default
#define QUALITY_EDGE_THRESHOLD_MIN 0.0625

uniform sampler2D f_src_rgb;
uniform sampler2D f_src_luma;
uniform float f_hq_subpix_quality;  // 1 - hq, 0 - off

in vec2 f_texture;

uniform vec2 f_texel_size;

out vec4 final_color;

vec4 FxaaPixelShader(
	vec2 pos,
	sampler2D tex_rgb,
	sampler2D tex_luma,
	vec2 fxaaQualityRcpFrame,
	float fxaaQualitySubpix,
	float fxaaQualityEdgeThreshold,
	float fxaaQualityEdgeThresholdMin
)
{
	vec2 posM;
	posM.x = pos.x;
	posM.y = pos.y;
	vec4 rgbyM;
  rgbyM.xyz = texture(tex_rgb, f_texture).xyz;
  rgbyM.w = texture(tex_luma, f_texture).x;
  float lumaS = texture(tex_luma, f_texture + (vec2(+0, +1) * f_texel_size)).x;
  float lumaE = texture(tex_luma, f_texture + (vec2(+1, +0) * f_texel_size)).x;
  float lumaN = texture(tex_luma, f_texture + (vec2(+0, -1) * f_texel_size)).x;
  float lumaW = texture(tex_luma, f_texture + (vec2(-1, +0) * f_texel_size)).x;
	float maxSM = max(lumaS, rgbyM.w);
	float minSM = min(lumaS, rgbyM.w);
	float maxESM = max(lumaE, maxSM);
	float minESM = min(lumaE, minSM);
	float maxWN = max(lumaN, lumaW);
	float minWN = min(lumaN, lumaW);
	float rangeMax = max(maxWN, maxESM);
	float rangeMin = min(minWN, minESM);
	float rangeMaxScaled = rangeMax * fxaaQualityEdgeThreshold;
	float range = rangeMax - rangeMin;
	float rangeMaxClamped = max(fxaaQualityEdgeThresholdMin, rangeMaxScaled);

	bool earlyExit = range < rangeMaxClamped;
	if(earlyExit)
		return rgbyM;

  float lumaNW = texture(tex_luma, f_texture + (vec2(-1, -1) * f_texel_size)).x;
  float lumaSE = texture(tex_luma, f_texture + (vec2(+1, +1) * f_texel_size)).x;
  float lumaNE = texture(tex_luma, f_texture + (vec2(+1, -1) * f_texel_size)).x;
  float lumaSW = texture(tex_luma, f_texture + (vec2(-1, +1) * f_texel_size)).x;
	float lumaNS = lumaN + lumaS;
	float lumaWE = lumaW + lumaE;
	float subpixRcpRange = 1.0/range;
	float subpixNSWE = lumaNS + lumaWE;
	float edgeHorz1 = (-2.0 * rgbyM.w) + lumaNS;
	float edgeVert1 = (-2.0 * rgbyM.w) + lumaWE;
	float lumaNESE = lumaNE + lumaSE;
	float lumaNWNE = lumaNW + lumaNE;
	float edgeHorz2 = (-2.0 * lumaE) + lumaNESE;
	float edgeVert2 = (-2.0 * lumaN) + lumaNWNE;
	float lumaNWSW = lumaNW + lumaSW;
	float lumaSWSE = lumaSW + lumaSE;
	float edgeHorz4 = (abs(edgeHorz1) * 2.0) + abs(edgeHorz2);
	float edgeVert4 = (abs(edgeVert1) * 2.0) + abs(edgeVert2);
	float edgeHorz3 = (-2.0 * lumaW) + lumaNWSW;
	float edgeVert3 = (-2.0 * lumaS) + lumaSWSE;
	float edgeHorz = abs(edgeHorz3) + edgeHorz4;
	float edgeVert = abs(edgeVert3) + edgeVert4;
	float subpixNWSWNESE = lumaNWSW + lumaNESE;
	float lengthSign = fxaaQualityRcpFrame.x;
	bool horzSpan = edgeHorz >= edgeVert;
	float subpixA = subpixNSWE * 2.0 + subpixNWSWNESE;
	if(!horzSpan) lumaN = lumaW;
	if(!horzSpan) lumaS = lumaE;
	if(horzSpan) lengthSign = fxaaQualityRcpFrame.y;
	float subpixB = (subpixA * (1.0/12.0)) - rgbyM.w;
	float gradientN = lumaN - rgbyM.w;
	float gradientS = lumaS - rgbyM.w;
	float lumaNN = lumaN + rgbyM.w;
	float lumaSS = lumaS + rgbyM.w;
	bool pairN = abs(gradientN) >= abs(gradientS);
	float gradient = max(abs(gradientN), abs(gradientS));
	if(pairN) lengthSign = -lengthSign;
	float subpixC = clamp(abs(subpixB) * subpixRcpRange, 0.0, 1.0);
	vec2 posB;
	posB.x = posM.x;
	posB.y = posM.y;
	vec2 offNP;
	offNP.x = (!horzSpan) ? 0.0 : fxaaQualityRcpFrame.x;
	offNP.y = ( horzSpan) ? 0.0 : fxaaQualityRcpFrame.y;
	if(!horzSpan) posB.x += lengthSign * 0.5;
	if( horzSpan) posB.y += lengthSign * 0.5;
	vec2 posN;
	posN.x = posB.x - offNP.x * 1.0;
	posN.y = posB.y - offNP.y * 1.0;
	vec2 posP;
	posP.x = posB.x + offNP.x * 1.0;
	posP.y = posB.y + offNP.y * 1.0;
	float subpixD = ((-2.0)*subpixC) + 3.0;
  float lumaEndN = texture(tex_luma, posN).x;
	float subpixE = subpixC * subpixC;
  float lumaEndP = texture(tex_luma, posP).x;
	if(!pairN) lumaNN = lumaSS;
	float gradientScaled = gradient * 1.0/4.0;
	float lumaMM = rgbyM.w - lumaNN * 0.5;
	float subpixF = subpixD * subpixE;
	bool lumaMLTZero = lumaMM < 0.0;
	lumaEndN -= lumaNN * 0.5;
	lumaEndP -= lumaNN * 0.5;
	bool doneN = abs(lumaEndN) >= gradientScaled;
	bool doneP = abs(lumaEndP) >= gradientScaled;
	if(!doneN) posN.x -= offNP.x * 1.5;
	if(!doneN) posN.y -= offNP.y * 1.5;
	bool doneNP = (!doneN) || (!doneP);
	if(!doneP) posP.x += offNP.x * 1.5;
	if(!doneP) posP.y += offNP.y * 1.5;
	if(doneNP) {
    if(!doneN) lumaEndN = texture(tex_luma, posN.xy).x;
    if(!doneP) lumaEndP = texture(tex_luma, posP.xy).x;
		if(!doneN) lumaEndN = lumaEndN - lumaNN * 0.5;
		if(!doneP) lumaEndP = lumaEndP - lumaNN * 0.5;
		doneN = abs(lumaEndN) >= gradientScaled;
		doneP = abs(lumaEndP) >= gradientScaled;
		if(!doneN) posN.x -= offNP.x * 2.0;
		if(!doneN) posN.y -= offNP.y * 2.0;
		doneNP = (!doneN) || (!doneP);
		if(!doneP) posP.x += offNP.x * 2.0;
		if(!doneP) posP.y += offNP.y * 2.0;
		if(doneNP) {
      if(!doneN) lumaEndN = texture(tex_luma, posN.xy).x;
      if(!doneP) lumaEndP = texture(tex_luma, posP.xy).x;
			if(!doneN) lumaEndN = lumaEndN - lumaNN * 0.5;
			if(!doneP) lumaEndP = lumaEndP - lumaNN * 0.5;
			doneN = abs(lumaEndN) >= gradientScaled;
			doneP = abs(lumaEndP) >= gradientScaled;
			if(!doneN) posN.x -= offNP.x * 2.0;
			if(!doneN) posN.y -= offNP.y * 2.0;
			doneNP = (!doneN) || (!doneP);
			if(!doneP) posP.x += offNP.x * 2.0;
			if(!doneP) posP.y += offNP.y * 2.0;
			if(doneNP) {
        if(!doneN) lumaEndN = texture(tex_luma, posN.xy).x;
        if(!doneP) lumaEndP = texture(tex_luma, posP.xy).x;
				if(!doneN) lumaEndN = lumaEndN - lumaNN * 0.5;
				if(!doneP) lumaEndP = lumaEndP - lumaNN * 0.5;
				doneN = abs(lumaEndN) >= gradientScaled;
				doneP = abs(lumaEndP) >= gradientScaled;
				if(!doneN) posN.x -= offNP.x * 4.0;
				if(!doneN) posN.y -= offNP.y * 4.0;
				doneNP = (!doneN) || (!doneP);
				if(!doneP) posP.x += offNP.x * 4.0;
				if(!doneP) posP.y += offNP.y * 4.0;
				if(doneNP) {
          if(!doneN) lumaEndN = texture(tex_luma, posN.xy).x;
          if(!doneP) lumaEndP = texture(tex_luma, posP.xy).x;
					if(!doneN) lumaEndN = lumaEndN - lumaNN * 0.5;
					if(!doneP) lumaEndP = lumaEndP - lumaNN * 0.5;
					doneN = abs(lumaEndN) >= gradientScaled;
					doneP = abs(lumaEndP) >= gradientScaled;
					if(!doneN) posN.x -= offNP.x * 12.0;
					if(!doneN) posN.y -= offNP.y * 12.0;
					doneNP = (!doneN) || (!doneP);
					if(!doneP) posP.x += offNP.x * 12.0;
					if(!doneP) posP.y += offNP.y * 12.0;
				}
			}
		}
	}

	float dstN = posM.x - posN.x;
	float dstP = posP.x - posM.x;
	if(!horzSpan) dstN = posM.y - posN.y;
	if(!horzSpan) dstP = posP.y - posM.y;

	bool goodSpanN = (lumaEndN < 0.0) != lumaMLTZero;
	float spanLength = (dstP + dstN);
	bool goodSpanP = (lumaEndP < 0.0) != lumaMLTZero;
	float spanLengthRcp = 1.0/spanLength;

	bool directionN = dstN < dstP;
	float dst = min(dstN, dstP);
	bool goodSpan = directionN ? goodSpanN : goodSpanP;
	float subpixG = subpixF * subpixF;
	float pixelOffset = (dst * (-spanLengthRcp)) + 0.5;
	float subpixH = subpixG * fxaaQualitySubpix;

	float pixelOffsetGood = goodSpan ? pixelOffset : 0.0;
	float pixelOffsetSubpix = max(pixelOffsetGood, subpixH);
	if(!horzSpan) posM.x += pixelOffsetSubpix * lengthSign;
	if( horzSpan) posM.y += pixelOffsetSubpix * lengthSign;

  return vec4(texture(tex_rgb, posM).xyz, 1.0);
}

void main() {
  final_color = FxaaPixelShader(f_texture, f_src_rgb, f_src_luma, f_texel_size,
    f_hq_subpix_quality, QUALITY_EDGE_THRESHOLD, QUALITY_EDGE_THRESHOLD_MIN);
}
