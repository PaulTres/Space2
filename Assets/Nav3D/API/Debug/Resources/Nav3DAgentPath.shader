//Line shader for Nav3DAgentPathDrawer, drawn with two materials so a path stays readable where geometry hides it: a dimmer pass
//behind the depth buffer, the full one in front. Both edges of the line are a rim in a contrasting tone of the line's own
//colour (dark on a light colour, light on a dark one), so the line reads on a bright sky and on dark geometry alike.
Shader "Nav3D/AgentPath"
{
    Properties
    {
        _Color        ("Tint", Color)                    = (1,1,1,1)
        _OccludedFade ("Alpha multiplier", Range(0,1))   = 1
        [Enum(UnityEngine.Rendering.CompareFunction)] _ZTest ("Depth test", Float) = 4
        _Outline      ("Outline fraction", Range(0,0.5)) = 0.125
    }

    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "IgnoreProjector" = "True" }

        Blend SrcAlpha OneMinusSrcAlpha
        ZWrite Off
        Cull Off
        Lighting Off

        CGINCLUDE

        #include "UnityCG.cginc"

        fixed4 _Color;
        float  _OccludedFade;
        float  _Outline;

        struct appdata
        {
            float4 vertex : POSITION;
            fixed4 color  : COLOR;
            float2 uv     : TEXCOORD0;
        };

        struct v2f
        {
            float4 pos   : SV_POSITION;
            fixed4 color : COLOR;
            float2 uv    : TEXCOORD0;
        };

        v2f vert(appdata _In)
        {
            v2f output;

            output.pos   = UnityObjectToClipPos(_In.vertex);
            output.color = _In.color * _Color;
            output.uv    = _In.uv;

            return output;
        }

        //the rim tone of a colour: the colour pulled toward black when it is light, toward white when it is dark
        fixed3 RimTone(fixed3 _Rgb)
        {
            float luminance = dot(_Rgb, float3(0.2126, 0.7152, 0.0722));

            return luminance > 0.5 ? _Rgb * 0.12 : lerp(_Rgb, fixed3(1, 1, 1), 0.8);
        }

        //the colour across the line's width: the rim over _Outline of the width at each edge, the colour itself between
        fixed4 Shade(v2f _In)
        {
            fixed4 result = _In.color;

            float edge = min(_In.uv.y, 1.0 - _In.uv.y);
            float aa   = fwidth(edge);
            float core = smoothstep(_Outline - aa, _Outline + aa, edge);

            result.rgb = lerp(RimTone(result.rgb), result.rgb, core);

            return result;
        }

        ENDCG

        Pass
        {
            ZTest [_ZTest]

            CGPROGRAM

            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment frag

            fixed4 frag(v2f _In) : SV_Target
            {
                fixed4 result = Shade(_In);

                result.a *= _OccludedFade;

                return result;
            }

            ENDCG
        }
    }
}
