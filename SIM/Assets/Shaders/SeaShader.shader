Shader "Custom/SeaShader"
{
    Properties
    {
        _Color ("Color", Color) = (0, 0.4, 0.7, 1)
        _MainTex ("Albedo (RGB)", 2D) = "white" {}
        _Glossiness ("Smoothness", Range(0,1)) = 0.8
        _Metallic ("Metallic", Range(0,1)) = 0.2
        _Seconds ("Time Offset", Float) = 0.0
        _AmplitudeScale ("Amplitude Scale", Float) = 1.0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 200

        CGPROGRAM
        // 1. Added 'vertex:vert' to tell Unity we are modifying positions
        // 2. Added 'addshadow' so the waves cast and receive shadows correctly
        #pragma surface surf Standard fullforwardshadows vertex:vert addshadow
        #pragma target 3.0

        sampler2D _MainTex;

        struct Input
        {
            float2 uv_MainTex;
            float worldHeight; // Pass height to color the water
        };

        half _Glossiness;
        half _Metallic;
        fixed4 _Color;
        float _Seconds;
        float _AmplitudeScale;

        // --- WAVE MATH (Exact match to your C++ logic) ---

        float calculateWave(float3 pos, float freq, float speed, float amp) {
            return sin(pos.x * freq + _Seconds * speed) * cos(pos.z * freq * 0.8 + _Seconds * speed) * amp;
        }

        float getTotalHeight(float3 pos) {
            float h = calculateWave(pos, 0.02, 1.2, 0.8 * _AmplitudeScale);
            h += calculateWave(pos, 0.04, 1.5, 0.5 * _AmplitudeScale);
            h += calculateWave(pos, 0.10, 2.2, 0.2 * _AmplitudeScale);
            h += calculateWave(pos, 0.18, 1.8, 0.15 * _AmplitudeScale);
            h += calculateWave(pos, 0.45, 3.0, 0.05 * _AmplitudeScale);
            h += calculateWave(pos, 0.80, 4.2, 0.02 * _AmplitudeScale);
            return h;
        }

        void vert(inout appdata_full v, out Input o) {
            UNITY_INITIALIZE_OUTPUT(Input, o);
            
            // Get World Position for the math
            float3 worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
            
            // 1. Calculate height at current point
            float h = getTotalHeight(worldPos);
            
            // 2. Calculate Normals using your Epsilon (Finite Difference) method
            float eps = 0.1;
            float hX = getTotalHeight(worldPos + float3(eps, 0, 0));
            float hZ = getTotalHeight(worldPos + float3(0, 0, eps));
            
            float3 tangentX = float3(eps, hX - h, 0);
            float3 tangentZ = float3(0, hZ - h, eps);
            
            // Update the vertex normal so lighting works
            v.normal = normalize(cross(tangentZ, tangentX));
            
            // 3. Offset the actual vertex position (Local space)
            v.vertex.y += h;
            o.worldHeight = h;
        }

        void surf (Input IN, inout SurfaceOutputStandard o)
        {
            // Subtle color variation based on wave height
            fixed4 c = tex2D (_MainTex, IN.uv_MainTex) * _Color;
            float foam = saturate(IN.worldHeight * 0.5); 
            
            o.Albedo = c.rgb + (foam * 0.1); // Add a tiny bit of brightness to crests
            o.Metallic = _Metallic;
            o.Smoothness = _Glossiness;
            o.Alpha = c.a;
        }
        ENDCG
    }
    FallBack "Diffuse"
}