# HighOmega Game Engine

# Powering:
* [![The HighOmega v3.0 debut demo](https://www.youtube.com/watch?v=8IRNQupyoIs)](https://www.youtube.com/watch?v=8IRNQupyoIs)
![Debut Demo gif](marketing/github_debutdemo.gif)

* [![C.L.A.S.H: Colonial Life Advancing Self-sustained Hemisphere - Chapter 1](https://store.steampowered.com/app/4796200/)](https://store.steampowered.com/app/4796200/) (Please consider buying a copy, your support is deeply appreciated!)
![C.L.A.S.H gif](marketing/github_clash.gif)

* [![SauRay(TM) Antiwallhack middleware](https://sauray.tech)](https://sauray.tech)
![SauRay(TM) gif](marketing/github_sauray.gif)

# Features:

* Its own physics solver:
  - Sequential impulse based solver with shock propagation and constraint support.
  - Spring based cloth and pressure based soft bodies.
  - Support for buoyancy via bobbies. Features linear and non-linear fluid drag.
  - Fast collision detection directly against detailed geometry using CWBVH8: https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs .
* Built-in geometry library used for live in-game cutting and shatter effects.
* A world streaming system with parallel asset loading and preparation.
* Built in multimedia functionalities and UI system. Only external dependencies are OpenAL-Soft, a modified version of LibKTX2 (https://github.com/toomuchvoltage/KTX-Software) and SDL2.
* Efficient and multi-threaded: separate threads for streaming, rendering, audio, entities, intersection detection, collision resolution, live boolean operations on geometry and compute based tessellation.
* Two-file Vulkan wrapper providing for easy rendering backend substitution. Uses Vulkan 1.1 with extensions.
* A rendering pipeline featuring:
  - A hardware pathtraced/denoised path via cross-vendor Khronos raytracing extensions.
  - A software pathtraced/denoised path via SDF-BVHs (SDF leaves sitting on a CWBVH8).
  - Cascaded cubic irradiance caches/probes in worldspace.
  - Both paths heavily optimized with very few CPU bottlenecks.
  - Visibility buffer based rendering with deferred materials. Driven via compute-based two-pass occlusion culling and indirect rendering.
  - Per-object MoBlur, volumetric fog, cut-off screenspace decals, DoF and much more.
* SSE4.1, AVX and F16C optimized vector math library.
* Single header simple serialization/de-serialization library alongside support for .ini files.
* Very flexible mesh file format with support for world streaming data, indexed geometry, skinning data, armature setup, keyframing information and inclusion of arbitrary meta-data.
* Slim readable source code: ~32k lines of C++ and ~14k lines of GLSL shaders.

# How to use:

The prerequisites are:

* VulkanSDK `1.4.313.2` or higher.
* Visual Studio Community 2022.

Simply open `HighOmega.sln`, compile and enjoy the demo. Minimum supported video card for the engine is an nVidia GeForce 1050Ti.

**NOTE:** Some source art assets for the demo were removed due to restrictions on redistribution. Re-running `publishscript.bat` with the parameter `redo` may remove their generated assets by mistake. Simply discard local changes through Git in that event.

**IF YOU CHANGE ANY SOURCE ART (.TGAs):** Delete their `.ktx` files and (re-)run `HighOmega/publishscript/publishscript.bat` to re-compress. They will use less VRAM and sample faster at runtime.

# Content creation

![SauRay(TM) gif](marketing/github_testmap.gif)

The primary platform for creating content for this engine is Blender.
Install `Blender 4.0` (or higher) and the `3MDconvert.py` plugin found in `HighOmega/source_material/exporter`.
Since nearly all meta-data parsed by the engine are on `Object custom properties`, installing the `Copy Attributes Menu` addon is also recommended.
Setting `pipelineSetupReturn.newMapBelong = "source_material/dev_test_maps/test_zones/"` in `main.cpp` will take you to the engine test map used for testing various features.
Opening `HighOmega/source_material/dev_test_maps/test_zones/test_zones.blend` will show you how this environment is made. It must be exported with the `Terrain export` flag checked on the export dialog.
Unchecking this usually means that you are working on an external module that can be linked via linked collections.
This map has plenty of examples of how this is done, including linked collections inside linked collections that can recursively bring in rigid bodies with the custody chain intact.

# Modifying shaders

The shaders are located in `HighOmega/source_material/shaders`. If you wish to modify them:

* Re-run `spirv-compiler.bat`.
* Followed by re-running `spirv-optimizer.bat` in `HighOmega/shaders/`. This both optimizes the shaders as well as regenerates `encodedshaders.h` which is embedded in the binary and used by the engine.
* Finally, recompile the solution once more. It will only recompile `gl.cpp` in all likelihood and relink.

# License

M.I.T.
