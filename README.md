# SauRay<sup>TM</sup> for CS:GO (via IPC through SourceMod)

This is a branch of HighOmega hosting [SauRay<sup>TM</sup>](http://sauray.tech) for [our TF2 SourceMod plugin](https://github.com/toomuchvoltage/SauRay/tree/master/TF2). Please note that SauRay<sup>TM</sup> branches of this repository -- such as this -- only contain implementation details relevant to the associated game.

If you are unfamiliar with SauRay<sup>TM</sup>, please read [our technical brief](http://toomuchvoltage.com/pub/sauray_techbrief/sauray_techbrief.pdf) or visit [our website](http://sauray.tech).

# License

Implementation details pertaining to SauRay<sup>TM</sup> are covered under [US20220219086A1](https://patents.google.com/patent/US20220219086A1) and are provided under the following license:

```
Copyright © 2023 TooMuchVoltage Software Inc. This notice shall always be coupled with any SauRay(TM) implementation and must be redistributed alongside it.

This implementation of US20220219086A1 is provided royalty free for either of the following:

* Games with gross revenues of under one(1) million dollars CAD.
* Games with at least a publically distributed moddable server binary with which SauRay(TM) is successfully integrable.

Public distribution requires either a public download link or a relatively simple registration and download process. If you are unsure of your registration process's straightforwardness, reach out directly.

Free open-source games (i.e. Cube/Sauerbraten or Xonotic) automatically qualify since successful SauRay(TM) integration is ultimately feasible with sufficient effort.

Open source games with non-Libre licenses (i.e. non-GPL, non-MIT) also qualify as long as the license is no further restrictive than that of Quake(idTech) II's. If unsure of whether your source code redistribution license is permissive enough, please reach out directly.

For games where at least the distributed server component is either open-source or moddable (in a manner permissible by the IP owner) the game must be sufficiently thin-client so that a SauRay(TM) integration does not result in crashes or defects that largely break the game in most multiplayer game modes. If you are unsure of whether your distributed binaries qualify for this category, please get in touch directly.

We can be reached at the email address: sauray@toomuchvoltage.com or using the contact information found on the website http://sauray.tech .

If your game does not qualify under either of the above categories, contact us for a commercial license. The covered source files are protected by copyright and the aforementioned terms will apply beyond the life of US20220219086A1.

All games using US20220219086A1 or this implementation of it must clearly declare that they're using it in a way noticeable and comprehensible by an average player of the game in the English language.

Beyond what is stated in http://toomuchvoltage.com/pub/sauray_techbrief/sauray_techbrief.pdf this source code does not provide any warranties of merchantability or fitness for any particular purpose.
```

All else -- i.e. regular HighOmega source code -- is provided under the MIT license. A complete version of HighOmega will be provided under the MIT license at a later date.

# Why IPC?

Source1 is a 32 bit engine and thus native SourceMod extensions will have to be 32 bit DLLs/SOs as well. Cross-vendor raytracing via VK_KHR_ray_tracing requires a 64-bit Vulkan instance. This method of integration shows that non-invasive/fully-decoupled integrations of SauRay<sup>TM</sup> are also possible.

# Requirements

Vulkan SDK and Visual Studio 2022 are required to compile. A video card with hardware accelerated ray-tracing is required. Note that with increased player count, raytracing and compute demands of the card increase. Currently our implementation can host a single competitive match per an RTX 2080Ti at 128 ticks/second or two at 64 ticks/second with a 640x640 resolution for every player.

# Extracted Maps

Any maps extracted using the content creation pipeline outlined [here](https://github.com/toomuchvoltage/SauRay/tree/master/TF2) must be placed in `assets/maps/tf2`. Currently `koth_badlands.bsp` is converted and can be found as `koth_badlands.txt` in that directory. This implies that this content is only sufficient for the badlands maps and only in the King of The Hill gameplay mode. Extract more content to support more modes/maps! All other converted maps should also have the `.txt` extension.

# How to Compile and Use

Open the Visual Studio project and compile. Pick `Release` unless you're debugging. This produces the binary that our SourceMod plugin will talk to in `x64/Release`. When running the binary, please provide a window number via a single command line argument (default is 1): i.e. `HighOmega.exe 2`. This allows several instances to run on the same machine for different server instances and with different settings (i.e. different resolutions or debug flags). Ensure that SauRay<sup>TM</sup> is running before the server is launched as TF2's `srcds` does not have hibernation like CS:GO's and will run the server loop even if there are no players present on the server.

In the event that you modify raytracing shaders in `source_material/shaders`:
* Run `spirv-compiler.bat` in the same folder
* Run `spirv-optimizer.bat` in the `shaders` folder for generating a new `encodedshaders.h`
* Recompile the project

# Contact Info

Either email [sauray@toomuchvoltage.com](mailto:sauray@toomuchvoltage.com) or visit the contact page on [SauRay<sup>TM</sup>'s website](http://sauray.tech).
For extension requests, you can also open issues on this GitHub repository as well.