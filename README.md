# SauRay<sup>TM</sup> for vkQuake2

This is a branch of HighOmega hosting [SauRay<sup>TM</sup>](http://sauray.tech) for [our vkQuake2 flavor](https://github.com/toomuchvoltage/SauRay/tree/master/vkQuake2). Please note that SauRay<sup>TM</sup> branches of this repository -- such as this -- only contain implementation details relevant to the associated game.

If you are unfamiliar with SauRay<sup>TM</sup>, please read [our technical brief](http://toomuchvoltage.com/pub/sauray_techbrief/sauray_techbrief.pdf) or visit [our website](http://sauray.tech).

# License

The point of this license is protection and not to be rampantly litigious. If you are really worried about reading patented source code: it's `sauray.cpp`, clearly marked parts of `render.cpp` and the raytracing shaders.
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

# Requirements

Vulkan SDK and Visual Studio 2022 are required to compile. A video card with hardware accelerated ray-tracing is required. Note that with increased player count, raytracing and compute demands of the card increase. However, given that Quake II servers have a tickrate of 10 (100ms per state update) and that SauRay<sup>TM</sup> can (and probably should) run async in most cases, it should be rather difficult to hit or exceed this limit.

# How to Compile and Use

Open the Visual Studio project and compile. Pick `Release` unless you're debugging. This project produces a static library (in `x64/Release`) that [our vkQuake2 flavor](https://github.com/toomuchvoltage/SauRay/tree/master/vkQuake2) compiles against (needs to be dropped in `ext/lib`).

In the event that you modify raytracing shaders in `source_material/shaders`:
* Run `spirv-compiler.bat` in the same folder
* Run `spirv-optimizer.bat` in the `shaders` folder for generating a new `encodedshaders.h`
* Recompile the project

# Contact Info

Either email [sauray@toomuchvoltage.com](mailto:sauray@toomuchvoltage.com) or visit the contact page on [SauRay<sup>TM</sup>'s website](http://sauray.tech).
For extension requests, you can also open issues on this GitHub repository as well.