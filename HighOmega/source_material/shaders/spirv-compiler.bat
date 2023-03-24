for %%A in (*.vert) do glslangvalidator -V %%A -o %%A.spv
for %%A in (*.tesc) do glslangvalidator -V %%A -o %%A.spv
for %%A in (*.tese) do glslangvalidator -V %%A -o %%A.spv
for %%A in (*.geom) do glslangvalidator -V %%A -o %%A.spv
for %%A in (*.frag) do glslangvalidator -V %%A -o %%A.spv
for %%A in (*.comp) do glslangvalidator -V %%A -o %%A.spv

for %%A in (*.rgen) do glslangvalidator --target-env spirv1.4 -V %%A -o %%A.spv
for %%A in (*.rchit) do glslangvalidator --target-env spirv1.4 -V %%A -o %%A.spv
for %%A in (*.rmiss) do glslangvalidator --target-env spirv1.4 -V %%A -o %%A.spv
for %%A in (*.rahit) do glslangvalidator --target-env spirv1.4 -V %%A -o %%A.spv

move /Y *.spv ..\..\shaders