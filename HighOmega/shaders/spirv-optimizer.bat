for %%A in (*.spv) do spirv-opt --scalar-block-layout --target-env=spv1.4 -O %%A -o %%A
TxtToCPP.exe