void HIGHOMEGA::WORLD::DestroyWorld(bool mapChange)
{
	...
	RTXPass::DestroyRTX();
	--> GridPass::DestroyGrid();
	...
}