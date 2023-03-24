/*
	Copyright (c) 2023 TooMuchVoltage Software Inc.

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#ifndef HIGHOMEGA_SAURAY_H
#define HIGHOMEGA_SAURAY_H

#ifdef __cplusplus
#include "world.h"

extern "C" {
#endif
	extern int sauray_debug_mode;
	extern unsigned int sauray_debug_w;
	extern unsigned int sauray_debug_h;
	void sauray_setdebug(unsigned int debug_w, unsigned int debug_h);
	int sauray_start(unsigned int max_players, unsigned int player_trace_res, unsigned int sauray_temporal_history_amount);
	int sauray_feedmap_quake2(char *mapName);
	void sauray_player_csgo(unsigned int player_id, unsigned int teamId, float weaponLen,
		float absmin_x, float absmin_y, float absmin_z,
		float absmax_x, float absmax_y, float absmax_z,
		float e1x, float e1y, float e1z,
		float e2x, float e2y, float e2z,
		float look1x, float look1y, float look1z,
		float up1x, float up1y, float up1z,
		float look2x, float look2y, float look2z,
		float up2x, float up2y, float up2z,
		float yfov, float whr);
	void sauray_start_smoke_csgo(float ox, float oy, float oz);
	int sauray_can_see_quake2(unsigned int viewer, unsigned int subject);
	void sauray_randomize_audio_source(unsigned int listenerId,
		float listenerX, float listenerY, float listenerZ,
		float originX, float originY, float originZ,
		float* retOriginX, float* retOriginY, float* retOriginZ,
		float randDistance, float updateDistanceThreshold);
	void sauray_reset_round_csgo(); // BLAS/TLAS force refresh
	void sauray_remove_player(unsigned int player);
	int sauray_loop();         // Non-threaded mode..
	int sauray_thread_start(); // Threaded mode...
	int sauray_thread_join();

#ifdef __cplusplus
}
#endif

#endif